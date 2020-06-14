/*
 * raspi_gpios_device.c:
 *      Copyright (c) 2020 Ricardo Bustos
 ***********************************************************************
 *
 *    Customized raspi GPIOS driver for the raspberry pi 3+ is a free 
 *    software: you can redistribute it and/or modify it under the terms 
 *    of the GNU Lesser General Public License as published by the Free 
 *    Software Foundation, either version 3 of the License, or 
 *    (at your option) any later version.
 *
 *    Customized raspi GPIOS driver for the raspberry pi 3+ is distributed 
 *    in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 *    without even the implied warranty of MERCHANTABILITY or FITNESS 
 *    FOR A PARTICULAR PURPOSE.
 *    See the GNU Lesser General Public License for more details.
 *
 ***********************************************************************
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <uapi/asm-generic/errno-base.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <asm/io.h>

#define RASPI_MODULE_N     "customized_raspi_gpios"
#define RASPI_NUM_GPIOS    (27 - 2) // range of GPIOs is from 2 to 26. GPIO 0 and 1 are reserved for advanced use.
#define RASPI_LAST_GPIO    26
#define RASPI_FIRST_GPIO   2

#define KBUF_SIZE 32

enum direction {in, out};
enum state {low, high};

/* Main structure used per device file */

struct raspi_gpio_device {
    int num_gpio;
    enum direction dir;
    enum state st;
    struct cdev cdev;
    spinlock_t lock;
};

/* Main characteristics of GPIO device file */

struct params_gpio {
    enum direction dir;
    int num_gpio;
};

static dev_t dev;
static struct class *raspi_gpio_class;

/* Main structure allocated dynamically with
   the number of GPIOs entered as parameters
*/

static struct raspi_gpio_device **raspi_gpio_devices;

static struct params_gpio params_gpios[RASPI_NUM_GPIOS];

static int out_gpios[RASPI_NUM_GPIOS];
static int in_gpios[RASPI_NUM_GPIOS];

static int num_out, num_in;

/* Module parameters */

module_param_array(out_gpios, int, &num_out, S_IRUSR|S_IWUSR);
module_param_array(in_gpios, int, &num_in, S_IRUSR|S_IWUSR);

/* Declaration of Entry point functions */

static int raspi_gpios_open(struct inode *inode, struct file *filp);

static ssize_t raspi_gpios_read ( struct file *filp,
 char *buf,
 size_t count,
 loff_t *f_pos);

static ssize_t raspi_gpios_write (struct file *filp,
 const char *buf,
 size_t count,
 loff_t *f_pos);

static int raspi_gpios_release(struct inode *inode, struct file *filp);

/* File operations structure per file device */

static struct file_operations raspi_gpio_fops = {
    .owner = THIS_MODULE,
    .open = raspi_gpios_open,
    .release = raspi_gpios_release,
    .read = raspi_gpios_read,
    .write = raspi_gpios_write,
};

/*
 * raspi_gpios_open:
 *      This function is executed from userspace when calling open.
 *      It stores in a pointer the structure of the opened device 
 *      for later usage.
 *
 *      Returns zero if no error was found. Otherwise, returns
 *      a negative integer which indicates an specific error.
 *********************************************************************************
 */
static int raspi_gpios_open(struct inode *inode, struct file *filp) {

    struct raspi_gpio_device *raspi_gpio_devp = NULL;

    raspi_gpio_devp = container_of(inode->i_cdev, struct raspi_gpio_device, cdev);
   
    filp->private_data = raspi_gpio_devp;

    return 0;
}

/*
 * raspi_gpios_release:
 *      This function is executed from userspace when calling close.
 *
 *      Returns zero if no error was found. Otherwise, returns
 *      a negative integer which indicates an specific error.
 *********************************************************************************
 */
static int raspi_gpios_release(struct inode *inode, struct file *filp) {

//    struct raspi_gpio_device *raspi_gpio_devp = NULL;
//    raspi_gpio_devp = container_of(inode->i_cdev, struct raspi_gpio_device, cdev);

    return 0;
}

/*
 * raspi_gpios_read:
 *      This function is executed from userspace when calling read.
 *      If the GPIO is an input it will let the user read from
 *      the device file representing thie GPIO.
 *
 *      Depending on the state of the GPIO, it will return either:
 *          0 (LOW)
 *          1 (HIGH)
 *
 *      Returns the number of bytes read from the device
 *********************************************************************************
 */
static ssize_t raspi_gpios_read ( struct file *filp,char *buf, size_t count, loff_t *f_pos) {

    ssize_t retval = 0;
    char byte;
    struct raspi_gpio_device *raspi_gpio_devp = NULL;

    raspi_gpio_devp = filp->private_data;

    if (raspi_gpio_devp != NULL) {

    	if (raspi_gpio_devp->dir == in) {

	    for (retval = 0; retval < count; ++retval) {
                byte = '0' + gpio_get_value(raspi_gpio_devp->num_gpio);

                if(put_user(byte, buf+retval))
                    break;
            }

	    printk(KERN_INFO "%s: Reading from GPIO: %d\n", RASPI_MODULE_N, raspi_gpio_devp->num_gpio);
	}	
    }

    return retval;
}

/*
 * raspi_gpios_write:
 *      This function is executed from userspace when calling write.
 *      If the GPIO is an output it will let the user write into
 *      the device file representing thie GPIO.
 *
 *      It only allows to set values of either:
 *          0 (LOW)
 *          1 (HIGH)
 * 
 *      Returns the number of bytes that are written into the device
 *********************************************************************************
 */
static ssize_t raspi_gpios_write (struct file *filp, const char *buf, size_t count, loff_t *f_pos) {
    
    int ret;
    unsigned len;
    char kbuf[KBUF_SIZE];
    struct raspi_gpio_device *raspi_gpio_devp = NULL;

    raspi_gpio_devp = filp->private_data;

    if (raspi_gpio_devp != NULL) {

	if (raspi_gpio_devp->dir == out) {

            len = count >= KBUF_SIZE ? KBUF_SIZE-1 : count-1;

            ret = copy_from_user(kbuf, buf, count);

            kbuf[len] = '\0';

	    printk(KERN_INFO "%s: Writing string: %s to GPIO: %d\n", RASPI_MODULE_N, kbuf, raspi_gpio_devp->num_gpio);

	    if ( (strcmp(kbuf, "1") == 0)    || 
		 (strcmp(kbuf, "HIGH") == 0) || 
		 (strcmp(kbuf, "high") == 0) ||
		 (strcmp(kbuf, "High") == 0) ) {

    		spin_lock(&raspi_gpio_devp->lock);
                gpio_direction_output(raspi_gpio_devp->num_gpio, high);
                spin_unlock(&raspi_gpio_devp->lock);

	    } else if ( (strcmp(kbuf, "0") == 0)   || 
                        (strcmp(kbuf, "LOW") == 0) ||
		        (strcmp(kbuf, "low") == 0) ||
		        (strcmp(kbuf, "Low") == 0) ) {

                spin_lock(&raspi_gpio_devp->lock);
                gpio_direction_output(raspi_gpio_devp->num_gpio, low);
                spin_unlock(&raspi_gpio_devp->lock);

	    }		    
        }
    }

    return count;
}

/*
 * raspi_gpios_validate_params:
 *      This function checks the parameters set by the user
 *      when loading the driver.
 *
 *      Returns zero if no error was found. Otherwise, returns
 *      a negative integer which indicates an specific error.
 *********************************************************************************
 */
static int raspi_gpios_validate_params(void) {
    
    int i, j;
    int k = 0;

    if (num_out + num_in == 0) {
        printk(KERN_ERR "%s: Error: no gpio parameter(s) set\n", RASPI_MODULE_N);
        return -EINVAL;	
    }

    // Checking intputs
    for (i = 0; i < num_in; i++) {
        if (in_gpios[i] < RASPI_FIRST_GPIO || in_gpios[i] > RASPI_LAST_GPIO) {
            printk(KERN_ERR "%s: Error: invalid value [%d] for parameter in_gpios\n", RASPI_MODULE_N, in_gpios[i]);
            return -EINVAL;
        } else {
            params_gpios[k].num_gpio = in_gpios[i];
	    params_gpios[k].dir = in;
	    k++;
	}
    }

    // Checking outputs
    for (j = 0; j < num_out; j++) {
        if (out_gpios[j] <= 1 || out_gpios[j] >= RASPI_LAST_GPIO) {
            printk(KERN_ERR "%s: Error: invalid value [%d] for parameter out_gpios\n", RASPI_MODULE_N, out_gpios[j]);
            return -EINVAL;
	} else {
	    params_gpios[k].num_gpio = out_gpios[j];
	    params_gpios[k].dir = out;
	    k++;
	}
    }

    // Checking for repeated input with outputs
    for (i = 0; i < num_in; i++) {
        for (j = 0; j < num_out; j++) {
            if ( in_gpios[i] == out_gpios[j]) {
                printk(KERN_ERR "%s: Error: it is not possible to assign the same gpio as input and output. parameters: in_gpios [%d] out_gpios [%d]\n", RASPI_MODULE_N, in_gpios[i], out_gpios[j]);
                return -EINVAL;
	    }
        }
    }

    return 0;
}

/*
 * raspi_gpios_init:
 *      This function is called when loading the driver.
 *      It allocates and creates the neccesary devices.
 *
 *      When inserting it requires at least one paramater of either:
 *          in_gpios
 *          out_gpios      
 *
 *      e.g.
 *         sudo insmod raspi_gpios_device.ko in_gpios=11,14,15 out_gpios=2,4,6  
 *
 *      Returns zero if no error was found. Otherwise, returns
 *      a negative integer which indicates an specific error.
 *********************************************************************************
 */
static int __init raspi_gpios_init(void) {

    int ret, i, j;

    ret = raspi_gpios_validate_params();

    if (ret < 0) {
        goto error_return;
    }

    ret = alloc_chrdev_region(&dev, 0, RASPI_NUM_GPIOS, RASPI_MODULE_N);
      
    if ( ret < 0) {	    
        printk(KERN_ERR "%s: Error: Cannot register device\n", RASPI_MODULE_N);
	goto error_return;
    }

    raspi_gpio_class = class_create(THIS_MODULE, RASPI_MODULE_N);

    if (raspi_gpio_class == NULL) {
	printk(KERN_ERR "%s: Error: Cannot create class\n", RASPI_MODULE_N);
	ret = -ENOMEM;
	goto unreg_chr_region;
    }

    raspi_gpio_devices = (struct raspi_gpio_device **)
		          kmalloc( (num_out + num_in) *
		                    sizeof(struct raspi_gpio_device *),
                                    GFP_KERNEL );


    if (raspi_gpio_devices == NULL) {
        printk(KERN_ERR "%s: Error: Unable to allocate dynamic memory for raspi_gpio_devices\n", 
                         RASPI_MODULE_N);
        ret = -ENOMEM;
        goto destroy_class;
    }

    for (i = 0; i < num_out + num_in; i++) {

        raspi_gpio_devices[i] = kmalloc(sizeof(struct raspi_gpio_device),
                                        GFP_KERNEL);

        if ( raspi_gpio_devices[i] == NULL) {
            printk(KERN_ERR "%s: Error: Unable to allocate dynamic memory for a raspi_gpio_device indexed = %d\n", 
                             RASPI_MODULE_N, i);

            ret = -ENOMEM;
            goto unalloc_gpio_dev;
        }

        spin_lock_init(&raspi_gpio_devices[i]->lock);
        raspi_gpio_devices[i]->dir = params_gpios[i].dir;
        raspi_gpio_devices[i]->num_gpio = params_gpios[i].num_gpio;

        if (raspi_gpio_devices[i]->dir == in) {

            ret = gpio_request_one(raspi_gpio_devices[i]->num_gpio, GPIOF_IN, NULL);
 
            if (ret < 0) {
                printk(KERN_ERR "%s: Error: couldn't request GPIO [%d] as input\n", 
                                 RASPI_MODULE_N, raspi_gpio_devices[i]->num_gpio);

                goto free_raspi_gpio;
            }
        } else {

            ret = gpio_request_one(raspi_gpio_devices[i]->num_gpio, GPIOF_OUT_INIT_LOW, NULL);

            if (ret < 0) {
                printk(KERN_ERR "%s: Error: couldn't request GPIO [%d] as output\n",
                                 RASPI_MODULE_N, raspi_gpio_devices[i]->num_gpio);

                goto free_raspi_gpio;
            }
        }

        cdev_init(&raspi_gpio_devices[i]->cdev, &raspi_gpio_fops);

        ret = cdev_add(&raspi_gpio_devices[i]->cdev,(dev + i), 1);
            
        if (ret < 0) {
            printk(KERN_ERR "%s: Error: couldn't add device indexed = %d to system\n", 
                             RASPI_MODULE_N, i);
            goto destroy_dev;
        }

        if (device_create( raspi_gpio_class,
                           NULL,
                           MKDEV(MAJOR(dev),
                           MINOR(dev)+i),
                           NULL,
                           "CusRaspiGpio%d",
                           params_gpios[i].num_gpio) == NULL) {

            printk(KERN_ERR "%s: Error: couldn't create device for GPIO [%d]\n", 
                             RASPI_MODULE_N, params_gpios[i].num_gpio);
            ret = -ENOMEM;
            goto destroy_dev;
        }
    }

    printk(KERN_INFO "%s: Kernel module successfully initialized\n", RASPI_MODULE_N);

    return 0;

destroy_dev:
    device_destroy( raspi_gpio_class, 
		    MKDEV(MAJOR(dev), 
	            MINOR(dev)+i));
free_raspi_gpio:

    if (raspi_gpio_devices[i] != NULL) {
        gpio_free(raspi_gpio_devices[i]->num_gpio);
	kfree(raspi_gpio_devices[i]);
	raspi_gpio_devices[i] = NULL;
    }

unalloc_gpio_dev:
    for (j = 0; j < i; j++) {

        device_destroy( raspi_gpio_class,
			MKDEV(MAJOR(dev),
			MINOR(dev)+j));	

        if (raspi_gpio_devices[j] != NULL) {
             gpio_free(raspi_gpio_devices[j]->num_gpio);
             kfree(raspi_gpio_devices[j]);
	     raspi_gpio_devices[j] = NULL; 
	}
    }

    kfree(raspi_gpio_devices);
    raspi_gpio_devices = NULL;

destroy_class:
    
    class_destroy(raspi_gpio_class);

unreg_chr_region:

    unregister_chrdev_region(dev, RASPI_NUM_GPIOS);

error_return:

    printk(KERN_INFO "%s: Unable to initialize kernel module\n", RASPI_MODULE_N);

    return ret;
}

/*
 * raspi_gpios_exit:
 *     This function is called when unloading the driver.
 *     It frees all memory allocated and devices created.
 *
 *     sudo rmmod raspi_gpios_device
 *
 *********************************************************************************
 */
static void __exit raspi_gpios_exit(void) {

    int i;

    for (i = 0; i < num_out+num_in; i++) {

	device_destroy( raspi_gpio_class,
                        MKDEV(MAJOR(dev), 
			MINOR(dev)+i));

	gpio_free(raspi_gpio_devices[i]->num_gpio);

	kfree(raspi_gpio_devices[i]);
        raspi_gpio_devices[i] = NULL;	
    }

    kfree(raspi_gpio_devices);
    raspi_gpio_devices = NULL;

    unregister_chrdev_region(dev, RASPI_NUM_GPIOS);

    class_destroy(raspi_gpio_class);

    printk(KERN_INFO "%s: kernel module successfully removed\n", RASPI_MODULE_N);
}

module_init(raspi_gpios_init);
module_exit(raspi_gpios_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ricardo Bustos");
MODULE_VERSION("1.0");
MODULE_DESCRIPTION("Customized GPIOs char device driver for Raspberry Pi 3");
