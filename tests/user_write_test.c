/*
 * user_write_test.c:
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

#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

#define CUS_GPIO_PATH    "/dev/CusRaspiGpio"

#define RASPI_LAST_GPIO    26
#define RASPI_FIRST_GPIO   2

#define PATH_SIZE 32

int main(int argc, char *argv[]) {
	int ret = 0;
	int fd;

	char dev_path[PATH_SIZE];
	char buf[2];
	size_t nbytes;
        int gpio_int;

        if (argc != 3) {
            printf("Error: you need to provide the GPIO and the value you wish to write to it\n");
            exit(0);
        }

        gpio_int = atoi(argv[1]);
	if (gpio_int < RASPI_FIRST_GPIO || gpio_int > RASPI_LAST_GPIO) {
	    printf("Error: you need to provide a valid GPIO number\n");
	    exit(0);
	}

        if ((strcmp(argv[2], "1")    == 0) ||
            (strcmp(argv[2], "HIGH") == 0) ) {
            buf[0] = '1';
	    buf[1] = '\0';  

        } else if ((strcmp(argv[2], "0") == 0) ||
                   (strcmp(argv[2], "LOW") == 0) ) {
            buf[0] = '0';
	    buf[1] = '\0';

	} else {
            printf("Error: invalid value to write to GPIO %d\n", gpio_int);
            exit(0);
        }

        strcpy(dev_path, CUS_GPIO_PATH);
        strcat(dev_path, argv[1]);

	fd = open(dev_path, O_RDWR);
	if (fd < 0) {
            printf("Error: Unable to open: %s\n", dev_path);
            exit(0);
        }

        nbytes = sizeof(buf);

        ret = write(fd, buf, nbytes);
        if (ret <= 0) {
            printf("Error: Could not write into: %s\n", dev_path);
    	    close(fd);
        }

        printf("Wrote %s to device: %s\n", buf, dev_path);

	close(fd);

	return 0;
}
