########################################################################
# Makefile:
#      Copyright (c) 2020 Ricardo Bustos
#
#    Customized raspi GPIOS driver for the raspberry pi 3+ is a free
#    software: you can redistribute it and/or modify it under the terms
#    of the GNU Lesser General Public License as published by the Free
#    Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    Customized raspi GPIOS driver for the raspberry pi 3+ is distributed
#    in the hope that it will be useful, but WITHOUT ANY WARRANTY;
#    without even the implied warranty of MERCHANTABILITY or FITNESS
#    FOR A PARTICULAR PURPOSE.
#    See the GNU Lesser General Public License for more details.
#
##########################################################################

obj-m += raspi_gpios_device.o

KDIR = /lib/modules/$(shell uname -r)/build

all:
	make -C $(KDIR) M=$(shell pwd) modules
clean:
	make -C $(KDIR) M=$(shell pwd) clean
