KERNEL_HEADER=/home/bertrik/code/course/linux

obj-m += focaltech_ft5x.o

all:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C $(KERNEL_HEADER) M=$(PWD) modules
#	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C $(KERNEL_HEADER) M=$(PWD) clean
