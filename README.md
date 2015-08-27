# KernelDriverCourse2015
Files for the Linux Kernel Driver Programming Course 2015, see https://revspace.nl/KernelDriverProgrammingCourse2015

focaltech_tf5x
==============
This is a simplified driver that talks to a focaltech ft5x touchscreen controller over i2c, interprets the touch data and creates events for the linux input event subsystem.
It uses the device tree.
This driver makes the touch screen work on a Chuwi V7 CW0825.

NOTE: before this works, you also need an updated DTS for the Chuwi, see the .patch file.

To build:
* edit the Makefile to make it point to your kernel source tree
* run make
  make ARCH=arm CROSS_COMPILE=<your cross-compiler prefix>
  
To use:
* copy the .ko file to the Chuwi storage (e.g. SD card)
* on the Chuwi, after booting, insert the kernel module
  insmod focaltech_ft5x.ko
* touch the screen and see the cursor move
