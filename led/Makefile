kVERS = $(shell uname -r)

KERN_DIR = /home/adrain/100ask/100ask_imx6ull-sdk/Linux-4.9.88/build
export ARCH=arm
export CROSS_COMPILE=/opt/20161016/gcc-linaro-6.2.1-2016.11-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
#kernel modules
obj-m += led_dts.o

#specify flags for the module compilation
#EXTRA_CFLAGS = -g -O0


build: kernel_modules

kernel_modules:
    #make -C /lib/modules/$(KVERS)/build M=$(CURDIR) modules
	make -C $(KERN_DIR) M=`pwd` modules

clean:
    #make -C /lib/modules/$(KVERS)/builds M=$(CURDIR) clean

	make -C $(KERN_DIR) M=`pwd` modules clean
	rm -rf modules.order
	rm -f globalmem.ko


