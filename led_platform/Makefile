	KERN_DIR=/home/yu/Documents/linux_sdk/kernel
all:
	make -C $(KERN_DIR) M=$(shell pwd) modules
	$(CROSS_COMPILE)gcc -o led_platform_test led_platform_test.c
clean:
	make -C $(KERN_DIR) M=$(shell pwd) modules clean
	rm -rf modules.order
obj-m += led_platform.o
