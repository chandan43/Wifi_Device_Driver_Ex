DRIVER_KMOD = rtl8180

obj-m := $(DRIVER_KMOD).o

$(DRIVER_KMOD)-y := dev.o

KDIR=/lib/modules/$(shell uname -r)/build

all: 
	$(MAKE) -C $(KDIR) M=$(PWD) modules

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a 
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean 
