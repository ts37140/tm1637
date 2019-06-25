obj-m		:= tm1637.o
tm1637-objs	:= tm1637_drv.o
ARCH		:= arm
PWD		:= $(shell pwd)

modules:
	make -C $(KERN_SRC) C=1 M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CCPREFIX) modules

clean:
	make -C $(KERN_SRC) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CCPREFIX) clean

raspi_install:
	scp tm1637.ko $(RASPI_USER)@$(RASPI_ADDR):$(RASPI_PATH)