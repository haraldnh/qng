# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)
obj-m := qng.o
# Otherwise we were called directly from the command
# line; invoke the kernel build system.
else
KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	rm -rf modules.order .qng.ko.cmd qng.o Module.symvers qng.mod.c \
		.qng.o.cmd qng.mod.o qng.ko .qng.mod.o.cmd .tmp_versions
endif
