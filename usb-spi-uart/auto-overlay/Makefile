ifneq ($(KERNELRELEASE),)
# SPDX-License-Identifier: GPL-2.0
obj-m += dtbocfg.o
dtbocfg-m := dtbocfg-core.o test.dtb.o

else

KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

endif

.PHONY: clean

clean:
	rm -f modules.order Module.symvers *.o *.ko *.mod.* *.mod *.S *.dtb *.o.dtb *.dtbo

# enable creation of __symbols__ node
DTC_FLAGS_overlay += -@
DTC_FLAGS_overlay_bad_phandle += -@
DTC_FLAGS_overlay_bad_symbol += -@
DTC_FLAGS_overlay_base += -@
DTC_FLAGS_testcases += -@

# suppress warnings about intentional errors
DTC_FLAGS_testcases += -Wno-interrupts_property \
	-Wno-node_name_vs_property_name \
	-Wno-interrupt_map

# Apply overlays statically with fdtoverlay.  This is a build time test that
# the overlays can be applied successfully by fdtoverlay.  This does not
# guarantee that the overlays can be applied successfully at run time by
# unittest, but it provides a bit of build time test coverage for those
# who do not execute unittest.
#
# The overlays are applied on top of static_base_1.dtb and static_base_2.dtb to
# create static_test_1.dtb and static_test_2.dtb.  If fdtoverlay detects an
# error than the kernel build will fail.  static_test_1.dtb and
# static_test_2.dtb are not consumed by unittest.
#
# Some unittest overlays deliberately contain errors that unittest checks for.
# These overlays will cause fdtoverlay to fail, and are thus not included
# in the static test:
#			  overlay_bad_add_dup_node.dtbo \
#			  overlay_bad_add_dup_prop.dtbo \
#			  overlay_bad_phandle.dtbo \
#			  overlay_bad_symbol.dtbo \


DTC_FLAGS_test += -Wno-interrupts_property \
	-Wno-node_name_vs_property_name \
	-Wno-interrupt_map

static_test_1-dtbs := static_test_1.dtb $(apply_static_overlay_1)
#static_test_2-dtbs := static_base_2.dtb $(apply_static_overlay_2)

#dtb-y += test.dtb
