Please use the following steps to integrate LIS2DS12 driver into your kernel:

- copy the include and drivers folders into the root of kernel source:
	cp -r drivers <KERNEL_SRC_ROOT>/
	cp -r include <KERNEL_SRC_ROOT>/

- edit input Kconfig (drivers/input/misc/Kconfig) to include lis2ds12 config
  adding the following line:
	source "drivers/input/misc/lis2ds12/Kconfig"

- edit input Makefile (drivers/input/misc/Makefile) adding the following line:
	obj-y += lis2ds12/


To enable this driver you have to make the needed changes in the board file or
into platform device tree.
An example of binding into device tree is shown below:

&i2c4 {
	pinctrl-names = "default";

	lis2ds12@1d {
		pinctrl-names = "default";
		pinctrl-0 = <&mcspi1_pins>;
		compatible = "st,lis2ds12";
		reg = <0x1d>;
		interrupts = <8 0x0>;
		interrupt-parent = <&gpio5>;

		st,drdy-int-pin = <1>;
	};
};
