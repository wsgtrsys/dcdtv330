#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x7e7b8b55, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x409bb863, __VMLINUX_SYMBOL_STR(dvb_usb_device_exit) },
	{ 0xfed1aabc, __VMLINUX_SYMBOL_STR(param_ops_short) },
	{ 0xc304f9d7, __VMLINUX_SYMBOL_STR(param_ops_int) },
	{ 0x4476941, __VMLINUX_SYMBOL_STR(param_array_ops) },
	{ 0x43c91cf4, __VMLINUX_SYMBOL_STR(usb_deregister) },
	{ 0x42dd6235, __VMLINUX_SYMBOL_STR(usb_register_driver) },
	{ 0x9d669763, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0xe4a48145, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0xf029ebee, __VMLINUX_SYMBOL_STR(usb_control_msg) },
	{ 0x12da5bb2, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0x498c0450, __VMLINUX_SYMBOL_STR(mutex_lock_interruptible) },
	{ 0x6e9dd606, __VMLINUX_SYMBOL_STR(__symbol_put) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x402b8281, __VMLINUX_SYMBOL_STR(__request_module) },
	{ 0x868784cb, __VMLINUX_SYMBOL_STR(__symbol_get) },
	{ 0x8d85e911, __VMLINUX_SYMBOL_STR(dvb_usb_device_init) },
	{ 0x2e5810c6, __VMLINUX_SYMBOL_STR(__aeabi_unwind_cpp_pr1) },
	{ 0xb1ad28e0, __VMLINUX_SYMBOL_STR(__gnu_mcount_nc) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=dvb-usb";

MODULE_ALIAS("usb:v04B4p1206d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v695Cp1206d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "DFD177A3C0CA7E051C0F17C");
