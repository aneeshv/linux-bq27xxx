#
# Makefile for nfc devices
#

obj-$(CONFIG_PN544_NFC)		+= pn544.o
obj-$(CONFIG_NFC_PN533)		+= pn533.o
obj-$(CONFIG_NFC_WILINK)	+= nfcwilink.o

#ifdef CONFIG_MACH_AM335XEVM_WILINK8
obj-$(CONFIG_ST_NFC)            += wl_nfc.o
#endif

ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG
