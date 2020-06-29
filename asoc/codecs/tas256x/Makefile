EXTRA_CFLAGS= -I$(shell pwd)
snd-soc-tas256x-objs := codec/src/tas256x-codec.o codec/src/tas256x-regmap.o tas256x/src/tas256x.o /src/tas256x-logic.o #algo/src/tas25xx-calib.o algo/src/tas25xx-algo-common.o
#obj-$(CONFIG_TISA_DEBUGFS_INTF) += algo/src/tas25xx-algo-debugfs.o src/smartpa-debug-common.o
#obj-$(CONFIG_TISA_BIN_INTF) += algo/src/tas25xx-algo-bin-intf.o
obj-$(CONFIG_SND_SOC_TAS256X)	+= snd-soc-tas256x.o
