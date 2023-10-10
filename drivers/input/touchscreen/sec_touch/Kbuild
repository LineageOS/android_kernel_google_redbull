obj-$(CONFIG_TOUCHSCREEN_SEC_TS)   += sec_touch.o
sec_touch-objs   += sec_cmd.o sec_ts.o sec_ts_fw.o sec_ts_fn.o
ifneq ($(CONFIG_TOUCHSCREEN_SEC_TS_DEBUG),)
sec_touch-objs   += sec_ts_only_vendor.o
endif
