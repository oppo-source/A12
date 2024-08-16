ifeq ($(TARGET_PRODUCT),full_oppo6765)
DEFS_PLATFORM = -DCONFIG_OPPO_FINGERPRINT_PLATFORM=6765
ccflags-y += -I$(srctree)/drivers/misc/mediatek/masp/asfv2/asf_inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/masp/asfv2/asf_export_inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/masp/asfv2/module
endif

ccflags-y += $(DEFS_PLATFORM)

obj-y     +=  oppo_fp_common.o

### Platform marco
ifeq ($(CONFIG_OPPO_FINGERPRINT_QCOM),y)
ccflags-y += -DQCOM_PLATFORM
else ifeq ($(CONFIG_OPPO_FINGERPRINT_MTK),y)
ccflags-y += -DMTK_PLATFORM
endif
