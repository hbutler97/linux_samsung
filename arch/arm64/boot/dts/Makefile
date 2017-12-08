dtb-$(CONFIG_ARCH_THUNDER) += thunder-88xx.dtb
dtb-$(CONFIG_ARCH_VEXPRESS) += rtsm_ve-aemv8a.dtb foundation-v8.dtb
dtb-$(CONFIG_ARCH_XGENE) += apm-mustang.dtb
ifeq ($(CONFIG_SOC_EXYNOS8890),y)
ifeq ($(CONFIG_SOC_EXYNOS8890_EVT1),y)
dtb-y += audi-poc.dtb s5e8890_south_am.dtb exynos8890-sadk8890.dtb
else
dtb-y += exynos8890-espresso8890.dtb exynos8890-espresso8890_rev01.dtb exynos8890-smdk8890.dtb
endif
endif

targets += dtbs

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif
targets += $(DTB_LIST)

dtbs: $(addprefix $(obj)/, $(DTB_LIST))

clean-files := dts/*.dtb *.dtb
