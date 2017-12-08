#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

ccflags-y := -Iinclude/drm -Idrivers/gpu/drm/exynos
exynosdrm-y := exynos_drm_drv.o exynos_drm_crtc.o exynos_drm_fbdev.o \
		exynos_drm_fb.o exynos_drm_core.o \
		exynos_drm_plane.o exynos_drm_gem.o decon_reg_8890.o \
		dsim_reg_8890.o vpp_reg_8890.o exynos_drm_vpp.o

exynosdrm-$(CONFIG_DRM_EXYNOS_DECON)	+= exynos_drm_decon.o
exynosdrm-$(CONFIG_DRM_EXYNOS_DSI)	+= exynos_drm_dsi.o
exynosdrm-$(CONFIG_DRM_EXYNOS_HDMI)	+= exynos_drm_hdmi.o hdmi_reg.o hdmiphy_conf.o hdcp_drv.o hdmi_ddc.o

obj-$(CONFIG_DRM_EXYNOS)		+= exynosdrm.o
