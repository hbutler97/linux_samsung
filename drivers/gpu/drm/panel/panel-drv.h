/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <linux/backlight.h>

#define MIN_BRIGHTNESS		0
#define MAX_BRIGHTNESS		255
#define DEFAULT_BRIGHTNESS	80

struct exynos_panel;
struct exynos_panel_funcs {
	int (*power_on)(struct exynos_panel *ctx);
	int (*power_off)(struct exynos_panel *ctx);
	int (*init)(struct exynos_panel *ctx);
	int (*disp_on)(struct exynos_panel *ctx);
	int (*disp_off)(struct exynos_panel *ctx);
};

struct exynos_panel {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *bl_dev;
	const struct exynos_panel_funcs *funcs;

	struct mipi_dsi_device dsi;

	struct {
		unsigned int prepare; /* To wait after power-on */
		unsigned int enable; /* To wait before display-on */
		unsigned int reset; /* To wait before reset release */
		unsigned int disable; /* To wait after display-off */
		unsigned int unprepare; /* To wait before power off */
	} delay;
	struct {
		struct gpio_desc *reset;
		struct gpio_desc *ready;
		struct gpio_desc *power;
	} gpio;

	struct regulator_bulk_data supplies[2];
	struct videomode vm;
	u32 width_mm;
	u32 height_mm;

	struct {
		u32 up_scale; /* To use scaling in DDI(Display Driver IC) side */
		u32 mic_bypass; /* To bypass mic encoder logic in DDI side */
	} attr;
};

int __panel_helper_dcs_write(struct exynos_panel *ctx, const u8 cmd,
		const void *data, size_t len);

/***************************/
/* Public Helper Functions */
/***************************/
static inline struct exynos_panel *panel_to_context(struct drm_panel *panel)
{
	return container_of(panel, struct exynos_panel, panel);
}
#define panel_helper_dcs_write(ctx, cmd, seq...) \
({\
	static const u8 c = cmd;\
	static const u8 d[] = { seq };\
	__panel_helper_dcs_write(ctx, c, d, ARRAY_SIZE(d));\
})
bool panel_helper_is_connected(struct exynos_panel *ctx);
int panel_helper_probe(struct exynos_panel *ctx, struct mipi_dsi_device *dsi);
int panel_helper_remove(struct mipi_dsi_device *dsi);
