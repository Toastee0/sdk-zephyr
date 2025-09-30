/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2018 Intel Corporation
 * Copyright (c) 2024 TOKITA Hiroshi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#define LOG_LEVEL 4
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#include <zephyr/kernel.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>

#define STRIP_NODE		DT_ALIAS(led_strip)

#if DT_NODE_HAS_PROP(DT_ALIAS(led_strip), chain_length)
#define STRIP_NUM_PIXELS	DT_PROP(DT_ALIAS(led_strip), chain_length)
#else
#error Unable to determine length of LED strip
#endif

#define DELAY_TIME K_MSEC(CONFIG_SAMPLE_LED_UPDATE_DELAY)

#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

static const struct led_rgb colors[] = {
	RGB(CONFIG_SAMPLE_LED_BRIGHTNESS, 0x00, 0x00), /* red */
	RGB(0x00, CONFIG_SAMPLE_LED_BRIGHTNESS, 0x00), /* green */
	RGB(0x00, 0x00, CONFIG_SAMPLE_LED_BRIGHTNESS), /* blue */
};

static struct led_rgb pixels[STRIP_NUM_PIXELS];

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

int main(void)
{
	size_t color = 0;
	int rc;

	/* Wait for FLPR core to start up and retry device initialization */
	LOG_INF("Waiting for FLPR core to initialize...");
	
	for (int retry = 0; retry < 10; retry++) {
		k_sleep(K_MSEC(50));
		if (device_is_ready(strip)) {
			LOG_INF("Found LED strip device %s (attempt %d)", strip->name, retry + 1);
			break;
		}
		LOG_WRN("LED strip device not ready, retrying... (attempt %d/10)", retry + 1);
	}
	
	if (!device_is_ready(strip)) {
		LOG_ERR("LED strip device %s failed to initialize after 10 attempts", strip->name);
		return 0;
	}

	LOG_INF("Starting red/green alternating pattern");
	
	while (1) {
		/* Set all LEDs to red */
		LOG_INF("Setting all LEDs to red");
		for (size_t i = 0; i < STRIP_NUM_PIXELS; i++) {
			pixels[i].r = CONFIG_SAMPLE_LED_BRIGHTNESS;
			pixels[i].g = 0;
			pixels[i].b = 0;
		}
		
		rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
		if (rc) {
			LOG_ERR("couldn't update strip to red: %d", rc);
		} else {
			LOG_INF("Successfully updated %d LEDs to red", STRIP_NUM_PIXELS);
		}
		
		k_sleep(K_MSEC(1000));
		
		/* Set all LEDs to green */
		LOG_INF("Setting all LEDs to green");
		for (size_t i = 0; i < STRIP_NUM_PIXELS; i++) {
			pixels[i].r = 0;
			pixels[i].g = CONFIG_SAMPLE_LED_BRIGHTNESS;
			pixels[i].b = 0;
		}
		
		rc = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
		if (rc) {
			LOG_ERR("couldn't update strip to green: %d", rc);
		} else {
			LOG_INF("Successfully updated %d LEDs to green", STRIP_NUM_PIXELS);
		}
		
		k_sleep(K_MSEC(1000));
	}
	
	return 0;
}
