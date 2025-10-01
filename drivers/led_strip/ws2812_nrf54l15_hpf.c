/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * Copyright (c) 2018 Intel Corporation
 * Copyright (c) 2019 Nordic Semiconductor ASA
 * Copyright (c) 2021 Seagate Technology LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT worldsemi_ws2812_nrf54l15_hpf

#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/ipc/ipc_service.h>

#include <string.h>

#define LOG_LEVEL CONFIG_LED_STRIP_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ws2812_nrf54l15_hpf);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/dt-bindings/led/led.h>
#include <zephyr/sys/util_macro.h>

/* Include HPF data structures from Nordic SDK */
#include <drivers/led_strip/hpf_ws2812.h>

struct ws2812_nrf54l15_hpf_cfg {
	uint32_t pin;
	uint8_t port;
	uint8_t num_colors;
	const uint8_t *color_mapping;
	size_t length;
	/* Communication backend configuration */
	const struct device *mbox_dev;
	uint32_t mbox_channel;
	bool use_icmsg; /* true for ICMSG, false for MBOX */
	const char *ipc_ept_name;
};

struct ws2812_nrf54l15_hpf_data {
	struct k_sem tx_sem;
	/* ICMSG backend data */
	struct ipc_ept ept;
};

/* Get shared memory pointer */
static hpf_ws2812_mbox_data_t *get_shared_memory(void)
{
	/* Get TX shared memory - this should match device tree sram_tx region */
	return (hpf_ws2812_mbox_data_t *)DT_REG_ADDR(DT_NODELABEL(sram_tx));
}

/* MBOX backend communication functions */
static int ws2812_hpf_mbox_send_update(const struct device *dev, 
                                       uint32_t pin, uint8_t port, uint32_t num_leds)
{
	const struct ws2812_nrf54l15_hpf_cfg *config = dev->config;
	struct ws2812_nrf54l15_hpf_data *data = dev->data;
	hpf_ws2812_mbox_data_t *shared_data;
	int ret;

	if (!config->mbox_dev) {
		return -ENODEV;
	}

	shared_data = get_shared_memory();
	if (!shared_data) {
		LOG_ERR("Failed to get shared memory");
		return -ENOMEM;
	}

	/* Wait for FLPR to be ready with timeout (100ms) */
	int timeout_ms = 100;
	while (shared_data->lock.data_size != 0 && timeout_ms > 0) {
		k_busy_wait(1000); /* 1ms per iteration */
		timeout_ms--;
	}
	if (timeout_ms == 0) {
		LOG_ERR("Timeout waiting for FLPR to consume data");
		return -ETIMEDOUT;
	}

	/* Acquire lock atomically */
	if (!atomic_cas(&shared_data->lock.locked, DATA_LOCK_STATE_READY, DATA_LOCK_STATE_BUSY)) {
		LOG_ERR("Failed to acquire shared memory lock");
		return -EBUSY;
	}

	/* Set control data */
	shared_data->control.opcode = HPF_WS2812_UPDATE;
	shared_data->control.pin = pin;
	shared_data->control.port = port;
	shared_data->control.num_leds = num_leds;

	/* Set data size to indicate data is available */
	shared_data->lock.data_size = sizeof(hpf_ws2812_control_packet_t) + 
	                              (num_leds * sizeof(hpf_ws2812_pixel_t));

	/* Release lock with data */
	atomic_set(&shared_data->lock.locked, DATA_LOCK_STATE_WITH_DATA);

	/* Signal FLPR via MBOX */
	ret = mbox_send(config->mbox_dev, config->mbox_channel, NULL);
	if (ret < 0) {
		LOG_ERR("MBOX send failed: %d", ret);
		/* Reset lock on failure */
		atomic_set(&shared_data->lock.locked, DATA_LOCK_STATE_READY);
		return ret;
	}

	return 0;
}

/* ICMSG backend communication functions */
static void icmsg_ept_bound(void *priv)
{
	struct ws2812_nrf54l15_hpf_data *data = (struct ws2812_nrf54l15_hpf_data *)priv;
	
	k_sem_give(&data->tx_sem);
}

static void icmsg_ept_recv(const void *msg_data, size_t len, void *priv)
{
	/* Handle response from FLPR core if needed */
	LOG_DBG("Received response from FLPR, len: %zu", len);
}

static int ws2812_hpf_icmsg_send_update(const struct device *dev,
                                        uint32_t pin, uint8_t port, uint32_t num_leds)
{
	struct ws2812_nrf54l15_hpf_data *data = dev->data;
	hpf_ws2812_mbox_data_t *shared_data;
	hpf_ws2812_control_packet_t control;
	int ret;

	shared_data = get_shared_memory();
	if (!shared_data) {
		LOG_ERR("Failed to get shared memory");
		return -ENOMEM;
	}

	/* Prepare control packet */
	control.opcode = HPF_WS2812_UPDATE;
	control.pin = pin;
	control.port = port;
	control.num_leds = num_leds;

	/* Send control packet via ICMSG */
	ret = ipc_service_send(&data->ept, &control, sizeof(control));
	if (ret < 0) {
		LOG_ERR("ICMSG send failed: %d", ret);
		return ret;
	}

	return 0;
}

/* Send update command using configured backend */
static int send_hpf_update(const struct device *dev, uint32_t pin, uint8_t port, uint32_t num_leds)
{
	const struct ws2812_nrf54l15_hpf_cfg *config = dev->config;

	if (config->use_icmsg) {
		return ws2812_hpf_icmsg_send_update(dev, pin, port, num_leds);
	} else {
		return ws2812_hpf_mbox_send_update(dev, pin, port, num_leds);
	}
}

static int ws2812_nrf54l15_hpf_update_rgb(const struct device *dev,
					  struct led_rgb *pixels,
					  size_t num_pixels)
{
	const struct ws2812_nrf54l15_hpf_cfg *config = dev->config;
	struct ws2812_nrf54l15_hpf_data *data = dev->data;
	hpf_ws2812_mbox_data_t *shared_data;
	size_t i;
	int ret;

	if (num_pixels > config->length) {
		LOG_ERR("Too many pixels: %zu (max: %zu)", num_pixels, config->length);
		return -EINVAL;
	}

	if (num_pixels > HPF_WS2812_MAX_LEDS) {
		LOG_ERR("Too many pixels for shared buffer: %zu (max: %d)", num_pixels, HPF_WS2812_MAX_LEDS);
		return -EINVAL;
	}

	/* Take semaphore to ensure exclusive access */
	ret = k_sem_take(&data->tx_sem, K_MSEC(100));
	if (ret != 0) {
		LOG_ERR("Failed to acquire tx semaphore: %d", ret);
		return ret;
	}

	shared_data = get_shared_memory();
	if (!shared_data) {
		LOG_ERR("Failed to get shared memory");
		k_sem_give(&data->tx_sem);
		return -ENOMEM;
	}

	/* Wait for previous transfer to complete with timeout (100ms) */
	int timeout_ms = 100;
	while (shared_data->lock.data_size != 0 && timeout_ms > 0) {
		k_busy_wait(1000); /* 1ms per iteration */
		timeout_ms--;
	}
	if (timeout_ms == 0) {
		LOG_ERR("Timeout waiting for FLPR to consume data");
		k_sem_give(&data->tx_sem);
		return -ETIMEDOUT;
	}

	/* Convert RGB pixels to GRB format in shared memory */
	for (i = 0; i < num_pixels; i++) {
		const struct led_rgb current_pixel = pixels[i];
		
		/* WS2812 expects GRB format */
		shared_data->pixels[i].g = current_pixel.g;
		shared_data->pixels[i].r = current_pixel.r;
		shared_data->pixels[i].b = current_pixel.b;
	}

	/* Send update command to FLPR */
	ret = send_hpf_update(dev, config->pin, config->port, num_pixels);

	k_sem_give(&data->tx_sem);

	return ret;
}

static size_t ws2812_nrf54l15_hpf_length(const struct device *dev)
{
	const struct ws2812_nrf54l15_hpf_cfg *config = dev->config;

	return config->length;
}

static const DEVICE_API(led_strip, ws2812_nrf54l15_hpf_api) = {
	.update_rgb = ws2812_nrf54l15_hpf_update_rgb,
	.length = ws2812_nrf54l15_hpf_length,
};

/*
 * Retrieve the channel to color mapping (e.g. RGB, BGR, GRB, ...) from the
 * "color-mapping" DT property.
 */
#define WS2812_COLOR_MAPPING(idx)					\
static const uint8_t ws2812_nrf54l15_hpf_##idx##_color_mapping[] =	\
	DT_INST_PROP(idx, color_mapping)

#define WS2812_NUM_COLORS(idx) (DT_INST_PROP_LEN(idx, color_mapping))

#define WS2812_NRF54L15_HPF_DEVICE(idx)				\
									\
	static int ws2812_nrf54l15_hpf_##idx##_init(const struct device *dev) \
	{								\
		const struct ws2812_nrf54l15_hpf_cfg *cfg = dev->config; \
		struct ws2812_nrf54l15_hpf_data *data = dev->data;	\
		uint8_t i;						\
		int ret;						\
									\
		/* Initialize semaphore */					\
		k_sem_init(&data->tx_sem, 1, 1);			\
									\
		/* Validate color mapping */				\
		for (i = 0; i < cfg->num_colors; i++) {			\
			switch (cfg->color_mapping[i]) {		\
			case LED_COLOR_ID_WHITE:			\
			case LED_COLOR_ID_RED:				\
			case LED_COLOR_ID_GREEN:			\
			case LED_COLOR_ID_BLUE:				\
				break;					\
			default:					\
				LOG_ERR("%s: invalid channel to color mapping." \
					" Check the color-mapping DT property",	\
					dev->name);			\
				return -EINVAL;				\
			}						\
		}							\
									\
		/* Initialize shared memory (used by both backends) */	\
		hpf_ws2812_mbox_data_t *shared_data = get_shared_memory(); \
		if (!shared_data) {					\
			LOG_ERR("Failed to get shared memory");	\
			return -ENOMEM;					\
		}							\
									\
		/* Initialize shared data lock */			\
		shared_data->lock.data_size = 0;			\
		atomic_set(&shared_data->lock.locked, DATA_LOCK_STATE_READY); \
									\
		/* Initialize communication backend */			\
		if (cfg->use_icmsg) {					\
			/* Initialize ICMSG endpoint */			\
			struct ipc_ept_cfg ept_cfg = {			\
				.name = cfg->ipc_ept_name,		\
				.cb = {					\
					.bound = icmsg_ept_bound,	\
					.received = icmsg_ept_recv,	\
				},					\
				.priv = data,				\
			};						\
									\
			ret = ipc_service_open_instance(cfg->mbox_dev);	\
			if (ret < 0 && ret != -EALREADY) {		\
				LOG_ERR("IPC service open failed: %d", ret); \
				return ret;				\
			}						\
									\
			ret = ipc_service_register_endpoint(cfg->mbox_dev, \
							     &data->ept, &ept_cfg); \
			if (ret < 0) {					\
				LOG_ERR("IPC endpoint register failed: %d", ret); \
				return ret;				\
			}						\
									\
			/* Wait for endpoint to be bound */		\
			ret = k_sem_take(&data->tx_sem, K_SECONDS(1));	\
			if (ret != 0) {					\
				LOG_ERR("ICMSG endpoint binding timeout");	\
				return -ETIMEDOUT;			\
			}						\
		} else {						\
			/* MBOX backend - device should be ready */	\
			if (!device_is_ready(cfg->mbox_dev)) {		\
				LOG_ERR("MBOX device not ready");	\
				return -ENODEV;				\
			}						\
		}							\
									\
		LOG_INF("WS2812 nRF54L15 HPF driver initialized: "	\
			"pin P%d.%02d, %zu LEDs, %s backend",		\
			cfg->port, cfg->pin, cfg->length,		\
			cfg->use_icmsg ? "ICMSG" : "MBOX");		\
									\
		return 0;						\
	}								\
									\
	BUILD_ASSERT(WS2812_NUM_COLORS(idx) <= sizeof(struct led_rgb),	\
		"Too many channels in color-mapping; "			\
		"currently not supported by the ws2812_nrf54l15_hpf driver"); \
									\
	WS2812_COLOR_MAPPING(idx);					\
									\
	static const struct ws2812_nrf54l15_hpf_cfg ws2812_nrf54l15_hpf_##idx##_cfg = { \
		.pin = DT_INST_PROP(idx, pin),				\
		.port = DT_INST_PROP(idx, port),			\
		.num_colors = WS2812_NUM_COLORS(idx),			\
		.color_mapping = ws2812_nrf54l15_hpf_##idx##_color_mapping, \
		.length = DT_INST_PROP(idx, chain_length),		\
		.mbox_dev = DEVICE_DT_GET(DT_INST_PHANDLE(idx, mbox_dev)), \
		.mbox_channel = DT_INST_PROP(idx, mbox_channel),	\
		.use_icmsg = DT_INST_PROP_OR(idx, use_icmsg, false),	\
		.ipc_ept_name = DT_INST_PROP_OR(idx, ipc_ept_name, "ws2812_hpf"), \
	};								\
									\
	static struct ws2812_nrf54l15_hpf_data ws2812_nrf54l15_hpf_##idx##_data; \
									\
	DEVICE_DT_INST_DEFINE(idx,					\
			    ws2812_nrf54l15_hpf_##idx##_init,		\
			    NULL,					\
			    &ws2812_nrf54l15_hpf_##idx##_data,		\
			    &ws2812_nrf54l15_hpf_##idx##_cfg, POST_KERNEL, \
			    CONFIG_LED_STRIP_INIT_PRIORITY,		\
			    &ws2812_nrf54l15_hpf_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_NRF54L15_HPF_DEVICE)