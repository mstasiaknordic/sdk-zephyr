/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nordic_nrfx_tdm

#if CONFIG_CLOCK_CONTROL_NRF
#include <hal/nrf_clock.h>
#endif
#include <nrfx_tdm.h>
#include <haly/nrfy_gpio.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/dt-bindings/clock/nrf-auxpll.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <dmm.h>
#include <soc.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(tdm_nrf, CONFIG_I2S_LOG_LEVEL);

#define NRFX_TDM_TX_CHANNELS_MASK                                                                  \
	GENMASK(TDM_CONFIG_CHANNEL_MASK_Tx0Enable_Pos + TDM_CONFIG_CHANNEL_NUM_NUM_Max,            \
		TDM_CONFIG_CHANNEL_MASK_Tx0Enable_Pos)
#define NRFX_TDM_RX_CHANNELS_MASK                                                                  \
	GENMASK(TDM_CONFIG_CHANNEL_MASK_Rx0Enable_Pos + TDM_CONFIG_CHANNEL_NUM_NUM_Max,            \
		TDM_CONFIG_CHANNEL_MASK_Rx0Enable_Pos)

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(audiopll))
#define NODE_ACLK      DT_NODELABEL(audiopll)
#define ACLK_FREQUENCY DT_PROP_OR(NODE_ACLK, frequency, 0)
#elif DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(audio_auxpll))
#define NODE_AUDIO_AUXPLL     DT_NODELABEL(audio_auxpll)
#define ACLK_NORDIC_FREQUENCY DT_PROP(NODE_AUDIO_AUXPLL, nordic_frequency)
BUILD_ASSERT((ACLK_NORDIC_FREQUENCY == NRF_AUXPLL_FREQ_DIV_AUDIO_48K) ||
	     (ACLK_NORDIC_FREQUENCY == NRF_AUXPLL_FREQ_DIV_AUDIO_44K1),
	     "Unsupported Audio AUXPLL frequency selection for TDM");
#define ACLK_FREQUENCY CLOCK_CONTROL_NRF_AUXPLL_GET_FREQ(NODE_AUDIO_AUXPLL)
#elif DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(aclk))
#define NODE_ACLK      DT_NODELABEL(aclk)
#define ACLK_FREQUENCY DT_PROP_OR(NODE_ACLK, clock_frequency, 0)
#else
#define ACLK_FREQUENCY 0
#endif

struct stream_cfg {
	struct i2s_config cfg;
	nrfx_tdm_config_t nrfx_cfg;
};

struct tdm_buf {
	size_t size;
	void *dmm_buf;
};

struct tdm_drv_cfg {
	nrfx_tdm_data_handler_t data_handler;
	const struct pinctrl_dev_config *pcfg;
	void *mem_reg;
	uint32_t mck_frequency;
	uint32_t pclk_frequency;
	enum clock_source {
		PCLK,
		ACLK
	} sck_src, mck_src;
};

struct tdm_drv_data {
	nrfx_tdm_t tdm;
#if CONFIG_CLOCK_CONTROL_NRFS_AUDIOPLL || DT_NODE_HAS_STATUS_OKAY(NODE_AUDIO_AUXPLL)
	const struct device *audiopll;
	struct nrf_clock_spec aclk_spec;
#elif CONFIG_CLOCK_CONTROL_NRF
	struct onoff_manager *clk_mgr;
#endif
	struct onoff_client clk_cli;
	struct stream_cfg tx;
	struct k_msgq tx_queue;
	struct stream_cfg rx;
	struct k_msgq rx_queue;
	struct k_msgq rx_mem_slab_queue;
	struct k_msgq tx_mem_slab_queue;
	const struct tdm_drv_cfg *drv_cfg;
	const uint32_t *last_tx_buffer;
	void *last_tx_mem_slab;
	void *last_rx_mem_slab;
	enum i2s_state state;
	enum i2s_dir active_dir;
	bool stop;       /* stop after the current (TX or RX) block */
	bool discard_rx; /* discard further RX blocks */
	volatile bool next_tx_buffer_needed;
	bool tx_configured: 1;
	bool rx_configured: 1;
	bool request_clock: 1;
};

static int audio_clock_request(struct tdm_drv_data *drv_data)
{
#if DT_NODE_HAS_STATUS_OKAY(NODE_ACLK) && CONFIG_CLOCK_CONTROL_NRF
	return onoff_request(drv_data->clk_mgr, &drv_data->clk_cli);
#elif (DT_NODE_HAS_STATUS_OKAY(NODE_ACLK) && CONFIG_CLOCK_CONTROL_NRFS_AUDIOPLL) || \
	  DT_NODE_HAS_STATUS_OKAY(NODE_AUDIO_AUXPLL)
	return nrf_clock_control_request(drv_data->audiopll, &drv_data->aclk_spec,
					 &drv_data->clk_cli);
#else
	(void)drv_data;

	return -ENOTSUP;
#endif
}

static int audio_clock_release(struct tdm_drv_data *drv_data)
{
#if DT_NODE_HAS_STATUS_OKAY(NODE_ACLK) && CONFIG_CLOCK_CONTROL_NRF
	return onoff_release(drv_data->clk_mgr);
#elif (DT_NODE_HAS_STATUS_OKAY(NODE_ACLK) && CONFIG_CLOCK_CONTROL_NRFS_AUDIOPLL) || \
	  DT_NODE_HAS_STATUS_OKAY(NODE_AUDIO_AUXPLL)
	return nrf_clock_control_release(drv_data->audiopll, &drv_data->aclk_spec);
#else
	(void)drv_data;

	return -ENOTSUP;
#endif
}

static bool get_next_tx_buffer(struct tdm_drv_data *drv_data, nrfx_tdm_buffers_t *buffers)
{
	struct tdm_buf buf;
	int ret = k_msgq_get(&drv_data->tx_queue, &buf, K_NO_WAIT);

	if (ret != 0) {
		return false;
	}
	buffers->p_tx_buffer = buf.dmm_buf;
	buffers->tx_buffer_size = buf.size / sizeof(uint32_t);
	buffers->rx_buffer_size = buf.size / sizeof(uint32_t);
	return true;
}

static bool get_next_rx_buffer(struct tdm_drv_data *drv_data, nrfx_tdm_buffers_t *buffers)
{
	const struct tdm_drv_cfg *drv_cfg = drv_data->drv_cfg;
	void *mem_slab;
	int ret = k_mem_slab_alloc(drv_data->rx.cfg.mem_slab, &mem_slab, K_NO_WAIT);

	if (ret < 0) {
		LOG_ERR("Failed to allocate next RX buffer: %d", ret);
		return false;
	}
	drv_data->last_rx_mem_slab = mem_slab;
	ret = dmm_buffer_in_prepare(drv_cfg->mem_reg, mem_slab,
				    buffers->rx_buffer_size * sizeof(uint32_t),
				    (void **)&buffers->p_rx_buffer);
	if (ret < 0) {
		LOG_ERR("Failed to prepare buffer: %d", ret);
		return false;
	}

	return true;
}

static void free_tx_buffer(struct tdm_drv_data *drv_data, struct tdm_buf *buf, void * mem_slab)
{
	const struct tdm_drv_cfg *drv_cfg = drv_data->drv_cfg;

	(void)dmm_buffer_out_release(drv_cfg->mem_reg, buf->dmm_buf);
	if (mem_slab != NULL) {
		k_mem_slab_free(drv_data->tx.cfg.mem_slab, mem_slab);
	}
	LOG_DBG("Freed TX %p", mem_slab);
}

static void free_rx_buffer(struct tdm_drv_data *drv_data, struct tdm_buf *buf, void * mem_slab)
{
	const struct tdm_drv_cfg *drv_cfg = drv_data->drv_cfg;

	(void)dmm_buffer_in_release(drv_cfg->mem_reg, mem_slab, buf->size, buf->dmm_buf);
	if (mem_slab != NULL) {
		k_mem_slab_free(drv_data->rx.cfg.mem_slab, mem_slab);
	}
	LOG_DBG("Freed RX %p", mem_slab);
}

static bool supply_next_buffers(struct tdm_drv_data *drv_data, nrfx_tdm_buffers_t *next)
{
	if (drv_data->active_dir != I2S_DIR_TX) { /* -> RX active */
		int ret = k_msgq_put(&drv_data->rx_mem_slab_queue, &drv_data->last_rx_mem_slab, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("Unable to put mem slab in queue");
			return false;
		}

		if (!get_next_rx_buffer(drv_data, next)) {
			drv_data->state = I2S_STATE_ERROR;
			nrfx_tdm_stop(&drv_data->tdm, false);
			return false;
		}
		/* Set buffer size if there is no TX buffer (which effectively
		 * controls how many bytes will be received).
		 */
		if (drv_data->active_dir == I2S_DIR_RX) {
			next->rx_buffer_size = drv_data->rx.cfg.block_size / sizeof(uint32_t);
		}
	}

	drv_data->last_tx_buffer = next->p_tx_buffer;

	LOG_DBG("Next buffers: %p/%p", next->p_tx_buffer, next->p_rx_buffer);
	return (nrfx_tdm_next_buffers_set(&drv_data->tdm, next) == 0);
}

static void purge_queue(const struct device *dev, enum i2s_dir dir)
{
	struct tdm_drv_data *drv_data = dev->data;
	struct tdm_buf buf;
	void * mem_slab;

	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		while (k_msgq_get(&drv_data->tx_queue, &buf, K_NO_WAIT) == 0) {
			free_tx_buffer(drv_data, &buf, NULL);
		}

		while (k_msgq_get(&drv_data->tx_mem_slab_queue, &mem_slab, K_NO_WAIT) == 0) {
			k_mem_slab_free(drv_data->tx.cfg.mem_slab, mem_slab);
		}
	}

	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		while (k_msgq_get(&drv_data->rx_queue, &buf, K_NO_WAIT) == 0) {
			free_rx_buffer(drv_data, &buf, NULL);
		}

		while (k_msgq_get(&drv_data->rx_mem_slab_queue, &mem_slab, K_NO_WAIT) == 0) {
			k_mem_slab_free(drv_data->rx.cfg.mem_slab, mem_slab);
		}
	}
}

static void tdm_uninit(struct tdm_drv_data *drv_data)
{
	nrfx_tdm_stop(&drv_data->tdm, false);
	nrfx_tdm_uninit(&drv_data->tdm);
}

static int tdm_nrf_configure(const struct device *dev, enum i2s_dir dir,
			     const struct i2s_config *tdm_cfg)
{
	nrfx_tdm_config_t nrfx_cfg;
	struct tdm_drv_data *drv_data = dev->data;
	const struct tdm_drv_cfg *drv_cfg = dev->config;
	uint32_t chan_mask = 0;
	uint8_t extra_channels = 0;
	uint8_t max_num_of_channels = NRFX_TDM_NUM_OF_CHANNELS;

	if (drv_data->state != I2S_STATE_READY) {
		LOG_ERR("Cannot configure in state: %d", drv_data->state);
		return -EINVAL;
	}

	if (tdm_cfg->frame_clk_freq == 0) { /* -> reset state */
		purge_queue(dev, dir);
		if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
			drv_data->tx_configured = false;
			memset(&drv_data->tx, 0, sizeof(drv_data->tx));
		}
		if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
			drv_data->rx_configured = false;
			memset(&drv_data->rx, 0, sizeof(drv_data->rx));
		}
		return 0;
	}

	__ASSERT_NO_MSG(tdm_cfg->mem_slab != NULL && tdm_cfg->block_size != 0);

	if ((tdm_cfg->block_size % sizeof(uint32_t)) != 0 ||
		(tdm_cfg->block_size < (TDM_MIN_TRANSFER_SIZE * 4))) {
		LOG_ERR("This device can only transmit full 32-bit words greater than %u bytes.",
			(TDM_MIN_TRANSFER_SIZE * 4));
		return -EINVAL;
	}

	switch (tdm_cfg->word_size) {
	case 8:
		nrfx_cfg.sample_width = NRF_TDM_SWIDTH_8BIT;
		break;
	case 16:
		nrfx_cfg.sample_width = NRF_TDM_SWIDTH_16BIT;
		break;
	case 24:
		nrfx_cfg.sample_width = NRF_TDM_SWIDTH_24BIT;
		break;
	case 32:
		nrfx_cfg.sample_width = NRF_TDM_SWIDTH_32BIT;
		break;
	default:
		LOG_ERR("Unsupported word size: %u", tdm_cfg->word_size);
		return -EINVAL;
	}

	switch (tdm_cfg->format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		nrfx_cfg.alignment = NRF_TDM_ALIGN_LEFT;
		nrfx_cfg.fsync_polarity = NRF_TDM_POLARITY_NEGEDGE;
		nrfx_cfg.sck_polarity = NRF_TDM_POLARITY_POSEDGE;
		nrfx_cfg.fsync_duration = NRF_TDM_FSYNC_DURATION_CHANNEL;
		nrfx_cfg.channel_delay = NRF_TDM_CHANNEL_DELAY_1CK;
		max_num_of_channels = 2;
		break;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		nrfx_cfg.alignment = NRF_TDM_ALIGN_LEFT;
		nrfx_cfg.fsync_polarity = NRF_TDM_POLARITY_POSEDGE;
		nrfx_cfg.sck_polarity = NRF_TDM_POLARITY_POSEDGE;
		nrfx_cfg.fsync_duration = NRF_TDM_FSYNC_DURATION_CHANNEL;
		nrfx_cfg.channel_delay = NRF_TDM_CHANNEL_DELAY_NONE;
		max_num_of_channels = 2;
		break;
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		nrfx_cfg.alignment = NRF_TDM_ALIGN_RIGHT;
		nrfx_cfg.fsync_polarity = NRF_TDM_POLARITY_POSEDGE;
		nrfx_cfg.sck_polarity = NRF_TDM_POLARITY_POSEDGE;
		nrfx_cfg.fsync_duration = NRF_TDM_FSYNC_DURATION_CHANNEL;
		nrfx_cfg.channel_delay = NRF_TDM_CHANNEL_DELAY_NONE;
		max_num_of_channels = 2;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
		nrfx_cfg.alignment = NRF_TDM_ALIGN_LEFT;
		nrfx_cfg.fsync_polarity = NRF_TDM_POLARITY_NEGEDGE;
		nrfx_cfg.sck_polarity = NRF_TDM_POLARITY_NEGEDGE;
		nrfx_cfg.fsync_duration = NRF_TDM_FSYNC_DURATION_SCK;
		nrfx_cfg.channel_delay = NRF_TDM_CHANNEL_DELAY_NONE;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		nrfx_cfg.alignment = NRF_TDM_ALIGN_LEFT;
		nrfx_cfg.fsync_polarity = NRF_TDM_POLARITY_POSEDGE;
		nrfx_cfg.sck_polarity = NRF_TDM_POLARITY_NEGEDGE;
		nrfx_cfg.fsync_duration = NRF_TDM_FSYNC_DURATION_SCK;
		nrfx_cfg.channel_delay = NRF_TDM_CHANNEL_DELAY_NONE;
		break;
	default:
		LOG_ERR("Unsupported data format: 0x%02x", tdm_cfg->format);
		return -EINVAL;
	}

	if ((tdm_cfg->format & I2S_FMT_DATA_ORDER_LSB) || (tdm_cfg->format & I2S_FMT_BIT_CLK_INV) ||
	    (tdm_cfg->format & I2S_FMT_FRAME_CLK_INV)) {
		LOG_ERR("Unsupported stream format: 0x%02x", tdm_cfg->format);
		return -EINVAL;
	}

	if (tdm_cfg->channels == 1 && nrfx_cfg.fsync_duration == NRF_TDM_FSYNC_DURATION_CHANNEL) {
		/* For I2S mono standard, two channels are to be sent.
		 * The unused half period of LRCK will contain zeros.
		 */
		extra_channels = 1;
	} else if (tdm_cfg->channels > max_num_of_channels) {
		LOG_ERR("Unsupported number of channels: %u", tdm_cfg->channels);
		return -EINVAL;
	}

	nrfx_cfg.channel_number = tdm_cfg->channels + extra_channels;
	chan_mask = BIT_MASK(tdm_cfg->channels);

	if ((tdm_cfg->options & I2S_OPT_BIT_CLK_SLAVE) &&
	    (tdm_cfg->options & I2S_OPT_FRAME_CLK_SLAVE)) {
		nrfx_cfg.mode = NRF_TDM_MODE_SLAVE;
	} else if (!(tdm_cfg->options & I2S_OPT_BIT_CLK_SLAVE) &&
		   !(tdm_cfg->options & I2S_OPT_FRAME_CLK_SLAVE)) {
		nrfx_cfg.mode = NRF_TDM_MODE_MASTER;
	} else {
		LOG_ERR("Unsupported operation mode: 0x%02x", tdm_cfg->options);
		return -EINVAL;
	}

	nrfx_tdm_prescalers_t prescalers;
	uint32_t sck_freq = tdm_cfg->word_size * tdm_cfg->frame_clk_freq * (tdm_cfg->channels + extra_channels);
	uint32_t mck_freq = drv_cfg->mck_frequency;
	uint32_t sck_src_freq = (drv_cfg->sck_src == ACLK) ? ACLK_FREQUENCY : drv_cfg->pclk_frequency;
	uint32_t mck_src_freq = (drv_cfg->mck_src == ACLK) ? ACLK_FREQUENCY : drv_cfg->pclk_frequency;

	/* Calculate SCK clock prescaler. */
	nrfx_tdm_clk_params_t clk_params = {
		.base_clock_freq = sck_src_freq,
		.mck_freq = sck_freq,
		.sck_freq = sck_freq,
	};

	if (nrfx_cfg.mode == NRF_TDM_MODE_MASTER) {
		if (nrfx_tdm_prescalers_calc(&clk_params, &prescalers) != 0) {
			LOG_ERR("Failed to find suitable TDM SCK clock configuration.");
			return -EINVAL;
		}
		nrfx_cfg.prescalers.sck_div = prescalers.sck_div;
		nrfx_cfg.sck_src = (drv_cfg->sck_src == ACLK) ? NRF_TDM_SRC_ACLK : NRF_TDM_SRC_PCLK32M;
		nrfx_cfg.sck_bypass = (nrfx_cfg.prescalers.sck_div > TDM_CONFIG_SCK_DIV_SCKDIV_Max);
	}

	/* Calculate MCK clock prescaler. */
	if ((FIELD_GET(TDM_PSEL_MCK_CONNECT_Msk, nrf_tdm_mck_pin_get(drv_data->tdm.p_reg)) ==
	     TDM_PSEL_MCK_CONNECT_Connected) &&
	    drv_cfg->mck_frequency != 0) {
		clk_params.base_clock_freq = mck_src_freq;
		clk_params.mck_freq = mck_freq;
		if (nrfx_tdm_prescalers_calc(&clk_params, &prescalers) != 0) {
			LOG_ERR("Failed to find suitable TDM MCK clock configuration.");
			return -EINVAL;
		}
		nrfx_cfg.prescalers.mck_div = prescalers.mck_div;
		nrfx_cfg.mck_src = (drv_cfg->mck_src == ACLK) ? NRF_TDM_SRC_ACLK : NRF_TDM_SRC_PCLK32M;
		nrfx_cfg.mck_bypass = (nrfx_cfg.prescalers.mck_div > TDM_CONFIG_MCK_DIV_DIV_Max);
	}

	/* Unless the PCLK source is used,
	 * it is required to request the proper clock to be running
	 * before starting the transfer itself.
	 */
	drv_data->request_clock = (drv_cfg->sck_src != PCLK) || (drv_cfg->mck_src != PCLK);

	nrfx_cfg.skip_gpio_cfg = true;
	nrfx_cfg.skip_psel_cfg = true;

	if ((tdm_cfg->options & I2S_OPT_LOOPBACK) || (tdm_cfg->options & I2S_OPT_PINGPONG)) {
		LOG_ERR("Unsupported options: 0x%02x", tdm_cfg->options);
		return -EINVAL;
	}
	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		drv_data->tx.cfg = *tdm_cfg;
		drv_data->tx.nrfx_cfg = nrfx_cfg;
		drv_data->tx.nrfx_cfg.tx_channel_mask = FIELD_PREP(NRFX_TDM_TX_CHANNELS_MASK, chan_mask);
		drv_data->tx_configured = true;
	}

	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		drv_data->rx.cfg = *tdm_cfg;
		drv_data->rx.nrfx_cfg = nrfx_cfg;
		drv_data->rx.nrfx_cfg.rx_channel_mask = FIELD_PREP(NRFX_TDM_RX_CHANNELS_MASK, chan_mask);
		drv_data->rx_configured = true;
	}

	return 0;
}

static const struct i2s_config *tdm_nrf_config_get(const struct device *dev, enum i2s_dir dir)
{
	struct tdm_drv_data *drv_data = dev->data;

	if (dir == I2S_DIR_TX && drv_data->tx_configured) {
		return &drv_data->tx.cfg;
	}
	if (dir == I2S_DIR_RX && drv_data->rx_configured) {
		return &drv_data->rx.cfg;
	}

	return NULL;
}

static int tdm_nrf_read(const struct device *dev, void **mem_block, size_t *size)
{
	struct tdm_drv_data *drv_data = dev->data;
	const struct tdm_drv_cfg *drv_cfg = drv_data->drv_cfg;
	struct tdm_buf buf;
	void * mem_slab;
	int ret;

	if (!drv_data->rx_configured) {
		LOG_ERR("Device is not configured");
		return -EIO;
	}
	ret = k_msgq_get(&drv_data->rx_queue, &buf,
			 (drv_data->state == I2S_STATE_ERROR)
				 ? K_NO_WAIT
				 : SYS_TIMEOUT_MS(drv_data->rx.cfg.timeout));
	if (ret == -ENOMSG) {
		return -EIO;
	}

	ret = k_msgq_get(&drv_data->rx_mem_slab_queue, &mem_slab,
			 (drv_data->state == I2S_STATE_ERROR)
				 ? K_NO_WAIT
				 : SYS_TIMEOUT_MS(drv_data->rx.cfg.timeout));
	if (ret == -ENOMSG) {
		return -EIO;
	}

	LOG_DBG("Released RX %p", mem_slab);

	if (ret == 0) {
		(void)dmm_buffer_in_release(drv_cfg->mem_reg, mem_slab, buf.size, buf.dmm_buf);
		*mem_block = mem_slab;
		*size = buf.size;
	}
	return ret;
}

static int tdm_nrf_write(const struct device *dev, void *mem_block, size_t size)
{
	struct tdm_drv_data *drv_data = dev->data;
	const struct tdm_drv_cfg *drv_cfg = dev->config;
	struct tdm_buf buf = {.size = size};
	int ret;

	if (!drv_data->tx_configured) {
		LOG_ERR("Device is not configured");
		return -EIO;
	}

	if (drv_data->state != I2S_STATE_RUNNING && drv_data->state != I2S_STATE_READY) {
		LOG_ERR("Cannot write in state: %d", drv_data->state);
		return -EIO;
	}

	if (size > drv_data->tx.cfg.block_size) {
		LOG_ERR("This device can only write blocks up to %u bytes",
			drv_data->tx.cfg.block_size);
		return -EIO;
	}

	ret = k_msgq_put(&drv_data->tx_mem_slab_queue, &mem_block, SYS_TIMEOUT_MS(drv_data->tx.cfg.timeout));
	if (ret < 0) {
		return ret;
	}

	ret = dmm_buffer_out_prepare(drv_cfg->mem_reg, mem_block, buf.size,
				     (void **)&buf.dmm_buf);
	ret = k_msgq_put(&drv_data->tx_queue, &buf, SYS_TIMEOUT_MS(drv_data->tx.cfg.timeout));
	if (ret < 0) {
		return ret;
	}
	drv_data->last_tx_mem_slab = mem_block;
	/* Check if interrupt wanted to get next TX buffer before current buffer
	 * was queued. Do not move this check before queuing because doing so
	 * opens the possibility for a race condition between this function and
	 * data_handler() that is called in interrupt context.
	 */
	if (drv_data->state == I2S_STATE_RUNNING && drv_data->next_tx_buffer_needed) {
		nrfx_tdm_buffers_t next = {0};

		if (!get_next_tx_buffer(drv_data, &next)) {
			/* Log error because this is definitely unexpected.
			 * Do not return error because the caller is no longer
			 * responsible for releasing the buffer.
			 */
			LOG_ERR("Cannot reacquire queued buffer");
			return 0;
		}

		drv_data->next_tx_buffer_needed = false;

		LOG_DBG("Next TX %p", next.p_tx_buffer);

		if (!supply_next_buffers(drv_data, &next)) {
			LOG_ERR("Cannot supply buffer");
			return -EIO;
		}
	}
	return 0;
}

static int start_transfer(struct tdm_drv_data *drv_data)
{
	nrfx_tdm_buffers_t initial_buffers = {0};
	int ret = 0;

	if (drv_data->active_dir != I2S_DIR_RX && /* -> TX to be started */
	    !get_next_tx_buffer(drv_data, &initial_buffers)) {
		LOG_ERR("No TX buffer available");
		ret = -ENOMEM;
	} else if (drv_data->active_dir != I2S_DIR_TX && /* -> RX to be started */
		   !get_next_rx_buffer(drv_data, &initial_buffers)) {
		/* Failed to allocate next RX buffer */
		ret = -ENOMEM;
	} else {
		if (drv_data->active_dir == I2S_DIR_RX) {
			initial_buffers.rx_buffer_size =
				drv_data->rx.cfg.block_size / sizeof(uint32_t);
		}

		drv_data->last_tx_buffer = initial_buffers.p_tx_buffer;
		ret = nrfx_tdm_start(&drv_data->tdm, &initial_buffers);
	}
	if (ret < 0) {
		tdm_uninit(drv_data);
		if (drv_data->request_clock) {
			(void)audio_clock_release(drv_data);
		}

		if (initial_buffers.p_tx_buffer) {
			struct tdm_buf buf = {.dmm_buf = (void *)initial_buffers.p_tx_buffer,
					      .size = initial_buffers.tx_buffer_size *
						      sizeof(uint32_t)};
			free_tx_buffer(drv_data, &buf, NULL);
		}
		if (initial_buffers.p_rx_buffer) {
			struct tdm_buf buf = {.dmm_buf = (void *)initial_buffers.p_rx_buffer,
					      .size = initial_buffers.rx_buffer_size *
						      sizeof(uint32_t)};
			free_rx_buffer(drv_data, &buf, NULL);
		}

		drv_data->state = I2S_STATE_ERROR;
	}
	return ret;
}

static bool channels_configuration_check(uint32_t tx, uint32_t rx)
{
	tx = FIELD_GET(NRFX_TDM_TX_CHANNELS_MASK, tx);
	rx = FIELD_GET(NRFX_TDM_RX_CHANNELS_MASK, rx);

	return (tx == rx);
}

static void clock_started_callback(struct onoff_manager *mgr, struct onoff_client *cli,
				   uint32_t state, int res)
{
	struct tdm_drv_data *drv_data = CONTAINER_OF(cli, struct tdm_drv_data, clk_cli);

	/* The driver state can be set back to READY at this point if the DROP
	 * command was triggered before the clock has started. Do not start
	 * the actual transfer in such case.
	 */
	if (drv_data->state == I2S_STATE_READY) {
		tdm_uninit(drv_data);
		(void)audio_clock_release(drv_data);
	} else {
		(void)start_transfer(drv_data);
	}
}

static int trigger_start(const struct device *dev)
{
	struct tdm_drv_data *drv_data = dev->data;
	const struct tdm_drv_cfg *drv_cfg = dev->config;
	int ret;
	const nrfx_tdm_config_t *nrfx_cfg = (drv_data->active_dir == I2S_DIR_TX)
						   ? &drv_data->tx.nrfx_cfg
						   : &drv_data->rx.nrfx_cfg;

	ret = nrfx_tdm_init(&drv_data->tdm, nrfx_cfg, drv_cfg->data_handler);
	if (ret < 0) {
		LOG_ERR("Failed to initialize TDN: %d", ret);
		return ret;
	}

	drv_data->state = I2S_STATE_RUNNING;

	/* If it is required to use certain HF clock, request it to be running
	 * first. If not, start the transfer directly.
	 */
	if (drv_data->request_clock) {
		sys_notify_init_callback(&drv_data->clk_cli.notify, clock_started_callback);
		ret = audio_clock_request(drv_data);
		if (ret < 0) {
			tdm_uninit(drv_data);
			drv_data->state = I2S_STATE_READY;

			LOG_ERR("Failed to request clock: %d", ret);
			return -EIO;
		}
	} else {
		ret = start_transfer(drv_data);
		if (ret < 0) {
			LOG_ERR("Failed to start transfer: %d", ret);
			return ret;
		}
	}

	return 0;
}

static int tdm_nrf_trigger(const struct device *dev, enum i2s_dir dir, enum i2s_trigger_cmd cmd)
{
	struct tdm_drv_data *drv_data = dev->data;
	bool configured = false;
	bool cmd_allowed;

	/* This driver does not use the I2S_STATE_NOT_READY value.
	 * Instead, if a given stream is not configured, the respective
	 * flag (tx_configured or rx_configured) is cleared.
	 */

	if (dir == I2S_DIR_BOTH) {
		configured = drv_data->tx_configured && drv_data->rx_configured;
	} else if (dir == I2S_DIR_TX) {
		configured = drv_data->tx_configured;
	} else if (dir == I2S_DIR_RX) {
		configured = drv_data->rx_configured;
	}

	if (!configured) {
		LOG_ERR("Device is not configured");
		return -EIO;
	}

	if (dir == I2S_DIR_BOTH) {
		if (!channels_configuration_check(drv_data->tx.nrfx_cfg.tx_channel_mask,
						  drv_data->rx.nrfx_cfg.rx_channel_mask)) {
			LOG_ERR("TX and RX channels configurations are different");
			return -EIO;
		}
		/* The TX and RX channel masks are to be stored in a single TDM register.
		 * In case of I2S_DIR_BOTH, only the rx.nrfx_cfg structure is used, so
		 * it must also contain the TX channel mask.
		 */
		uint32_t tx_rx_merged =
			drv_data->tx.nrfx_cfg.tx_channel_mask | drv_data->rx.nrfx_cfg.rx_channel_mask;
		drv_data->tx.nrfx_cfg.tx_channel_mask = tx_rx_merged;
		drv_data->tx.nrfx_cfg.rx_channel_mask = tx_rx_merged;
		drv_data->rx.nrfx_cfg.tx_channel_mask = tx_rx_merged;
		drv_data->rx.nrfx_cfg.rx_channel_mask = tx_rx_merged;
		if (memcmp(&drv_data->tx.nrfx_cfg, &drv_data->rx.nrfx_cfg,
			   sizeof(drv_data->rx.nrfx_cfg)) != 0 ||
		    (drv_data->tx.cfg.block_size != drv_data->rx.cfg.block_size)) {
			LOG_ERR("TX and RX configurations are different");
			return -EIO;
		}
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		cmd_allowed = (drv_data->state == I2S_STATE_READY);
		break;
	case I2S_TRIGGER_STOP:
	case I2S_TRIGGER_DRAIN:
		cmd_allowed = (drv_data->state == I2S_STATE_RUNNING);
		break;
	case I2S_TRIGGER_DROP:
		cmd_allowed = configured;
		break;
	case I2S_TRIGGER_PREPARE:
		cmd_allowed = (drv_data->state == I2S_STATE_ERROR);
		break;
	default:
		LOG_ERR("Invalid trigger: %d", cmd);
		return -EINVAL;
	}

	if (!cmd_allowed) {
		LOG_ERR("Not allowed");
		return -EIO;
	}

	/* For triggers applicable to the RUNNING state (i.e. STOP, DRAIN,
	 * and DROP), ensure that the command is applied to the streams
	 * that are currently active (this device cannot e.g. stop only TX
	 * without stopping RX).
	 */
	if (drv_data->state == I2S_STATE_RUNNING && drv_data->active_dir != dir) {
		LOG_ERR("Inappropriate trigger (%d/%d), active stream(s): %d", cmd, dir,
			drv_data->active_dir);
		return -EINVAL;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		drv_data->stop = false;
		drv_data->discard_rx = false;
		drv_data->active_dir = dir;
		drv_data->next_tx_buffer_needed = false;
		return trigger_start(dev);

	case I2S_TRIGGER_STOP:
		drv_data->state = I2S_STATE_STOPPING;
		drv_data->stop = true;
		return 0;

	case I2S_TRIGGER_DRAIN:
		drv_data->state = I2S_STATE_STOPPING;
		/* If only RX is active, DRAIN is equivalent to STOP. */
		drv_data->stop = (drv_data->active_dir == I2S_DIR_RX);
		return 0;

	case I2S_TRIGGER_DROP:
		if (drv_data->state != I2S_STATE_READY) {
			drv_data->discard_rx = true;
			nrfx_tdm_stop(&drv_data->tdm, true);
		}
		purge_queue(dev, dir);
		drv_data->state = I2S_STATE_READY;
		return 0;

	case I2S_TRIGGER_PREPARE:
		purge_queue(dev, dir);
		drv_data->state = I2S_STATE_READY;
		return 0;

	default:
		LOG_ERR("Invalid trigger: %d", cmd);
		return -EINVAL;
	}
}

static void data_handler(const struct device *dev, const nrfx_tdm_buffers_t *released, uint32_t status)
{
	struct tdm_drv_data *drv_data = dev->data;
	bool stop_transfer = false;
	struct tdm_buf buf = {.dmm_buf = NULL, .size = 0};
	int ret;
	void * mem_slab;

	if (released != NULL) {
		__ASSERT_NO_MSG(released->rx_buffer_size == released->tx_buffer_size);
		buf.size = released->rx_buffer_size * sizeof(uint32_t);
	}

	if (status & NRFX_TDM_STATUS_TRANSFER_STOPPED) {
		if (drv_data->state == I2S_STATE_STOPPING) {
			drv_data->state = I2S_STATE_READY;
		}
		if (drv_data->last_tx_buffer) {
			/* Usually, these pointers are equal, i.e. the last TX
			 * buffer that were to be transferred is released by the
			 * driver after it stops. The last TX buffer pointer is
			 * then set to NULL here so that the buffer can be freed
			 * below, just as any other TX buffer released by the
			 * driver. However, it may happen that the buffer is not
			 * released this way, for example, when the transfer
			 * ends with an error because an RX buffer allocation
			 * fails. In such case, the last TX buffer needs to be
			 * freed here.
			 */

			if ((released != NULL) != 0 &&
			    (drv_data->last_tx_buffer != released->p_tx_buffer)) {
				buf.dmm_buf = (void *)drv_data->last_tx_buffer;
				/* At this point, where user provided no more TX buffers,
				 * the last TX mem_slab should also be the last mem_slab in queue.
				 */
				if (k_msgq_get(&drv_data->tx_mem_slab_queue, &mem_slab, K_NO_WAIT) == 0){
					free_tx_buffer(drv_data, &buf, mem_slab);
				}
			}
			drv_data->last_tx_buffer = NULL;
		}

		if (drv_data->last_rx_mem_slab) {
			if (drv_data->stop && drv_data->discard_rx) {
				(void)k_mem_slab_free(drv_data->rx.cfg.mem_slab, drv_data->last_rx_mem_slab);
			} else {
				(void)k_msgq_put(&drv_data->rx_mem_slab_queue, &drv_data->last_rx_mem_slab, K_NO_WAIT);
			}
			drv_data->last_rx_mem_slab = NULL;
		}

		tdm_uninit(drv_data);
		if (drv_data->request_clock) {
			(void)audio_clock_release(drv_data);
		}
	}

	if (released == NULL) {
		/* This means that buffers for the next part of the transfer
		 * were not supplied and the previous ones cannot be released
		 * yet, as pointers to them were latched in the TDM registers.
		 * It is not an error when the transfer is to be stopped (those
		 * buffers will be released after the transfer actually stops).
		 */
		if (drv_data->state != I2S_STATE_STOPPING) {
			drv_data->state = I2S_STATE_ERROR;

		}
		nrfx_tdm_stop(&drv_data->tdm, false);
		return;
	}
	if (released->p_rx_buffer) {
		buf.dmm_buf = (void *)released->p_rx_buffer;
		if (drv_data->discard_rx) {
			k_msgq_get(&drv_data->rx_mem_slab_queue, &mem_slab, K_NO_WAIT);
			free_rx_buffer(drv_data, &buf, mem_slab);
		} else {
			ret = k_msgq_put(&drv_data->rx_queue, &buf, K_NO_WAIT);
			if (ret < 0) {
				LOG_ERR("No room in RX queue");
				drv_data->state = I2S_STATE_ERROR;
				stop_transfer = true;
				k_msgq_get(&drv_data->rx_mem_slab_queue, &mem_slab, K_NO_WAIT);
				free_rx_buffer(drv_data, &buf, mem_slab);
			} else {

				/* If the TX direction is not active and
				 * the transfer should be stopped after
				 * the current block, stop the reception.
				 */
				if (drv_data->active_dir == I2S_DIR_RX && drv_data->stop) {
					drv_data->discard_rx = true;
					stop_transfer = true;
				}
			}
		}
	}

	if (released->p_tx_buffer) {
		buf.dmm_buf = (void *)released->p_tx_buffer;
		/* If the last buffer that was to be transferred has just been
		 * released, it is time to stop the transfer.
		 */
		if (released->p_tx_buffer == drv_data->last_tx_buffer) {
			drv_data->discard_rx = true;
			stop_transfer = true;
		} else {
			ret = k_msgq_get(&drv_data->tx_mem_slab_queue, &mem_slab, K_NO_WAIT);
			if (ret < 0) {
				LOG_ERR("No memory slab in TX queue");
				stop_transfer = true;
			}
			free_tx_buffer(drv_data, &buf, mem_slab);
		}
	}

	if (stop_transfer) {
		nrfx_tdm_stop(&drv_data->tdm, false);
	} else if (status & NRFX_TDM_STATUS_NEXT_BUFFERS_NEEDED) {
		nrfx_tdm_buffers_t next = {0};

		if (drv_data->active_dir != I2S_DIR_RX) { /* -> TX active */
			if (drv_data->stop) {
				/* If the stream is to be stopped, don't get
				 * the next TX buffer from the queue, instead
				 * supply the one used last time (it won't be
				 * transferred, the stream will stop right
				 * before this buffer would be started again).
				 */
				next.p_tx_buffer = drv_data->last_tx_buffer;
				next.tx_buffer_size = 1;
			} else if (get_next_tx_buffer(drv_data, &next)) {
				/* Next TX buffer successfully retrieved from
				 * the queue, nothing more to do here.
				 */
			} else if (drv_data->state == I2S_STATE_STOPPING) {
				/* If there are no more TX blocks queued and
				 * the current state is STOPPING (so the DRAIN
				 * command was triggered) it is time to finish
				 * the transfer.
				 */
				drv_data->stop = true;
				/* Supply the same buffer as last time; it will
				 * not be transferred anyway, as the transfer
				 * will be stopped earlier.
				 */
				next.p_tx_buffer = drv_data->last_tx_buffer;
				next.tx_buffer_size = 1;
			} else {
				/* Next TX buffer cannot be supplied now.
				 * Defer it to when the user writes more data.
				 */
				drv_data->next_tx_buffer_needed = true;
				return;
			}
		}
		(void)supply_next_buffers(drv_data, &next);
	}
}

static void clock_manager_init(const struct device *dev)
{
#if DT_NODE_HAS_STATUS_OKAY(NODE_ACLK) && CONFIG_CLOCK_CONTROL_NRFS_AUDIOPLL
	struct tdm_drv_data *drv_data = dev->data;

	drv_data->audiopll = DEVICE_DT_GET(NODE_ACLK);
	drv_data->aclk_spec.frequency = ACLK_FREQUENCY;
#elif DT_NODE_HAS_STATUS_OKAY(NODE_AUDIO_AUXPLL)
	struct tdm_drv_data *drv_data = dev->data;

	drv_data->audiopll = DEVICE_DT_GET(NODE_AUDIO_AUXPLL);
	drv_data->aclk_spec.frequency = ACLK_FREQUENCY;
#elif CONFIG_CLOCK_CONTROL_NRF && (NRF_CLOCK_HAS_HFCLKAUDIO || NRF_CLOCK_HAS_HFCLK24M)
	clock_control_subsys_t subsys;
	struct tdm_drv_data *drv_data = dev->data;

	IF_ENABLED(NRF_CLOCK_HAS_HFCLKAUDIO, (subsys = CLOCK_CONTROL_NRF_SUBSYS_HFAUDIO;))
	IF_ENABLED(NRF_CLOCK_HAS_HFCLK24M, (subsys = CLOCK_CONTROL_NRF_SUBSYS_HF24M;))
	drv_data->clk_mgr = z_nrf_clock_control_get_onoff(subsys);
	__ASSERT_NO_MSG(drv_data->clk_mgr != NULL);
#else
	(void)dev;
#endif /* CONFIG_CLOCK_CONTROL_NRF && (NRF_CLOCK_HAS_HFCLKAUDIO || NRF_CLOCK_HAS_HFCLK24M) */
}

static int data_init(const struct device *dev)
{
	struct tdm_drv_data *drv_data = dev->data;
	const struct tdm_drv_cfg *drv_cfg = dev->config;

	drv_data->state = I2S_STATE_READY;
	int err = pinctrl_apply_state(drv_cfg->pcfg, PINCTRL_STATE_DEFAULT);

	if (err < 0) {
		return err;
	}
	drv_data->drv_cfg = drv_cfg;
	return err;
}

static DEVICE_API(i2s, tdm_nrf_drv_api) = {
	.configure = tdm_nrf_configure,
	.config_get = tdm_nrf_config_get,
	.read = tdm_nrf_read,
	.write = tdm_nrf_write,
	.trigger = tdm_nrf_trigger,
};

#define TDM(inst)             DT_DRV_INST(inst)
#define TDM_SCK_CLK_SRC(inst) DT_STRING_TOKEN(TDM(inst), sck_clock_source)
#define TDM_MCK_CLK_SRC(inst) DT_STRING_TOKEN(TDM(inst), mck_clock_source)
#define PCLK_NODE(inst)       DT_CLOCKS_CTLR(TDM(inst))

#define TDM_NRF_DEVICE(inst)									\
	static struct tdm_buf tx_msgs##inst[CONFIG_I2S_NRF_TDM_TX_BLOCK_COUNT];			\
	static struct tdm_buf rx_msgs##inst[CONFIG_I2S_NRF_TDM_RX_BLOCK_COUNT];			\
	static void *rx_mem_slabs##inst[CONFIG_I2S_NRF_TDM_RX_BLOCK_COUNT];			\
	static void *tx_mem_slabs##inst[CONFIG_I2S_NRF_TDM_TX_BLOCK_COUNT];			\
	static void tdm_##inst##data_handler(nrfx_tdm_buffers_t const *p_released,		\
					     uint32_t status)					\
	{											\
		data_handler(DEVICE_DT_INST_GET(inst), p_released, status);			\
	}											\
	PINCTRL_DT_DEFINE(TDM(inst));								\
	static const struct tdm_drv_cfg tdm_nrf_cfg##inst = {					\
		.data_handler = tdm_##inst##data_handler,					\
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(TDM(inst)),					\
		.sck_src = TDM_SCK_CLK_SRC(inst),						\
		.mck_src = TDM_MCK_CLK_SRC(inst),						\
		.mck_frequency = DT_PROP_OR(TDM(inst), mck_frequency, 0),			\
		.pclk_frequency = DT_PROP(PCLK_NODE(inst), clock_frequency),			\
		.mem_reg = DMM_DEV_TO_REG(TDM(inst)),						\
	};											\
	static struct tdm_drv_data tdm_nrf_data##inst = {					\
		.tdm = NRFX_TDM_INSTANCE(DT_INST_REG_ADDR(inst))				\
	};											\
	static int tdm_nrf_init##inst(const struct device *dev)					\
	{											\
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),			\
			    nrfx_tdm_irq_handler, &tdm_nrf_data##inst.tdm, 0);			\
												\
		int err = data_init(dev);							\
		if (err < 0) {									\
			return err;								\
		}										\
		k_msgq_init(&tdm_nrf_data##inst.tx_queue, (char *)tx_msgs##inst,		\
			    sizeof(struct tdm_buf), ARRAY_SIZE(tx_msgs##inst));			\
		k_msgq_init(&tdm_nrf_data##inst.rx_queue, (char *)rx_msgs##inst,		\
			    sizeof(struct tdm_buf), ARRAY_SIZE(rx_msgs##inst));			\
		k_msgq_init(&tdm_nrf_data##inst.rx_mem_slab_queue, (char *)rx_mem_slabs##inst,	\
			    sizeof(void *), ARRAY_SIZE(rx_mem_slabs##inst));			\
		k_msgq_init(&tdm_nrf_data##inst.tx_mem_slab_queue, (char *)tx_mem_slabs##inst,	\
			    sizeof(void *), ARRAY_SIZE(tx_mem_slabs##inst));			\
		clock_manager_init(dev);							\
		return 0;									\
	}											\
	BUILD_ASSERT((TDM_SCK_CLK_SRC(inst) != ACLK && TDM_MCK_CLK_SRC(inst) != ACLK) ||	\
			     (DT_NODE_HAS_STATUS_OKAY(NODE_ACLK) ||				\
			      DT_NODE_HAS_STATUS_OKAY(NODE_AUDIO_AUXPLL)),			\
		     "Clock source ACLK requires the audiopll/audio_auxpll node.");		\
	NRF_DT_CHECK_NODE_HAS_REQUIRED_MEMORY_REGIONS(TDM(inst));				\
	DEVICE_DT_INST_DEFINE(inst, tdm_nrf_init##inst, NULL, &tdm_nrf_data##inst,		\
			      &tdm_nrf_cfg##inst, POST_KERNEL,					\
			      CONFIG_I2S_INIT_PRIORITY, &tdm_nrf_drv_api);

DT_INST_FOREACH_STATUS_OKAY(TDM_NRF_DEVICE)
