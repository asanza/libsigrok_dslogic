/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2015 Bartosz Golaszewski <bgolaszewski@baylibre.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "protocol.h"

SR_PRIV struct sr_dev_driver baylibre_acme_driver_info;
static struct sr_dev_driver *di = &baylibre_acme_driver_info;

static const uint32_t devopts[] = {
	SR_CONF_CONTINUOUS | SR_CONF_SET,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_MSEC | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PROBE_FACTOR | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_POWER_OFF | SR_CONF_GET | SR_CONF_SET,
};

#define MAX_SAMPLE_RATE 500 /* In Hz */

static const uint64_t samplerates[] = {
	SR_HZ(1),
	SR_HZ(MAX_SAMPLE_RATE),
	SR_HZ(1),
};

static int init(struct sr_context *sr_ctx)
{
	return std_init(sr_ctx, di, LOG_PREFIX);
}

static GSList *scan(GSList *options)
{
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_dev_inst *sdi;
	GSList *devices;
	gboolean status;
	int i;

	(void)options;

	drvc = di->priv;
	devices = NULL;

	devc = g_malloc0(sizeof(struct dev_context));
	devc->samplerate = SR_HZ(10);

	sdi = g_malloc0(sizeof(struct sr_dev_inst));
	sdi->status = SR_ST_INACTIVE;
	sdi->vendor = g_strdup("BayLibre");
	sdi->model = g_strdup("ACME");
	sdi->driver = di;
	sdi->priv = devc;

	status = bl_acme_is_sane();
	if (!status)
		goto err_out;

	/*
	 * Iterate over all ACME connectors and check if any probes
	 * are present.
	 */
	for (i = 0; i < MAX_PROBES; i++) {
		/*
		 * First check if there's an energy probe on this connector. If
		 * not, and we're already at the fifth probe - see if we can
		 * detect a temperature probe.
		 */
		status = bl_acme_detect_probe(bl_acme_get_enrg_addr(i),
					      PROBE_NUM(i), ENRG_PROBE_NAME);
		if (status) {
			/* Energy probe detected. */
			status = bl_acme_register_probe(sdi, PROBE_ENRG,
					bl_acme_get_enrg_addr(i), PROBE_NUM(i));
			if (!status) {
				sr_err("Error registering power probe %d",
				       PROBE_NUM(i));
				continue;
			}
		} else if (i >= TEMP_PRB_START_INDEX) {
			status = bl_acme_detect_probe(bl_acme_get_temp_addr(i),
					      PROBE_NUM(i), TEMP_PROBE_NAME);
			if (status) {
				/* Temperature probe detected. */
				status = bl_acme_register_probe(sdi,PROBE_TEMP,
					bl_acme_get_temp_addr(i), PROBE_NUM(i));
				if (!status) {
					sr_err("Error registering temp "
					       "probe %d", PROBE_NUM(i));
					continue;
				}
			}
		}
	}

	/*
	 * Let's assume there's no ACME device present if no probe
	 * has been registered.
	 */
	if (sdi->channel_groups == NULL)
		goto err_out;

	devices = g_slist_append(devices, sdi);
	drvc->instances = g_slist_append(drvc->instances, sdi);

	return devices;

err_out:
	g_free(devc);
	sr_dev_inst_free(sdi);

	return NULL;
}

static GSList *dev_list(void)
{
	return ((struct drv_context *)(di->priv))->instances;
}

static int dev_clear(void)
{
	return std_dev_clear(di, NULL);
}

static int dev_open(struct sr_dev_inst *sdi)
{
	(void)sdi;

	/* Nothing to do here. */
	sdi->status = SR_ST_ACTIVE;

	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi)
{
	(void)sdi;

	/* Nothing to do here. */
	sdi->status = SR_ST_INACTIVE;

	return SR_OK;
}

static int cleanup(void)
{
	dev_clear();

	return SR_OK;
}

static int config_get(uint32_t key, GVariant **data,
		      const struct sr_dev_inst *sdi,
		      const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	int ret;
	uint64_t shunt;
	gboolean power_off;

	devc = sdi->priv;

	ret = SR_OK;
	switch (key) {
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(devc->limit_samples);
		break;
	case SR_CONF_LIMIT_MSEC:
		*data = g_variant_new_uint64(devc->limit_msec);
		break;
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->samplerate);
		break;
	case SR_CONF_PROBE_FACTOR:
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		ret = bl_acme_get_shunt(cg, &shunt);
		if (ret == SR_OK)
			*data = g_variant_new_uint64(shunt);
		break;
	case SR_CONF_POWER_OFF:
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		ret = bl_acme_read_power_state(cg, &power_off);
		if (ret == SR_OK)
			*data = g_variant_new_boolean(power_off);
		break;
	default:
		return SR_ERR_NA;
	}

	return ret;
}

static int config_set(uint32_t key, GVariant *data,
		      const struct sr_dev_inst *sdi,
		      const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	uint64_t samplerate;
	int ret;

	if (sdi->status != SR_ST_ACTIVE)
		return SR_ERR_DEV_CLOSED;

	devc = sdi->priv;

	ret = SR_OK;
	switch (key) {
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
		devc->limit_msec = 0;
		sr_dbg("Setting sample limit to %" PRIu64, devc->limit_samples);
		break;
	case SR_CONF_LIMIT_MSEC:
		devc->limit_msec = g_variant_get_uint64(data) * 1000;
		devc->limit_samples = 0;
		sr_dbg("Setting time limit to %" PRIu64"ms", devc->limit_msec);
		break;
	case SR_CONF_SAMPLERATE:
		samplerate = g_variant_get_uint64(data);
		if (samplerate > MAX_SAMPLE_RATE) {
			sr_err("Maximum sample rate is %d", MAX_SAMPLE_RATE);
			ret = SR_ERR_SAMPLERATE;
			break;
		}
		devc->samplerate = samplerate;
		sr_dbg("Setting samplerate to %" PRIu64, devc->samplerate);
		break;
	case SR_CONF_PROBE_FACTOR:
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		ret = bl_acme_set_shunt(cg, g_variant_get_uint64(data));
		break;
	case SR_CONF_POWER_OFF:
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		ret = bl_acme_set_power_off(cg, g_variant_get_boolean(data));
		break;
	default:
		ret = SR_ERR_NA;
	}

	return ret;
}

static int config_list(uint32_t key, GVariant **data,
		       const struct sr_dev_inst *sdi,
		       const struct sr_channel_group *cg)
{
	GVariant *gvar;
	GVariantBuilder gvb;
	int ret;

	(void)sdi;
	(void)cg;

	ret = SR_OK;
	switch (key) {
	case SR_CONF_DEVICE_OPTIONS:
		*data = g_variant_new_fixed_array(G_VARIANT_TYPE_UINT32,
			devopts, ARRAY_SIZE(devopts), sizeof(uint32_t));
		break;
	case SR_CONF_SAMPLERATE:
		g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
		gvar = g_variant_new_fixed_array(G_VARIANT_TYPE("t"),
			samplerates, ARRAY_SIZE(samplerates), sizeof(uint64_t));
		g_variant_builder_add(&gvb, "{sv}", "samplerate-steps", gvar);
		*data = g_variant_builder_end(&gvb);
		break;
	default:
		return SR_ERR_NA;
	}

	return ret;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi, void *cb_data)
{
	struct dev_context *devc;

	(void)cb_data;

	if (sdi->status != SR_ST_ACTIVE)
		return SR_ERR_DEV_CLOSED;

	devc = sdi->priv;
	devc->samples_read = 0;

	if (pipe(devc->pipe_fds)) {
		sr_err("Error setting up pipe");
		return SR_ERR;
	}

	devc->channel = g_io_channel_unix_new(devc->pipe_fds[0]);
	g_io_channel_set_flags(devc->channel, G_IO_FLAG_NONBLOCK, NULL);
	g_io_channel_set_encoding(devc->channel, NULL, NULL);
	g_io_channel_set_buffered(devc->channel, FALSE);

	sr_session_source_add_channel(sdi->session, devc->channel,
		G_IO_IN | G_IO_ERR, 1, bl_acme_receive_data, (void *)sdi);

	/* Send header packet to the session bus. */
	std_session_send_df_header(sdi, LOG_PREFIX);
	devc->start_time = g_get_monotonic_time();

	return SR_OK;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi, void *cb_data)
{
	struct sr_datafeed_packet packet;
	struct dev_context *devc;

	(void)cb_data;

	devc = sdi->priv;

	if (sdi->status != SR_ST_ACTIVE)
		return SR_ERR_DEV_CLOSED;

	sr_session_source_remove_channel(sdi->session, devc->channel);
	g_io_channel_shutdown(devc->channel, FALSE, NULL);
	g_io_channel_unref(devc->channel);
	devc->channel = NULL;

	/* Send last packet. */
	packet.type = SR_DF_END;
	sr_session_send(sdi, &packet);

	return SR_OK;
}

SR_PRIV struct sr_dev_driver baylibre_acme_driver_info = {
	.name = "baylibre-acme",
	.longname = "BayLibre ACME (Another Cute Measurement Equipment)",
	.api_version = 1,
	.init = init,
	.cleanup = cleanup,
	.scan = scan,
	.dev_list = dev_list,
	.dev_clear = dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
	.priv = NULL,
};
