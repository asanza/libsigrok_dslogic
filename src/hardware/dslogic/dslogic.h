/*
* This file is part of the libsigrok project.
*
* Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
* Copyright (C) 2013 DreamSourceLab <dreamsourcelab@dreamsourcelab.com>
* Copyright (C) 2025 Diego F. Asanza <d.asanza@gmail.com>
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

#ifndef LIBDSLOGIC_HARDWARE_DSLOGIC_H
#define LIBDSLOGIC_HARDWARE_DSLOGIC_H

#include <glib.h>
#include <libsigrok.h>
#include "devdefs.h"
#include "protocol.h"

/* Message logging helpers with subsystem-specific prefix string. */
#define LOG_PREFIX "DSLogic Hardware: "
#define ds_log(l, s, args...) ds_log(l, LOG_PREFIX s, ## args)
#define ds_spew(s, args...) ds_spew(LOG_PREFIX s, ## args)
#define ds_dbg(s, args...) ds_dbg(LOG_PREFIX s, ## args)
#define ds_info(s, args...) ds_info(LOG_PREFIX s, ## args)
#define ds_warn(s, args...) ds_warn(LOG_PREFIX s, ## args)
#define ds_err(s, args...) ds_err(LOG_PREFIX s, ## args)

#define USB_INTERFACE		0
#define USB_CONFIGURATION	1
#define TRIGGER_TYPE 		"01"

#define MAX_RENUM_DELAY_MS	3000

#define DSLOGIC_REQUIRED_VERSION_MAJOR	1

#define MAX_8BIT_SAMPLE_RATE	DS_MHZ(24)
#define MAX_16BIT_SAMPLE_RATE	DS_MHZ(12)

/* 6 delay states of up to 256 clock ticks */
#define MAX_SAMPLE_DELAY	(6 * 256)

#define DEV_CAPS_16BIT_POS	0

#define DEV_CAPS_16BIT		(1 << DEV_CAPS_16BIT_POS)

#define MAX_ANALOG_PROBES_NUM 9
#define MAX_DSO_PROBES_NUM 2

#define DEFAULT_SAMPLERATE SR_MHZ(100)
#define DEFAULT_SAMPLELIMIT SR_MB(16)

#define SR_KB(n) ((n) * (uint64_t)(1024ULL))
#define SR_MB(n) ((n) * (uint64_t)(1048576ULL))


static const dslogic_profile supported_fx2[3] = {
	/*
	 * DSLogic
	 */
	{0x2A0E, 0x0001, NULL, "DSLogic", NULL,
		"DSLogic.fw",
		"DSLogic33.bin",
		"DSLogic50.bin",
		DEV_CAPS_16BIT},

	{ 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};
#define NUM_SIMUL_TRANSFERS	64
#define MAX_EMPTY_TRANSFERS	(NUM_SIMUL_TRANSFERS * 2)

struct dev_context;
SR_PRIV gboolean dslogic_check_conf_profile(libusb_device *dev);

// NEW_API
SR_PRIV struct dev_context *dslogic_dev_new(void);
SR_PRIV clk_source dslogic_get_clock_source(const struct sr_dev_inst* sdi);
SR_PRIV void dslogic_set_profile(struct dev_context* devc,const dslogic_profile* prof);
SR_PRIV void dslogic_set_firmware_updated(const struct sr_dev_inst* sdi);
SR_PRIV int dslogic_dev_open(struct sr_dev_inst* sdi, const struct sr_dev_driver* di);
SR_PRIV int dslogic_program_fpga(const struct sr_dev_inst* sdi);
SR_PRIV uint64_t dslogic_get_sample_limit(const struct sr_dev_inst* sdi);
SR_PRIV int dslogic_set_sample_limit(const struct sr_dev_inst* sdi, uint64_t value);
SR_PRIV int dslogic_set_voltage_threshold(const struct sr_dev_inst* sdi, voltage_range value);
SR_PRIV voltage_range dslogic_get_voltage_threshold(const struct sr_dev_inst* sdi);
SR_PRIV dev_mode dslogic_get_device_mode(const struct sr_dev_inst* sdi);
SR_PRIV int dslogic_set_device_mode(const struct sr_dev_inst* sdi, dev_mode value);
SR_PRIV int dslogic_set_samplerate(const struct sr_dev_inst* sdi, uint64_t samplerate);
SR_PRIV uint64_t dslogic_get_sample_rate(const struct sr_dev_inst* sdi);
SR_PRIV void dslogic_acquisition_stop(const struct sr_dev_inst* sdi);
SR_PRIV uint64_t dslogic_get_capture_ratio(const struct sr_dev_inst* sdi);
SR_PRIV int dslogic_set_capture_ratio(const struct sr_dev_inst* sdi, uint64_t ratio);
SR_PRIV int dslogic_start_acquisition(const struct sr_dev_inst* sdi,
                                     const struct sr_dev_driver * di,
                                     sr_receive_data_callback receive_data,
                                      void* cb_data);
SR_PRIV int dslogic_get_sample_count(const struct sr_dev_inst* sdi);
SR_PRIV dslogic_status dslogic_get_device_status(const struct sr_dev_inst* sdi);
SR_PRIV void dslogic_clear_trigger_stages(const struct sr_dev_inst* sdi);
SR_PRIV void dslogic_set_sample_wide(const struct sr_dev_inst* sdi, int wide);
SR_PRIV void dslogic_abort_acquisition(const struct sr_dev_inst* sdi);
SR_PRIV int dslogic_get_sample_wide(const struct sr_dev_inst* sdi);
SR_PRIV gboolean dslogic_increase_empty_sample_count(const struct sr_dev_inst* sdi);
SR_PRIV gboolean dslogic_sample_complete(const struct sr_dev_inst* sdi);
SR_PRIV void dslogic_process_data(const struct sr_dev_inst* sdi, uint8_t* data, int data_size);
SR_PRIV void dslogic_set_trigger_stage(const struct sr_dev_inst* sdi);
SR_PRIV void dslogic_reset_empty_transfer_count(const struct sr_dev_inst* sdi);

#endif
