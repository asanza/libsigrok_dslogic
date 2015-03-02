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

struct DSLogic_setting {
	uint32_t sync;
	uint16_t mode_header;                   // 0
	uint16_t mode;
	uint32_t divider_header;                // 1-2
	uint32_t divider;
	uint32_t count_header;                  // 3-4
	uint32_t count;
	uint32_t trig_pos_header;               // 5-6
	uint32_t trig_pos;
	uint16_t trig_glb_header;               // 7
	uint16_t trig_glb;
	uint32_t trig_adp_header;               // 10-11
	uint32_t trig_adp;
	uint32_t trig_sda_header;               // 12-13
	uint32_t trig_sda;
	uint32_t trig_mask0_header;              // 16
	uint16_t trig_mask0[NUM_TRIGGER_STAGES];
	uint32_t trig_mask1_header;              // 17
	uint16_t trig_mask1[NUM_TRIGGER_STAGES];
	//uint32_t trig_mask2_header;              // 18
	//uint16_t trig_mask2[NUM_TRIGGER_STAGES];
	//uint32_t trig_mask3_header;              // 19
	//uint16_t trig_mask3[NUM_TRIGGER_STAGES];
	uint32_t trig_value0_header;             // 20
	uint16_t trig_value0[NUM_TRIGGER_STAGES];
	uint32_t trig_value1_header;             // 21
	uint16_t trig_value1[NUM_TRIGGER_STAGES];
	//uint32_t trig_value2_header;             // 22
	//uint16_t trig_value2[NUM_TRIGGER_STAGES];
	//uint32_t trig_value3_header;             // 23
	//uint16_t trig_value3[NUM_TRIGGER_STAGES];
	uint32_t trig_edge0_header;              // 24
	uint16_t trig_edge0[NUM_TRIGGER_STAGES];
	uint32_t trig_edge1_header;              // 25
	uint16_t trig_edge1[NUM_TRIGGER_STAGES];
	//uint32_t trig_edge2_header;              // 26
	//uint16_t trig_edge2[NUM_TRIGGER_STAGES];
	//uint32_t trig_edge3_header;              // 27
	//uint16_t trig_edge3[NUM_TRIGGER_STAGES];
	uint32_t trig_count0_header;             // 28
	uint16_t trig_count0[NUM_TRIGGER_STAGES];
	uint32_t trig_count1_header;             // 29
	uint16_t trig_count1[NUM_TRIGGER_STAGES];
	//uint32_t trig_count2_header;             // 30
	//uint16_t trig_count2[NUM_TRIGGER_STAGES];
	//uint32_t trig_count3_header;             // 31
	//uint16_t trig_count3[NUM_TRIGGER_STAGES];
	uint32_t trig_logic0_header;             // 32
	uint16_t trig_logic0[NUM_TRIGGER_STAGES];
	uint32_t trig_logic1_header;             // 33
	uint16_t trig_logic1[NUM_TRIGGER_STAGES];
	//uint32_t trig_logic2_header;             // 34
	//uint16_t trig_logic2[NUM_TRIGGER_STAGES];
	//uint32_t trig_logic3_header;             // 35
	//uint16_t trig_logic3[NUM_TRIGGER_STAGES];
	uint32_t end_sync;
};

struct dev_context;
SR_PRIV int fpga_setting(const struct sr_dev_inst *sdi);
SR_PRIV int fpga_config(struct libusb_device_handle *hdl, const char *filename);
SR_PRIV gboolean check_conf_profile(libusb_device *dev);
SR_PRIV int dev_status_get(struct sr_dev_inst *sdi, struct DSLogic_status *status);
SR_PRIV int dev_test(struct sr_dev_inst *sdi);
SR_PRIV void receive_trigger_pos(struct libusb_transfer *transfer);
SR_PRIV unsigned int get_timeout(struct dev_context *devc);
SR_PRIV unsigned int get_number_of_transfers(struct dev_context *devc);
SR_PRIV unsigned int to_bytes_per_ms(struct dev_context *devc);
SR_PRIV size_t get_buffer_size(struct dev_context *devc);
SR_PRIV int dev_transfer_start(const struct sr_dev_inst *sdi);

// NEW_API
SR_PRIV struct dev_context *dslogic_dev_new(void);
SR_PRIV gboolean dslogic_identify_by_vid_and_pid(struct dev_context* devc, int vid, int pid);
SR_PRIV void dslogic_set_profile(struct dev_context* devc,const dslogic_profile* prof);
SR_PRIV void dslogic_set_firmware_updated(struct dev_context* devc);
SR_PRIV int dslogic_dev_open(struct sr_dev_inst* sdi, struct sr_dev_driver* di);
SR_PRIV int dslogic_configure_fpga(struct sr_dev_inst* sdi);
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
SR_PRIV int dslogic_send_fpga_settings(const struct sr_dev_inst* sdi, void* cb_data);
SR_PRIV int dslogic_set_usb_transfer(struct sr_dev_inst* sdi,
                                     struct sr_dev_driver * di,
                                     sr_receive_data_callback receive_data);
SR_PRIV int dslogic_get_sample_count(const struct sr_dev_inst* sdi);
SR_PRIV dslogic_status dslogic_get_device_status(const struct sr_dev_inst* sdi);
SR_PRIV void dslogic_clear_trigger_stages(const struct sr_dev_inst* sdi);
SR_PRIV void dslogic_set_sample_wide(const struct sr_dev_inst* sdi, int wide);
SR_PRIV void abort_acquisition(const struct sr_dev_inst* sdi);
#endif
