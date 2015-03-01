/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
 * Copyright (C) 2013 DreamSourceLab <dreamsourcelab@dreamsourcelab.com>
 * Copyright (C) 2015 Diego F. Asanza <f.asanza@gmail.com>
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

#include <sys/stat.h>
#include <errno.h>
#include <glib/gstdio.h>
#include <libusb.h>
#include "dslogic.h"
#include "protocol.h"
#include "transfer.h"
#include <errno.h>
#include <string.h>
#include "libsigrok-internal.h"
#include <math.h>


SR_PRIV void abort_acquisition(struct dev_context *devc) {
	int i, ret;
	struct sr_usb_dev_inst *usb;

	devc->num_samples = -1;

	sr_info("%s: Stopping", __func__);

	/* Stop GPIF acquisition */
	usb = ((struct sr_dev_inst *) devc->cb_data)->conn;
	if ((ret = command_stop_acquisition (usb->devhdl)) != SR_OK)
		sr_err("Stop DSLogic acquisition failed!");
	else
		sr_info("Stop DSLogic acquisition!");

	/* Cancel exist transfers */
	for (i = devc->num_transfers - 1; i >= 0; i--) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}
}

SR_PRIV int fpga_setting(const struct sr_dev_inst *sdi) {
    struct dev_context *devc;
    struct sr_usb_dev_inst *usb;
    struct libusb_device_handle *hdl;
    struct DSLogic_setting setting;
    int ret;
    int transferred;
    int result;
    int i;

    devc = sdi->priv;
    usb = sdi->conn;
    hdl = usb->devhdl;

    setting.sync = 0xf5a5f5a5;
    setting.mode_header = 0x0001;
    setting.divider_header = 0x0102ffff;
    setting.count_header = 0x0302ffff;
    setting.trig_pos_header = 0x0502ffff;
    setting.trig_glb_header = 0x0701;
    setting.trig_adp_header = 0x0a02ffff;
    setting.trig_sda_header = 0x0c02ffff;
    setting.trig_mask0_header = 0x1010ffff;
    setting.trig_mask1_header = 0x1110ffff;
    //setting.trig_mask2_header = 0x1210ffff;
    //setting.trig_mask3_header = 0x1310ffff;
    setting.trig_value0_header = 0x1410ffff;
    setting.trig_value1_header = 0x1510ffff;
    //setting.trig_value2_header = 0x1610ffff;
    //setting.trig_value3_header = 0x1710ffff;
    setting.trig_edge0_header = 0x1810ffff;
    setting.trig_edge1_header = 0x1910ffff;
    //setting.trig_edge2_header = 0x1a10ffff;
    //setting.trig_edge3_header = 0x1b10ffff;
    setting.trig_count0_header = 0x1c10ffff;
    setting.trig_count1_header = 0x1d10ffff;
    //setting.trig_count2_header = 0x1e10ffff;
    //setting.trig_count3_header = 0x1f10ffff;
    setting.trig_logic0_header = 0x2010ffff;
    setting.trig_logic1_header = 0x2110ffff;
    //setting.trig_logic2_header = 0x2210ffff;
    //setting.trig_logic3_header = 0x2310ffff;
    setting.end_sync = 0xfa5afa5a;

    //setting.mode = (test_mode ? 0x8000 : 0x0000) + trigger->trigger_en + (sdi->mode << 4);
    setting.mode = 0<<15; //((devc->op_mode == SR_OP_INTERNAL_TEST) << 15);
    setting.mode += 0<<14; //((devc->op_mode == SR_OP_EXTERNAL_TEST) << 14);
    setting.mode += 0<<13; //((devc->op_mode == SR_OP_LOOPBACK_TEST) << 13);
    setting.mode += 0; //trigger->trigger_en;
    setting.mode += 0<<4; //((sdi->mode > 0) << 4); 0=logic, 1= dso; 2 = analog
    setting.mode += ((devc->clock_source == CLOCK_EXT_CLK) << 1);
    setting.mode += (devc->clock_edge << 1);
    setting.mode += (((devc->current_samplerate == SR_MHZ(200) && 1/*sdi->mode != DSO*/) || (0/*sdi->mode == ANALOG*/)) << 5);
    setting.mode += ((devc->current_samplerate == SR_MHZ(400)) << 6);
    setting.mode += 0<<7; //((sdi->mode == ANALOG) << 7);
    setting.mode += 0<<8; //((devc->filter == SR_FILTER_1T) << 8); no filter
    setting.divider = (uint32_t) ceil(SR_MHZ(100) * 1.0 / devc->current_samplerate);
    setting.count = (uint32_t) (devc->sample_limit);
    setting.trig_pos = (uint32_t)(/*trigger->trigger_pos*/1 / 100.0f * devc->sample_limit); //danot sure about it.
    setting.trig_glb = 0; //trigger->trigger_stages; //no trigger stages
    setting.trig_adp = setting.count - setting.trig_pos - 1;
    setting.trig_sda = 0x0;
    if (1) {//trigger->trigger_mode == SIMPLE_TRIGGER) {
        setting.trig_mask0[0] = 0; //ds_trigger_get_mask0(TriggerStages);
        setting.trig_mask1[0] = 0; //ds_trigger_get_mask1(TriggerStages);

        setting.trig_value0[0] = 0; //ds_trigger_get_value0(TriggerStages);
        setting.trig_value1[0] = 0; //ds_trigger_get_value1(TriggerStages);

        setting.trig_edge0[0] = 0; //ds_trigger_get_edge0(TriggerStages);
        setting.trig_edge1[0] = 0; //ds_trigger_get_edge1(TriggerStages);

        setting.trig_count0[0] = 0; //trigger->trigger0_count[TriggerStages];
        setting.trig_count1[0] = 0; // trigger->trigger1_count[TriggerStages];

        setting.trig_logic0[0] = 0; //(trigger->trigger_logic[TriggerStages] << 1) + trigger->trigger0_inv[TriggerStages];
        setting.trig_logic1[0] = 0; //(trigger->trigger_logic[TriggerStages] << 1) + trigger->trigger1_inv[TriggerStages];

        for (i = 1; i < NUM_TRIGGER_STAGES; i++) {
            setting.trig_mask0[i] = 0xff;
            setting.trig_mask1[i] = 0xff;

            setting.trig_value0[i] = 0;
            setting.trig_value1[i] = 0;

            setting.trig_edge0[i] = 0;
            setting.trig_edge1[i] = 0;

            setting.trig_count0[i] = 0;
            setting.trig_count1[i] = 0;

            setting.trig_logic0[i] = 2;
            setting.trig_logic1[i] = 2;
        }
    } else {/*
        for (i = 0; i < NUM_TRIGGER_STAGES; i++) {
            setting.trig_mask0[i] = ds_trigger_get_mask0(i);
            setting.trig_mask1[i] = ds_trigger_get_mask1(i);

            setting.trig_value0[i] = ds_trigger_get_value0(i);
            setting.trig_value1[i] = ds_trigger_get_value1(i);

            setting.trig_edge0[i] = ds_trigger_get_edge0(i);
            setting.trig_edge1[i] = ds_trigger_get_edge1(i);

            setting.trig_count0[i] = trigger->trigger0_count[i];
            setting.trig_count1[i] = trigger->trigger1_count[i];

            setting.trig_logic0[i] = (trigger->trigger_logic[i] << 1) + trigger->trigger0_inv[i];
            setting.trig_logic1[i] = (trigger->trigger_logic[i] << 1) + trigger->trigger1_inv[i];
        }*/
    }

    result = SR_OK;
    ret = libusb_bulk_transfer(hdl, 2 | LIBUSB_ENDPOINT_OUT,
            (unsigned char*)&setting, sizeof (struct DSLogic_setting),
            &transferred, 1000);

    if (ret < 0) {
        sr_err("Unable to setting FPGA of DSLogic: %s.",
                libusb_error_name(ret));
        result = SR_ERR;
    } else if (transferred != sizeof (struct DSLogic_setting)) {
        sr_err("Setting FPGA error: expacted transfer size %d; actually %d",
                sizeof (struct DSLogic_setting), transferred);
        result = SR_ERR;
    }

    if (result == SR_OK)
        sr_info("FPGA setting done");

    return result;
}

SR_PRIV int fpga_config(struct libusb_device_handle *hdl, const char *filename) {
    FILE *fw;
    int offset, chunksize, ret, result;
    unsigned char *buf;
    int transferred;
    uint64_t filesize;
    struct stat f_stat;

    sr_info("Configure FPGA using %s", filename);
    if ((fw = g_fopen(filename, "rb")) == NULL) {
        sr_err("Unable to open FPGA bit file %s for reading: %s",
                filename, strerror(errno));
        return SR_ERR;
    }
    if (stat(filename, &f_stat) == -1)
        return SR_ERR;

    filesize = (uint64_t) f_stat.st_size;
    if (!(buf = g_try_malloc(filesize))) {
        sr_err("FPGA configure bit malloc failed.");
        return SR_ERR;
    }

    result = SR_OK;
    offset = 0;
    while (1) {
        chunksize = fread(buf, 1, filesize, fw);
        if (chunksize == 0)
            break;

        ret = libusb_bulk_transfer(hdl, 2 | LIBUSB_ENDPOINT_OUT,
                buf, chunksize,
                &transferred, 1000);

        if (ret < 0) {
            sr_err("Unable to configure FPGA of DSLogic: %s.",
                    libusb_error_name(ret));
            result = SR_ERR;
            break;
        } else if (transferred != chunksize) {
            sr_err("Configure FPGA error: expacted transfer size %d; actually %d",
                    chunksize, transferred);
            result = SR_ERR;
            break;
        }
        sr_info("Configure %d bytes", chunksize);
        offset += chunksize;
    }
    fclose(fw);
    if (result == SR_OK)
        sr_info("FPGA configure done");

    return result;
}

/**
 * Check the USB configuration to determine if this is an DSLogic device.
 *
 * @return TRUE if the device's configuration profile match DSLogic
 *         configuration, FALSE otherwise.
 */
SR_PRIV gboolean check_conf_profile(libusb_device *dev) {
    struct libusb_device_descriptor des;
    struct libusb_device_handle *hdl;
    gboolean ret;
    unsigned char strdesc[64];

    hdl = NULL;
    ret = FALSE;
    while (!ret) {
        /* Assume the FW has not been loaded, unless proven wrong. */
        if (libusb_get_device_descriptor(dev, &des) != 0)
            break;

        if (libusb_open(dev, &hdl) != 0)
            break;

        if (libusb_get_string_descriptor_ascii(hdl,
                des.iManufacturer, strdesc, sizeof (strdesc)) < 0)
            break;
        if (strncmp((const char *) strdesc, "DreamSourceLab", 14))
            break;

        if (libusb_get_string_descriptor_ascii(hdl,
                des.iProduct, strdesc, sizeof (strdesc)) < 0)
            break;
        if (strncmp((const char *) strdesc, "DSLogic", 7))
            break;

        /* If we made it here, it must be an DSLogic. */
        ret = TRUE;
    }
    if (hdl)
        libusb_close(hdl);

    return ret;
}

SR_PRIV int dev_status_get(struct sr_dev_inst *sdi, struct DSLogic_status *status) {
    printf("get_status\n");
    if (sdi) {
        struct sr_usb_dev_inst *usb;
        int ret;

        usb = sdi->conn;
        ret = command_get_status(usb->devhdl, status);
        if (ret != SR_OK) {
            sr_err("Device don't exist!");
            return SR_ERR;
        } else {
            return SR_OK;
        }
    } else {
        return SR_ERR;
    }
}

SR_PRIV int dev_test(struct sr_dev_inst *sdi) {
    printf("set_test\n");
    if (sdi) {
        struct sr_usb_dev_inst *usb;
        struct version_info vi;
        int ret;

        usb = sdi->conn;
        ret = command_get_fw_version(usb->devhdl, &vi);
        if (ret != SR_OK) {
            sr_err("Device don't exist!");
            return SR_ERR;
        } else {
            return SR_OK;
        }
    } else {
        return SR_ERR;
    }
}

SR_PRIV void receive_trigger_pos(struct libusb_transfer *transfer) {
	struct dev_context *devc;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;
	//struct sr_datafeed_dso dso;
	//struct sr_datafeed_analog analog;
	struct ds_trigger_pos *trigger_pos;
	int ret;

	devc = transfer->user_data;
	sr_info("receive trigger pos handle...");

	if (devc->num_samples == -1) {
		free_transfer(transfer);
		return;
	}

	sr_info("receive_trigger_pos(): status %d; timeout %d; received %d bytes.",
	        transfer->status, transfer->timeout, transfer->actual_length);

	if (devc->status != DSLOGIC_ERROR) {
		trigger_pos = (struct ds_trigger_pos *) transfer->buffer;
		switch (transfer->status) {
			case LIBUSB_TRANSFER_COMPLETED:
				packet.type = SR_DF_TRIGGER;
				packet.payload = trigger_pos;
				sr_session_send(devc->cb_data, &packet);

				if (1) {//(*(struct sr_dev_inst *)(devc->cb_data)).mode == LOGIC) {
		packet.type = SR_DF_LOGIC;
		packet.payload = &logic;
		logic.unitsize = devc->sample_wide ? 2 : 1;
		logic.length = sizeof (trigger_pos->first_block);
		//logic.data_error = 0;
		logic.data = trigger_pos->first_block;
		devc->num_samples += logic.length / logic.unitsize;
	} /*else if ((*(struct sr_dev_inst *)(devc->cb_data)).mode == DSO){
		packet.type = SR_DF_DSO;
		packet.payload = &dso;
		dso.probes = (*(struct sr_dev_inst *)(devc->cb_data)).probes;
		dso.num_samples = sizeof(trigger_pos->first_block) / (devc->sample_wide ? 2 : 1);
		dso.mq = SR_MQ_VOLTAGE;
		dso.unit = SR_UNIT_VOLT;
		dso.mqflags = SR_MQFLAG_AC;
		dso.data = trigger_pos->first_block;;
		} else {
			packet.type = SR_DF_ANALOG;
			packet.payload = &analog;
			analog.probes = (*(struct sr_dev_inst *)(devc->cb_data)).probes;
			analog.num_samples = sizeof(trigger_pos->first_block) / (devc->sample_wide ? 2 : 1);
			analog.mq = SR_MQ_VOLTAGE;
			analog.unit = SR_UNIT_VOLT;
			analog.mqflags = SR_MQFLAG_AC;
			analog.data = trigger_pos->first_block;;
		}*/

				sr_session_send(devc->cb_data, &packet);

				devc->status = DSLOGIC_TRIGGERED;
				free_transfer(transfer);
				devc->num_transfers = 0;
				break;
			default:
				//abort_acquisition(devc);
				free_transfer(transfer);
				devc->status = DSLOGIC_ERROR;
				break;
		}

		//        if (devc->status != DSLOGIC_START) {
		//            g_free(transfer->buffer);
		//            transfer->buffer = NULL;
		//            libusb_free_transfer(transfer);
		//        }
		//        if (devc->status == DSLOGIC_STOP) {
		//            finish_acquisition(devc);
		//        }

		if (devc->status == DSLOGIC_TRIGGERED) {
			if ((ret = dev_transfer_start(devc->cb_data)) != SR_OK) {
				sr_err("%s: could not start data transfer"
				       "(%d)%d", __func__, ret, errno);
			}
		}
	}
}

SR_PRIV unsigned int get_timeout(struct dev_context *devc) {
	(void)devc;
	//size_t total_size;
	//unsigned int timeout;

	//total_size = get_buffer_size(devc) * get_number_of_transfers(devc);
	//timeout = total_size / to_bytes_per_ms(devc);
	//return timeout + timeout / 4; /* Leave a headroom of 25% percent. */
	return 1000;
}

SR_PRIV int dev_transfer_start(const struct sr_dev_inst *sdi) {
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct libusb_transfer *transfer;
	unsigned int i;
	//unsigned int timeout;
	unsigned int num_transfers;
	int ret;
	unsigned char *buf;
	size_t size;

	devc = sdi->priv;
	usb = sdi->conn;

	//timeout = get_timeout(devc);
	num_transfers = get_number_of_transfers(devc);
	size = /*(sdi->mode == ANALOG) ? cons_buffer_size : ((sdi->mode == DSO) ? dso_buffer_size : */ get_buffer_size(devc);
	devc->submitted_transfers = 0;

	devc->transfers = g_try_malloc0(sizeof (*devc->transfers) * num_transfers);
	if (!devc->transfers) {
		sr_err("USB transfers malloc failed.");
		return SR_ERR_MALLOC;
	}

	devc->num_transfers = num_transfers;
	for (i = 0; i < num_transfers; i++) {
		if (!(buf = g_try_malloc(size))) {
			sr_err("USB transfer buffer malloc failed.");
			return SR_ERR_MALLOC;
		}
		transfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(transfer, usb->devhdl,
		                          6 | LIBUSB_ENDPOINT_IN, buf, size,
		                          dslogic_receive_transfer, devc, 0);
		if ((ret = libusb_submit_transfer(transfer)) != 0) {
			sr_err("Failed to submit transfer: %s.",
			       libusb_error_name(ret));
			libusb_free_transfer(transfer);
			g_free(buf);
			abort_acquisition(devc);
			return SR_ERR;
		}
		devc->transfers[i] = transfer;
		devc->submitted_transfers++;
	}

	devc->status = DSLOGIC_DATA;

	return SR_OK;
}

SR_PRIV unsigned int get_number_of_transfers(struct dev_context *devc) {
	unsigned int n;
	/* Total buffer size should be able to hold about 100ms of data. */
	n = 100 * to_bytes_per_ms(devc) / get_buffer_size(devc);

	if (n > NUM_SIMUL_TRANSFERS)
		return NUM_SIMUL_TRANSFERS;

	return n;
}

SR_PRIV unsigned int to_bytes_per_ms(struct dev_context *devc) {
	if (devc->current_samplerate > SR_MHZ(100))
		return SR_MHZ(100) / 1000 * (devc->sample_wide ? 2 : 1);
	else
		return devc->current_samplerate / 1000 * (devc->sample_wide ? 2 : 1);
}

SR_PRIV size_t get_buffer_size(struct dev_context *devc) {
	size_t s;

	/*
	 * The buffer should be large enough to hold 20ms of data and
	 * a multiple of 512.
	 */
	s = 20 * to_bytes_per_ms(devc);
	return (s + 511) & ~511;
}