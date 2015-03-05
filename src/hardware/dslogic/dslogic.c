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
#include <errno.h>
#include <string.h>
#include "libsigrok.h"
#include "libsigrok-internal.h"
#include <math.h>
#include "devdefs.h"
#include "fpga.h"

#ifndef _WIN32
#undef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif

struct ds_trigger_pos {
    uint32_t real_pos;
    uint32_t ram_saddr;
    unsigned char first_block[504];
};

struct dev_context {
    const dslogic_profile *profile;
    /*
    * Since we can't keep track of an DSLogic device after upgrading
    * the firmware (it renumerates into a different device address
    * after the upgrade) this is like a global lock. No device will open
    * until a proper delay after the last device was upgraded.
    */
    int64_t fw_updated;
    /* Device/capture settings */
    uint64_t current_samplerate;
    uint64_t sample_limit;
    clk_source clock_source; // EXTERNAL = TRUE
    clk_edge clock_edge;   // RISING = 0, FALLING = 1
    voltage_range voltage_threshold;
    gboolean filter;
    uint32_t capture_ratio;
    dev_mode device_mode;
    /* device internals */
    int64_t sample_count;
    unsigned int num_transfers;
    int submitted_transfers;
    int empty_transfer_count;
    void *cb_data;
    struct libusb_transfer **transfers;
    dslogic_status status;
    /* wide of sample. TRUE = 16 channels (2 bytes),
     * FALSE = 8 channels */
    gboolean sample_width;
    gboolean trigger_enabled;

    /*sigrok context*/
    struct sr_context* ctx;

    /* trigger */
    struct fpga_trigger_settings trigger_settings;
    uint16_t trigger_mask[NUM_TRIGGER_STAGES];
    uint16_t trigger_value[NUM_TRIGGER_STAGES];
    uint16_t trigger_buffer[NUM_TRIGGER_STAGES];

};

static int dev_transfer_start(const struct sr_dev_inst *sdi);
static void finish_acquisition(const struct sr_dev_inst *sdi);
static unsigned int get_number_of_transfers(struct dev_context *devc);
static unsigned int to_bytes_per_ms(struct dev_context *devc);
static size_t get_buffer_size(struct dev_context *devc);
static void dslogic_receive_transfer(struct libusb_transfer *transfer);

static gboolean dslogic_identify_by_vid_and_pid(struct dev_context* devc, int vid, int pid);

static void finish_acquisition(const struct sr_dev_inst *sdi) {
    struct dev_context *devc = sdi->priv;
    struct sr_datafeed_packet packet;
    sr_err("finish acquisition: send SR_DF_END packet");
    /* Terminate session. */
    packet.type = SR_DF_END;
    sr_session_send(devc->cb_data, &packet);
    sr_err("finish acquisition: remove fds from polling");
    if (devc->num_transfers != 0) {
        devc->num_transfers = 0;
        g_free(devc->transfers);
    }
    usb_source_remove(sdi->session, devc->ctx);
}

static void clear_transfer(const struct sr_dev_inst* sdi, const struct libusb_transfer* transfer){
    g_assert(sdi);
    g_assert(transfer);
    unsigned int i;
    struct dev_context* devc = sdi->priv;
    for (i = 0; i < devc->num_transfers; i++) {
        if (devc->transfers[i] == transfer) {
            devc->transfers[i] = NULL;
            break;
        }
    }
    devc->submitted_transfers--;
    if (devc->submitted_transfers == 0 && devc->status != DSLOGIC_TRIGGERED)
        finish_acquisition(sdi);
}

static void free_transfer(struct libusb_transfer *transfer)
{
    struct sr_dev_inst *sdi = transfer->user_data;
    g_free(transfer->buffer);
    transfer->buffer = NULL;
    libusb_free_transfer(transfer);
    clear_transfer(sdi, transfer);
}

SR_PRIV void dslogic_abort_acquisition(const struct sr_dev_inst* sdi) {
    g_assert(sdi);
    struct dev_context *devc = sdi->priv;
    int i, ret;
    struct sr_usb_dev_inst *usb;
    devc->sample_count = -1;
    sr_info("%s: Stopping", __func__);
    /* Stop GPIF acquisition */
    usb = ((struct sr_dev_inst *) devc->cb_data)->conn;
    if ((ret = command_stop_acquisition (usb->devhdl)) != SR_OK)
        sr_err("Stop DSLogic acquisition failed!");
    else
        sr_info("Stop DSLogic acquisition!");
    /* Cancel exist transfers */
    for (i = devc->num_transfers - 1; i >= 0; i--) {
        if (devc->transfers[i]){
            libusb_cancel_transfer(devc->transfers[i]);
        }
    }
}

static int set_fpga_setting(const struct sr_dev_inst *sdi) {
    struct dev_context *devc;
    struct sr_usb_dev_inst *usb;
    struct libusb_device_handle *hdl;
    struct dslogic_fpga_setting* setting;
    setting = dslogic_fpga_new_setting();
    int ret;
    int transferred;

    devc = sdi->priv;
    usb = sdi->conn;
    hdl = usb->devhdl;

    dslogic_fpga_set_mode(setting, devc->device_mode);
    dslogic_fpga_set_samplerate(setting,devc->current_samplerate, devc->sample_limit);
    dslogic_fpga_set_trigger(setting, (uint32_t)(devc->capture_ratio/ 100.0f * devc->sample_limit),
                             devc->trigger_enabled, &devc->trigger_settings);
    int setting_size = dslogic_get_fpga_setting_size();
    ret = libusb_bulk_transfer(hdl, 2 | LIBUSB_ENDPOINT_OUT,
                               (unsigned char*)setting, setting_size,
                               &transferred, 1000);
    if (ret < 0) {
        sr_err("Unable to setting FPGA of DSLogic: %s.",
               libusb_error_name(ret));
        ret = SR_ERR;
    } else if (transferred != setting_size) {
        sr_err("Setting FPGA error: expacted transfer size %d; actually %d",
               setting_size, transferred);
        ret = SR_ERR;
    }

    if (ret == SR_OK)
        sr_info("FPGA setting done");

    dslogic_fpga_setting_free(setting);
    return ret;
}

static int fpga_config(struct libusb_device_handle *hdl, const char *filename) {
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
SR_PRIV gboolean dslogic_check_conf_profile(libusb_device *dev) {
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

static void receive_trigger_pos(struct libusb_transfer *transfer) {
    struct sr_dev_inst *sdi = transfer->user_data;
    struct dev_context *devc = sdi->priv;
    struct sr_datafeed_packet packet;
    struct sr_datafeed_logic logic;
    struct ds_trigger_pos *trigger_pos;
    int ret;
    sr_info("receive trigger pos handle...");
    if (devc->sample_count == -1) {
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
            packet.type = SR_DF_LOGIC;
            packet.payload = &logic;
            logic.unitsize = devc->sample_width ? 2 : 1;
            logic.length = sizeof (trigger_pos->first_block);
            logic.data = trigger_pos->first_block;
            devc->sample_count += logic.length / logic.unitsize;
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
        if (devc->status == DSLOGIC_TRIGGERED) {
            if ((ret = dev_transfer_start(devc->cb_data)) != SR_OK) {
                sr_err("%s: could not start data transfer"
                       "(%d)%d", __func__, ret, errno);
            }
        }
    }
}

static unsigned int get_timeout(struct dev_context *devc) {
    (void)devc;
    //size_t total_size;
    //unsigned int timeout;

    //total_size = get_buffer_size(devc) * get_number_of_transfers(devc);
    //timeout = total_size / to_bytes_per_ms(devc);
    //return timeout + timeout / 4; /* Leave a headroom of 25% percent. */
    return 1000;
}

static int dev_transfer_start(const struct sr_dev_inst *sdi) {
    struct dev_context *devc;
    struct sr_usb_dev_inst *usb;
    struct libusb_transfer *transfer;
    unsigned int i;
    unsigned int num_transfers;
    int ret;
    unsigned char *buf;
    size_t size;

    devc = sdi->priv;
    usb = sdi->conn;

    num_transfers = get_number_of_transfers(devc);
    size = get_buffer_size(devc);
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
                                  dslogic_receive_transfer, (void*)sdi, 0);
        if ((ret = libusb_submit_transfer(transfer)) != 0) {
            sr_err("Failed to submit transfer: %s.",
                   libusb_error_name(ret));
            libusb_free_transfer(transfer);
            g_free(buf);
            dslogic_abort_acquisition(sdi);
            return SR_ERR;
        }
        devc->transfers[i] = transfer;
        devc->submitted_transfers++;
    }

    devc->status = DSLOGIC_DATA;

    return SR_OK;
}

static unsigned int get_number_of_transfers(struct dev_context *devc) {
    unsigned int n;
    /* Total buffer size should be able to hold about 100ms of data. */
    n = 100 * to_bytes_per_ms(devc) / get_buffer_size(devc);

    if (n > NUM_SIMUL_TRANSFERS)
        return NUM_SIMUL_TRANSFERS;

    return n;
}

static unsigned int to_bytes_per_ms(struct dev_context *devc) {
    if (devc->current_samplerate > SR_MHZ(100))
        return SR_MHZ(100) / 1000 * (devc->sample_width ? 2 : 1);
    else
        return devc->current_samplerate / 1000 * (devc->sample_width ? 2 : 1);
}

static size_t get_buffer_size(struct dev_context *devc) {
    size_t s;

    /*
     * The buffer should be large enough to hold 20ms of data and
     * a multiple of 512.
     */
    s = 20 * to_bytes_per_ms(devc);
    return (s + 511) & ~511;
}

static int open_device(struct sr_dev_inst *sdi, const struct sr_dev_driver* di) {
    libusb_device **devlist;
    struct sr_usb_dev_inst *usb;
    struct libusb_device_descriptor des;
    struct dev_context *devc;
    struct drv_context *drvc;
    struct version_info vi;
    //int skip;
    int ret, i, device_count;
    uint8_t revid;
    char connection_id[64];

    drvc = di->priv;
    devc = sdi->priv;
    usb = sdi->conn;

    if (sdi->status == SR_ST_ACTIVE)
        /* Device is already in use. */
        return SR_ERR;
    device_count = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
    if (device_count < 0) {
        sr_err("Failed to get device list: %s.",
               libusb_error_name(device_count));
        return SR_ERR;
    }

    for (i = 0; i < device_count; i++) {
        if ((ret = libusb_get_device_descriptor(devlist[i], &des))) {
            sr_err("Failed to get device descriptor: %s.",
                   libusb_error_name(ret));
            continue;
        }

        if (!dslogic_identify_by_vid_and_pid(devc, des.idVendor, des.idProduct))
            continue;

        if (sdi->status == SR_ST_INITIALIZING || sdi->status == SR_ST_INACTIVE) {
            /* check device by its physical usb bus/port address */
            usb_get_port_path(devlist[i], connection_id, sizeof (connection_id));
            if (strcmp(sdi->connection_id, connection_id))
                /* This is not the one. */
                continue;
        }

        if (!(ret = libusb_open(devlist[i], &usb->devhdl))) {
            if (usb->address == 0xff)
                /*
                 * First time we touch this device after FW
                 * upload, so we don't know the address yet.
                 */
                usb->address = libusb_get_device_address(devlist[i]);
        } else {
            sr_err("pFailed to open device: %s.",
                   libusb_error_name(ret));
            break;
        }

        ret = command_get_fw_version(usb->devhdl, &vi);
        if (ret != SR_OK) {
            sr_err("Failed to get firmware version.");
            break;
        }

        ret = command_get_revid_version(usb->devhdl, &revid);
        if (ret != SR_OK) {
            sr_err("Failed to get REVID.");
            break;
        }
        /*
         * Changes in major version mean incompatible/API changes, so
         * bail out if we encounter an incompatible version.
         * Different minor versions are OK, they should be compatible.
         */
        if (vi.major != DSLOGIC_REQUIRED_VERSION_MAJOR) {
            sr_err("Expected firmware version %d.x, "
                   "got %d.%d.", DSLOGIC_REQUIRED_VERSION_MAJOR,
                   vi.major, vi.minor);
            break;
        }

        sdi->status = SR_ST_ACTIVE;
        sr_info("Opened device %a on %d.%d, "
                "interface %d, firmware %d.%d.",
                sdi->connection_id, usb->bus, usb->address,
                USB_INTERFACE, vi.major, vi.minor);

        sr_info("Detected REVID=%d, it's a Cypress CY7C68013%s.",
                revid, (revid != 1) ? " (FX2)" : "A (FX2LP)");

        break;
    }

    libusb_free_device_list(devlist, 1);
    if (sdi->status != SR_ST_ACTIVE)
        return SR_ERR;
    return SR_OK;
}

// NEW API
//TODO: Remove
static const char* config_path = "/home/diego/media/DSLogic/dslogic-gui/res/";

SR_PRIV struct dev_context *dslogic_dev_new(void) {
    struct dev_context *devc;
    if (!(devc = g_try_malloc(sizeof (struct dev_context)))) {
        sr_err("Device context malloc failed.");
        return NULL;
    }
    int i,j;
    devc->profile = NULL;
    devc->fw_updated = 0;
    devc->current_samplerate = DEFAULT_SAMPLERATE;
    devc->sample_limit = DEFAULT_SAMPLELIMIT;
    devc->sample_width = 1;
    devc->clock_source = CLOCK_INTERNAL;
    devc->clock_edge = RISING;
    devc->capture_ratio = 0;
    devc->voltage_threshold = VOLTAGE_RANGE_18_33_V;
    devc->filter = FALSE;
    devc->device_mode = NORMAL_MODE;
    devc->trigger_enabled = FALSE;
    for (i = 0; i <= NUM_TRIGGER_STAGES; i++) {
        for (j = 0; j < NUM_TRIGGER_STAGES; j++) {
            devc->trigger_settings.trigger0[i][j] = TRIGGER_FIRED;
            devc->trigger_settings.trigger1[i][j] = TRIGGER_FIRED;
        }
        devc->trigger_settings.trigger0_count[i] = 0;
        devc->trigger_settings.trigger1_count[i] = 0;
        devc->trigger_settings.trigger0_inv[i] = 0;
        devc->trigger_settings.trigger1_inv[i] = 0;
        devc->trigger_settings.trigger_logic[i] = 1;
    }
    return devc;
}


static gboolean dslogic_identify_by_vid_and_pid(struct dev_context* devc, int vid, int pid){
    return !(vid != devc->profile->vid
            || pid != devc->profile->pid);
}

SR_PRIV void dslogic_set_profile(struct dev_context* devc,const dslogic_profile* prof){
    devc->profile = prof;
}

SR_PRIV void dslogic_set_firmware_updated(const struct sr_dev_inst* sdi){
    g_assert(sdi);
    struct dev_context* devc = sdi->priv;
    devc->fw_updated = g_get_monotonic_time();
}

SR_PRIV clk_source dslogic_get_clock_source(const struct sr_dev_inst* sdi){
    g_assert(sdi);
    struct dev_context* devc = sdi->priv;
    return devc->clock_source;
}

SR_PRIV void dslogic_set_clock_source(const struct sr_dev_inst* sdi,clk_source source){
    g_assert(sdi);
    struct dev_context* devc = sdi->priv;
    devc->clock_source = source;
}

SR_PRIV void dslogic_set_clock_edge(const struct sr_dev_inst* sdi, clk_edge edge){
    g_assert(sdi);
    struct dev_context* devc = sdi->priv;
    devc->clock_edge = edge;
}

SR_PRIV clk_edge dslogic_get_clock_edge(const struct sr_dev_inst* sdi){
    g_assert(sdi);
    struct dev_context* devc = sdi->priv;
    return devc->clock_edge;
}

SR_PRIV int dslogic_dev_open(struct sr_dev_inst* sdi, const struct sr_dev_driver* di){
    struct dev_context* devc = sdi->priv;
    uint64_t timediff_us, timediff_ms, ret;
    if (devc->fw_updated > 0) {
        sr_info("Waiting for device to reset.");
        /* Takes >= 300ms for the FX2 to be gone from the USB bus. */
        g_usleep(300 * 1000);
        timediff_ms = 0;
        while (timediff_ms < MAX_RENUM_DELAY_MS) {
            if ((ret = open_device(sdi, di)) == SR_OK)
                break;
            g_usleep(100 * 1000);

            timediff_us = g_get_monotonic_time() - devc->fw_updated;
            timediff_ms = timediff_us / 1000;
            sr_spew("Waited %" PRIi64 "ms.", timediff_ms);
        }
        if (ret != SR_OK) {
            sr_err("Device failed to renumerate.");
            return SR_ERR;
        }
        sr_info("Device came back after %" PRIi64 "ms.", timediff_ms);
    } else {
        sr_info("Firmware upload was not needed.");
        ret = open_device(sdi,di);
    }
    return ret;
}

SR_PRIV int dslogic_program_fpga(const struct sr_dev_inst* sdi){
    struct dev_context* devc = sdi->priv;
    struct sr_usb_dev_inst* usb = sdi->conn;
    int ret;
    if ((ret = command_fpga_config(usb->devhdl)) != SR_OK) {
        sr_err("Send FPGA configure command failed!");
    } else {
        /* Takes >= 10ms for the FX2 to be ready for FPGA configure. */
        g_usleep(10 * 1000);
        char filename[256];
        switch (devc->voltage_threshold) {
        case VOLTAGE_RANGE_18_33_V:
            sprintf(filename, "%s%s", config_path, devc->profile->fpga_bit33);
            break;
        case VOLTAGE_RANGE_5_V:
            sprintf(filename, "%s%s", config_path, devc->profile->fpga_bit50);
            break;
        default:
            sr_err("wrong voltage settings");
            return SR_ERR;
        }
        const char *fpga_bit = filename;
        ret = fpga_config(usb->devhdl, fpga_bit);
        if (ret != SR_OK) {
            sr_err("Configure FPGA failed!");
        }
    }
    return ret;
}

SR_PRIV uint64_t dslogic_get_sample_limit(const struct sr_dev_inst* sdi){
    struct dev_context* devc = sdi->priv;
    return devc->sample_limit;
}

SR_PRIV int dslogic_set_sample_limit(const struct sr_dev_inst* sdi, uint64_t value){
    g_assert(sdi);
    if(value > SR_MB(16))
        return SR_ERR_ARG;
    struct dev_context* devc = sdi->priv;
    devc->sample_limit = value;
    sr_dbg("New Sample Limit: %d", devc->sample_limit);
    return SR_OK;
}

SR_PRIV voltage_range dslogic_get_voltage_threshold(const struct sr_dev_inst* sdi){
    g_assert(sdi);
    struct dev_context* devc = sdi->priv;
    return devc->voltage_threshold;
}

SR_PRIV int dslogic_set_voltage_threshold(const struct sr_dev_inst* sdi, voltage_range value){
    g_assert(sdi);
    struct dev_context* devc = sdi->priv;
    int ret = SR_OK;
    devc->voltage_threshold = value;
    ret = dslogic_program_fpga(sdi);
    return ret;
}

SR_PRIV dev_mode dslogic_get_device_mode(const struct sr_dev_inst* sdi){
    g_assert(sdi);
    struct dev_context* devc = sdi->priv;
    return devc->device_mode;
}

SR_PRIV int dslogic_set_device_mode(const struct sr_dev_inst* sdi, dev_mode value){
    g_assert(sdi);
    struct dev_context* devc = sdi->priv;
    devc->device_mode = value;
    return SR_OK;
}

SR_PRIV uint64_t dslogic_get_sample_rate(const struct sr_dev_inst* sdi){
    g_assert(sdi);
    struct dev_context* devc = sdi->priv;
    return devc->current_samplerate;
}

SR_PRIV int dslogic_set_samplerate(const struct sr_dev_inst* sdi, uint64_t samplerate){
    g_assert(sdi);
    struct dev_context* devc = sdi->priv;
    if(samplerate > SR_MHZ(400))
        return SR_ERR_ARG;
    devc->current_samplerate = samplerate;
    if(samplerate >= SR_MHZ(200))
        devc->sample_width = FALSE;
    else
        devc->sample_width = TRUE;
    sr_dbg("New samplerate: %d, Sample Width: %d", devc->current_samplerate,
           devc->sample_width + 1);
    return SR_OK;
}

SR_PRIV void dslogic_acquisition_stop(const struct sr_dev_inst* sdi){
    g_assert(sdi);
    struct dev_context *devc;
    devc = sdi->priv;
    devc->status = DSLOGIC_STOP;
    sr_info("%s: Stopping", __func__);
    dslogic_abort_acquisition(sdi);
}

SR_PRIV uint64_t dslogic_get_capture_ratio(const struct sr_dev_inst* sdi){
    g_assert(sdi);
    struct dev_context *devc = sdi->priv;
    return devc->capture_ratio;
}

SR_PRIV int dslogic_set_capture_ratio(const struct sr_dev_inst* sdi, uint64_t ratio){
    g_assert(sdi);
    struct dev_context *devc = sdi->priv;
    if(ratio > 100)
        return SR_ERR_ARG;
    devc->capture_ratio = ratio;
    return SR_OK;
}

SR_PRIV int dslogic_get_sample_count(const struct sr_dev_inst* sdi){
    g_assert(sdi);
    struct dev_context *devc = sdi->priv;
    return devc->sample_count;
}

SR_PRIV dslogic_status dslogic_get_device_status(const struct sr_dev_inst* sdi){
    g_assert(sdi);
    struct dev_context *devc = sdi->priv;
    return devc->status;
}

SR_PRIV void dslogic_clear_trigger_stages(const struct sr_dev_inst* sdi){
    g_assert(sdi);
    struct dev_context *devc = sdi->priv;
    int i;
    devc->trigger_settings.stage_count = 0;
}

SR_PRIV void dslogic_set_trigger_stage(const struct sr_dev_inst* sdi,
                                       int stage_count){
    g_assert(sdi);
    struct dev_context *devc = sdi->priv;
    devc->trigger_settings.stage_count = stage_count;
    if(stage_count < 0)
        devc->trigger_enabled = FALSE;
    else
        devc->trigger_enabled = TRUE;
    /* clear all past triggers: TODO: move own function? */
    int i;
    for(i = 0; i < NUM_TRIGGER_STAGES; i++){
        devc->trigger_settings.trigger0[NUM_TRIGGER_STAGES][i] = TRIGGER_FIRED;
        devc->trigger_settings.trigger1[NUM_TRIGGER_STAGES][i] = TRIGGER_FIRED;
    }
}

SR_PRIV void dslogic_set_trigger(const struct sr_dev_inst* sdi,
                                       int stage, int probe,  int trigger){
    struct dev_context *devc = sdi->priv;    
    devc->trigger_settings.trigger0[NUM_TRIGGER_STAGES][probe] = trigger;
    devc->trigger_settings.trigger1[NUM_TRIGGER_STAGES][probe] = TRIGGER_FIRED; //Dont care
}

SR_PRIV int dslogic_get_sample_wide(const struct sr_dev_inst* sdi){
    g_assert(sdi);
    struct dev_context *devc = sdi->priv;
    return devc->current_samplerate <= SR_MHZ(100) ? 2 : devc->sample_width ? 2 : 1;
}

SR_PRIV void dslogic_set_trigger_mask(const struct sr_dev_inst* sdi,
                                      int stage, int probe_bit){
    struct dev_context* devc = sdi->priv;
    devc->trigger_mask[stage] |= probe_bit;
}

SR_PRIV void dslogic_set_trigger_value(const struct sr_dev_inst* sdi,
                                       int stage, int probe_bit){
    struct dev_context* devc = sdi->priv;
    devc->trigger_value[stage] |= probe_bit;
}

SR_PRIV void dslogic_reset_empty_transfer_count(const struct sr_dev_inst* sdi){
    struct dev_context* devc = sdi->priv;
    devc->empty_transfer_count = 0;
}

SR_PRIV gboolean dslogic_increase_empty_sample_count(const struct sr_dev_inst* sdi){
    g_assert(sdi);
    struct dev_context* devc = sdi->priv;
    devc->empty_transfer_count++;
    gboolean ret = devc->empty_transfer_count > MAX_EMPTY_TRANSFERS;
    if (ret) {
        devc->status = DSLOGIC_ERROR;
    }
    return ret;
}

SR_PRIV gboolean dslogic_sample_complete(const struct sr_dev_inst* sdi){
    g_assert(sdi);
    struct dev_context* devc = sdi->priv;
    return devc->sample_limit &&
            (unsigned int)devc->sample_count >= devc->sample_limit;
}

SR_PRIV void dslogic_process_data(const struct sr_dev_inst* sdi, uint8_t* data, int data_size){
    g_assert(sdi);
    struct dev_context* devc = sdi->priv;
    int sample_width = dslogic_get_sample_wide(sdi);
    int cur_sample_count = data_size / sample_width;
    struct sr_datafeed_packet packet;
    struct sr_datafeed_logic logic;
    int i;
    int trigger_offset = 0;
    if (devc->trigger_settings.stage_count >= 0) {
        for (i = 0; i < cur_sample_count; i++) {
            const uint16_t cur_sample = devc->sample_width ?
                        *((const uint16_t*) data + i) :
                        *((const uint8_t*) data + i);
            if ((cur_sample & devc->trigger_mask[devc->trigger_settings.stage_count]) ==
                    devc->trigger_value[devc->trigger_settings.stage_count]) {
                // Match on this trigger stage.
                devc->trigger_buffer[devc->trigger_settings.stage_count] = cur_sample;
                devc->trigger_settings.stage_count++;
                if (devc->trigger_settings.stage_count == NUM_TRIGGER_STAGES ||
                        devc->trigger_mask[devc->trigger_settings.stage_count] == 0) {
                    // Match on all trigger stages, we're done.
                    trigger_offset = i + 1;
                    /*
                     * TODO: Send pre-trigger buffer to session bus.
                     * Tell the frontend we hit the trigger here.
                     */
                    packet.type = SR_DF_TRIGGER;
                    packet.payload = NULL;
                    sr_session_send(devc->cb_data, &packet);
                    /*
                     * Send the samples that triggered it,
                     * since we're skipping past them.
                     */
                    packet.type = SR_DF_LOGIC;
                    packet.payload = &logic;
                    logic.unitsize = sizeof (*devc->trigger_buffer);
                    logic.length = devc->trigger_settings.stage_count * logic.unitsize;
                    logic.data = devc->trigger_buffer;
                    sr_session_send(devc->cb_data, &packet);
                    devc->trigger_settings.stage_count = TRIGGER_FIRED;
                    break;
                }
            } else if (devc->trigger_settings.stage_count > 0) {
                /*
                 * We had a match before, but not in the next sample. However, we may
                 * have a match on this stage in the next bit -- trigger on 0001 will
                 * fail on seeing 00001, so we need to go back to stage 0 -- but at
                 * the next sample from the one that matched originally, which the
                 * counter increment at the end of the loop takes care of.
                 */
                i -= devc->trigger_settings.stage_count;
                if (i < -1)
                    i = -1; // Oops, went back past this buffer.
                // Reset trigger stage.
                devc->trigger_settings.stage_count = 0;
            }
        }
    }
    if (devc->trigger_settings.stage_count == TRIGGER_FIRED) {
        /* Send the incoming transfer to the session bus. */
        int trigger_offset_bytes = trigger_offset * sample_width;
        packet.type = SR_DF_LOGIC;
        packet.payload = &logic;
        logic.length = data_size - trigger_offset_bytes;
        logic.unitsize = sample_width;
        logic.data = data + trigger_offset_bytes;
        if ((devc->sample_limit && devc->sample_count < (int64_t)devc->sample_limit)) {
            const uint64_t remain_length = (devc->sample_limit - devc->sample_count) * sample_width;
            logic.length = min(logic.length, remain_length);
            // send data to session bus
            sr_session_send(devc->cb_data, &packet);
        }

        devc->sample_count += cur_sample_count;
        if ( devc->sample_limit &&
                (unsigned int)devc->sample_count >= devc->sample_limit) {
            devc->status = DSLOGIC_STOP;
            return;
        }
    } else {
        // TODO: Buffer pre-trigger data in capture
        //ratio-sized buffer.
    }
}

SR_PRIV int dslogic_start_acquisition(const struct sr_dev_inst* sdi,
                                     const struct sr_dev_driver * di,
                                     sr_receive_data_callback receive_data,
                                      void* cb_data){
    g_assert(sdi);
    g_assert(di);
    struct dev_context* devc = sdi->priv;
    struct drv_context* drvc = di->priv;
    struct sr_usb_dev_inst* usb = sdi->conn;
    struct ds_trigger_pos* trigger_pos;
    struct libusb_transfer *transfer;
    int ret;
    /* Stop Previous GPIF acquisition */
    devc->cb_data = cb_data;
    devc->sample_count = 0;
    devc->empty_transfer_count = 0;
    devc->status = DSLOGIC_INIT;
    devc->num_transfers = 0;
    devc->submitted_transfers = 0;
    if ((ret = command_stop_acquisition(usb->devhdl)) != SR_OK) {
        sr_err("Stop DSLogic acquisition failed!");
        dslogic_abort_acquisition(sdi);
        return ret;
    } else {
        sr_info("Previous DSLogic acquisition stopped!");
    }
    /* Setting FPGA before acquisition start*/
    if ((ret = command_fpga_setting(usb->devhdl,
                    dslogic_get_fpga_setting_size() / sizeof (uint16_t))) != SR_OK) {
        sr_err("Send FPGA setting command failed!");
    } else {
        if ((ret = set_fpga_setting(sdi)) != SR_OK) {
            sr_err("Configure FPGA failed!");
            dslogic_abort_acquisition(sdi);
            return ret;
        }
    }
    if(ret!=SR_OK) return ret;
    if ((ret = command_start_acquisition(usb->devhdl,devc->current_samplerate,
                                         devc->sample_width, TRUE)) != SR_OK) {
        dslogic_abort_acquisition(sdi);
        return ret;
    }
    devc->transfers = g_try_malloc0(sizeof (*devc->transfers));
    if (!devc->transfers) {
        sr_err("USB trigger_pos transfer malloc failed.");
        return SR_ERR_MALLOC;
    }
    //poll trigger status transfer
    if (!(trigger_pos = g_try_malloc0(sizeof (struct ds_trigger_pos)))) {
        sr_err("USB trigger_pos buffer malloc failed.");
        return SR_ERR_MALLOC;
    }
    devc->num_transfers = 1;
    transfer = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(transfer, usb->devhdl,
                              6 | LIBUSB_ENDPOINT_IN,
                              (unsigned char*)trigger_pos,
                              sizeof (struct ds_trigger_pos),
                              receive_trigger_pos, (void*)sdi, 0);
    if ((ret = libusb_submit_transfer(transfer)) != 0) {
        sr_err("Failed to submit trigger_pos transfer: %s.",
               libusb_error_name(ret));
        libusb_free_transfer(transfer);
        g_free(trigger_pos);
        dslogic_abort_acquisition(sdi);
        return SR_ERR;
    }
    devc->ctx = drvc->sr_ctx;
    usb_source_add(sdi->session, drvc->sr_ctx, get_timeout(devc), receive_data, (void*) sdi);
    devc->transfers[0] = transfer;
    devc->submitted_transfers++;
    devc->status = DSLOGIC_START;
    // Send header packet to the session bus.
    //std_session_send_df_header(cb_data, LOG_PREFIX);
    std_session_send_df_header(sdi, LOG_PREFIX);
    return SR_OK;
}

static void dslogic_set_error_status(const struct sr_dev_inst* sdi){
      struct dev_context* devc = sdi->priv;
      devc->status = DSLOGIC_ERROR;
}

static void dslogic_resubmit_transfer(struct libusb_transfer *transfer) {
    int ret;

    if ((ret = libusb_submit_transfer(transfer)) == LIBUSB_SUCCESS)
        return;

    free_transfer(transfer);
    /* TODO: Stop session? */

    sr_err("%s: %s", __func__, libusb_error_name(ret));
}

static void dslogic_receive_transfer(struct libusb_transfer *transfer) {
    gboolean packet_has_error = FALSE;
    struct sr_dev_inst* sdi = transfer->user_data;
    uint8_t *cur_buf;
    GTimeVal cur_time;
    g_get_current_time(&cur_time);

    sr_dbg("receive_transfer: current time %d sec %d usec",
           cur_time.tv_sec, cur_time.tv_usec);
    /*
     * If acquisition has already ended, just free any queued up
     * transfer that come in.
     */
    if (dslogic_get_sample_count(sdi) == -1) {
        sr_dbg("receive_data: already ended");
        free_transfer(transfer);
        return;
    }

    sr_dbg("receive_transfer: status %d; timeout %d; received %d bytes.",
           transfer->status, transfer->timeout, transfer->actual_length);

    /* Save incoming transfer before reusing the transfer struct. */
    cur_buf = transfer->buffer;
    switch (transfer->status) {
        case LIBUSB_TRANSFER_NO_DEVICE:
            //abort_acquisition(devc);
            free_transfer(transfer);
            dslogic_set_error_status(sdi);
            return;
        case LIBUSB_TRANSFER_COMPLETED:
        case LIBUSB_TRANSFER_TIMED_OUT: /* We may have received some data though. */
            break;
        default:
            packet_has_error = TRUE;
            break;
    }

    if (transfer->actual_length == 0 || packet_has_error) {
        if(dslogic_increase_empty_sample_count(sdi)){
            /*
             * The FX2 gave up. End the acquisition, the frontend
             * will work out that the samplecount is short.
             */
            sr_dbg("receive_transfer: error");
            free_transfer(transfer);
        } else {
            sr_dbg("receive_transfer: resubmit");
            dslogic_resubmit_transfer(transfer);
        }
        return;
    } else {
        sr_dbg("receive_transfer: empty_transfer_count=0");
        dslogic_reset_empty_transfer_count(sdi);
    }
    dslogic_process_data(sdi, cur_buf, transfer->actual_length);
    if(dslogic_sample_complete(sdi))
        free_transfer(transfer);
    else
        dslogic_resubmit_transfer(transfer);
}
