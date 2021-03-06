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

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <assert.h>
#include <glib/gstdio.h>
#include <libusb.h>
#include <inttypes.h>
#include "libsigrok.h"
#include "libsigrok-internal.h"
#include "dslogic.h"
#include "protocol.h"

static const struct {
    voltage_range range;
	gdouble low;
	gdouble high;
} volt_thresholds[] = {
	{ VOLTAGE_RANGE_18_33_V, 0.7, 1.4},
	{ VOLTAGE_RANGE_5_V, 1.4, 3.6},
};

static const int32_t scanopts[] = {
	SR_CONF_CONN,
};

static const uint32_t drvopts[] = {
    SR_CONF_LOGIC_ANALYZER,
    SR_CONF_OSCILLOSCOPE,
};

/* device options */
static const uint32_t devopts[] = {
	SR_CONF_CONTINUOUS ,
    SR_CONF_LIMIT_SAMPLES | SR_CONF_SET | SR_CONF_GET | SR_CONF_LIST,
    SR_CONF_VOLTAGE_THRESHOLD | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
    SR_CONF_PATTERN_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
    SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
    SR_CONF_CAPTURE_RATIO | SR_CONF_GET | SR_CONF_SET,
    SR_CONF_EXTERNAL_CLOCK | SR_CONF_SET | SR_CONF_GET,
    SR_CONF_CONN | SR_CONF_GET,
    SR_CONF_TRIGGER_MATCH | SR_CONF_LIST,
    //SR_CONF_CLOCK_EDGE | SR_CONF_SET | SR_CONF_GET | SR_CONF_LIST,
    //SR_CONF_TRIGGER_SOURCE | SR_CONF_LIST | SR_CONF_SET | SR_CONF_GET,
    /*SR_CONF_FILTER | SR_CONF_SET | SR_CONF_GET | SR_CONF_LIST,
    */// SR_CONF_DEVICE_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
    //SR_CONF_TEST_MODE | SR_CONF_SET | SR_CONF_GET | SR_CONF_LIST,
};

/* Names assigned to available trigger sources.  Indices must match
 * trigger_source enum values.
 */
static const char *const trigger_source_names[] = { "CH", "TRG" };

#define STR_PATTERN_NONE     "None"
#define STR_PATTERN_EXTERNAL "External"
#define STR_PATTERN_INTERNAL "Internal"

/* Supported methods of test pattern outputs */
enum {
    /**
     * Capture pins 31:16 (unbuffered wing) output a test pattern
     * that can captured on pins 0:15.
     */
    PATTERN_EXTERNAL,
    /** Route test pattern internally to capture buffer. */
    PATTERN_INTERNAL,
};

static const char *patterns[] = {
    STR_PATTERN_NONE,
    STR_PATTERN_EXTERNAL,
    STR_PATTERN_INTERNAL,
};

/* Helper for mapping a string-typed configuration value to an index
 * within a table of possible values.
 */
static int lookup_index(GVariant *value, const char *const *table, int len)
{
	const char *entry;
	int i;

	entry = g_variant_get_string(value, NULL);
	if (!entry)
		return -1;

	// Linear search is fine for very small tables. 
	for (i = 0; i < len; ++i) {
		if (strcmp(entry, table[i]) == 0)
			return i;
	}
	return -1;
}

static const int32_t trigger_matches[] = {
	SR_TRIGGER_ZERO,
	SR_TRIGGER_ONE,
	SR_TRIGGER_RISING,
	SR_TRIGGER_FALLING,
	SR_TRIGGER_EDGE,
};

/* Names assigned to available trigger slope choices.  Indices must
 * match the signal_edge enum values.
 */
static const char *const signal_edge_names[] = { "rising", "falling" };

static const char *channel_names[] = {
	"0", "1", "2", "3", "4", "5", "6", "7",
	"8", "9", "10", "11", "12", "13", "14", "15",
	NULL,
};

static const int cons_buffer_size = 1024 * 16;
SR_PRIV struct sr_dev_driver dslogic_driver_info;

static const uint64_t samplerates[] = {
    SR_KHZ(10),	SR_KHZ(20),	SR_KHZ(50),	SR_KHZ(100),
    SR_KHZ(200), SR_KHZ(500), SR_MHZ(1), SR_MHZ(2),
    SR_MHZ(5),	SR_MHZ(10), SR_MHZ(20),	SR_MHZ(25),
    SR_MHZ(50),	SR_MHZ(100), SR_MHZ(200), SR_MHZ(400),
};

static const uint64_t samplecounts[] = {
    SR_KB(1), SR_KB(2),	SR_KB(8), SR_KB(8),
    SR_KB(16), SR_KB(32), SR_KB(64), SR_KB(128),
    SR_KB(256), SR_KB(512),	SR_MB(1), SR_MB(2),
    SR_MB(4), SR_MB(8),	SR_MB(16),
};

//TODO: Remove
const char* config_path = "/home/diego/media/DSLogic/dslogic-gui/res/";

static int receive_data(int fd, int revents, void *cb_data) {
    //static int i = 0;
    struct timeval tv;
    struct drv_context *drvc;
    const struct sr_dev_inst *sdi;
    (void) fd;
    (void) revents;
    sdi = cb_data;
    drvc = dslogic_driver_info.priv;
    if (dslogic_get_sample_count(sdi) != -1 &&(
            dslogic_get_device_status(sdi) == DSLOGIC_STOP ||
            dslogic_get_device_status(sdi) == DSLOGIC_ERROR)) {
        sr_info("%s: Stopping", __func__);
        dslogic_abort_acquisition(sdi);
    }
    tv.tv_sec = tv.tv_usec = 0;
    libusb_handle_events_timeout(drvc->sr_ctx->libusb_ctx, &tv);
    return TRUE;
}

static int configure_probes(const struct sr_dev_inst *sdi) {
	printf("configure_probes\n");
	struct sr_channel *probe;
    struct sr_trigger *trigger;
    struct sr_trigger_stage *stage;
    struct sr_trigger_match *match;

    GSList *l,*m;
    dslogic_clear_trigger_stages(sdi);
    /* if no triggers, return happily */
	for (l = sdi->channels; l; l = l->next) {
		probe = (struct sr_channel *) l->data;
		if (probe->enabled == FALSE)
			continue;
    }
    if (!(trigger = sr_session_trigger_get(sdi->session))){
        dslogic_set_trigger_stage(sdi, TRIGGER_FIRED);
        return SR_OK;
    }

    int num_stages = g_slist_length(trigger->stages);
    if(num_stages > NUM_TRIGGER_STAGES){
        sr_err("This device only supports %d trigger stages.",
               NUM_TRIGGER_STAGES);
        return SR_ERR;
    }
    dslogic_set_trigger_stage(sdi, num_stages);
    for(l = trigger->stages; l; l = l->next){
        stage = l->data;
        for(m=stage->matches;m;m = m->next){
            match = m->data;
            if(!match->channel->enabled)
                continue;
            dslogic_set_trigger_mask(sdi, stage->stage,
                                     1 << match->channel->index);
            switch(match->match){
            case SR_TRIGGER_ONE:
                if(match->match == SR_TRIGGER_ONE)
                    dslogic_set_trigger_value(sdi, stage->stage,
                                              1 << match->channel->index);
                break;
            case SR_TRIGGER_FALLING:
                break;
            case SR_TRIGGER_RISING:
                break;
            case SR_TRIGGER_EDGE:
                break;
            case SR_TRIGGER_ZERO:
                break;
            }
        }
	}
	return SR_OK;
}

static int dev_clear(void) {
    return std_dev_clear(&dslogic_driver_info, NULL);
}

static int init(struct sr_context *sr_ctx) {
    return std_init(sr_ctx, &dslogic_driver_info, LOG_PREFIX);
}

static int set_probes(struct sr_dev_inst *sdi, int num_probes) {
	int j;
	struct sr_channel *probe;
	for (j = 0; j < num_probes; j++) {
		probe = sr_channel_new(j, SR_CHANNEL_LOGIC, TRUE, channel_names[j]);
		if (!probe) return SR_ERR;
		sdi->channels = g_slist_append(sdi->channels, probe);
	}
    sr_dbg("Setting Probes: %d ", num_probes);
	return SR_OK;
}

static int adjust_probes(const struct sr_dev_inst *sdi, int num_probes) {
	GSList *l;
	struct sr_channel *probe;
	GSList *p;
	(void)p;
    assert(num_probes > 0);
    for(l = sdi->channels; l; l=l->next){
        probe = (struct sr_channel*)l->data;
        if(num_probes-- > 0)
            probe->enabled = TRUE;
        else
            probe->enabled = FALSE;
    }
    sr_dbg("Adjusted Probes: %d probes found", 16 + num_probes);
	return SR_OK;
}

static GSList *scan(GSList *options) {
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_dev_inst *sdi;
	struct sr_usb_dev_inst *usb;
    struct sr_config *src;
    const dslogic_profile *prof;
	GSList *l, *devices, *conn_devices;
	struct libusb_device_descriptor des;
	libusb_device **devlist;
	int devcnt, num_logic_probes, ret, i, j;
	const char *conn;
	char connection_id[64];
    drvc = dslogic_driver_info.priv;

    conn = NULL;
    for (l = options; l; l = l->next) {
		src = l->data;
		switch (src->key) {
			case SR_CONF_CONN:
				conn = g_variant_get_string(src->data, NULL);
				break;
		}
    }
	if (conn)
		conn_devices = sr_usb_find(drvc->sr_ctx->libusb_ctx, conn);
	else
		conn_devices = NULL;
	/* Find all DSLogic compatible devices and upload firmware to them. */
	devices = NULL;
	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	for (i = 0; devlist[i]; i++) {
		if (conn) {
			usb = NULL;
			for (l = conn_devices; l; l = l->next) {
				usb = l->data;
				if (usb->bus == libusb_get_bus_number(devlist[i])
				    && usb->address == libusb_get_device_address(devlist[i]))
					break;
			}
			if (!l)
				/* This device matched none of the ones that
				 * matched the conn specification. */
				continue;
		}

		if ((ret = libusb_get_device_descriptor(devlist[i], &des)) != 0) {
			sr_warn("Failed to get device descriptor: %s.",
			        libusb_error_name(ret));
			continue;
		}
		usb_get_port_path(devlist[i], connection_id, sizeof (connection_id));
		prof = NULL;
		for (j = 0; supported_fx2[j].vid; j++) {
			if (des.idVendor == supported_fx2[j].vid &&
			    des.idProduct == supported_fx2[j].pid) {
				prof = &supported_fx2[j];
			}
		}
		/* Skip if the device was not found. */
		if (!prof)
			continue;
		devcnt = g_slist_length(drvc->instances);
		sdi = g_malloc0(sizeof (struct sr_dev_inst));
		sdi->status = SR_ST_INITIALIZING;
		sdi->vendor = g_strdup("Dreamsourcelab");
		sdi->model = g_strdup("DSLogic");
		// TODO: Read this strings from device???
		sdi->version = g_strdup("1.0.0");
        sdi->driver = &dslogic_driver_info;
		sdi->connection_id = g_strdup(connection_id);
		if (!sdi)
			return NULL;
		/* Fill in probelist according to this device's profile. */
		num_logic_probes = prof->dev_caps & DEV_CAPS_16BIT ? 16 : 8;
		if (set_probes(sdi, num_logic_probes) != SR_OK)
			return NULL;
        devc = dslogic_dev_new();
        dslogic_set_profile(devc, prof);
		sdi->priv = devc;
		drvc->instances = g_slist_append(drvc->instances, sdi);
		devices = g_slist_append(devices, sdi);
        if (dslogic_check_conf_profile(devlist[i])) {
			/* Already has the firmware, so fix the new address. */
			sr_dbg("Found an DSLogic device.");
			sdi->status = SR_ST_INACTIVE;
			sdi->inst_type = SR_INST_USB;
			sdi->conn = sr_usb_dev_inst_new(libusb_get_bus_number(devlist[i]),
			                                libusb_get_device_address(devlist[i]), NULL);
		} else {
			char filename[256];
			sprintf(filename, "%s%s", config_path, prof->firmware);
			const char *firmware = filename;
			if (ezusb_upload_firmware(devlist[i], USB_CONFIGURATION,
			                          firmware) == SR_OK)
				/* Store when this device's FW was updated. */
                dslogic_set_firmware_updated(sdi);
			else
				sr_err("Firmware upload failed for "
				       "device %d.", devcnt);
			sdi->inst_type = SR_INST_USB;
			sdi->conn = sr_usb_dev_inst_new(libusb_get_bus_number(devlist[i]),
			                                0xff, NULL);
		}
	}
	libusb_free_device_list(devlist, 1);
	g_slist_free_full(conn_devices, (GDestroyNotify) sr_usb_dev_inst_free);
	return devices;
}

static GSList *dev_list(void) {
    return ((struct drv_context *) (dslogic_driver_info.priv))->instances;
}

static int dev_open(struct sr_dev_inst *sdi) {
	struct sr_usb_dev_inst *usb;
	usb = sdi->conn;
    int ret = dslogic_dev_open(sdi, &dslogic_driver_info);
	if (ret != SR_OK) {
		sr_err("Unable to open device.");
		return SR_ERR;
	}
	ret = libusb_claim_interface(usb->devhdl, USB_INTERFACE);
	if (ret != 0) {
		switch (ret) {
			case LIBUSB_ERROR_BUSY:
				sr_err("Unable to claim USB interface. Another "
				       "program or driver has already claimed it.");
				break;
			case LIBUSB_ERROR_NO_DEVICE:
				sr_err("Device has been disconnected.");
				break;
			default:
				sr_err("Unable to claim interface: %s.",
				       libusb_error_name(ret));
				break;
		}
		return SR_ERR;
	}
    ret = dslogic_program_fpga(sdi);
	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi) {
	struct sr_usb_dev_inst *usb;
	usb = sdi->conn;
	if (usb->devhdl == NULL)
		return SR_ERR;
	sr_info("DSLogic: Closing device %s on %d.%d interface %d.",
	        sdi->connection_id, usb->bus, usb->address, USB_INTERFACE);
	libusb_release_interface(usb->devhdl, USB_INTERFACE);
	libusb_close(usb->devhdl);
	usb->devhdl = NULL;
	sdi->status = SR_ST_INACTIVE;
	return SR_OK;
}

static int cleanup(void) {
	int ret;
	struct drv_context *drvc;
    if (!(drvc = dslogic_driver_info.priv))
		return SR_OK;
	ret = dev_clear();
	g_free(drvc);
    dslogic_driver_info.priv = NULL;
	return ret;
}

static int config_get(uint32_t id, GVariant **data, const struct sr_dev_inst *sdi,
                      const struct sr_channel_group *cg){
	struct sr_usb_dev_inst *usb;
	GVariant * range[2];
	char str[128];
	unsigned int i;
    dev_mode mode;
	(void)cg;
	switch (id) {
        case SR_CONF_LIMIT_SAMPLES:
            *data = g_variant_new_uint64(dslogic_get_sample_limit(sdi));
            break;
        case SR_CONF_VOLTAGE_THRESHOLD:
            for (i = 0; i < ARRAY_SIZE(volt_thresholds); i++) {
                if(volt_thresholds[i].range != dslogic_get_voltage_threshold(sdi))
                    continue;
                range[0] = g_variant_new_double(volt_thresholds[i].low);
                range[1] = g_variant_new_double(volt_thresholds[i].high);
                *data = g_variant_new_tuple(range, 2);
                break;
            }
            break;
        case SR_CONF_PATTERN_MODE:
            mode = dslogic_get_device_mode(sdi);
            if(mode == TEST_EXTERNAL)
                *data = g_variant_new_string(STR_PATTERN_EXTERNAL);
            else if(mode == TEST_INTERNAL)
                *data = g_variant_new_string(STR_PATTERN_INTERNAL);
            else
                *data = g_variant_new_string(STR_PATTERN_NONE);
            break;
        break;
        case SR_CONF_CAPTURE_RATIO:
            *data = g_variant_new_uint64(dslogic_get_capture_ratio(sdi));
        break;
        case SR_CONF_CONN:
			if (!sdi || !sdi->conn)
				return SR_ERR_ARG;
			usb = sdi->conn;
			if (usb->address == 255)
				/* Device still needs to re-enumerate after firmware
				 * upload, so we don't know its (future) address. */
				return SR_ERR;
			snprintf(str, 128, "%d.%d", usb->bus, usb->address);
			*data = g_variant_new_string(str);
			break;
		case SR_CONF_SAMPLERATE:
            *data = g_variant_new_uint64(dslogic_get_sample_rate(sdi));
            break;
        case SR_CONF_EXTERNAL_CLOCK:
            *data = g_variant_new_boolean(dslogic_get_clock_source(sdi) == CLOCK_EXT_CLK);
			break;
		case SR_CONF_CLOCK_EDGE:
            i = dslogic_get_clock_edge(sdi);
            if (i >= G_N_ELEMENTS(signal_edge_names))
				return SR_ERR_BUG;
            *data = g_variant_new_string(signal_edge_names[i]);
			break;
        case SR_CONF_FILTER:
			if (!sdi)
				return SR_ERR;
//			devc = sdi->priv;
            //*data = g_variant_new_boolean (devc->filter);
			break;
		case SR_CONF_TIMEBASE:
			if (!sdi)
				return SR_ERR;
//			devc = sdi->priv;
            //*data = g_variant_new_uint64(devc->timebase);
			break;
		case SR_CONF_TRIGGER_SLOPE:
			if (!sdi)
				return SR_ERR;
//			devc = sdi->priv;
            //*data = g_variant_new_byte(devc->trigger_slope);
			break;
		case SR_CONF_TRIGGER_SOURCE:
			if (!sdi)
				return SR_ERR;
//			devc = sdi->priv;
            //idx = devc->trigger_source;
            if (i >= G_N_ELEMENTS(trigger_source_names))
                return SR_ERR_BUG;
            *data = g_variant_new_string(trigger_source_names[i]);
            break;
		default:
			return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_set(uint32_t id, GVariant *data, const struct sr_dev_inst *sdi,
                      const struct sr_channel_group *cg) {
	(void)cg;
    int ret = SR_OK;
	unsigned int i;
	gdouble low, high;
	if (sdi->status != SR_ST_ACTIVE)
		return SR_ERR;

    int idx;
    const char* stropt;
    switch(id){
    case SR_CONF_TRIGGER_SOURCE:
        idx = lookup_index(data, trigger_source_names,
                   G_N_ELEMENTS(trigger_source_names));
        if (idx < 0)
            return SR_ERR_ARG;
        //devc->trigger_source = idx;
        break;
    case SR_CONF_PATTERN_MODE:
        stropt = g_variant_get_string(data, NULL);
        ret = SR_OK;
        if (!strcmp(stropt, STR_PATTERN_NONE)) {
            sr_info("Disabling test modes.");
            dslogic_set_device_mode(sdi,NORMAL_MODE);
        }else if (!strcmp(stropt, STR_PATTERN_INTERNAL)) {
            sr_info("Enabling internal test mode.");
            dslogic_set_device_mode(sdi,TEST_INTERNAL);
        } else if (!strcmp(stropt, STR_PATTERN_EXTERNAL)) {
            sr_info("Enabling external test mode.");
            dslogic_set_device_mode(sdi,TEST_EXTERNAL);
        } else {
            ret = SR_ERR;
        }
        break;
    case SR_CONF_CAPTURE_RATIO:
        ret = dslogic_set_capture_ratio(sdi, g_variant_get_uint64(data));
        break;
    case SR_CONF_SAMPLERATE:
       ret = dslogic_set_samplerate(sdi,g_variant_get_uint64(data));
       if (dslogic_get_sample_rate(sdi) >= SR_MHZ(200)) {
           adjust_probes(sdi, SR_MHZ(1600) / dslogic_get_sample_rate(sdi));
       } else {
           adjust_probes(sdi, 16);
       }
       break;
    case SR_CONF_FILTER:
        break;
    case SR_CONF_CLOCK_EDGE:
        idx = lookup_index(data, signal_edge_names,
                   G_N_ELEMENTS(signal_edge_names));
        if (idx < 0)
            return SR_ERR_ARG;
        dslogic_set_clock_edge(sdi, idx);
        break;
    case SR_CONF_EXTERNAL_CLOCK:
        dslogic_set_clock_source(sdi,(g_variant_get_boolean(data))? CLOCK_EXT_CLK : CLOCK_INTERNAL);
        break;
    case SR_CONF_LIMIT_SAMPLES:
        ret = dslogic_set_sample_limit(sdi, g_variant_get_uint64(data));
        break;
    case SR_CONF_VOLTAGE_THRESHOLD:
        g_variant_get(data, "(dd)", &low, &high);
        ret = SR_ERR_ARG;
        for (i = 0; i < ARRAY_SIZE(volt_thresholds); i++) {
            if (fabs(volt_thresholds[i].low - low) < 0.1 &&
                fabs(volt_thresholds[i].high - high) < 0.1) {
                ret = dslogic_set_voltage_threshold(sdi, volt_thresholds[i].range);
                break;
            }
        }
        break;
    default:
        ret = SR_ERR_NA;
    }
    return ret;
}

static int config_list(uint32_t key, GVariant **data, const struct sr_dev_inst *sdi,
                       const struct sr_channel_group *cg) {
	GVariant *gvar, *range[2];
	GVariantBuilder gvb;
	unsigned int i;
	(void) sdi;
	(void) cg;
	switch (key) {
        case SR_CONF_LIMIT_SAMPLES:
            range[0] = g_variant_new_uint64(samplecounts[0]);
            range[1] = g_variant_new_uint64(samplecounts[sizeof(samplecounts)/sizeof(uint64_t)-1]);
            *data = g_variant_new_tuple(range, 2);
            break;
        case SR_CONF_PATTERN_MODE:
            *data = g_variant_new_strv(patterns, ARRAY_SIZE(patterns));
            break;
		case SR_CONF_SCAN_OPTIONS:
			*data = g_variant_new_fixed_array(G_VARIANT_TYPE_INT32,
			                                  scanopts, ARRAY_SIZE(scanopts), sizeof (int32_t));
			break;
		case SR_CONF_DEVICE_OPTIONS:
			if (!sdi)
				*data = g_variant_new_fixed_array(G_VARIANT_TYPE_UINT32,
				                                  drvopts, ARRAY_SIZE(drvopts), sizeof (uint32_t));
			else
				*data = g_variant_new_fixed_array(G_VARIANT_TYPE_UINT32,
				                                  devopts, ARRAY_SIZE(devopts), sizeof (uint32_t));
			break;
		case SR_CONF_SAMPLERATE:
            g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
            gvar = g_variant_new_fixed_array(G_VARIANT_TYPE("t"), samplerates,
                                             ARRAY_SIZE(samplerates), sizeof (uint64_t));
            g_variant_builder_add(&gvb, "{sv}", "samplerates", gvar);
			*data = g_variant_builder_end(&gvb);
			break;
		case SR_CONF_TRIGGER_MATCH:
            *data = g_variant_new_fixed_array(G_VARIANT_TYPE_INT32,  trigger_matches,
                                              ARRAY_SIZE(trigger_matches), sizeof (int32_t));
			break;
		case SR_CONF_VOLTAGE_THRESHOLD:
			g_variant_builder_init(&gvb, G_VARIANT_TYPE_ARRAY);
			for (i = 0; i < ARRAY_SIZE(volt_thresholds); i++) {
				range[0] = g_variant_new_double(volt_thresholds[i].low);
				range[1] = g_variant_new_double(volt_thresholds[i].high);
				gvar = g_variant_new_tuple(range, 2);
				g_variant_builder_add_value(&gvb, gvar);
			}
			*data = g_variant_builder_end(&gvb);
			break;
        case SR_CONF_FILTER:
            g_variant_builder_init(&gvb, G_VARIANT_TYPE_ARRAY);
            range[0] = g_variant_new_boolean(TRUE);
            range[1] = g_variant_new_boolean(FALSE);
            g_variant_builder_add_value(&gvb, range[0]);
            g_variant_builder_add_value(&gvb, range[1]);
            *data = g_variant_builder_end(&gvb);
            break;
		case SR_CONF_TRIGGER_SLOPE:
		case SR_CONF_CLOCK_EDGE:
			*data = g_variant_new_strv(signal_edge_names,
					   G_N_ELEMENTS(signal_edge_names));
		break;
        case SR_CONF_TRIGGER_SOURCE:
            *data = g_variant_new_strv(trigger_source_names,
                       G_N_ELEMENTS(trigger_source_names));
        break;
		default:
			return SR_ERR_NA;
	}
	return SR_OK;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi, void *cb_data) {
	if (sdi->status != SR_ST_ACTIVE)
		return SR_ERR_DEV_CLOSED;
    if (configure_probes(sdi) != SR_OK) {
		sr_err("Failed to configure probes.");
		return SR_ERR;
	}
    int ret = dslogic_start_acquisition(sdi, &dslogic_driver_info, receive_data,
                                        cb_data);
    return ret;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi, void *cb_data) {
	(void) cb_data;
    dslogic_acquisition_stop(sdi);
	return SR_OK;
}

SR_PRIV struct sr_dev_driver dslogic_driver_info = {
	.name = "dslogic",
    .longname = "Dreamsourcelab DSLogic",
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

