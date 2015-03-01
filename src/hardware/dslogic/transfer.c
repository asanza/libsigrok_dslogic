#include "transfer.h"
#include <libusb.h>
#include <libsigrok.h>
#include "libsigrok-internal.h"
#include "devdefs.h"

#ifndef _WIN32
#undef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif

SR_PRIV void finish_acquisition(struct dev_context *devc) {
	struct sr_datafeed_packet packet;
	//int i, ret;
	//struct sr_usb_dev_inst *usb;

	sr_err("finish acquisition: send SR_DF_END packet");
	/* Terminate session. */
	packet.type = SR_DF_END;
    //sr_session_send(devc->cb_data, &packet);

	sr_err("finish acquisition: remove fds from polling");
	/* Remove fds from polling. */
	// for (i = 0; devc->usbfd[i] != -1; i++)
	//    sr_source_remove(devc->usbfd[i]);
	//g_free(devc->usbfd);

/*	if (devc->num_transfers != 0) {
		devc->num_transfers = 0;
		g_free(devc->transfers);
    }*/
}

SR_PRIV void free_transfer(struct libusb_transfer *transfer) {
	struct dev_context *devc;
	unsigned int i;

	devc = transfer->user_data;

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);

/*	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			break;
		}
	}

	devc->submitted_transfers--;
	if (devc->submitted_transfers == 0 && devc->status != DSLOGIC_TRIGGERED)
        finish_acquisition(devc);*/
}

SR_PRIV void dslogic_receive_transfer(struct libusb_transfer *transfer) {
	gboolean packet_has_error = FALSE;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;
	/*
	 struct sr_datafeed_dso dso;
	 */
	//struct sr_datafeed_analog analog;
	struct dev_context *devc;
	int trigger_offset;
	int i, sample_width, cur_sample_count;
	int trigger_offset_bytes;
	uint8_t *cur_buf;
	GTimeVal cur_time;

	//g_get_current_time(&cur_time);
	sr_info("receive_transfer: current time %d sec %d usec", cur_time.tv_sec, cur_time.tv_usec);


	devc = transfer->user_data;

	/*
	 * If acquisition has already ended, just free any queued up
	 * transfer that come in.
	 */
    /*if (devc->sample_count == -1) {
		free_transfer(transfer);
		return;
    }*/

	sr_info("receive_transfer(): status %d; timeout %d; received %d bytes.",
	        transfer->status, transfer->timeout, transfer->actual_length);

	/* Save incoming transfer before reusing the transfer struct. */
	cur_buf = transfer->buffer;
/*	sample_width = devc->current_samplerate <= SR_MHZ(100) ? 2 :
		devc->sample_wide ? 2 : 1;
		cur_sample_count = transfer->actual_length / sample_width;

		switch (transfer->status) {
			case LIBUSB_TRANSFER_NO_DEVICE:
				//abort_acquisition(devc);
				free_transfer(transfer);
				devc->status = DSLOGIC_ERROR;
				return;
			case LIBUSB_TRANSFER_COMPLETED:
				case LIBUSB_TRANSFER_TIMED_OUT: /* We may have received some data though. */
/*					break;
			default:
				packet_has_error = TRUE;
				break;
		}

		if (transfer->actual_length == 0 || packet_has_error) {
			devc->empty_transfer_count++;
			if (devc->empty_transfer_count > MAX_EMPTY_TRANSFERS) {
				/*
				 * The FX2 gave up. End the acquisition, the frontend
				 * will work out that the samplecount is short.
				 */
				//abort_acquisition(devc);
/*				free_transfer(transfer);
				devc->status = DSLOGIC_ERROR;
			} else {
				resubmit_transfer(transfer);
			}
			return;
		} else {
			devc->empty_transfer_count = 0;
		}
		//trigger_offset = 0;
		if (devc->trigger_stage >= 0) {
			for (i = 0; i < cur_sample_count; i++) {
				const uint16_t cur_sample = devc->sample_wide ?
					*((const uint16_t*) cur_buf + i) :
					*((const uint8_t*) cur_buf + i);
					if ((cur_sample & devc->trigger_mask[devc->trigger_stage]) ==
					    devc->trigger_value[devc->trigger_stage]) {
						/* Match on this trigger stage. */
/*						devc->trigger_buffer[devc->trigger_stage] = cur_sample;
						devc->trigger_stage++;
						if (devc->trigger_stage == NUM_TRIGGER_STAGES ||
						    devc->trigger_mask[devc->trigger_stage] == 0) {
							/* Match on all trigger stages, we're done. */
							//trigger_offset = i + 1;
							/*
							 * TODO: Send pre-trigger buffer to session bus.
							 * Tell the frontend we hit the trigger here.
							 */
/*							packet.type = SR_DF_TRIGGER;
							packet.payload = NULL;
							sr_session_send(devc->cb_data, &packet);
							/*
							 * Send the samples that triggered it,
							 * since we're skipping past them.
							 */
/*							packet.type = SR_DF_LOGIC;
							packet.payload = &logic;
							logic.unitsize = sizeof (*devc->trigger_buffer);
							logic.length = devc->trigger_stage * logic.unitsize;
							logic.data = devc->trigger_buffer;
							sr_session_send(devc->cb_data, &packet);
							devc->trigger_stage = TRIGGER_FIRED;
							break;
						}
					} else if (devc->trigger_stage > 0) {
						/*
						 * We had a match before, but not in the next sample. However, we may
						 * have a match on this stage in the next bit -- trigger on 0001 will
						 * fail on seeing 00001, so we need to go back to stage 0 -- but at
						 * the next sample from the one that matched originally, which the
						 * counter increment at the end of the loop takes care of.
                         */
/*						i -= devc->trigger_stage;
						if (i < -1)
							i = -1; /* Oops, went back past this buffer. */
                        /* Reset trigger stage. */
/*						devc->trigger_stage = 0;
					}
			}
		}
		if (devc->trigger_stage == TRIGGER_FIRED) {
			/* Send the incoming transfer to the session bus. */
/*			trigger_offset_bytes = trigger_offset * sample_width;
			if (1){//(*(struct sr_dev_inst *)(devc->cb_data)).mode == LOGIC) {
				packet.type = SR_DF_LOGIC;
				packet.payload = &logic;
				logic.length = transfer->actual_length - trigger_offset_bytes;
				logic.unitsize = sample_width;
				//logic.data_error = 0;
				logic.data = cur_buf + trigger_offset_bytes;
			}/* else if ((*(struct sr_dev_inst *)(devc->cb_data)).mode == DSO) {
				packet.type = SR_DF_DSO;
				packet.payload = &dso;
				dso.probes = (*(struct sr_dev_inst *)(devc->cb_data)).channels;
				dso.num_samples = transfer->actual_length / sample_width;
				dso.mq = SR_MQ_VOLTAGE;
				dso.unit = SR_UNIT_VOLT;
				dso.mqflags = SR_MQFLAG_AC;
				dso.data = cur_buf + trigger_offset_bytes;
			} else {
				packet.type = SR_DF_ANALOG;
				packet.payload = &analog;
				analog.probes = (*(struct sr_dev_inst *)(devc->cb_data)).channels;
				analog.num_samples = transfer->actual_length / sample_width;
				analog.mq = SR_MQ_VOLTAGE;
				analog.unit = SR_UNIT_VOLT;
				analog.mqflags = SR_MQFLAG_AC;
				analog.data = cur_buf + trigger_offset_bytes;
			}*/
/*			if ((devc->sample_limit && devc->sample_count < devc->sample_limit) ||
//			    0/*(*(struct sr_dev_inst *)(devc->cb_data)).mode != LOGIC ) {
/*				const uint64_t remain_length= (devc->sample_limit - devc->sample_count) * sample_width;
				logic.length = min(logic.length, remain_length);
				// in test mode, check data content
				//if (devc->op_mode == SR_OP_INTERNAL_TEST) {
					//for (i = 0; i < logic.length / sample_width; i++) {
					/*for (i = 0; i < logic.length / 2; i++) {
						//                    const uint16_t cur_sample = devc->sample_wide ?
						//                        *((const uint16_t*)cur_buf + i) :
						//                        *((const uint8_t*)cur_buf + i);
						const uint16_t cur_sample = *((const uint16_t*)cur_buf + i);
						//if (test_init == 1) {
							//test_sample_value = cur_sample;
							//test_init = 0;
						//}
						//if (cur_sample != test_sample_value) {
							//logic.data_error = 1;
							break;
						//}
						//test_sample_value++;
					}*/
				//}
				//if (devc->op_mode == SR_OP_EXTERNAL_TEST) {
					/*for (i = 0; i < logic.length / 2; i++) {
						const uint16_t cur_sample = *((const uint16_t*)cur_buf + i);
						//if (test_init == 1) {
							//test_sample_value = cur_sample;
							//test_init = 0;
						//}
						//if (cur_sample != test_sample_value) {
							//logic.data_error = 1;
							//sr_err("exp: %d; act: %d", test_sample_value, cur_sample);
							break;
						//}
						//test_sample_value = (test_sample_value + 1) % 65001;
						//test_sample_value = test_sample_value + 1;
					}*/
				//}
				// send data to session bus 
				//sr_session_send(devc->cb_data, &packet);
/*			}

			devc->sample_count += cur_sample_count;
            if (/*(*(struct sr_dev_inst *)(devc->cb_data)).mode == LOGIC1 &&
/*			    devc->sample_limit &&
			    (unsigned int)devc->sample_count >= devc->sample_limit) {
				//abort_acquisition(devc);
				free_transfer(transfer);
				devc->status = DSLOGIC_STOP;
				return;
			}
		} else {
			// TODO: Buffer pre-trigger data in capture
			//ratio-sized buffer.
		}
        resubmit_transfer(transfer);*/
}

SR_PRIV void resubmit_transfer(struct libusb_transfer *transfer) {
	int ret;

	if ((ret = libusb_submit_transfer(transfer)) == LIBUSB_SUCCESS)
		return;

	free_transfer(transfer);
	/* TODO: Stop session? */

	sr_err("%s: %s", __func__, libusb_error_name(ret));
}
