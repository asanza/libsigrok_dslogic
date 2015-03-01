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

SR_PRIV void resubmit_transfer(struct libusb_transfer *transfer) {
	int ret;

	if ((ret = libusb_submit_transfer(transfer)) == LIBUSB_SUCCESS)
		return;

	free_transfer(transfer);
	/* TODO: Stop session? */

	sr_err("%s: %s", __func__, libusb_error_name(ret));
}
