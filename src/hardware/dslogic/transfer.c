#include "transfer.h"
#include <libusb.h>
#include <libsigrok.h>
#include "libsigrok-internal.h"
#include "devdefs.h"

#ifndef _WIN32
#undef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif

SR_PRIV void dslogic_resubmit_transfer(struct libusb_transfer *transfer) {
	int ret;

	if ((ret = libusb_submit_transfer(transfer)) == LIBUSB_SUCCESS)
		return;

	free_transfer(transfer);
	/* TODO: Stop session? */

	sr_err("%s: %s", __func__, libusb_error_name(ret));
}

SR_PRIV void dslogic_receive_transfer(struct libusb_transfer *transfer) {
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

SR_PRIV void free_transfer(struct libusb_transfer *transfer)
{
    struct sr_dev_inst *sdi = transfer->user_data;
    unsigned int i;

    g_free(transfer->buffer);
    transfer->buffer = NULL;
    libusb_free_transfer(transfer);
    dslogic_clear_transfer(sdi, transfer);
}
