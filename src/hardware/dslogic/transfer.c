#include "transfer.h"
#include <libusb.h>
#include <libsigrok.h>
#include "libsigrok-internal.h"
#include "devdefs.h"

#ifndef _WIN32
#undef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif

SR_PRIV void resubmit_transfer(struct libusb_transfer *transfer) {
	int ret;

	if ((ret = libusb_submit_transfer(transfer)) == LIBUSB_SUCCESS)
		return;

	free_transfer(transfer);
	/* TODO: Stop session? */

	sr_err("%s: %s", __func__, libusb_error_name(ret));
}
