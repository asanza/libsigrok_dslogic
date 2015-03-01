#ifndef LIBDSLOGIC_HARDWARE_TRANSFER_H
#define LIBDSLOGIC_HARDWARE_TRANSFER_H
#include <libusb.h>
#include <libsigrok.h>
#include "devdefs.h"
#include "dslogic.h"
#define NUM_SIMUL_TRANSFERS	64
#define MAX_EMPTY_TRANSFERS	(NUM_SIMUL_TRANSFERS * 2)

SR_PRIV void free_transfer(struct libusb_transfer *transfer);
SR_PRIV void dslogic_receive_transfer(struct libusb_transfer *transfer);
SR_PRIV void finish_acquisition(struct dev_context *devc);
SR_PRIV void resubmit_transfer(struct libusb_transfer *transfer);

#endif
