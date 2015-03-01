#ifndef LIBDSLOGIC_HARDWARE_DEVDEFS_H
#define LIBDSLOGIC_HARDWARE_DEVDEFS_H

#define LOG_PREFIX "DSLogic Hardware: "

#define NUM_TRIGGER_STAGES	16
/* Software trigger implementation: positive values indicate trigger stage. */
#define TRIGGER_FIRED          -1

enum {
	DSLOGIC_ERROR = -1,
	DSLOGIC_INIT = 0,
	DSLOGIC_START = 1,
	DSLOGIC_TRIGGERED = 2,
	DSLOGIC_DATA = 3,
	DSLOGIC_STOP = 4,
};

struct DSLogic_profile {
	uint16_t vid;
	uint16_t pid;
	const char *vendor;
	const char *model;
	const char *model_version;
	const char *firmware;
	const char *fpga_bit33;
	const char *fpga_bit50;
	uint32_t dev_caps;
};

struct dev_context {
	const struct DSLogic_profile *profile;
    /*
    * Since we can't keep track of an DSLogic device after upgrading
    * the firmware (it renumerates into a different device address
    * after the upgrade) this is like a global lock. No device will open
    * until a proper delay after the last device was upgraded.
    */
	int64_t fw_updated;
	/* libsigrok context */
	struct sr_context* ctx;
	/* Device/capture settings */
    uint64_t current_samplerate;
    uint64_t sample_limit;
    gboolean clock_source; // EXTERNAL = TRUE
    uint32_t clock_edge;   // RISING = 0, FALLING = 1
    uint16_t voltage_threshold;
	gboolean filter;
    int trigger_stage;
    uint16_t trigger_mask[NUM_TRIGGER_STAGES];
	uint16_t trigger_value[NUM_TRIGGER_STAGES];
	uint16_t trigger_buffer[NUM_TRIGGER_STAGES];
	uint64_t timebase;
	uint8_t trigger_slope;
	uint8_t trigger_source;

	//TODO: replace trigger_hpos with capture ratio
	uint32_t trigger_hpos;
	uint32_t capture_ratio;
	
	gboolean zero;

	int num_samples;
	int submitted_transfers;
	int empty_transfer_count;

	void *cb_data;
	unsigned int num_transfers;
	struct libusb_transfer **transfers;
	//int *usbfd;

	int pipe_fds[2];
	GIOChannel *channel;

	int status;

    /* unknow functions */
    gboolean sample_wide;
};

/** Available clock sources.
 */
enum clock_source {
	CLOCK_INTERNAL,
	CLOCK_EXT_CLK,
};

#endif
