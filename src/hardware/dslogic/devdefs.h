#ifndef LIBDSLOGIC_HARDWARE_DEVDEFS_H
#define LIBDSLOGIC_HARDWARE_DEVDEFS_H

#define LOG_PREFIX "DSLogic Hardware: "

#define NUM_TRIGGER_STAGES	16
/* Software trigger implementation: positive values indicate trigger stage. */
#define TRIGGER_FIRED          -1

typedef enum {
	DSLOGIC_ERROR = -1,
	DSLOGIC_INIT = 0,
	DSLOGIC_START = 1,
	DSLOGIC_TRIGGERED = 2,
	DSLOGIC_DATA = 3,
	DSLOGIC_STOP = 4,
}dslogic_status;

/** Available clock sources.
 */
typedef enum {
    CLOCK_INTERNAL,
    CLOCK_EXT_CLK,
}clk_source;

typedef enum {
    RISING,
    FALLING
}clk_edge;

typedef enum  {
    VOLTAGE_RANGE_18_33_V,	/* 1.8V and 3.3V logic */
    VOLTAGE_RANGE_5_V,	/* 5V logic */
    VOLTAGE_RANGE_UNKNOWN,
}voltage_range;

struct dslogic_profile {
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
    const struct dslogic_profile *profile;
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
    clk_source clock_source; // EXTERNAL = TRUE
    clk_edge clock_edge;   // RISING = 0, FALLING = 1
    voltage_range voltage_threshold;
    gboolean filter;
    uint32_t capture_ratio;
    /* device internals */
    int sample_count;
    unsigned int num_transfers;
    int submitted_transfers;
    int empty_transfer_count;
    void *cb_data;
    struct libusb_transfer **transfers;
    dslogic_status status;

    /* unknow functions */
    gboolean sample_wide;
    int trigger_stage;
    uint16_t trigger_mask[NUM_TRIGGER_STAGES];
    uint16_t trigger_value[NUM_TRIGGER_STAGES];
    uint16_t trigger_buffer[NUM_TRIGGER_STAGES];
    uint64_t timebase;
    uint8_t trigger_slope;
    uint8_t trigger_source;
};


#endif
