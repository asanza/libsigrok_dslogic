#ifndef LIBDSLOGIC_HARDWARE_DEVDEFS_H
#define LIBDSLOGIC_HARDWARE_DEVDEFS_H

#define LOG_PREFIX "DSLogic Hardware: "

#define NUM_TRIGGER_STAGES	16
/* Software trigger implementation: positive values indicate trigger stage. */
#define TRIGGER_FIRED          -1

#include <stdint.h>

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

typedef enum {
    NORMAL_MODE,
    TEST_INTERNAL,
    TEST_EXTERNAL,
}dev_mode;

typedef struct DSlogic_profile {
	uint16_t vid;
	uint16_t pid;
	const char *vendor;
	const char *model;
	const char *model_version;
	const char *firmware;
	const char *fpga_bit33;
	const char *fpga_bit50;
	uint32_t dev_caps;
}dslogic_profile;


#endif
