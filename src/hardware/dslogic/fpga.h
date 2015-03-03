#ifndef LIBDSLOGIC_HW_FPGA_H_
#define LIBDSLOGIC_HW_FPGA_H_
#include "libsigrok.h"
#include "libsigrok-internal.h"
#include "devdefs.h"
#include <stdint.h>

SR_PRIV struct dslogic_fpga_setting;
SR_PRIV struct dslogic_fpga_setting* dslogic_fpga_new_setting(void);
SR_PRIV void dslogic_fpga_setting_free(struct dslogic_fpga_setting* setting);
SR_PRIV void dslogic_fpga_set_mode(struct dslogic_fpga_setting* setting, dev_mode mode);
SR_PRIV void dslogic_fpga_set_samplerate(struct dslogic_fpga_setting* setting, uint64_t current_sample_rate,
                                         uint64_t sample_limit);
SR_PRIV void dslogic_fpga_set_trigger(struct dslogic_fpga_setting* setting);
SR_PRIV int dslogic_get_fpga_setting_size(void);
SR_PRIV void dslogic_fpga_set_clock(struct dslogic_fpga_setting* setting,
                                    clk_source clock_source, clk_edge clock_edge);
SR_PRIV void dslogic_fpga_set_filter(struct dslogic_fpga_setting* setting,
                                     gboolean filter);
#endif
