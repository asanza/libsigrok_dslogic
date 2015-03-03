#include "fpga.h"
#include "devdefs.h"
#include <math.h>

struct dslogic_fpga_setting {
        uint32_t sync;
        uint16_t mode_header;                   // 0
        uint16_t mode;
        uint32_t divider_header;                // 1-2
        uint32_t divider;
        uint32_t count_header;                  // 3-4
        uint32_t count;
        uint32_t trig_pos_header;               // 5-6
        uint32_t trig_pos;
        uint16_t trig_glb_header;               // 7
        uint16_t trig_glb;
        uint32_t trig_adp_header;               // 10-11
        uint32_t trig_adp;
        uint32_t trig_sda_header;               // 12-13
        uint32_t trig_sda;
        uint32_t trig_mask0_header;              // 16
        uint16_t trig_mask0[NUM_TRIGGER_STAGES];
        uint32_t trig_mask1_header;              // 17
        uint16_t trig_mask1[NUM_TRIGGER_STAGES];
        //uint32_t trig_mask2_header;              // 18
        //uint16_t trig_mask2[NUM_TRIGGER_STAGES];
        //uint32_t trig_mask3_header;              // 19
        //uint16_t trig_mask3[NUM_TRIGGER_STAGES];
        uint32_t trig_value0_header;             // 20
        uint16_t trig_value0[NUM_TRIGGER_STAGES];
        uint32_t trig_value1_header;             // 21
        uint16_t trig_value1[NUM_TRIGGER_STAGES];
        //uint32_t trig_value2_header;             // 22
        //uint16_t trig_value2[NUM_TRIGGER_STAGES];
        //uint32_t trig_value3_header;             // 23
        //uint16_t trig_value3[NUM_TRIGGER_STAGES];
        uint32_t trig_edge0_header;              // 24
        uint16_t trig_edge0[NUM_TRIGGER_STAGES];
        uint32_t trig_edge1_header;              // 25
        uint16_t trig_edge1[NUM_TRIGGER_STAGES];
        //uint32_t trig_edge2_header;              // 26
        //uint16_t trig_edge2[NUM_TRIGGER_STAGES];
        //uint32_t trig_edge3_header;              // 27
        //uint16_t trig_edge3[NUM_TRIGGER_STAGES];
        uint32_t trig_count0_header;             // 28
        uint16_t trig_count0[NUM_TRIGGER_STAGES];
        uint32_t trig_count1_header;             // 29
        uint16_t trig_count1[NUM_TRIGGER_STAGES];
        //uint32_t trig_count2_header;             // 30
        //uint16_t trig_count2[NUM_TRIGGER_STAGES];
        //uint32_t trig_count3_header;             // 31
        //uint16_t trig_count3[NUM_TRIGGER_STAGES];
        uint32_t trig_logic0_header;             // 32
        uint16_t trig_logic0[NUM_TRIGGER_STAGES];
        uint32_t trig_logic1_header;             // 33
        uint16_t trig_logic1[NUM_TRIGGER_STAGES];
        //uint32_t trig_logic2_header;             // 34
        //uint16_t trig_logic2[NUM_TRIGGER_STAGES];
        //uint32_t trig_logic3_header;             // 35
        //uint16_t trig_logic3[NUM_TRIGGER_STAGES];
        uint32_t end_sync;
};

SR_PRIV int dslogic_get_fpga_setting_size(void){
    return sizeof (struct dslogic_fpga_setting);
}

SR_PRIV struct dslogic_fpga_setting* dslogic_fpga_new_setting(void){
    struct dslogic_fpga_setting* setting = g_try_malloc(sizeof(struct dslogic_fpga_setting));
    g_assert(setting);
    setting->sync = 0xf5a5f5a5;
    setting->mode_header = 0x0001;
    setting->divider_header = 0x0102ffff;
    setting->count_header = 0x0302ffff;
    setting->trig_pos_header = 0x0502ffff;
    setting->trig_glb_header = 0x0701;
    setting->trig_adp_header = 0x0a02ffff;
    setting->trig_sda_header = 0x0c02ffff;
    setting->trig_mask0_header = 0x1010ffff;
    setting->trig_mask1_header = 0x1110ffff;
    //setting->trig_mask2_header = 0x1210ffff;
    //setting->trig_mask3_header = 0x1310ffff;
    setting->trig_value0_header = 0x1410ffff;
    setting->trig_value1_header = 0x1510ffff;
    //setting->trig_value2_header = 0x1610ffff;
    //setting->trig_value3_header = 0x1710ffff;
    setting->trig_edge0_header = 0x1810ffff;
    setting->trig_edge1_header = 0x1910ffff;
    //setting->trig_edge2_header = 0x1a10ffff;
    //setting->trig_edge3_header = 0x1b10ffff;
    setting->trig_count0_header = 0x1c10ffff;
    setting->trig_count1_header = 0x1d10ffff;
    //setting->trig_count2_header = 0x1e10ffff;
    //setting->trig_count3_header = 0x1f10ffff;
    setting->trig_logic0_header = 0x2010ffff;
    setting->trig_logic1_header = 0x2110ffff;
    //setting->trig_logic2_header = 0x2210ffff;
    //setting->trig_logic3_header = 0x2310ffff;
    setting->end_sync = 0xfa5afa5a;
    return setting;
}

SR_PRIV void dslogic_fpga_setting_free(struct dslogic_fpga_setting* setting){
    g_free(setting);
}

SR_PRIV void dslogic_fpga_set_mode(struct dslogic_fpga_setting* setting, dev_mode mode){
    /* clear old mode bits */
    setting->mode &= 0x1FFF;
    switch (mode){
    case NORMAL_MODE:
        break;
    case TEST_EXTERNAL:
        setting->mode += 1 << 14;
        break;
    case TEST_INTERNAL:
        setting->mode += 1 << 15;
        break;
    /* case LOOPBACK_MODE:
     * setting->mode += 1 << 13;
     * break;
     */
    default:
        g_assert(0);
    }
    setting->mode &= ~( 1 << 4 ); // set analog mode. (set bit to 1 for analog or dso);
    setting->mode &= ~( 1 << 7 ); // set bit to 1 for analog ;
}

SR_PRIV void dslogic_fpga_set_clock(struct dslogic_fpga_setting* setting,
                                    clk_source clock_source, clk_edge clock_edge){
    setting->mode &= ~(1 << 1);
    switch(clock_source){
    case CLOCK_EXT_CLK:
        setting->mode += (1 << 1);
        break;
    case CLOCK_INTERNAL:
        break;
    default:
        g_assert(0);
    }

    switch(clock_edge){
    case RISING:
        setting->mode += (1 << 1);
        break;
    case FALLING:
        break;
    }
}

SR_PRIV void dslogic_fpga_set_filter(struct dslogic_fpga_setting* setting,
                                     gboolean filter){
    setting->mode &= ~(1 << 8);
    setting->mode += filter << 8;
}

SR_PRIV void dslogic_fpga_set_samplerate(struct dslogic_fpga_setting* setting, uint64_t current_samplerate,
                                         uint64_t sample_limit){
    setting->divider = (uint32_t) ceil(SR_MHZ(100) * 1.0 / current_samplerate);
    setting->count = (uint32_t) (sample_limit);
    setting->mode += 0 << 5; //(((devc->current_samplerate == SR_MHZ(200) && 1/*sdi->mode != DSO*/) || (0/*sdi->mode == ANALOG*/)) << 5);
    setting->mode += 0 << 6; //((devc->current_samplerate == SR_MHZ(400)) << 6);
}

SR_PRIV void dslogic_fpga_set_trigger(struct dslogic_fpga_setting* setting, uint32_t pretrigger_samples,
                                      gboolean enabled){
    setting->trig_pos = pretrigger_samples; //danot sure about it.
    setting->trig_adp = setting->count - setting->trig_pos - 1;
    setting->mode += enabled; //trigger->trigger_en;
    setting->trig_glb = 0; //trigger->trigger_stages; //no trigger stages
    setting->trig_sda = 0x0;
    int i;
    if (1) {//trigger->trigger_mode == SIMPLE_TRIGGER) {
        setting->trig_mask0[0] = 0; //ds_trigger_get_mask0(TriggerStages);
        setting->trig_mask1[0] = 0; //ds_trigger_get_mask1(TriggerStages);

        setting->trig_value0[0] = 0; //ds_trigger_get_value0(TriggerStages);
        setting->trig_value1[0] = 0; //ds_trigger_get_value1(TriggerStages);

        setting->trig_edge0[0] = 0; //ds_trigger_get_edge0(TriggerStages);
        setting->trig_edge1[0] = 0; //ds_trigger_get_edge1(TriggerStages);

        setting->trig_count0[0] = 0; //trigger->trigger0_count[TriggerStages];
        setting->trig_count1[0] = 0; // trigger->trigger1_count[TriggerStages];

        setting->trig_logic0[0] = 0; //(trigger->trigger_logic[TriggerStages] << 1) + trigger->trigger0_inv[TriggerStages];
        setting->trig_logic1[0] = 0; //(trigger->trigger_logic[TriggerStages] << 1) + trigger->trigger1_inv[TriggerStages];

        for (i = 1; i < NUM_TRIGGER_STAGES; i++) {
            setting->trig_mask0[i] = 0xff;
            setting->trig_mask1[i] = 0xff;

            setting->trig_value0[i] = 0;
            setting->trig_value1[i] = 0;

            setting->trig_edge0[i] = 0;
            setting->trig_edge1[i] = 0;

            setting->trig_count0[i] = 0;
            setting->trig_count1[i] = 0;

            setting->trig_logic0[i] = 2;
            setting->trig_logic1[i] = 2;
        }
    } else {/*
        for (i = 0; i < NUM_TRIGGER_STAGES; i++) {
            setting->trig_mask0[i] = ds_trigger_get_mask0(i);
            setting->trig_mask1[i] = ds_trigger_get_mask1(i);

            setting->trig_value0[i] = ds_trigger_get_value0(i);
            setting->trig_value1[i] = ds_trigger_get_value1(i);

            setting->trig_edge0[i] = ds_trigger_get_edge0(i);
            setting->trig_edge1[i] = ds_trigger_get_edge1(i);

            setting->trig_count0[i] = trigger->trigger0_count[i];
            setting->trig_count1[i] = trigger->trigger1_count[i];

            setting->trig_logic0[i] = (trigger->trigger_logic[i] << 1) + trigger->trigger0_inv[i];
            setting->trig_logic1[i] = (trigger->trigger_logic[i] << 1) + trigger->trigger1_inv[i];
        }*/
    }
}

