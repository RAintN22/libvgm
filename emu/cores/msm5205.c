// license:BSD-3-Clause
// copyright-holders:eito,ValleyBell, Mao
/**********************************************************************************************
    OKI MSM5205 ADPCM (Full Working Implementation)
***********************************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../../stdtype.h"
#include "../snddef.h"
#include "../EmuHelper.h"
#include "../EmuCores.h"
#include "../logging.h"
#include "msm5205.h"

#define PIN_RESET   0x80
#define PIN_4B3B    0x40
#define PIN_S2      0x20
#define PIN_S1      0x10
#define PIN_DATA    0x0F

// ========== Function Prototypes ==========
static UINT8 device_start_msm5205(const DEV_GEN_CFG *cfg, DEV_INFO *retDevInf);
static void device_stop_msm5205(void *chip);
static void device_reset_msm5205(void *chip);
static void msm5205_update(void *param, UINT32 samples, DEV_SMPL **outputs);
static UINT32 msm5205_get_rate(void *chip);
static void msm5205_set_clock(void *chip, UINT32 clock);
static void msm5205_write(void *chip, UINT8 offset, UINT8 data);
static void msm5205_set_mute_mask(void *chip, UINT32 MuteMask);
static void msm5205_set_srchg_cb(void *chip, DEVCB_SRATE_CHG CallbackFunc, void *DataPtr);
static void msm5205_set_log_cb(void *chip, DEVCB_LOG func, void *param);

// ========== Core Structure ==========
typedef struct _msm5205_state {
    DEV_DATA _devData;
    DEV_LOGGER logger;
    
    UINT32  master_clock;
    INT32   signal;
    INT32   step;
    
    UINT8   data_buf[8];
    UINT8   data_in_last;
    UINT8   data_buf_pos;
    UINT8   data_empty;
    
    UINT8   output_mask;
    UINT8   Muted;
    
    DEVCB_SRATE_CHG SmpRateFunc;
    void*   SmpRateData;
} msm5205_state;

// ========== Global Tables ==========
static const int index_shift[8] = {-1, -1, -1, -1, 2, 4, 6, 8};
static int diff_lookup[49*16];
static UINT8 tables_computed = 0;

// ========== Device Definition ==========
static DEVDEF_RWFUNC devFunc[] = {
    {RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, msm5205_write},
    {RWF_CLOCK | RWF_WRITE, DEVRW_VALUE, 0, msm5205_set_clock},
    {RWF_SRATE | RWF_READ, DEVRW_VALUE, 0, msm5205_get_rate},
    {RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, msm5205_set_mute_mask},
    {0x00, 0x00, 0, NULL}
};

static DEV_DEF devDef = {
    "MSM5205", "MAME", FCC_MAME,
    device_start_msm5205,
    device_stop_msm5205,
    device_reset_msm5205,
    msm5205_update,
    NULL,
    msm5205_set_mute_mask,
    NULL,
    msm5205_set_srchg_cb,
    msm5205_set_log_cb,
    NULL,
    devFunc
};

const DEV_DEF *devDefList_MSM5205[] = { &devDef, NULL };

// ========== Helper Functions ==========
static void compute_tables(void) {
    static const int nbl2bit[16][4] = {
        {1,0,0,0}, {1,0,0,1}, {1,0,1,0}, {1,0,1,1},
        {1,1,0,0}, {1,1,0,1}, {1,1,1,0}, {1,1,1,1},
        {-1,0,0,0}, {-1,0,0,1}, {-1,0,1,0}, {-1,0,1,1},
        {-1,1,0,0}, {-1,1,0,1}, {-1,1,1,0}, {-1,1,1,1}
    };

    if (tables_computed) return;

    for (int step = 0; step <= 48; step++) {
        int stepval = (int)floor(16.0 * pow(11.0 / 10.0, (double)step));
        for (int nib = 0; nib < 16; nib++) {
            diff_lookup[step*16 + nib] = nbl2bit[nib][0] *
                (stepval   * nbl2bit[nib][1] +
                 stepval/2 * nbl2bit[nib][2] +
                 stepval/4 * nbl2bit[nib][3] +
                 stepval/8);
        }
    }
    tables_computed = 1;
}

INLINE UINT32 get_prescaler(msm5205_state *info) {
    return (info->data_in_last & PIN_S1) ? 
        ((info->data_in_last & PIN_S2) ? 48 : 64) : 
        ((info->data_in_last & PIN_S2) ? 96 : 192);
}

// ========== Core ADPCM Processing ==========
static INT16 clock_adpcm(msm5205_state *chip, UINT8 data) {
    if (data & PIN_RESET) {
        chip->step = 0;
        chip->signal = 0;
        return 0;
    }
    
    if (!(data & PIN_4B3B)) data <<= 1;
    data &= PIN_DATA;

    int sample = diff_lookup[chip->step * 16 + (data & 15)];
    chip->signal = ((sample << 8) + (chip->signal * 245)) >> 8;

    chip->signal = (chip->signal > 2047) ? 2047 : 
                  ((chip->signal < -2048) ? -2048 : chip->signal);
    
    chip->step += index_shift[data & 7];
    chip->step = (chip->step > 48) ? 48 : 
                ((chip->step < 0) ? 0 : chip->step);
    
    return (INT16)(chip->signal << 4);
}

// ========== Device Interface ==========
static UINT8 device_start_msm5205(const DEV_GEN_CFG *cfg, DEV_INFO *retDevInf) {
    msm5205_state *info;
    
    compute_tables();
    
    info = (msm5205_state*)calloc(1, sizeof(msm5205_state));
    if (!info) return 0xFF;

    info->master_clock = cfg->clock;
    info->signal = -2;
    info->step = 0;
    info->Muted = 0;
    info->data_empty = 0xFF;
    info->data_in_last = PIN_S2;
    info->data_buf[0] = info->data_in_last;

    info->_devData.chipInf = info;
    INIT_DEVINF(retDevInf, &info->_devData, msm5205_get_rate(info), &devDef);
    return 0x00;
}

static void device_stop_msm5205(void *chip) {
    free((msm5205_state*)chip);
}

static void device_reset_msm5205(void *chip) {
    msm5205_state *info = (msm5205_state*)chip;
    
    info->signal = -2;
    info->step = 0;
    memset(info->data_buf, 0, sizeof(info->data_buf));
    info->data_buf_pos = 0;
    info->data_empty = 0xFF;
    info->data_in_last = PIN_S2;
    info->data_buf[0] = info->data_in_last;
    
    if (info->SmpRateFunc)
        info->SmpRateFunc(info->SmpRateData, msm5205_get_rate(info));
}

// ========== Audio Generation ==========
static void msm5205_update(void *param, UINT32 samples, DEV_SMPL **outputs) {
    msm5205_state *info = (msm5205_state*)param;
    DEV_SMPL *bufL = outputs[0];
    DEV_SMPL *bufR = outputs[1];
    UINT32 i;

    for (i = 0; i < samples; i++) {
        INT16 sample = 0;
        
        if (!info->Muted && !(info->data_in_last & PIN_RESET)) {
            UINT8 read_pos = info->data_buf_pos & 0x0F;
            UINT8 write_pos = (info->data_buf_pos >> 4) & 0x07;
            
            if (read_pos != write_pos) {
                UINT8 data = info->data_buf[read_pos];
                sample = clock_adpcm(info, data);
                info->data_buf_pos = (write_pos << 4) | ((read_pos + 1) & 0x07);
            } else {
                sample = (info->signal * 15) / 16;
            }
        }
        
        bufL[i] = bufR[i] = sample;
    }
}

// ========== I/O Handling ==========
static void msm5205_write(void *chip, UINT8 offset, UINT8 data) {
    msm5205_state *info = (msm5205_state*)chip;
    
    if (offset == 0) {
        UINT8 write_pos = (info->data_buf_pos >> 4) & 0x07;
        UINT8 read_pos = info->data_buf_pos & 0x07;
        
        if (((write_pos + 1) & 0x07) == read_pos) {
            emu_logf(&info->logger, DEVLOG_DEBUG, "MSM5205 FIFO overflow\n");
            return;
        }
        
        info->data_buf[write_pos] = data;
        info->data_buf_pos = ((write_pos + 1) << 4) | read_pos;
    } else {
        UINT8 old = info->data_in_last;
        info->data_in_last = data;
        
        if ((old ^ data) & (PIN_S1|PIN_S2|PIN_RESET)) {
            if (info->SmpRateFunc)
                info->SmpRateFunc(info->SmpRateData, msm5205_get_rate(info));
            
            if ((old ^ data) & PIN_RESET) {
                info->signal = 0;
                info->step = 0;
            }
        }
    }
}

// ========== Configuration ==========
static UINT32 msm5205_get_rate(void *chip) {
    msm5205_state *info = (msm5205_state*)chip;
    return info->master_clock / get_prescaler(info);
}

static void msm5205_set_clock(void *chip, UINT32 clock) {
    msm5205_state *info = (msm5205_state*)chip;
    info->master_clock = clock;
    if (info->SmpRateFunc)
        info->SmpRateFunc(info->SmpRateData, msm5205_get_rate(info));
}

static void msm5205_set_mute_mask(void *chip, UINT32 MuteMask) {
    msm5205_state *info = (msm5205_state*)chip;
    info->Muted = MuteMask & 0x01;
}

static void msm5205_set_srchg_cb(void *chip, DEVCB_SRATE_CHG CallbackFunc, void *DataPtr) {
    msm5205_state *info = (msm5205_state*)chip;
    info->SmpRateFunc = CallbackFunc;
    info->SmpRateData = DataPtr;
}

static void msm5205_set_log_cb(void *chip, DEVCB_LOG func, void *param) {
    msm5205_state *info = (msm5205_state*)chip;
    dev_logger_set(&info->logger, info, func, param);
}