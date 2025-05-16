#include <stdlib.h>
#include <string.h>
#include "../../stdtype.h"
#include "../EmuStructs.h"
#include "../EmuCores.h"
#include "../snddef.h"
#include "../EmuHelper.h"
#include "k005289.h"


typedef struct _k005289_state {
    DEV_DATA _devData;

    struct {
        UINT16 pitch;       // Latched pitch value (12-bit)
        UINT16 freq;        // Current frequency
        UINT8 volume;       // Volume (0-15)
        UINT8 waveform;     // Waveform select (A5-A7)
        UINT16 counter;     // Frequency counter
        UINT8 addr;         // Waveform position (0-31)
    } voice[2];

    const UINT8* prom;      // Pointer to 512-byte PROM data
    UINT32 clock;           // Chip clock
    UINT32 rate;            // Sample rate
    UINT8 mute_mask;        // Mute mask (bit 0: voice 1, bit 1: voice 2)
} k005289_state;


// Function prototypes
static void k005289_update(void* param, UINT32 samples, DEV_SMPL** outputs);
static UINT8 device_start_k005289(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_k005289(void* chip);
static void device_reset_k005289(void* chip);
static void k005289_set_mute_mask(void* chip, UINT32 mute_mask);
static void k005289_write(void* chip, UINT8 address, UINT8 data);


static DEVDEF_RWFUNC devFunc[] = {
    { RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, k005289_write },
    { RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, k005289_set_mute_mask },
    { 0x00, 0x00, 0, NULL }
};

static DEV_DEF devDef = {
    "K005289", "MAME", FCC_MAME,
    device_start_k005289,
    device_stop_k005289,
    device_reset_k005289,
    k005289_update,
    NULL, NULL, NULL, NULL, NULL, NULL, devFunc
};

const DEV_DEF* devDefList_K005289[] = { &devDef, NULL };


// Device start
static UINT8 device_start_k005289(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf) {
    k005289_state* info;
    
    info = (k005289_state*)calloc(1, sizeof(k005289_state));
    if (info == NULL) return 0xFF;
    
    info->clock = cfg->clock;
    info->rate = info->clock / 16; // Typical clock divider
    info->prom = (UINT8*)cfg->data.ptr; // 512-byte PROM data
    
    device_reset_k005289(info);
    info->_devData.chipInf = info;
    INIT_DEVINF(retDevInf, &info->_devData, info->rate, &devDef);
    return 0x00;
}

// Device stop
static void device_stop_k005289(void* chip) {
    free(chip);
}

// Device reset
static void device_reset_k005289(void* chip) {
    k005289_state* info = (k005289_state*)chip;
    memset(info->voice, 0, sizeof(info->voice));
    info->mute_mask = 0x00;
}

// Sound generation
static void k005289_update(void* param, UINT32 samples, DEV_SMPL** outputs) {
    k005289_state* info = (k005289_state*)param;
    DEV_SMPL* buffer = outputs[0];
    UINT32 i;

    for (i = 0; i < samples; i++) {
        INT32 mix = 0;
        
        for (int ch = 0; ch < 2; ch++) {
            if (info->mute_mask & (1 << ch)) continue;
            
            // Update frequency counter
            if (info->voice[ch].counter-- == 0) {
                info->voice[ch].addr = (info->voice[ch].addr + 1) & 0x1F;
                info->voice[ch].counter = info->voice[ch].freq;
            }
            
            // Get sample from PROM (4-bit signed)
            UINT8 prom_offset = (ch * 0x100) + // Channel-specific PROM
                              (info->voice[ch].waveform << 5) + // Waveform select
                              info->voice[ch].addr; // Current position
            
            INT8 sample = (info->prom[prom_offset] & 0x0F) - 8; // Convert to signed
            mix += sample * info->voice[ch].volume;
        }
        
        buffer[i] = mix * 256; // Scale to 16-bit
    }
}

// Write handlers
static void k005289_write(void* chip, UINT8 address, UINT8 data) {
    k005289_state* info = (k005289_state*)chip;
    int ch = (address >= 0x02) ? 1 : 0; // Channel select
    
    switch (address) {
        // Control A (Channel 1)
        case 0x00:
            info->voice[0].volume = data & 0x0F;
            info->voice[0].waveform = (data >> 5) & 0x07;
            break;
            
        // Control B (Channel 2)
        case 0x01:
            info->voice[1].volume = data & 0x0F;
            info->voice[1].waveform = (data >> 5) & 0x07;
            break;
            
        // LD1/LD2 - Latch pitch
        case 0x02:
        case 0x03:
            info->voice[ch].pitch = 0xFFF - (address & 0x0FFF);
            break;
            
        // TG1/TG2 - Trigger frequency update
        case 0x04:
        case 0x05:
            info->voice[ch].freq = info->voice[ch].pitch;
            break;
    }
}

// Mute mask
static void k005289_set_mute_mask(void* chip, UINT32 mute_mask) {
    k005289_state* info = (k005289_state*)chip;
    info->mute_mask = mute_mask & 0x03;
}