
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../../stdtype.h"
#include "../EmuStructs.h"
#include "../EmuCores.h"
#include "../snddef.h"
#include "../EmuHelper.h"
#include "es5506.h"

// Constants
#define MAX_VOICES         32
#define MAX_REGIONS        4
#define ULAW_MAXBITS       8
#define MAX_SAMPLE_CHUNK   10000

// Control register bits
#define CONTROL_STOPMASK   0x0003
#define CONTROL_STOP0      0x0001
#define CONTROL_STOP1      0x0002
#define CONTROL_BS0        0x4000
#define CONTROL_BS1        0x8000
#define CONTROL_LOOPMASK   0x0018
#define CONTROL_LPE        0x0008
#define CONTROL_BLE        0x0010
#define CONTROL_CA_MASK    0x1C00
#define CONTROL_LP_MASK    0x0300
#define CONTROL_LP3        0x0100
#define CONTROL_LP4        0x0200
#define CONTROL_IRQE       0x0020
#define CONTROL_DIR        0x0040
#define CONTROL_IRQ        0x0080

// Helper macro
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

typedef struct {
    UINT32 control;
    UINT32 freqcount;
    UINT32 start;
    UINT32 end;
    UINT32 accum;
    UINT32 lvol;
    UINT32 rvol;
    UINT32 lvramp;
    UINT32 rvramp;
    UINT32 ecount;
    UINT32 k2;
    UINT32 k2ramp;
    UINT32 k1;
    UINT32 k1ramp;
    INT32 o4n1;
    INT32 o3n1;
    INT32 o3n2;
    INT32 o2n1;
    INT32 o2n2;
    INT32 o1n1;
    UINT32 exbank;
    UINT32 accum_mask;
    UINT8 filtcount;
    UINT8 Muted;
} ES5506_Voice;

typedef struct {
    DEV_DATA _devData;
    
    ES5506_Voice voice[MAX_VOICES];
    UINT16* region_base[MAX_REGIONS];
    UINT32 region_size[MAX_REGIONS];
    UINT32 master_clock;
    UINT32 write_latch;
    UINT32 read_latch;
    UINT8 current_page;
    UINT8 active_voices;
    UINT8 mode;
    UINT8 wst;
    UINT8 wend;
    UINT8 lrend;
    UINT8 irqv;
    UINT8 sndtype;
    
    INT16* ulaw_lookup;
    UINT16* volume_lookup;
    INT32* scratch;
    UINT8 output_channels;
    UINT32 output_rate;
    
    DEVCB_SRATE_CHG SmpRateFunc;
    void* SmpRateData;
} ES5506_Chip;

// Prototypes
static void generate_samples(ES5506_Chip* chip, INT32** outputs, int offset, int samples);
static void update_envelopes(ES5506_Voice* voice, int samples);
static void apply_filters(ES5506_Voice* voice, INT32* sample);
static void compute_tables(ES5506_Chip* chip);
static void check_loop_end(ES5506_Voice* voice, UINT32 accum);

// Device interface
static UINT8 device_start_es5506(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_es5506(void* info);
static void device_reset_es5506(void* info);
static void es5506_pcm_update(void* param, UINT32 samples, DEV_SMPL** outputs);
static void es5506_write(void* info, UINT8 offset, UINT8 data);
static UINT8 es5506_read(void* info, UINT8 offset);
static void es5506_write_rom(void* info, UINT32 offset, UINT32 length, const UINT8* data);
static void es5506_set_mute_mask(void* info, UINT32 MuteMask);
static void es5506_set_srchg_cb(void* info, DEVCB_SRATE_CHG CallbackFunc, void* DataPtr);

static DEVDEF_RWFUNC devFunc[] = {
    { RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, es5506_write },
    { RWF_REGISTER | RWF_READ,  DEVRW_A8D8, 0, es5506_read },
    { RWF_MEMORY | RWF_WRITE,   DEVRW_BLOCK, 0, es5506_write_rom },
    { RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, es5506_set_mute_mask },
    { 0x00, 0x00, 0, NULL }
};

static DEV_DEF devDef = {
    "ES5506", "MAME", FCC_MAME,
    device_start_es5506,
    device_stop_es5506,
    device_reset_es5506,
    es5506_pcm_update,
    NULL,
    es5506_set_mute_mask,
    NULL,
    es5506_set_srchg_cb,
    NULL,
    NULL,
    devFunc
};

const DEV_DEF* devDefList_ES5506[] = { &devDef, NULL };

//-------------------------------------------------
// Device initialization
//-------------------------------------------------

static UINT8 device_start_es5506(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf) {
    ES5506_Chip* chip = calloc(1, sizeof(ES5506_Chip));
    if (!chip) return 0xFF;

    chip->master_clock = cfg->clock;
    chip->output_channels = (cfg->flags & 0x0F) ? (cfg->flags & 0x0F) : 2;
    chip->sndtype = (cfg->flags >> 31) & 0x01;

    for (int i = 0; i < MAX_REGIONS; i++) {
        chip->region_base[i] = NULL;
        chip->region_size[i] = 0;
    }

    compute_tables(chip);
    chip->scratch = malloc(2 * MAX_SAMPLE_CHUNK * sizeof(INT32));

    chip->active_voices = chip->sndtype ? 31 : 24;
    chip->output_rate = chip->master_clock / (16 * (chip->active_voices + 1));
    
    es5506_set_mute_mask(chip, 0x00000000);
    chip->_devData.chipInf = chip;
    INIT_DEVINF(retDevInf, &chip->_devData, chip->output_rate, &devDef);

    // ----------- DUMMY ROM PATCH FOR VGM LOGGING -----------
    // If no sample ROM was loaded, fill each region with a repeating saw pattern.
    for (int r = 0; r < MAX_REGIONS; ++r) {
        if (!chip->region_base[r]) {
            chip->region_size[r] = 0x10000; // 64k words (128KB)
            chip->region_base[r] = (UINT16*)calloc(chip->region_size[r], 1);
            for (int i = 0; i < chip->region_size[r] / 2; ++i)
                chip->region_base[r][i] = (i & 0xFF) << 8; // Sawtooth waveform
        }
    }
    // -------------------------------------------------------

    return 0x00;
}

static void device_stop_es5506(void* info) {
    ES5506_Chip* chip = (ES5506_Chip*)info;
    if (!chip) return;

    free(chip->ulaw_lookup);
    free(chip->volume_lookup);
    free(chip->scratch);
    for (int i = 0; i < MAX_REGIONS; i++) free(chip->region_base[i]);
    free(chip);
}

static void device_reset_es5506(void* info) {
    ES5506_Chip* chip = (ES5506_Chip*)info;
    UINT32 accum_mask = chip->sndtype ? 0xFFFFFFFF : 0x7FFFFFFF;

    memset(chip->voice, 0, sizeof(chip->voice));
    for (int i = 0; i < MAX_VOICES; i++) {
        chip->voice[i].control = CONTROL_STOPMASK;
        chip->voice[i].lvol = 0xFFFF;
        chip->voice[i].rvol = 0xFFFF;
        chip->voice[i].accum_mask = accum_mask;
    }

    chip->active_voices = chip->sndtype ? 31 : 24;
    chip->output_rate = chip->master_clock / (16 * (chip->active_voices + 1));
    if (chip->SmpRateFunc)
        chip->SmpRateFunc(chip->SmpRateData, chip->output_rate);
}

//-------------------------------------------------
// Audio generation core
//-------------------------------------------------

static void es5506_pcm_update(void* param, UINT32 samples, DEV_SMPL** outputs) {
    ES5506_Chip* chip = (ES5506_Chip*)param;
    INT32* buffers[8];
    int offset = 0;

    for (int i = 0; i < chip->output_channels; i++) {
        memset(outputs[i], 0, samples * sizeof(DEV_SMPL));
        buffers[i] = (INT32*)outputs[i];
    }

    while (samples > 0) {
        int todo = (samples > MAX_SAMPLE_CHUNK) ? MAX_SAMPLE_CHUNK : samples;
        generate_samples(chip, buffers, offset, todo);
        offset += todo;
        samples -= todo;
    }
}

static void generate_samples(ES5506_Chip* chip, INT32** buffers, int offset, int samples) {
    for (int v = 0; v <= chip->active_voices; v++) {
        ES5506_Voice* voice = &chip->voice[v];
        if (voice->control & CONTROL_STOPMASK || voice->Muted)
            continue;

        UINT16* base = chip->region_base[voice->control >> 14];
        if (!base) continue;

        UINT32 freqcount = voice->freqcount;
        UINT32 accum = voice->accum & voice->accum_mask;
        INT32 lvol = chip->volume_lookup[voice->lvol >> 4];
        INT32 rvol = chip->volume_lookup[voice->rvol >> 4];
        UINT32 end = voice->end;
        UINT32 start = voice->start;

        for (int s = 0; s < samples; s++) {
            if ((voice->control & CONTROL_DIR) ? (accum < start) : (accum > end)) {
                check_loop_end(voice, accum);
                if (voice->control & CONTROL_STOPMASK) break;
            }

            UINT32 addr = (accum >> 11) & (voice->accum_mask >> 11);
            INT32 val1, val2;
            
            if (voice->control & 0x2000) {
                val1 = chip->ulaw_lookup[base[addr] >> (16 - ULAW_MAXBITS)];
                val2 = chip->ulaw_lookup[base[(addr + 1) & (voice->accum_mask >> 11)] >> (16 - ULAW_MAXBITS)];
            } else {
                val1 = (INT16)base[addr];
                val2 = (INT16)base[(addr + 1) & (voice->accum_mask >> 11)];
            }

            INT32 sample = ((val1 * (0x800 - (accum & 0x7FF)) + 
                           val2 * (accum & 0x7FF)) >> 11);

            apply_filters(voice, &sample);

            accum = (voice->control & CONTROL_DIR) ? 
                (accum - freqcount) : (accum + freqcount);
            accum &= voice->accum_mask;

            INT32 lout = (sample * lvol) >> 11;
            INT32 rout = (sample * rvol) >> 11;

            UINT8 ch = (voice->control & CONTROL_CA_MASK) >> 10;
            ch %= chip->output_channels;
            
            buffers[ch][offset + s] += lout;
            if (ch < chip->output_channels - 1)
                buffers[ch + 1][offset + s] += rout;
            else
                buffers[ch][offset + s] += rout;
        }

        voice->accum = accum;
        update_envelopes(voice, samples);
    }
}

static void check_loop_end(ES5506_Voice* voice, UINT32 accum) {
    UINT32 end = voice->end;
    UINT32 start = voice->start;
    
    if (voice->control & CONTROL_IRQE)
        voice->control |= CONTROL_IRQ;

    switch (voice->control & CONTROL_LOOPMASK) {
        case 0:
            voice->control |= CONTROL_STOP0;
            break;
            
        case CONTROL_LPE:
            accum = start + ((accum - start) % (end - start));
            break;
            
        case CONTROL_BLE:
            voice->control ^= CONTROL_DIR;
            accum = end - (accum - end);
            break;
            
        case (CONTROL_LPE | CONTROL_BLE):
            accum = start + (end - accum);
            break;
    }
    
    voice->accum = accum;
}

static void apply_filters(ES5506_Voice* voice, INT32* sample) {
    INT32 temp = *sample;
    
    // Pole 1
    temp = ((voice->k1 >> 2) * (temp - voice->o1n1) / 16384) + voice->o1n1;
    voice->o1n1 = temp;
    
    // Pole 2
    temp = ((voice->k1 >> 2) * (temp - voice->o2n1) / 16384) + voice->o2n1;
    voice->o2n2 = voice->o2n1;
    voice->o2n1 = temp;
    
    switch (voice->control & CONTROL_LP_MASK) {
        case 0:
            temp = temp - voice->o2n2 + 
                 ((voice->k2 >> 2) * voice->o3n1) / 32768 + voice->o3n1 / 2;
            voice->o3n2 = voice->o3n1;
            voice->o3n1 = temp;
            
            temp = temp - voice->o3n2 + 
                 ((voice->k2 >> 2) * voice->o4n1) / 32768 + voice->o4n1 / 2;
            voice->o4n1 = temp;
            break;
            
        case CONTROL_LP3:
            temp = ((voice->k1 >> 2) * (temp - voice->o3n1) / 16384) + voice->o3n1;
            voice->o3n2 = voice->o3n1;
            voice->o3n1 = temp;
            
            temp = temp - voice->o3n2 + 
                 ((voice->k2 >> 2) * voice->o4n1) / 32768 + voice->o4n1 / 2;
            voice->o4n1 = temp;
            break;
            
        case CONTROL_LP4:
            temp = ((voice->k2 >> 2) * (temp - voice->o3n1) / 16384) + voice->o3n1;
            voice->o3n2 = voice->o3n1;
            voice->o3n1 = temp;
            
            temp = ((voice->k2 >> 2) * (temp - voice->o4n1) / 16384) + voice->o4n1;
            voice->o4n1 = temp;
            break;
            
        case (CONTROL_LP3 | CONTROL_LP4):
            temp = ((voice->k1 >> 2) * (temp - voice->o3n1) / 16384) + voice->o3n1;
            voice->o3n2 = voice->o3n1;
            voice->o3n1 = temp;
            
            temp = ((voice->k2 >> 2) * (temp - voice->o4n1) / 16384) + voice->o4n1;
            voice->o4n1 = temp;
            break;
    }
    
    *sample = temp;
}

static void update_envelopes(ES5506_Voice* voice, int samples) {
    if (voice->ecount == 0) return;
    
    int count = (samples > voice->ecount) ? voice->ecount : samples;
    voice->ecount -= count;
    
    if (voice->lvramp) {
        voice->lvol += (INT8)voice->lvramp * count;
        voice->lvol = CLAMP(voice->lvol, 0, 0xFFFF);
    }
    if (voice->rvramp) {
        voice->rvol += (INT8)voice->rvramp * count;
        voice->rvol = CLAMP(voice->rvol, 0, 0xFFFF);
    }
    
    if (voice->k1ramp && (voice->k1ramp >= 0 || !(voice->filtcount & 7))) {
        voice->k1 += (INT8)voice->k1ramp * count;
        voice->k1 = CLAMP(voice->k1, 0, 0xFFFF);
    }
    if (voice->k2ramp && (voice->k2ramp >= 0 || !(voice->filtcount & 7))) {
        voice->k2 += (INT8)voice->k2ramp * count;
        voice->k2 = CLAMP(voice->k2, 0, 0xFFFF);
    }
    
    voice->filtcount += count;
}

//-------------------------------------------------
// Register I/O
//-------------------------------------------------

static void es5506_write(void* info, UINT8 offset, UINT8 data) {
    ES5506_Chip* chip = (ES5506_Chip*)info;
    ES5506_Voice* voice = &chip->voice[chip->current_page & 0x1F];
    UINT32 shift = 8 * (offset & 3);
    UINT32 mask = ~(0xFF << (24 - shift));
    UINT32 value = data << (24 - shift);

    chip->write_latch = (chip->write_latch & mask) | value;

    if (shift != 24) return;

    switch (chip->current_page) {
        case 0x00:
            switch (offset >> 2) {
                case 0: voice->control = chip->write_latch & 0xFFFF; break;
                case 1: voice->freqcount = chip->write_latch & 0x1FFFF; break;
                case 2: voice->start = chip->write_latch & 0xFFFFF800; break;
                case 3: voice->end = chip->write_latch & 0xFFFFFF80; break;
                case 4: voice->accum = chip->write_latch & voice->accum_mask; break;
                case 5: voice->o4n1 = (INT32)(chip->write_latch << 14) >> 14; break;
                case 6: voice->o3n1 = (INT32)(chip->write_latch << 14) >> 14; break;
                case 7: voice->o3n2 = (INT32)(chip->write_latch << 14) >> 14; break;
            }
            break;
            
        case 0x20:
            switch (offset >> 2) {
                case 0: voice->lvol = chip->write_latch & 0xFFFF; break;
                case 1: voice->rvol = chip->write_latch & 0xFFFF; break;
                case 2: voice->lvramp = (chip->write_latch >> 8) & 0xFF; break;
                case 3: voice->rvramp = (chip->write_latch >> 8) & 0xFF; break;
            }
            break;
            
        case 0x40:
            switch (offset >> 2) {
                case 0: voice->k2 = chip->write_latch & 0xFFFF; break;
                case 1: voice->k2ramp = (chip->write_latch >> 8) & 0xFF; break;
                case 2: voice->k1 = chip->write_latch & 0xFFFF; break;
                case 3: voice->k1ramp = (chip->write_latch >> 8) & 0xFF; break;
            }
            break;
    }

    chip->write_latch = 0;
}

static UINT8 es5506_read(void* info, UINT8 offset) {
    ES5506_Chip* chip = (ES5506_Chip*)info;
    ES5506_Voice* voice = &chip->voice[chip->current_page & 0x1F];
    UINT32 value = 0;

    switch (chip->current_page) {
        case 0x00:
            switch (offset >> 2) {
                case 0: value = voice->control; break;
                case 1: value = voice->freqcount; break;
                case 2: value = voice->start; break;
                case 3: value = voice->end; break;
                case 4: value = voice->accum; break;
                case 5: value = voice->o4n1 & 0x3FFFF; break;
                case 6: value = voice->o3n1 & 0x3FFFF; break;
                case 7: value = voice->o3n2 & 0x3FFFF; break;
            }
            break;
            
        case 0x20:
            switch (offset >> 2) {
                case 0: value = voice->lvol; break;
                case 1: value = voice->rvol; break;
                case 2: value = voice->lvramp << 8; break;
                case 3: value = voice->rvramp << 8; break;
            }
            break;
            
        case 0x40:
            switch (offset >> 2) {
                case 0: value = voice->k2; break;
                case 1: value = voice->k2ramp << 8; break;
                case 2: value = voice->k1; break;
                case 3: value = voice->k1ramp << 8; break;
            }
            break;
    }

    return (value >> (24 - (offset & 3)*8)) & 0xFF;
}

static void es5506_write_rom(void* info, UINT32 offset, UINT32 length, const UINT8* data) {
    ES5506_Chip* chip = (ES5506_Chip*)info;
    UINT8 region = (offset >> 28) & 0x03;
    UINT8 is8bit = (offset >> 31) & 0x01;
    offset &= 0x0FFFFFFF;

    if (is8bit) {
        offset *= 2;
        length *= 2;
        if (chip->region_size[region] < offset + length) {
            chip->region_base[region] = realloc(chip->region_base[region], offset + length);
            chip->region_size[region] = offset + length;
        }
        for (UINT32 i = 0; i < length/2; i++)
            chip->region_base[region][offset/2 + i] = data[i] << 8;
    } else {
        if (chip->region_size[region] < offset + length) {
            chip->region_base[region] = realloc(chip->region_base[region], offset + length);
            chip->region_size[region] = offset + length;
        }
        memcpy(chip->region_base[region] + offset/2, data, length);
    }
}

static void es5506_set_mute_mask(void* info, UINT32 MuteMask) {
    ES5506_Chip* chip = (ES5506_Chip*)info;
    for (int i = 0; i < MAX_VOICES; i++)
        chip->voice[i].Muted = (MuteMask >> i) & 1;
}

static void es5506_set_srchg_cb(void* info, DEVCB_SRATE_CHG CallbackFunc, void* DataPtr) {
    ES5506_Chip* chip = (ES5506_Chip*)info;
    chip->SmpRateFunc = CallbackFunc;
    chip->SmpRateData = DataPtr;
}

static void compute_tables(ES5506_Chip* chip) {
    chip->ulaw_lookup = malloc((1 << ULAW_MAXBITS) * sizeof(INT16));
    for (int i = 0; i < (1 << ULAW_MAXBITS); i++) {
        UINT16 rawval = (i << (16 - ULAW_MAXBITS)) | (1 << (15 - ULAW_MAXBITS));
        UINT8 exponent = rawval >> 13;
        UINT32 mantissa = (rawval << 3) & 0xffff;
        chip->ulaw_lookup[i] = (exponent == 0) ? 
            (INT16)(mantissa >> 7) : 
            (INT16)(((mantissa >> 1) | (~mantissa & 0x8000)) >> (7 - exponent));
    }

    chip->volume_lookup = malloc(4096 * sizeof(UINT16));
    for (int i = 0; i < 4096; i++) {
        UINT8 exponent = i >> 8;
        UINT32 mantissa = (i & 0xff) | 0x100;
        chip->volume_lookup[i] = (mantissa << 11) >> (20 - exponent);
    }
}
