// license:BSD-3-Clause
// copyright-holders:Aaron Giles, Paul Leaman, Miguel Angel Horna, libvgm contributors
/***************************************************************************

  Data East BSMT2000
  ==================

  Core for BSMT2000 sound chip, adapted for libvgm, by combining code from:
    - PinMAME (src/sound/bsmt2000.c, f88658fd)
    - MAME (src/devices/sound/bsmt2000.cpp/.h, 05eb28b6)
    - libvgm QSound core (emu/cores/qsound_mame.c, db7310dc)
  Interface and integration modeled after qsound_mame.c in libvgm.

  This file is self-contained for libvgm, and does not depend on MAME/PinMAME internals.

  The BSMT2000 is a custom TMS320C15 DSP with internal ROM and external sample ROM.
  It supports multiple PCM voices and a single ADPCM channel, with stereo output.

  TODO:
  - Expose all registers and ROM mapping
  - Implement correct reset/mode handling
  - More accurate interpolation
  - Properly handle all quirks from both MAME and PinMAME
  - Fine-tune external interface for libvgm conventions

***************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <math.h>

#include "../../stdtype.h"
#include "../EmuStructs.h"
#include "../SoundDevs.h"
#include "../EmuCores.h"
#include "../snddef.h"
#include "../EmuHelper.h"
#include "../logging.h"
#include "bsmt2000.h"

/* ==== Constants ==== */

#define BSMT2000_CLOCK        24000000    /* default 24MHz clock */
#define BSMT2000_CHANNELS     12          /* up to 12 PCM voices, plus ADPCM */
#define BSMT2000_ADPCM_INDEX  12          /* 0..11 = PCM, 12 = ADPCM */
#define BSMT2000_REGS         7           /* registers per voice */

#define BSMT2000_MAX_VOICES   (BSMT2000_CHANNELS + 1)  /* 12 PCM + 1 ADPCM */
#define BSMT2000_SAMPLE_CHUNK 10000

#define BSMT2000_ROM_BANKSIZE 0x10000     /* 64k per bank */
#define BSMT2000_REG_CURRPOS  0
#define BSMT2000_REG_RATE     1
#define BSMT2000_REG_LOOPEND  2
#define BSMT2000_REG_LOOPSTART 3
#define BSMT2000_REG_BANK     4
#define BSMT2000_REG_RIGHTVOL 5
#define BSMT2000_REG_LEFTVOL  6
#define BSMT2000_REG_TOTAL    7

/* ==== Register Map ==== */
static const UINT8 regmap[8][7] = {
    { 0x00, 0x18, 0x24, 0x30, 0x3c, 0x48, 0xff },
    { 0x00, 0x16, 0x21, 0x2c, 0x37, 0x42, 0x4d },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x18, 0x24, 0x30, 0x3c, 0x54, 0x60 },
    { 0x00, 0x10, 0x18, 0x20, 0x28, 0x38, 0x40 },
    { 0x00, 0x12, 0x1b, 0x24, 0x2d, 0x3f, 0x48 }
};

/* ==== Internal Voice State ==== */
typedef struct
{
    UINT16 reg[BSMT2000_REG_TOTAL]; // 7 registers per voice
    UINT16 fraction;                // sample position fraction
} bsmt2000_voice;

/* ==== Main Chip State ==== */
typedef struct _bsmt2000_state bsmt2000_state;
struct _bsmt2000_state
{
    DEV_DATA _devData;
    DEV_LOGGER logger;

    // Sample ROM
    INT8 *sample_rom;
    UINT32 sample_rom_length;
    UINT32 sample_rom_mask;
    UINT32 total_banks;

    // Voices and ADPCM
    bsmt2000_voice voice[BSMT2000_MAX_VOICES];
    UINT8 voices;         // actual number of voices (usually 11 or 12)
    UINT8 stereo;         // stereo output enabled?
    UINT8 adpcm;          // ADPCM enabled?
    UINT8 mode;           // current mode (0,1,5,6,7)
    UINT8 last_register;  // last register written
    UINT8 right_volume_set;  // Track if right volume was set
    // ADPCM state
    INT32 adpcm_current;
    INT32 adpcm_delta_n;
    UINT16 adpcm_77;      // for special volume case

    // Output sample rate
    double sample_rate;

    // Mute mask
    UINT8 Muted[BSMT2000_MAX_VOICES];

    // Misc
    // (can add more fields as needed)
};

/* ==== Prototypes ==== */
static void bsmt2000_update(void *param, UINT32 samples, DEV_SMPL **outputs);
static UINT8 device_start_bsmt2000(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf);
static void device_stop_bsmt2000(void *info);
static void device_reset_bsmt2000(void *info);

static void bsmt2000_w(void *info, UINT8 offset, UINT8 data);
static UINT8 bsmt2000_r(void *info, UINT8 offset);
static void bsmt2000_write_data(void *info, UINT8 address, UINT16 data);

static void bsmt2000_alloc_rom(void* info, UINT32 memsize);
static void bsmt2000_write_rom(void *info, UINT32 offset, UINT32 length, const UINT8* data);
static void bsmt2000_set_mute_mask(void *info, UINT32 MuteMask);
static UINT32 bsmt2000_get_mute_mask(void *info);
static void bsmt2000_set_log_cb(void* info, DEVCB_LOG func, void* param);

/* ==== Device Function Table ==== */
static DEVDEF_RWFUNC devFunc[] =
{
    {RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, bsmt2000_w},
    {RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, bsmt2000_r},
    {RWF_REGISTER | RWF_QUICKWRITE, DEVRW_A8D16, 0, bsmt2000_write_data},
    {RWF_MEMORY | RWF_WRITE, DEVRW_BLOCK, 0, bsmt2000_write_rom},
    {RWF_MEMORY | RWF_WRITE, DEVRW_MEMSIZE, 0, bsmt2000_alloc_rom},
    {RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, bsmt2000_set_mute_mask},
    {0x00, 0x00, 0, NULL}
};

/* ==== Device Definition ==== */
DEV_DEF devDef =
{
    "BSMT2000", "MAME", FCC_MAME,

    device_start_bsmt2000,
    device_stop_bsmt2000,
    device_reset_bsmt2000,
    bsmt2000_update,

    NULL, // SetOptionBits
    bsmt2000_set_mute_mask,
    NULL, // SetPanning
    NULL, // SetSampleRateChangeCallback
    bsmt2000_set_log_cb, // SetLoggingCallback
    NULL, // LinkDevice

    devFunc,    // rwFuncs
};

static const char* DeviceName(const DEV_GEN_CFG* devCfg)
{
	return "BSMT2000";
}

static UINT16 DeviceChannels(const DEV_GEN_CFG* devCfg)
{
	return BSMT2000_MAX_VOICES;
}

static const char** DeviceChannelNames(const DEV_GEN_CFG* devCfg)
{
	static const char* names[BSMT2000_MAX_VOICES] =
	{
		"PCM 1", "PCM 2", "PCM 3", "PCM 4", "PCM 5", "PCM 6", "PCM 7", "PCM 8",
		"PCM 9", "PCM 10", "PCM 11", "PCM 12",
		"ADPCM",
	};
}

const DEV_DECL sndDev_BSMT2000 =
{
	DEVID_BSMT2000,
	DeviceName,
	DeviceChannels,
	DeviceChannelNames,
	{	// cores
		&devDef,
		NULL
	}
};

/* ==== Utility Macros ==== */
#define MIN(x,y) ((x)<(y)?(x):(y))

/* ==== Internal Functions ==== */

static void init_voice(bsmt2000_voice *voice)
{
    memset(voice, 0, sizeof(*voice));
    voice->reg[BSMT2000_REG_LEFTVOL] = 0x7fff;
    voice->reg[BSMT2000_REG_RIGHTVOL] = 0x7fff;
}
static void init_all_voices(bsmt2000_state *chip)
{
    int i;
    for (i = 0; i < BSMT2000_MAX_VOICES; i++)
        init_voice(&chip->voice[i]);
}

/* ==== Device Start ==== */
static UINT8 device_start_bsmt2000(const DEV_GEN_CFG* cfg, DEV_INFO* retDevInf)
{
    bsmt2000_state *chip;
    chip = (bsmt2000_state *)calloc(1, sizeof(bsmt2000_state));
    if (!chip)
        return 0xFF;

    chip->sample_rom = NULL;
    chip->sample_rom_length = 0x00;
    chip->sample_rom_mask = 0x00;
    chip->total_banks = 0;

    chip->voices = 12;       // default: 12 PCM (may be overridden later)
    chip->stereo = 1;        // default: stereo enabled
    chip->adpcm = 1;         // default: ADPCM enabled

    chip->mode = 1;
    chip->sample_rate = cfg->clock / 1000.0;
    chip->last_register = 1; // Mode 1
    chip->adpcm_current = 0;
    chip->adpcm_delta_n = 10;
    chip->adpcm_77 = 0;

    init_all_voices(chip);
    bsmt2000_set_mute_mask(chip, 0x0000);

    chip->_devData.chipInf = chip;
    INIT_DEVINF(retDevInf, &chip->_devData, (UINT32)chip->sample_rate, &devDef);

    return 0x00;
}

/* ==== Device Stop ==== */
static void device_stop_bsmt2000(void *info)
{
    bsmt2000_state *chip = (bsmt2000_state *)info;
    free(chip->sample_rom);
    free(chip);
}

/* ==== Device Reset ==== */
static void device_reset_bsmt2000(void *info) {
    bsmt2000_state *chip = (bsmt2000_state *)info;
    UINT32 muteMask;

    muteMask = bsmt2000_get_mute_mask(chip);
    init_all_voices(chip);
    chip->adpcm_current = 0;
    chip->adpcm_delta_n = 10;
    chip->adpcm_77 = 0;
    chip->right_volume_set = 0; // Reset volume flag
    // Reset mode to last_register?
    chip->mode = chip->last_register;
    bsmt2000_set_mute_mask(chip, muteMask);
}

/* ==== Register/Command Interface ==== */

static void bsmt2000_w(void *info, UINT8 offset, UINT8 data)
{
    bsmt2000_state *chip = (bsmt2000_state *)info;
    // BSMT2000 is 16-bit, so emulate register/data port scheme
    static UINT16 latch = 0;
    switch (offset)
    {
    case 0:
        latch = (latch & 0x00ff) | (data << 8);
        break;
    case 1:
        latch = (latch & 0xff00) | data;
        break;
    case 2:
        bsmt2000_write_data(chip, data, latch);
        break;
    default:
        emu_logf(&chip->logger, DEVLOG_DEBUG, "unexpected bsmt2000 write to offset %d == %02X\n", offset, data);
        break;
    }
}

static UINT8 bsmt2000_r(void *info, UINT8 offset)
{
    /* Always ready (bit 7 = 1) */
    return 0x80;
}

/* ==== Data/Voice Register Write ==== */

static void bsmt2000_write_data(void *info, UINT8 address, UINT16 data) {
    bsmt2000_state *chip = (bsmt2000_state *)info;
    bsmt2000_voice *voice;
    int voice_index, regindex;
    UINT8 mode = chip->mode;
    chip->last_register = address;

    if (address >= 0x80) return;

    // Handle PCM voices (address < 0x6d)
    if (address < 0x6d) {
        regindex = 6;
        while (address < regmap[mode][regindex])
            regindex--;

        voice_index = address - regmap[mode][regindex];
        if (voice_index >= chip->voices) return;
        voice = &chip->voice[voice_index];
        if (regindex < 0 || regindex >= BSMT2000_REG_TOTAL) return;

        // Apply DE Rom banking fix for REG_BANK
        if (regindex == BSMT2000_REG_BANK) {
            UINT16 temp = (data & 0x07) | ((data & 0x18) << 1) | ((data & 0x20) >> 2);
            data = temp;
        }

        voice->reg[regindex] = data;

        if (regindex == BSMT2000_REG_RIGHTVOL)
            chip->right_volume_set = 1;

        if (regindex == BSMT2000_REG_CURRPOS)
            voice->fraction = 0;
    } 
    // Handle ADPCM voice registers (address >= 0x6d)
    else {
        voice = &chip->voice[BSMT2000_ADPCM_INDEX];
        switch (address) {
            case 0x6d: // REG_LOOPEND
                voice->reg[BSMT2000_REG_LOOPEND] = data;
                break;
            case 0x6e: // Main right volume
                voice->reg[BSMT2000_REG_RIGHTVOL] = data;
                chip->right_volume_set = 1;
                break;
            case 0x6f: // REG_BANK (ADPCM)
                data = (data & 0x07) | ((data & 0x18) << 1) | ((data & 0x20) >> 2);
                voice->reg[BSMT2000_REG_BANK] = data;
                break;
            case 0x73: // REG_RATE (ADPCM)
                voice->reg[BSMT2000_REG_RATE] = data;
                if (data != 0) {
                    chip->adpcm_current = 0;
                    chip->adpcm_delta_n = 10;
                }
                break;
            case 0x74: // Alternate right volume
                voice->reg[BSMT2000_REG_RIGHTVOL] = data;
                chip->right_volume_set = 1;
                break;
            case 0x75: // REG_CURRPOS (ADPCM)
                voice->reg[BSMT2000_REG_CURRPOS] = data;
                voice->fraction = 0;
                break;
            case 0x77: // ADPCM command 0x77
                chip->adpcm_77 = data;
                break;
            case 0x70: // Main left volume
            case 0x78: // Alternate left volume
                if (chip->stereo)
                    voice->reg[BSMT2000_REG_LEFTVOL] = data;
                break;
            default:
                break;
        }
    }
}

/* ==== Sample ROM Handling ==== */
static void bsmt2000_alloc_rom(void* info, UINT32 memsize)
{
    bsmt2000_state* chip = (bsmt2000_state *)info;
    if (chip->sample_rom_length == memsize)
        return;
    chip->sample_rom = (INT8*)realloc(chip->sample_rom, memsize);
    chip->sample_rom_length = memsize;
    chip->sample_rom_mask = pow2_mask(memsize);
    chip->total_banks = memsize / BSMT2000_ROM_BANKSIZE;
    memset(chip->sample_rom, 0xFF, memsize);
}

static void bsmt2000_write_rom(void *info, UINT32 offset, UINT32 length, const UINT8* data)
{
    bsmt2000_state* chip = (bsmt2000_state *)info;
    if (offset > chip->sample_rom_length)
        return;
    if (offset + length > chip->sample_rom_length)
        length = chip->sample_rom_length - offset;
    memcpy(chip->sample_rom + offset, data, length);
}

/* ==== Mute Mask ==== */
static void bsmt2000_set_mute_mask(void *info, UINT32 MuteMask)
{
    bsmt2000_state* chip = (bsmt2000_state *)info;
    UINT8 CurChn;
    for (CurChn = 0; CurChn < BSMT2000_MAX_VOICES; CurChn++)
        chip->Muted[CurChn] = (MuteMask >> CurChn) & 0x01;
}

static UINT32 bsmt2000_get_mute_mask(void *info)
{
    bsmt2000_state* chip = (bsmt2000_state *)info;
    UINT32 muteMask = 0;
    UINT8 CurChn;
    for (CurChn = 0; CurChn < BSMT2000_MAX_VOICES; CurChn++)
        muteMask |= (chip->Muted[CurChn] << CurChn);
    return muteMask;
}

/* ==== Logging ==== */
static void bsmt2000_set_log_cb(void* info, DEVCB_LOG func, void* param)
{
    bsmt2000_state* chip = (bsmt2000_state *)info;
    dev_logger_set(&chip->logger, chip, func, param);
}

/* ==== Sound Update ==== */

static void bsmt2000_update(void *param, UINT32 samples, DEV_SMPL **outputs)
{
    bsmt2000_state *chip = (bsmt2000_state *)param;
    INT64 left[BSMT2000_SAMPLE_CHUNK], right[BSMT2000_SAMPLE_CHUNK];
    INT16 *ldest = (INT16*)outputs[0];
    INT16 *rdest = (INT16*)outputs[1];
    bsmt2000_voice *voice;
    int samp, v, length = MIN(samples, BSMT2000_SAMPLE_CHUNK);

    if (!chip->sample_rom || !chip->sample_rom_length)
    {
        memset(outputs[0], 0, samples * sizeof(*outputs[0]));
        memset(outputs[1], 0, samples * sizeof(*outputs[1]));
        return;
    }
    memset(left, 0, length * sizeof(left[0]));
    memset(right, 0, length * sizeof(right[0]));

    // PCM voices
    for (v = 0; v < chip->voices; v++)
    {
        if (chip->Muted[v])
            continue;
        voice = &chip->voice[v];
        if (voice->reg[BSMT2000_REG_BANK] >= chip->total_banks)
            continue;
        INT8 *base = chip->sample_rom + voice->reg[BSMT2000_REG_BANK] * BSMT2000_ROM_BANKSIZE;
        UINT32 rate = voice->reg[BSMT2000_REG_RATE];
		INT32 rvol = voice->reg[BSMT2000_REG_RIGHTVOL];
		INT32 lvol = chip->stereo ? voice->reg[BSMT2000_REG_LEFTVOL] : rvol;
		if (chip->stereo && !chip->right_volume_set) {
			rvol = lvol;
		}
		if (chip->adpcm_77 > 0 && rvol == 0 && lvol == 0) {
			rvol = lvol = chip->adpcm_77;
		}
        UINT32 pos = voice->reg[BSMT2000_REG_CURRPOS];
        UINT32 frac = voice->fraction;
        for (samp = 0; samp < length; samp++)
        {
            INT32 val1 = base[pos];
            INT32 sample = val1 << 8;
            left[samp]  += sample * lvol;
            right[samp] += sample * rvol;
            // Update position
            frac += rate;
            pos += frac >> 11;
            frac &= 0x7ff;
            // Loop
            if (pos >= voice->reg[BSMT2000_REG_LOOPEND])
            {
                pos += voice->reg[BSMT2000_REG_LOOPSTART] - voice->reg[BSMT2000_REG_LOOPEND];
                frac = 0;
            }
        }
        voice->reg[BSMT2000_REG_CURRPOS] = (UINT16)pos;
        voice->fraction = (UINT16)frac;
    }
    // ADPCM voice
    if (chip->adpcm && !chip->Muted[BSMT2000_ADPCM_INDEX])
    {
        voice = &chip->voice[BSMT2000_ADPCM_INDEX];
        if (voice->reg[BSMT2000_REG_BANK] < chip->total_banks && voice->reg[BSMT2000_REG_RATE])
        {
            INT8 *base = chip->sample_rom + voice->reg[BSMT2000_REG_BANK] * BSMT2000_ROM_BANKSIZE;
			INT32 rvol = voice->reg[BSMT2000_REG_RIGHTVOL];
			INT32 lvol = chip->stereo ? voice->reg[BSMT2000_REG_LEFTVOL] : rvol;
			if (chip->stereo && !chip->right_volume_set) {
				rvol = lvol;
			}
			if (chip->adpcm_77 > 0 && rvol == 0 && lvol == 0) {
				rvol = lvol = chip->adpcm_77;
			}
            UINT32 pos = voice->reg[BSMT2000_REG_CURRPOS];
            UINT32 frac = voice->fraction;
            for (samp = 0; samp < length && pos < voice->reg[BSMT2000_REG_LOOPEND]; samp++)
            {
                left[samp]  += chip->adpcm_current * (lvol * 2);
                right[samp] += chip->adpcm_current * (rvol * 2);
                frac++;
                if (frac == 6)
                {
                    pos++;
                    frac = 0;
                }
                if (frac == 1 || frac == 4)
                {
                    static const INT32 delta_tab[16] = { 154, 154, 128, 102, 77, 58, 58, 58, 58, 58, 58, 58, 77, 102, 128, 154 };
                    INT32 delta, value;
                    value = base[pos];
                    if (frac == 1)
                        value >>= 4;
                    value &= 0xF;
                    if (value & 0x8)
                        value |= 0xFFFFFFF0;
                    delta = chip->adpcm_delta_n * value;
                    if (value > 0)
                        delta += chip->adpcm_delta_n >> 1;
                    else
                        delta -= chip->adpcm_delta_n >> 1;
                    chip->adpcm_current += delta;
                    if (chip->adpcm_current > 32767)
                        chip->adpcm_current = 32767;
                    else if (chip->adpcm_current < -32768)
                        chip->adpcm_current = -32768;
                    chip->adpcm_delta_n = (chip->adpcm_delta_n * delta_tab[value+8]) >> 6;
                    if (chip->adpcm_delta_n > 2000)
                        chip->adpcm_delta_n = 2000;
                    else if (chip->adpcm_delta_n < 1)
                        chip->adpcm_delta_n = 1;
                }
            }
            voice->reg[BSMT2000_REG_CURRPOS] = (UINT16)pos;
            voice->fraction = (UINT16)frac;
            if (pos >= voice->reg[BSMT2000_REG_LOOPEND])
                voice->reg[BSMT2000_REG_RATE] = 0;
        }
    }
    // Output clamp and write
    for (samp = 0; samp < length; samp++)
    {
        INT64 l = (left[samp] >> 16);
        INT64 r = (right[samp] >> 16);
        if (l > 32767) l = 32767;
        else if (l < -32768) l = -32768;
        if (r > 32767) r = 32767;
        else if (r < -32768) r = -32768;
        ldest[samp] = (INT16)l;
        rdest[samp] = (INT16)r;
    }
}
