/**********************************************************************************************
 *
 *   Ensoniq ES5505/6 driver
 *   by Aaron Giles
 *
 **********************************************************************************************/

#pragma once

#ifndef __ES5506_H__
#define __ES5506_H__


UINT8 es550x_r(UINT8 ChipID, offs_t offset);
void es550x_w(UINT8 ChipID, offs_t offset, UINT8 data);
void es550x_w16(UINT8 ChipID, offs_t offset, UINT16 data);

void es5506_update(UINT8 ChipID, stream_sample_t **outputs, int samples);
int device_start_es5506(UINT8 ChipID, int clock, int channels);
void device_stop_es5506(UINT8 ChipID);
void device_reset_es5506(UINT8 ChipID);

void es5506_write_rom(UINT8 ChipID, offs_t ROMSize, offs_t DataStart, offs_t DataLength,
					   const UINT8* ROMData);

void es5506_set_mute_mask(UINT8 ChipID, UINT32 MuteMask);
void es5506_set_srchg_cb(UINT8 ChipID, SRATE_CALLBACK CallbackFunc, void* DataPtr);


#endif /* __ES5506_H__ */
