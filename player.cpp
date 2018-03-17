#ifdef _WIN32
//#define _WIN32_WINNT	0x500	// for GetConsoleWindow()
#include <windows.h>
#ifdef _DEBUG
#include <crtdbg.h>
#endif
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <vector>
#include <string>

#ifdef _WIN32
extern "C" int __cdecl _getch(void);	// from conio.h
extern "C" int __cdecl _kbhit(void);
#else
#include <unistd.h>		// for STDIN_FILENO and usleep()
#include <termios.h>
#include <sys/time.h>	// for struct timeval in _kbhit()
#define	Sleep(msec)	usleep(msec * 1000)
#endif

#include <common_def.h>
#include "player/playerbase.hpp"
#include "player/s98player.hpp"
#include "player/droplayer.hpp"
#include "audio/AudioStream.h"
#include "audio/AudioStream_SpcDrvFuns.h"
#include "emu/Resampler.h"


int main(int argc, char* argv[]);
static UINT8 GetPlayerForFile(const char* fileName, PlayerBase** retPlayer);
static const char* GetFileTitle(const char* filePath);
static UINT32 CalcCurrentVolume(UINT32 playbackSmpl);
static UINT32 FillBuffer(void* drvStruct, void* userParam, UINT32 bufSize, void* Data);
static UINT8 FilePlayCallback(PlayerBase* player, void* userParam, UINT8 evtType, void* evtParam);
static UINT32 GetNthAudioDriver(UINT8 adrvType, INT32 drvNumber);
static UINT8 InitAudioSystem(void);
static UINT8 DeinitAudioSystem(void);
static UINT8 StartAudioDevice(void);
static UINT8 StopAudioDevice(void);
static UINT8 StartDiskWriter(const char* fileName);
static UINT8 StopDiskWriter(void);
#ifndef _WIN32
static void changemode(UINT8 noEcho);
static int _kbhit(void);
#define	_getch	getchar
#endif


static UINT32 smplSize;
static void* audDrv;
static void* audDrvLog;
static UINT32 smplAlloc;
static WAVE_32BS* smplData;
static volatile bool canRender;
static volatile bool isRendering;
static volatile bool renderDoWait;

static UINT32 sampleRate = 44100;
static UINT32 maxLoops = 2;
static volatile UINT8 playState;

static UINT32 idWavOut;
static UINT32 idWavOutDev;
static UINT32 idWavWrt;

static INT32 AudioOutDrv = 1;
static INT32 WaveWrtDrv = -1;

static UINT32 masterVol = 0x10000;	// fixed point 16.16
static UINT32 fadeSmplStart;
static UINT32 fadeSmplTime;

int main(int argc, char* argv[])
{
	int argbase;
	UINT8 retVal;
	PlayerBase* player;
	int curSong;
	
	if (argc < 2)
	{
		printf("Usage: %s inputfile\n", argv[0]);
		return 0;
	}
	argbase = 1;
	
	retVal = InitAudioSystem();
	if (retVal)
		return 1;
	retVal = StartAudioDevice();
	if (retVal)
	{
		DeinitAudioSystem();
		return 1;
	}
	playState = 0x00;
	
	for (curSong = argbase; curSong < argc; curSong ++)
	{
	
	printf("Loading %s ...  ", GetFileTitle(argv[curSong]));
	fflush(stdout);
	player = NULL;
	retVal = GetPlayerForFile(argv[curSong], &player);
	if (retVal)
	{
		if (player != NULL)
			delete player;
		printf("Error 0x%02X loading S98 file!\n", retVal);
		continue;
	}
	player->SetCallback(&FilePlayCallback, NULL);
	
	if (player->GetPlayerType() == FCC_S98)
	{
		S98Player* s98play = dynamic_cast<S98Player*>(player);
		const S98_HEADER* s98hdr = s98play->GetFileHeader();
		const char* s98Title = player->GetSongTitle();
		
		printf("S98 v%u, Total Length: %.2f s, Loop Length: %.2f s, Tick Rate: %u/%u", s98hdr->fileVer,
				player->Tick2Second(player->GetTotalTicks()), player->Tick2Second(player->GetLoopTicks()),
				s98hdr->tickMult, s98hdr->tickDiv);
		if (s98Title != NULL)
			printf("\nSong Title: %s", s98Title);
	}
	else if (player->GetPlayerType() == FCC_DRO)
	{
		DROPlayer* droplay = dynamic_cast<DROPlayer*>(player);
		const DRO_HEADER* drohdr = droplay->GetFileHeader();
		const char* hwType;
		
		if (drohdr->hwType == 0)
			hwType = "OPL2";
		else if (drohdr->hwType == 1)
			hwType = "DualOPL2";
		else if (drohdr->hwType == 2)
			hwType = "OPL3";
		else
			hwType = "unknown";
		printf("DRO v%u, Total Length: %.2f s, HW Type: %s", drohdr->verMajor,
				player->Tick2Second(player->GetTotalTicks()), hwType);
	}
	putchar('\n');
	
	isRendering = false;
	AudioDrv_SetCallback(audDrv, FillBuffer, player);
	player->SetSampleRate(sampleRate);
	player->Start();
	fadeSmplTime = player->GetSampleRate() * 4;
	fadeSmplStart = (UINT32)-1;
	
	StartDiskWriter("waveOut.wav");
	renderDoWait = false;
	canRender = true;
#ifndef _WIN32
	changemode(1);
#endif
	playState &= ~PLAYSTATE_END;
	while(! (playState & PLAYSTATE_END))
	{
		if (! (playState & PLAYSTATE_PAUSE))
		{
			printf("Playing %.2f / %.2f ...   \r", player->Sample2Second(player->GetCurSample()),
					player->Tick2Second(player->GetTotalPlayTicks(maxLoops)));
			fflush(stdout);
		}
		Sleep(50);
		
		if (_kbhit())
		{
			int inkey = _getch();
			int letter = toupper(inkey);
			
			if (letter == ' ' || letter == 'P')
			{
				playState ^= PLAYSTATE_PAUSE;
				if (playState & PLAYSTATE_PAUSE)
					AudioDrv_Pause(audDrv);
				else
					AudioDrv_Resume(audDrv);
			}
			else if (letter == 'R')	// restart
			{
				renderDoWait = true;
				while(isRendering)
					Sleep(1);
				player->Reset();
				renderDoWait = false;
			}
			else if (letter == 'B')	// previous file
			{
				if (curSong > argbase)
				{
					playState |= PLAYSTATE_END;
					curSong -= 2;
				}
			}
			else if (letter == 'N')	// next file
			{
				if (curSong + 1 < argc)
					playState |= PLAYSTATE_END;
			}
			else if (inkey == 0x1B || letter == 'Q')	// quit
			{
				playState |= PLAYSTATE_END;
				curSong = argc - 1;
			}
			else if (letter == 'F')	// fade out
			{
				fadeSmplStart = player->GetCurSample();
			}
		}
	}
#ifndef _WIN32
	changemode(0);
#endif
	renderDoWait = true;
	while(isRendering)
		Sleep(1);	// wait for render thread to finish
	StopDiskWriter();
	
	player->Stop();
	player->UnloadFile();
	
	}	// end for(curSong)
	canRender = false;
	renderDoWait = false;
	
	StopAudioDevice();
	DeinitAudioSystem();
	printf("Done.\n");
	
#if defined(_DEBUG) && (_MSC_VER >= 1400)
	// doesn't work well with C++ containers
	//if (_CrtDumpMemoryLeaks())
	//	_getch();
#endif
	
	return 0;
}

static UINT8 GetPlayerForFile(const char* fileName, PlayerBase** retPlayer)
{
	UINT8 retVal;
	PlayerBase* player;
	
	player = new S98Player;
	retVal = player->LoadFile(fileName);
	if (retVal < 0x80)
	{
		*retPlayer = player;
		return retVal;
	}
	delete player;
	
	player = new DROPlayer;
	retVal = player->LoadFile(fileName);
	if (retVal < 0x80)
	{
		*retPlayer = player;
		return retVal;
	}
	delete player;
	
	return 0xFF;
}

static const char* GetFileTitle(const char* filePath)
{
	const char* dirSep1;
	const char* dirSep2;
	
	dirSep1 = strrchr(filePath, '/');
	dirSep2 = strrchr(filePath, '\\');
	if (dirSep2 > dirSep1)
		dirSep1 = dirSep2;

	return (dirSep1 == NULL) ? filePath : (dirSep1 + 1);
}

#if 1
#define VOLCALC64
#define VOL_BITS	16	// use .X fixed point for working volume
#else
#define VOL_BITS	8	// use .X fixed point for working volume
#endif
#define VOL_SHIFT	(16 - VOL_BITS)	// shift for master volume -> working volume

// Pre- and post-shifts are used to reduce make the calculations as accurate as possible
// without causing the sample data (likely 24 bits) to overflow while applying the volume gain.
// Smaller values for VOL_PRESH are more accurate, but have a higher risk of overflows during calculations.
// (24 + VOL_POSTSH) must NOT be larger than 31
#define VOL_PRESH	4	// sample data pre-shift
#define VOL_POSTSH	(VOL_BITS - VOL_PRESH)	// post-shift after volume multiplication

static UINT32 CalcCurrentVolume(UINT32 playbackSmpl)
{
	UINT32 curVol;	// 16.16 fixed point
	
	// 1. master volume
	curVol = masterVol;
	
	// 2. apply fade-out factor
	if (playbackSmpl >= fadeSmplStart)
	{
		UINT32 fadeSmpls;
		UINT64 fadeVol;	// 64 bit for less type casts when doing multiplications with .16 fixed point
		
		fadeSmpls = playbackSmpl - fadeSmplStart;
		if (fadeSmpls >= fadeSmplTime)
			return 0x0000;	// going beyond fade time -> volume 0
		
		fadeVol = (UINT64)fadeSmpls * 0x10000 / fadeSmplTime;
		fadeVol = 0x10000 - fadeVol;	// fade from full volume to silence
		fadeVol = fadeVol * fadeVol;	// logarithmic fading sounds nicer
		curVol = (UINT32)((fadeVol * curVol) >> 32);
	}
	
	return curVol;
}

static UINT32 FillBuffer(void* drvStruct, void* userParam, UINT32 bufSize, void* data)
{
	PlayerBase* player = (PlayerBase*)userParam;
	UINT32 basePbSmpl;
	UINT32 smplCount;
	UINT32 smplRendered;
	INT16* SmplPtr16;
	UINT32 curSmpl;
	WAVE_32BS fnlSmpl;	// final sample value
	INT32 curVolume;
	
	smplCount = bufSize / smplSize;
	if (! smplCount)
		return 0;
	
	while(renderDoWait)
		Sleep(1);	// pause the thread while the main thread wants to do some actions
	if (! canRender)
	{
		memset(data, 0x00, smplCount * smplSize);
		return smplCount * smplSize;
	}
	
	isRendering = true;
	if (smplCount > smplAlloc)
		smplCount = smplAlloc;
	memset(smplData, 0, smplCount * sizeof(WAVE_32BS));
	basePbSmpl = player->GetCurSample();
	smplRendered = player->Render(smplCount, smplData);
	smplCount = smplRendered;
	
	curVolume = (INT32)CalcCurrentVolume(basePbSmpl) >> VOL_SHIFT;
	SmplPtr16 = (INT16*)data;
	for (curSmpl = 0; curSmpl < smplCount; curSmpl ++, basePbSmpl ++, SmplPtr16 += 2)
	{
		if (basePbSmpl >= fadeSmplStart)
		{
			UINT32 fadeSmpls;
			
			fadeSmpls = basePbSmpl - fadeSmplStart;
			if (fadeSmpls >= fadeSmplTime && ! (playState & PLAYSTATE_END))
			{
				playState |= PLAYSTATE_END;
				break;
			}
			
			curVolume = (INT32)CalcCurrentVolume(basePbSmpl) >> VOL_SHIFT;
		}
		
		// Input is about 24 bits (some cores might output a bit more)
		fnlSmpl = smplData[curSmpl];
		
#ifdef VOLCALC64
		fnlSmpl.L = (INT32)( ((INT64)fnlSmpl.L * curVolume) >> VOL_BITS );
		fnlSmpl.R = (INT32)( ((INT64)fnlSmpl.R * curVolume) >> VOL_BITS );
#else
		fnlSmpl.L = ((fnlSmpl.L >> VOL_PRESH) * curVolume) >> VOL_POSTSH;
		fnlSmpl.R = ((fnlSmpl.R >> VOL_PRESH) * curVolume) >> VOL_POSTSH;
#endif
		
		fnlSmpl.L >>= 8;	// 24 bit -> 16 bit
		fnlSmpl.R >>= 8;
		if (fnlSmpl.L < -0x8000)
			fnlSmpl.L = -0x8000;
		else if (fnlSmpl.L > +0x7FFF)
			fnlSmpl.L = +0x7FFF;
		if (fnlSmpl.R < -0x8000)
			fnlSmpl.R = -0x8000;
		else if (fnlSmpl.R > +0x7FFF)
			fnlSmpl.R = +0x7FFF;
		SmplPtr16[0] = (INT16)fnlSmpl.L;
		SmplPtr16[1] = (INT16)fnlSmpl.R;
	}
	isRendering = false;
	
	return curSmpl * smplSize;
}

static UINT8 FilePlayCallback(PlayerBase* player, void* userParam, UINT8 evtType, void* evtParam)
{
	switch(evtType)
	{
	case PLREVT_START:
		//printf("S98 playback started.\n");
		break;
	case PLREVT_STOP:
		//printf("S98 playback stopped.\n");
		break;
	case PLREVT_LOOP:
		{
			UINT32* curLoop = (UINT32*)evtParam;
			if (*curLoop >= maxLoops)
			{
				if (fadeSmplTime)
				{
					if (fadeSmplStart == (UINT32)-1)
						fadeSmplStart = player->GetCurSample();
				}
				else
				{
					printf("Loop End.\n");
					playState |= PLAYSTATE_END;
					return 0x01;
				}
			}
			printf("Loop %u.\n", 1 + *curLoop);
		}
		break;
	case PLREVT_END:
		playState |= PLAYSTATE_END;
		printf("Song End.\n");
		break;
	}
	return 0x00;
}

static UINT32 GetNthAudioDriver(UINT8 adrvType, INT32 drvNumber)
{
	UINT32 drvCount;
	UINT32 curDrv;
	INT32 typedDrv;
	AUDDRV_INFO* drvInfo;
	
	// go through all audio drivers get the ID of the requested Output/Disk Writer driver
	drvCount = Audio_GetDriverCount();
	for (typedDrv = 0, curDrv = 0; curDrv < drvCount; curDrv ++)
	{
		Audio_GetDriverInfo(curDrv, &drvInfo);
		if (drvInfo->drvType == adrvType)
		{
			if (typedDrv == drvNumber)
				return curDrv;
			typedDrv ++;
		}
	}
	
	return (UINT32)-1;
}

// initialize audio system and search for requested audio drivers
static UINT8 InitAudioSystem(void)
{
	AUDDRV_INFO* drvInfo;
	UINT8 retVal;
	
	printf("Opening Audio Device ...\n");
	retVal = Audio_Init();
	if (retVal == AERR_NODRVS)
		return retVal;
	
	idWavOut = GetNthAudioDriver(ADRVTYPE_OUT, AudioOutDrv);
	idWavOutDev = 0;	// default device
	if (idWavOut == (UINT32)-1)
	{
		printf("Requested Audio Output driver not found!\n");
		Audio_Deinit();
		return AERR_NODRVS;
	}
	idWavWrt = GetNthAudioDriver(ADRVTYPE_DISK, WaveWrtDrv);
	
	Audio_GetDriverInfo(idWavOut, &drvInfo);
	printf("Using driver %s.\n", drvInfo->drvName);
	retVal = AudioDrv_Init(idWavOut, &audDrv);
	if (retVal)
	{
		printf("WaveOut: Drv Init Error: %02X\n", retVal);
		Audio_Deinit();
		return retVal;
	}
	
	audDrvLog = NULL;
	if (idWavWrt != (UINT32)-1)
	{
		retVal = AudioDrv_Init(idWavWrt, &audDrvLog);
		if (retVal)
			audDrvLog = NULL;
	}
	
	return 0x00;
}

static UINT8 DeinitAudioSystem(void)
{
	UINT8 retVal;
	
	retVal = AudioDrv_Deinit(&audDrv);
	if (audDrvLog != NULL)
		AudioDrv_Deinit(&audDrvLog);
	Audio_Deinit();
	
	return retVal;
}

static UINT8 StartAudioDevice(void)
{
	AUDIO_OPTS* opts;
	UINT8 retVal;
	
	opts = AudioDrv_GetOptions(audDrv);
	opts->sampleRate = sampleRate;
	opts->numChannels = 2;
	opts->numBitsPerSmpl = 16;
	smplSize = opts->numChannels * opts->numBitsPerSmpl / 8;
	
	canRender = false;
	printf("Opening Device %u ...\n", idWavOutDev);
	retVal = AudioDrv_Start(audDrv, idWavOutDev);
	if (retVal)
	{
		printf("Dev Init Error: %02X\n", retVal);
		return retVal;
	}
	
	smplAlloc = AudioDrv_GetBufferSize(audDrv) / smplSize;
	smplData = (WAVE_32BS*)malloc(smplAlloc * sizeof(WAVE_32BS));
	
	return 0x00;
}

static UINT8 StopAudioDevice(void)
{
	UINT8 retVal;
	
	retVal = AudioDrv_Stop(audDrv);
	free(smplData);	smplData = NULL;
	
	return retVal;
}

static UINT8 StartDiskWriter(const char* fileName)
{
	AUDIO_OPTS* opts;
	UINT8 retVal;
	
	if (audDrvLog == NULL)
		return 0x00;
	
	opts = AudioDrv_GetOptions(audDrvLog);
	*opts = *AudioDrv_GetOptions(audDrv);
	
	WavWrt_SetFileName(AudioDrv_GetDrvData(audDrvLog), fileName);
	retVal = AudioDrv_Start(audDrvLog, 0);
	if (! retVal)
		AudioDrv_DataForward_Add(audDrv, audDrvLog);
	return retVal;
}

static UINT8 StopDiskWriter(void)
{
	if (audDrvLog == NULL)
		return 0x00;
	
	AudioDrv_DataForward_Remove(audDrv, audDrvLog);
	return AudioDrv_Stop(audDrvLog);
}


#ifndef _WIN32
static struct termios oldterm;
static UINT8 termmode = 0xFF;

static void changemode(UINT8 noEcho)
{
	if (termmode == 0xFF)
	{
		tcgetattr(STDIN_FILENO, &oldterm);
		termmode = 0;
	}
	if (termmode == noEcho)
		return;
	
	if (noEcho)
	{
		struct termios newterm;
		newterm = oldterm;
		newterm.c_lflag &= ~(ICANON | ECHO);
		tcsetattr(STDIN_FILENO, TCSANOW, &newterm);
		termmode = 1;
	}
	else
	{
		tcsetattr(STDIN_FILENO, TCSANOW, &oldterm);
		termmode = 0;
	}
	
	return;
}

static int _kbhit(void)
{
	struct timeval tv;
	fd_set rdfs;
	
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	
	FD_ZERO(&rdfs);
	FD_SET(STDIN_FILENO, &rdfs);
	select(STDIN_FILENO + 1, &rdfs, NULL, NULL, &tv);
	
	return FD_ISSET(STDIN_FILENO, &rdfs);;
}
#endif
