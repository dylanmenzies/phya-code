//
// AIOloadWaveFile.cpp	
//
// Wrapper for wave file loading done in wave.c.
//

#include "wave.h"
#include "AIOloadWaveFile.hpp"

BYTE *pbData;		// Pointer to wave data.

int AIOfreeWaveFile()
{
	GlobalFree(pbData);
	return 0;
}

       
int AIOloadWaveFile( char* fileName, CAIOwaveConfig* config)
{
	WAVEFORMATEX *pwfxInfo;			
	WAVEFORMATEX **ppwfxInfo = &pwfxInfo;
	BYTE **ppbData = &pbData;
	UINT bSize;
	int hr;


	hr = WaveLoadFile((TCHAR*)fileName, &bSize, ppwfxInfo, ppbData);

	if (hr !=0) return -1;

	config->m_start = (short *) pbData;
	config->m_nBytes = bSize;
	config->m_nChannels = pwfxInfo->nChannels;
	config->m_nBitsPerMonoSample = pwfxInfo->wBitsPerSample;
	config->m_nFrames = bSize / (pwfxInfo->wBitsPerSample / 8)
											/ pwfxInfo->nChannels;
	config->m_nFramesPerSecond = pwfxInfo->nSamplesPerSec;

	return 0;
}

