//
// CAIOwaveConfig.hpp
//




#if !defined(__CAIOwaveConfig_hpp)
#define __CAIOwaveConfig_hpp




// General description of waveformat used in files and streams.

class CAIOwaveConfig
{
public:
	int m_nFramesPerSecond;
	int m_nChannels;	// Number of channels. mono, stereo, quad etc.
	int m_nBitsPerMonoSample;
	int m_nBytes;
	int m_nFrames;		// Number of samples whether mono, stereo etc.
	short* m_start;

};




#endif
