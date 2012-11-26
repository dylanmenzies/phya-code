//
// CAIOoutStream.hpp
//
// Audio IO interface abstraction, implemented for DirectSound.
//
// Should replace this with wrapper for RTaudio by Gary Scavone, with multi-platform support.
//	 
// Buffering occurs at three levels:
// The user passes the result of signal processing to CAIOoutStream->writeSamples( ), in a
// buffer of typically about 100 frames. Writes build up in the Stream Buffer, which eventually
// empties to the 'Device Buffer' when full. Using the Stream Buffer ensures that the
// considerable system overheads incurred by each write to the device buffer are minimized,
// while not sacrificing any latency performance. A sensible arrangement is to make the stream
// buffer a third the size of the device buffer. If the response to a device buffer write request can
// be improved then the device buffer size can be reduced while keeping the stream buffer
// the same, and so reduce latency.


#if !defined(__CAIOoutStream_hpp)
#define __CAIOoutStream_hpp


#include <windows.h>
#include <dsound.h>

#include "CAIOwaveConfig.hpp"


class CAIOoutStreamConfig
{
public:

	CAIOwaveConfig m_wave;
	
	// NB buffers measured in sample 'frames' or multi-samples. ie in stereo buf size = 1
	// means 1 stereo sample = 2 simultaneous mono samples.

	int m_nStreamBufferFrames;	// Used for storing single samples past to writeFrame, with periodic writing to writeToDeviceBuffer.
	int	m_nDeviceBufferFrames;	// Size of the hardware which is receiving blocks, and possibly mixing etc.			
	int m_nMinDeviceBufferWriteSamples;
	int m_nMinDeviceBufferSamplesFilled;

	bool m_streaming;		// True for streamed (continuous audio). False for static, used for repeated samples.

	CAIOoutStreamConfig();


	// Static variable settings. Used as defaults. 

	static long CAIOoutStreamConfig::nFramesPerSecond;
	static int CAIOoutStreamConfig::nChannels;
	static int CAIOoutStreamConfig::nStreamBufferFrames;
	static int CAIOoutStreamConfig::nDeviceBufferFrames;	
	static int CAIOoutStreamConfig::nMinDeviceBufferWriteSamples;
	static int CAIOoutStreamConfig::nMinDeviceBufferSamplesFilled;

};




class CAIOoutStream
{

protected:

	LPDIRECTSOUND       m_pDirectSound;		// Try making this static so that we can reuse the same device for several buffers.
	LPDIRECTSOUNDBUFFER m_pDSBuffer;
	UINT                m_writePos;
	UINT                m_cbBufSize;


	CAIOoutStreamConfig m_config;			// Handy storage of all configuration variables.
	short *m_streamBuffer;
	long m_nStreamBufferSamples;
	long m_nStreamBufferSamplesFilled;		// 1 sample = 1 word = 2 bytes.
	long m_nDeviceBufferSamples;
	int  m_nDeviceBufferSamplesToFill;
	// In non-blocking write mode, determines the number of samples which
	// the user wants to add to the device buffer.
	// ->writeSampsWithoutBlocking will then repeatedly accept buffers, 
	// until enough have been accumulated to add to the device buffer.

	int m_nDeviceBufferSamplesFilledOld;	// Used to store the number of filled device samples at the end of the last
									// write to the device buffer by tickWithoutBlocking.
	bool m_started;					// Indicates whether stream has been created (following m_config set.)
	
	void create0(void);				// create which is common to different O/S 
	void create(void);				// Internal create routine called from CAIOoutStream variations.
    int writeToDeviceBuffer(short *buf, int bufsize);
    int writeToDeviceBufferWithoutBlocking(short *buf, int bufsize);

	bool m_firstAdaptiveWrite;

 public:

	CAIOoutStream();					// Use default m_config, create immediately.
	~CAIOoutStream();

	int open(CAIOoutStreamConfig*);		// Use supplied m_config to start up the stream.
	int open();							// Open using default definition.


	///// Blocking Write.

	int writeSample(short samp);
	int writeFrame(short s1, short s2);
 	int writeFrame(short s1, short s2, short s3, short s4);
	int flush(void);		// Make sure remaining samples are passed through.

	// Accumulate samples until enough are received to write to the device buffer.
	// Block until this amount can be written.

	int writeSamples(short* in, long nSamples);	
	int writeSamples(float* in, long nSamples);		
	int writeSamples(double* in, long nSamples);		

	///// Non-blocking Write.

	// Assess how much the device buffer needs refilling, based on delay since last fill.
	// Use in conjunction with tickWithoutBlocking to implement audio in the same thread
	// as eg fast dynamics.
	int calcnDeviceBufferSamplesToFill();
	int calcnDeviceBufferSamplesToFillAdaptively();
	int getnDeviceBufferSamplesFilled();
	
	// We assume the user knows there is room for this!
	void setnDeviceBufferSamplesToFill(int n) {	m_nDeviceBufferSamplesToFill = n;	}

	// Accumulate samples and write when m_nDeviceBufferSamplesToFill samples have
	// accumulated. Returns -1 when no more samples are required.

	int writeSamplesWithoutBlocking(short* in, long nSamples);	
	int writeSamplesWithoutBlocking(float* in, long nSamples);	
	int writeSamplesWithoutBlocking(double* in, long nSamples);	
										


};



inline 
int CAIOoutStream :: writeSample(short sample)
{
	int hr =0;

	m_streamBuffer[m_nStreamBufferSamplesFilled++] = sample;


	if (m_nStreamBufferSamplesFilled == m_nStreamBufferSamples) {
		hr = writeToDeviceBuffer(m_streamBuffer, m_nStreamBufferSamples);
		m_nStreamBufferSamplesFilled = 0;
	}

	return hr;
}

inline 
int CAIOoutStream :: writeFrame(short s1, short s2)
{
	int hr =0;

	m_streamBuffer[m_nStreamBufferSamplesFilled++] = s1;
	m_streamBuffer[m_nStreamBufferSamplesFilled++] = s2;


	if (m_nStreamBufferSamplesFilled == m_nStreamBufferSamples) {
		hr = writeToDeviceBuffer(m_streamBuffer, m_nStreamBufferSamples);
		m_nStreamBufferSamplesFilled = 0;
	}

	return hr;
}

inline 
int CAIOoutStream :: writeFrame(short s1, short s2, short s3, short s4)
{
	int hr =0;

	m_streamBuffer[m_nStreamBufferSamplesFilled++] = s1;
	m_streamBuffer[m_nStreamBufferSamplesFilled++] = s2;
	m_streamBuffer[m_nStreamBufferSamplesFilled++] = s3;
	m_streamBuffer[m_nStreamBufferSamplesFilled++] = s4;

	if (m_nStreamBufferSamplesFilled == m_nStreamBufferSamples) {
		hr = writeToDeviceBuffer(m_streamBuffer, m_nStreamBufferSamples);
		m_nStreamBufferSamplesFilled = 0;
	}

	return hr;
}




#endif