//
// CAIOoutStream.cpp
//
// Audio hardware I/O access.
//



#include "CAIOoutStream.hpp"


CAIOoutStreamConfig::CAIOoutStreamConfig()
{
	// Default 'sensible' settings.
	// Currently output will break if input block size changes dynamically.

	m_nStreamBufferFrames = CAIOoutStreamConfig::nStreamBufferFrames;
	m_nDeviceBufferFrames = CAIOoutStreamConfig::nDeviceBufferFrames;
	m_nMinDeviceBufferWriteSamples = CAIOoutStreamConfig::nMinDeviceBufferWriteSamples;
	m_nMinDeviceBufferSamplesFilled = CAIOoutStreamConfig::nMinDeviceBufferSamplesFilled;
	m_wave.m_nFramesPerSecond = CAIOoutStreamConfig::nFramesPerSecond;
	m_wave.m_nChannels = CAIOoutStreamConfig::nChannels;

	m_streaming = true;
}


CAIOoutStream :: CAIOoutStream()
{
	// m_config initialized in class header.
//	create0();	
	m_started = false;
}

int
CAIOoutStream :: open(CAIOoutStreamConfig* config) 
{
	// Overwrite default m_config..
	memcpy(&m_config, config, sizeof(CAIOoutStreamConfig));		// Make copy of user m_config,

	m_started = true;
	create0();	
	return 0;
}


int CAIOoutStream :: open() 
{
	m_started = true;
	create0();
	return 0;
}


// Create0 is init code common to all the operating systems.

void CAIOoutStream :: create0()
{
	m_nStreamBufferSamplesFilled = 0;
	m_nStreamBufferSamples = m_config.m_nStreamBufferFrames * m_config.m_wave.m_nChannels;
	m_nDeviceBufferSamples = m_config.m_nDeviceBufferFrames * m_config.m_wave.m_nChannels;
	m_nDeviceBufferSamplesFilledOld = 0;

	// Allow upto max device buffer size for the '->writeSamplesWithoutBlocking'.
	m_streamBuffer = (short*) calloc(m_nDeviceBufferSamples, sizeof(short));

	create();		// OS specific initialisation.
	m_started = true;
	m_firstAdaptiveWrite = true;
}

int CAIOoutStream :: flush()
{
	int hr, i, nBufs;

	// Flush the stream and send enough following zeroes to
	// make the device buffer quiet. Neccessary for Dsound, but maybe not
	// other device buffers.

	nBufs = m_config.m_nDeviceBufferFrames / m_config.m_nStreamBufferFrames;

	for(i=0; i<nBufs; i++) {
	
		while(m_nStreamBufferSamplesFilled < m_nStreamBufferSamples)
			m_streamBuffer[m_nStreamBufferSamplesFilled++] =0;

		hr = writeToDeviceBuffer(m_streamBuffer, m_nStreamBufferSamples);
		m_nStreamBufferSamplesFilled = 0;

	}
	
	return hr;
}





//DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD
//DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD
//DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD

#include <stdio.h>


void CAIOoutStream :: create()
{
  HRESULT result;
  HWND hWnd;
  DWORD dwDataLen;
  WAVEFORMATEX  wfFormat;
  DSBUFFERDESC  dsbdDesc, primarydsbDesc;
  LPDIRECTSOUNDBUFFER m_pDSPrimeBuffer;
  BYTE *pDSBuffData;


  // Initialize pointers to NULL
  m_pDirectSound = NULL;
  m_pDSBuffer = NULL;
  m_pDSPrimeBuffer = NULL;

  // Create the DS object
  if ((result = DirectSoundCreate(NULL, &m_pDirectSound, NULL)) != DS_OK)
    {
      fprintf(stderr, "Cannot open default sound device!!\n");
      exit(0);
    }

 hWnd = GetForegroundWindow();
  if ((result = m_pDirectSound->SetCooperativeLevel(hWnd, DSSCL_EXCLUSIVE)) != DS_OK)
    {
      fprintf(stderr, "DS Constructor: couldn't set cooperative level!\n");
      exit(0);
    }


 LPDIRECTSOUND m_pDirectSound2;
  if ((result = DirectSoundCreate(NULL, &m_pDirectSound2, NULL)) != DS_OK)
    {
      fprintf(stderr,"Cannot open default sound device!!\n");
      exit(0);
    }

 hWnd = GetForegroundWindow();
  if ((result = m_pDirectSound2->SetCooperativeLevel(hWnd, DSSCL_EXCLUSIVE)) != DS_OK)
    {
      fprintf(stderr,"DS Constructor: couldn't set cooperative level!\n");
      exit(0);
    }




  // Define the wave format structure
  wfFormat.wFormatTag = WAVE_FORMAT_PCM;
  wfFormat.nChannels = m_config.m_wave.m_nChannels;
  wfFormat.nSamplesPerSec = (unsigned long) m_config.m_wave.m_nFramesPerSecond;
  wfFormat.wBitsPerSample = 8 * sizeof(short);		// ie 16 bit per channel samples
  wfFormat.nBlockAlign = wfFormat.nChannels * wfFormat.wBitsPerSample / 8;
  wfFormat.nAvgBytesPerSec = wfFormat.nSamplesPerSec * wfFormat.nBlockAlign;
  wfFormat.cbSize = 0;

  // Setup the primary DS buffer description
  ZeroMemory(&primarydsbDesc, sizeof(DSBUFFERDESC));
  primarydsbDesc.dwSize = sizeof(DSBUFFERDESC);
  primarydsbDesc.dwFlags = DSBCAPS_PRIMARYBUFFER;
  primarydsbDesc.dwBufferBytes = 0;
  primarydsbDesc.lpwfxFormat = NULL;



  // Create the primary DS buffer
  if ((result = m_pDirectSound->CreateSoundBuffer(&primarydsbDesc,
				&m_pDSPrimeBuffer, NULL)) != DS_OK)
    {
      fprintf(stderr, "Cannot get the primary DS buffer address!\n");
      exit(0);
    }

  // Set the primary DS buffer sound format.  We have to do this because
  // the default primary buffer is 8-bit, 22kHz! 

  if ((result = m_pDSPrimeBuffer->SetFormat(&wfFormat)) != DS_OK)
    {
      fprintf(stderr, "Cannot set the primary DS buffer to proper sound format!\n");
	  	int a1=(result==DSERR_BADFORMAT); 
		int a2=(result==DSERR_INVALIDCALL);
		int a3=(result==DSERR_INVALIDPARAM); 
		int a4=(result==DSERR_OUTOFMEMORY); 
		int a5=(result==DSERR_PRIOLEVELNEEDED); 
		int a6=(result==DSERR_UNSUPPORTED); 
      exit(0);
    }


  // Setup the secondary DS buffer description
  m_cbBufSize = m_nDeviceBufferSamples * sizeof(short);
  ZeroMemory(&dsbdDesc, sizeof(DSBUFFERDESC));
  dsbdDesc.dwSize = sizeof(DSBUFFERDESC);
  dsbdDesc.dwFlags = DSBCAPS_GLOBALFOCUS;
  dsbdDesc.dwBufferBytes = m_cbBufSize;
  dsbdDesc.lpwfxFormat = &wfFormat;



  // Create the secondary DS buffer
  if ((result = m_pDirectSound->CreateSoundBuffer(
	  &dsbdDesc, &m_pDSBuffer, NULL)) != DS_OK)
    {
     fprintf(stderr, "DS Constructor: couldn't create sound buffer!\n");
      exit(0);
    }

  // Lock the DS buffer
  if ((result = m_pDSBuffer->Lock(0, m_cbBufSize, (LPVOID *)&pDSBuffData,
								  &dwDataLen, NULL, NULL, 0)) != DS_OK)
    {
      fprintf(stderr, "DS Constructor: couldn't lock sound buffer!\n");
      exit(0);
    }

  // Zero the DS buffer
  ZeroMemory(pDSBuffData, dwDataLen);

  // Unlock the DS buffer
  if ((result = m_pDSBuffer->Unlock(pDSBuffData, dwDataLen, NULL, 0)) != DS_OK)
    {
      fprintf(stderr, "DS Constructor: couldn't unlock sound buffer!\n");
      exit(0);
    }

  m_writePos = 0;  // reset last write position to start of buffer

  // Start the buffer playback
  if ((result = m_pDSBuffer->Play( 0, 0, DSBPLAY_LOOPING ) != DS_OK))
    {
      fprintf(stderr, "DS Constructor: couldn't play sound buffer!\n");
      exit(0);
    }
}




CAIOoutStream :: ~CAIOoutStream()
{

  // Cleanup the sound buffer
  if (m_pDSBuffer)
	{
	  m_pDSBuffer->Stop();
	  m_pDSBuffer->Release();
	  m_pDSBuffer = NULL;
	}

  // Cleanup the DS object
  if (m_pDirectSound)
	{
	  m_pDirectSound->Release();
	  m_pDirectSound = NULL;
	}
}




// Blocking write to device buffer.
// Sleeps until enough room is available for a write.
// Don't write more than the device buffer in one go!


int CAIOoutStream :: writeToDeviceBuffer(short *inBuf, int nInBufSamples)
{
  HRESULT hr;
  DWORD status;
  LPVOID lpbuf1 = NULL;
  LPVOID lpbuf2 = NULL;
  DWORD dwsize1 = 0;
  DWORD dwsize2 = 0;
  DWORD playPos, safePos, endWrite;
  DWORD millis;



  // Should be playing, right?
  hr = m_pDSBuffer->GetStatus( &status );
  if (!(status && DSBSTATUS_PLAYING))
    {
      fprintf(stderr, "Buffer not playing!\n");
    }

/////// Sleep until we have enough room in the device buffer.

// m_writePos is an index to the next sample to write to on the device buffer.
// This is maintained by the program, because DirectSound provides no direct access.
// NB m_writePos, playPos and endWrite are all in bytes.
 
  endWrite = m_writePos + nInBufSamples * sizeof(short);

  hr = m_pDSBuffer->GetCurrentPosition( &playPos, &safePos );
  if( hr != DS_OK ) return -1;

  if( playPos < m_writePos ) playPos += m_cbBufSize; // unwrap play offset


	// Use Sleep() to block until there is room to make a write.
	// Maybe better performance to wait on notification events, but details look complex so..


  while ( playPos < endWrite ) {	// First sleep may not be long enough, so use a while loop.

    // Calculate number of milliseconds until we will have room, as
    // time = (bytes) * (milliseconds/second) / ((bytes/sample) * (samples/second)),
    // rounded up.

		millis = (DWORD) (1.0 + ((endWrite - playPos) * 1000.0) / ( sizeof(short) * m_config.m_wave.m_nFramesPerSecond));

		Sleep( millis );

	// Wake up, find out where the playPos pointer has advanced to.

		hr = m_pDSBuffer->GetCurrentPosition( &playPos, &safePos );
		if( hr != DS_OK ) return -1;

		if( playPos < m_writePos ) playPos += m_cbBufSize;
		
  }


  // Lock free space in the DS
  hr = m_pDSBuffer->Lock (m_writePos, nInBufSamples * sizeof(short), 
							&lpbuf1, &dwsize1, &lpbuf2, &dwsize2, 0);

// printf("%d\n", dwsize1 + dwsize2);

  if (hr == DS_OK)
    {
		// Copy the buffer into the DS
		CopyMemory(lpbuf1, inBuf, dwsize1);
		if (lpbuf2) CopyMemory(lpbuf2, ((char*)inBuf) +dwsize1, dwsize2);	// NB (char*) because desize1 counts bytes!

		// Update our buffer offset and unlock sound buffer
		m_writePos = (m_writePos + dwsize1 + dwsize2) % m_cbBufSize;

// printf("writePos = %d\n", m_writePos);

      m_pDSBuffer->Unlock (lpbuf1, dwsize1, lpbuf2, dwsize2);
    }
  return 0;
}





int
CAIOoutStream :: getnDeviceBufferSamplesFilled()
{
	HRESULT hr;
	DWORD playPos, writePos, safePos;

	hr = m_pDSBuffer->GetCurrentPosition( &playPos, &safePos );
	if( hr != DS_OK ) return -1;

	writePos = m_writePos;	// Temp writePos so that..
	if( playPos > writePos ) writePos += m_cbBufSize; // Unwrap write offset.

	return (  writePos - playPos) /2;	// nBytes to nWords.

	//! What if playPos has gone past writePos? PlayPos cycles
	// regardless, so the returned value is incorrect. Maybe correct this
	// with interrupts on buffer empty, but...
}


int
CAIOoutStream :: calcnDeviceBufferSamplesToFillAdaptively()
{
	// Assess how many samples should be written to the device buffer,
	// by predicting the next audio frame uptake from the last.

	int nSamplesFilled;
	
	if (m_firstAdaptiveWrite) {
		nSamplesFilled = 0;
		m_firstAdaptiveWrite = false;
	}

	else nSamplesFilled	= getnDeviceBufferSamplesFilled();

	int nSamplesFilledNew;


	nSamplesFilledNew = m_nDeviceBufferSamplesFilledOld - nSamplesFilled 
								+ m_config.m_nMinDeviceBufferSamplesFilled;


//	printf("filled = %d      willBeFilled = %d\n", nSamplesFilled, nSamplesFilledNew);


	if (nSamplesFilledNew < m_config.m_nMinDeviceBufferSamplesFilled) 
		nSamplesFilledNew = m_config.m_nMinDeviceBufferSamplesFilled;

	// Make sure fillup does not exceed the device buffer.
	if (nSamplesFilledNew > m_config.m_nDeviceBufferFrames ) 
		nSamplesFilledNew = m_config.m_nDeviceBufferFrames;

	int nSamplesToFill = nSamplesFilledNew - nSamplesFilled;

	// Directsound messes up if small writes are made too frequently.
	if ( nSamplesToFill < m_config.m_nMinDeviceBufferWriteSamples ) return -1;	

	setnDeviceBufferSamplesToFill(nSamplesToFill);	
	m_nDeviceBufferSamplesFilledOld = nSamplesFilledNew;

	return 0;		// Ready for ->writeSamplesWithoutBlocking ...
}


int
CAIOoutStream :: calcnDeviceBufferSamplesToFill()
{
	// Calculate number of samples free in the device buffer.

	int nSamplesToFill = m_nDeviceBufferSamples - getnDeviceBufferSamplesFilled();

//	printf("samples to fill = %d\n", nSamplesToFill);

	// Directsound messes up if small writes are made too frequently.
	if ( nSamplesToFill < m_config.m_nMinDeviceBufferWriteSamples ) return -1;	

	setnDeviceBufferSamplesToFill(nSamplesToFill);	

	return 0;		// Ready for ->writeSamplesWithoutBlocking ...
}


// This is a simpler form of writeToDeviceBuffer.
// It assumes that the inBuf is small enough to fit into the available free space in the
// dsound secondary buffer.

int CAIOoutStream :: writeToDeviceBufferWithoutBlocking(short *inBuf, int nInBufSamples)
{
  HRESULT hr;
  DWORD status;
  LPVOID lpbuf1 = NULL;
  LPVOID lpbuf2 = NULL;
  DWORD dwsize1 = 0;
  DWORD dwsize2 = 0;


  // Should be playing, right?
  hr = m_pDSBuffer->GetStatus( &status );
  if (!(status && DSBSTATUS_PLAYING))
    {
      fprintf(stderr, "Buffer not playing!\n");
    }

  // Lock free space in the DS
  hr = m_pDSBuffer->Lock (m_writePos, nInBufSamples * sizeof(short), 
							&lpbuf1, &dwsize1, &lpbuf2, &dwsize2, 0);


  if (hr == DS_OK)
    {
		// Copy the buffer into the DS
		CopyMemory(lpbuf1, inBuf, dwsize1);
		if (lpbuf2) CopyMemory(lpbuf2, ((char*)inBuf) +dwsize1, dwsize2);	// NB (char*) because desize1 counts bytes!

		// Update our buffer offset and unlock sound buffer
		m_writePos = (m_writePos + dwsize1 + dwsize2) % m_cbBufSize;

		m_pDSBuffer->Unlock (lpbuf1, dwsize1, lpbuf2, dwsize2);
    }
  return 0;
}


//DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD
//DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD
//DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD





// This is a flexible block passing function. It can efficiently handle blocks that are
// both smaller and larger than the stream and device buffers.
// Note that normally you want buffer sizes writebuffer < streambuffer < devicebuffer.

// Can the overloaded functions be covered with a single function??
// Seems we can't pass pointers to free-type parameters 

int
CAIOoutStream :: writeSamples( short* in, long nSamples )	// Output one in-buffer directly
{

	// If enough samples left to fill Stream buffer, fill it then repeat with input samples remaining.
	while( m_nStreamBufferSamplesFilled + nSamples >= m_nStreamBufferSamples) {

		nSamples -= (m_nStreamBufferSamples - m_nStreamBufferSamplesFilled);	// Calculate before m_nStreamBufferSamplesFilled changes.

		while(m_nStreamBufferSamplesFilled < m_nStreamBufferSamples)
			m_streamBuffer[m_nStreamBufferSamplesFilled++] = (short)(*in++);

		writeToDeviceBuffer(m_streamBuffer, m_nStreamBufferSamples);

		m_nStreamBufferSamplesFilled = 0;
	}

	// Remaining samples won't fill the m_streamBuffer, so write all of them.
	while(nSamples-- >0) 
		m_streamBuffer[m_nStreamBufferSamplesFilled++] = (short)(*in++);


	return 0;
}


int
CAIOoutStream :: writeSamples( float* in, long nSamples )	// Output one in-buffer directly
{

	// If enough samples left to fill Stream buffer, fill it then repeat with input samples remaining.
	while( m_nStreamBufferSamplesFilled + nSamples >= m_nStreamBufferSamples) {

		nSamples -= (m_nStreamBufferSamples - m_nStreamBufferSamplesFilled);	// Calculate before m_nStreamBufferSamplesFilled changes.

		while(m_nStreamBufferSamplesFilled < m_nStreamBufferSamples)
			m_streamBuffer[m_nStreamBufferSamplesFilled++] = (short)(*in++);

		writeToDeviceBuffer(m_streamBuffer, m_nStreamBufferSamples);

		m_nStreamBufferSamplesFilled = 0;
	}

	// Remaining samples won't fill the m_streamBuffer, so write all of them.
	while(nSamples-- >0) 
		m_streamBuffer[m_nStreamBufferSamplesFilled++] = (short)(*in++);


	return 0;
}


int
CAIOoutStream :: writeSamples( double* in, long nSamples )	// Output one in-buffer directly
{

	// If enough samples left to fill Stream buffer, fill it then repeat with input samples remaining.
	while( m_nStreamBufferSamplesFilled + nSamples >= m_nStreamBufferSamples) {

		nSamples -= (m_nStreamBufferSamples - m_nStreamBufferSamplesFilled);	// Calculate before m_nStreamBufferSamplesFilled changes.

		while(m_nStreamBufferSamplesFilled < m_nStreamBufferSamples)
			m_streamBuffer[m_nStreamBufferSamplesFilled++] = (short)(*in++);

		writeToDeviceBuffer(m_streamBuffer, m_nStreamBufferSamples);

		m_nStreamBufferSamplesFilled = 0;
	}

	// Remaining samples won't fill the m_streamBuffer, so write all of them.
	while(nSamples-- >0) 
		m_streamBuffer[m_nStreamBufferSamplesFilled++] = (short)(*in++);


	return 0;
}


int
CAIOoutStream :: writeSamplesWithoutBlocking( short* in, long nSamples )
{
	int result = 0;		// Default result : not enough samples for desired fill yet.
	
	if( m_nStreamBufferSamplesFilled + nSamples >= m_nDeviceBufferSamplesToFill ) {

		nSamples -= (m_nDeviceBufferSamplesToFill - m_nStreamBufferSamplesFilled);	// Calculate before m_nStreamBufferSamplesFilled changes.

		while(m_nStreamBufferSamplesFilled < m_nDeviceBufferSamplesToFill)
			m_streamBuffer[m_nStreamBufferSamplesFilled++] = (short)(*in++);

		writeToDeviceBufferWithoutBlocking(m_streamBuffer, m_nDeviceBufferSamplesToFill);

		m_nStreamBufferSamplesFilled = 0;

		result = -1;	// Enough samples have been received for fill, and fill has been executed.
	}


	while(nSamples-- >0)
		m_streamBuffer[m_nStreamBufferSamplesFilled++] = (short)(*in++);


	return result;
}
	


int
CAIOoutStream :: writeSamplesWithoutBlocking( float* in, long nSamples )
{
	int result = 0;		// Default result : not enough samples for desired fill yet.
	
	if( m_nStreamBufferSamplesFilled + nSamples >= m_nDeviceBufferSamplesToFill ) {

		nSamples -= (m_nDeviceBufferSamplesToFill - m_nStreamBufferSamplesFilled);	// Calculate before m_nStreamBufferSamplesFilled changes.

		while(m_nStreamBufferSamplesFilled < m_nDeviceBufferSamplesToFill)
			m_streamBuffer[m_nStreamBufferSamplesFilled++] = (short)(*in++);

		writeToDeviceBufferWithoutBlocking(m_streamBuffer, m_nDeviceBufferSamplesToFill);

		m_nStreamBufferSamplesFilled = 0;

		result = -1;	// Enough samples have been received for fill, and fill has been executed.
	}


	while(nSamples-- >0)
		m_streamBuffer[m_nStreamBufferSamplesFilled++] = (short)(*in++);


	return result;
}
	


int
CAIOoutStream :: writeSamplesWithoutBlocking( double* in, long nSamples )
{
	int result = 0;		// Default result : not enough samples for desired fill yet.
	
	if( m_nStreamBufferSamplesFilled + nSamples >= m_nDeviceBufferSamplesToFill ) {

		nSamples -= (m_nDeviceBufferSamplesToFill - m_nStreamBufferSamplesFilled);	// Calculate before m_nStreamBufferSamplesFilled changes.

		while(m_nStreamBufferSamplesFilled < m_nDeviceBufferSamplesToFill)
			m_streamBuffer[m_nStreamBufferSamplesFilled++] = (short)(*in++);

		writeToDeviceBufferWithoutBlocking(m_streamBuffer, m_nDeviceBufferSamplesToFill);

		m_nStreamBufferSamplesFilled = 0;

		result = -1;	// Enough samples have been received for fill, and fill has been executed.
	}


	while(nSamples-- >0)
		m_streamBuffer[m_nStreamBufferSamplesFilled++] = (short)(*in++);


	return result;
}
	