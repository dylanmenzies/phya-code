// 
// modan.cpp
//
// Modal Analysis command line function.
// Simplified Macauly-Quaterieri analysis: modes assumed to stay within a single bin.
// Final log amp evolution approximated by straight lines.
// Suitable for extracting modes from the impulse response of
// hard objects. Note that objects such as wood exhibit a notable ammount of time-domain
// impulse response which is not well described exclusively with modes.
//
// Note although we link with the AIO wrapper, only winmm.lib is needed for the 
// wav file functions and not dsound.lib
//
//! not working for 24 bit wav files.


#define DEFAULT_N_SAMPLES_IN_FFT_WINDOW 4096
#define DEFAULT_FRACTIONAL_WINDOW_GAP	.1f
#define DEFAULT_NOISE_FLOOR				8	// 4
#define DEFAULT_MIN_PEAKS				5	// 3


//#define PULSE_THRESHOLD_AMP				4000


//  Scheme: 
// 
//  load wav file.
//  Create space for bin amplitudes for each time frame.
//  Scan wav and fill bins.
//  Scan bins for peaks, increment peak counts.
//  Look at significant bin counts and do least-squares to determine amp coupling and damping.



#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>



#include "AIOloadWaveFile.hpp"
#include "fft2.h"

//#include "ADBwriteToFile.hpp"

char s0[] = "MODAN modal analyser, (C) 2001-2009 Dylan Menzies.";



void
main(int argc, char *argv[ ])
{

	long nTimeSamplesPerWindow;
	long nFreqSamplesPerWindow;
	long nTimeSamplesBetweenWindows;

printf("\n%s\n\n", s0);


	if(argc <2) {
		printf("usage : \n modan <wav file> (<block size> <block spacing> <noise floor> <min num peaks>)\n\n");
		printf("<wav file> smooth section of a recording of hit resonant object eg bell.\n");
		printf("<block size> in samples, default %d, (will be rounded to nearest power of two)>\n",DEFAULT_N_SAMPLES_IN_FFT_WINDOW);
		printf("<block spacing> as fraction of a block, default %f\n",DEFAULT_FRACTIONAL_WINDOW_GAP);
		printf("<noise floor> spectral peaks below this level are ignored, default %d\n",DEFAULT_NOISE_FLOOR);
		printf("<min num peaks> for each freq bin, the number of successive peaks required to generate a resonance, default %d\n\n",DEFAULT_MIN_PEAKS);

		exit(-1);
	}

	char *soundfileName = argv[1];


	nTimeSamplesPerWindow = DEFAULT_N_SAMPLES_IN_FFT_WINDOW;
	if (argc > 2) sscanf( argv[2], "%d", &nTimeSamplesPerWindow );

	int fft2Power = (int)(log((float)nTimeSamplesPerWindow)/log(2.0) -1.0 +.5);	// +.5 rounds avoids numerical inaccuracy.

	nTimeSamplesPerWindow = (long)pow(2.0, fft2Power) * 2;			// Make sure its a power of two!!

	nFreqSamplesPerWindow = nTimeSamplesPerWindow / 2;		// Half because phase info discarded.

	nTimeSamplesBetweenWindows = (short) (nTimeSamplesPerWindow * DEFAULT_FRACTIONAL_WINDOW_GAP);
	if (argc > 3) {
		float f;
		sscanf( argv[3], "%f", &f );
		nTimeSamplesBetweenWindows = (short) ( f * nTimeSamplesPerWindow );
	}

	float noiseFloor = DEFAULT_NOISE_FLOOR;
	if (argc > 4) {
		float f;
		sscanf( argv[4], "%f", &f );
		noiseFloor = f;
	}

	int minPeaks = DEFAULT_MIN_PEAKS;
	if (argc > 5) {
		int d;
		sscanf( argv[5], "%d", &d );
		minPeaks = d;
	}



	CAIOwaveConfig wavConfig;

	if (AIOloadWaveFile( soundfileName, &wavConfig) != 0) {
		fprintf(stderr, "Error loading soundfile.\n");
		exit(-1);
	}

	if (wavConfig.m_nChannels != 1) {
		fprintf(stderr, "Error. Soundfile must contain only one channel.\n");
		exit(-1);
	}

	if (wavConfig.m_nBitsPerMonoSample != 16) {
		fprintf(stderr, "Error. Soundfile must be 16 bit PCM.\n");
		exit(-1);
	}

	// Create workspace.

	float* fftFrameBufferReal =	(float*)calloc( nTimeSamplesPerWindow, sizeof(float) );
	float* fftFrameBufferImag = (float*)calloc( nTimeSamplesPerWindow, sizeof(float) );
	float* window = (float*)malloc( nTimeSamplesPerWindow * sizeof(float) ); 
	float* amp = (float*)malloc( nFreqSamplesPerWindow * sizeof(float) );
	// Sum of amps of frames for each bin:
	float* ampSum = (float*)calloc( nFreqSamplesPerWindow, sizeof(float) );
	// First moment of amps:
	float* ampMoment = (float*)calloc( nFreqSamplesPerWindow, sizeof(float) );
	// Number of frames overwhich amp statistics have been taken:
	long* ampNum = (long*)calloc(  nFreqSamplesPerWindow, sizeof(long) );
	// Number of peaks found in each bin:
	long* nPeaks = (long*)calloc( nFreqSamplesPerWindow, sizeof(long) );

	// Result space.

	float* ampcoupling = (float*)malloc( nFreqSamplesPerWindow * sizeof(float) );
	float* damping = (float*)malloc( nFreqSamplesPerWindow * sizeof(float) );		// Sum of amps of frames for each bin.
	float* freq = (float*)malloc( nFreqSamplesPerWindow * sizeof(float) );	// First moment of amps.




	int n;

	// Create window to be applied before FFT.

	float f = 2.0f*3.142659f/(nTimeSamplesPerWindow-1);
	for(n=0; n<nTimeSamplesPerWindow; n++)
		// Hanning.
		window[n] = .5f*(1.0f-(float)cos(f*n));			// Narrow main lobe prefered to differentiate modes.
		// Hamming.
//		window[n] = .54-.46*cos(f*n);



	// Find impulse start.

	short* soundStart = wavConfig.m_start;
	short* pulseStart = soundStart;				// Could start analysis later in sound file.

//	while( *(pulseStart++) < PULSE_THRESHOLD_AMP );
//!	while( (pulseStart++ - soundStart) < 5000 );

	short* framePos = pulseStart;		// Cursor for working frame.
	short* maxFramePos = soundStart + wavConfig.m_nFrames - nTimeSamplesPerWindow;

	short nFrame = 0;		// Count the number of frames.

	short* samplePos;

	if (framePos > maxFramePos) {
		fprintf(stderr, "Audio too short for window size.\n");
		exit(-1);
	}



////// Main scan.

	// For each frame: 
	// 1. Calculate the FFT.
	// 2. Update statistics for least square fitting of modes identified later.
	// 3. Find peaks and update total peak count.

	while( framePos < maxFramePos ) {

		samplePos = framePos;

		// Copy wav data into FFT buffer.
		for(n=0; n< nTimeSamplesPerWindow; n++)
			fftFrameBufferReal[n] = window[n] * (float) *(samplePos++);


		fft2( fftFrameBufferReal, fftFrameBufferImag, fft2Power, (float)1.0, (int)0 );

		// Calc log of bin magnitudes.

		for(n=0; n < nFreqSamplesPerWindow; n++)
		{ 
			//! Should rationalize noiseFloor, eg dB rel max amp.
			amp[n] = 0.5f* (float)log( fftFrameBufferReal[n] * fftFrameBufferReal[n]
								+ fftFrameBufferImag[n] * fftFrameBufferImag[n] );

			if (amp[n] > noiseFloor) 
				{	// Above noise floor, so add to statistics.
					ampSum[n] += amp[n];
					ampMoment[n] += nFrame * amp[n];	//! could do this on ampSum later
					ampNum[n]++;
				}


////////// Visualization:
/*
			char line[256];
			sprintf(line, "%8d Hz %s",(n*wavConfig.m_nFramesPerSecond)/nTimeSamplesPerWindow,
				ADBbar((short)amp[n]));
			ADBwriteToFile(line, nFreqSamplesPerWindow);
*/
		}

int numPeaksInFrame = 0;	
		// Find peaks and increment peak counters.
		for(n=1; n< nFreqSamplesPerWindow -1; n++)
		{
			if (amp[n] > noiseFloor && amp[n-1] < amp[n] && amp[n] > amp[n+1] )
			{
				nPeaks[n] ++;
				numPeaksInFrame++;
			}
		}
//printf("%d\n", numPeaksInFrame);

		framePos += nTimeSamplesBetweenWindows;
		nFrame ++;
	}


	float gradient;		// Slope of linear fit.
	float offset;		// value of fit at n=0.

	int i;

/*
	FILE* fh = fopen("results", "w");

	for(i=0; i< nFreqSamplesPerWindow; i++) {
		gradient = ( ampMoment[i] - a * ampSum[i] ) * b;
		offset = ( ampSum[i] - c * gradient ) * d;
		fprintf(fh, "%10d %10d %10f %10f %10f\n", i, nPeaks[i], ampSum[i], offset, gradient);
//		fprintf(fh, "%s\n", ADBbar((short)ampSum[i]*.1) );
	}
*/

	int nModesFound = 0;
	float overlapFactor = (float)nTimeSamplesPerWindow / (float)nTimeSamplesBetweenWindows;
	float maxAmpcoupling = 0;
//	int maxPeaks;


/////// Final Scan

	// 1. Find peaks with peak counts greater than minimum.  //, starting with the biggest.
	// 2. Fit modal data using bin statistics.


		for(i=0; i< nFreqSamplesPerWindow; i++)
		{
//printf("%d\n", nPeaks[i]);
			if ( nPeaks[i] > minPeaks )
			{
				int n = ampNum[i];
				//! I worked the coefficients by hand so there could be errors..

				float a = (n-1)*.5f;
				float b = 12.0f/ ( n*(n-1)*(n-1) );
				float c = n*(n-1)*.5f;
				float d = 1.0f / n;

				gradient = ( ampMoment[i] - a * ampSum[i] ) * b;
				offset = ( ampSum[i] - c * gradient ) * d;

				if (gradient < 0.0)
				{
					freq[nModesFound] = (float)i / nTimeSamplesPerWindow * wavConfig.m_nFramesPerSecond;
					damping[nModesFound] = (float)wavConfig.m_nFramesPerSecond* (-gradient) 
											/ (float)nTimeSamplesBetweenWindows;

					float p = overlapFactor * (-gradient);	// Decay amp compensation factor.
					float a = ampcoupling[nModesFound] = expf(offset) * p / (1- expf(-p));
					if (a > maxAmpcoupling) maxAmpcoupling = a;
					nModesFound++;
				}
			}
		}

//// Order by number of peaks.
//	bool firstPeak = true;
//	do{
//		maxPeaks = 0;
//		int maxPeakIndex = 0;
//		for(i=0; i< nFreqSamplesPerWindow; i++)
//			if ( nPeaks[i] > maxPeaks ) { maxPeaks = nPeaks[i]; maxPeakIndex = i; };
//
//		if (firstPeak) {
////			printf("Max peak count = %d/n", maxPeaks);	
//			firstPeak = false;
//		}
//
//		if (maxPeaks == 0) break;	// No more peaks left.
//		nPeaks[maxPeakIndex] = 0;	// Exclude this peak next search.
//		i = maxPeakIndex;
//
//		int n = ampNum[i];
//		//! I worked the coefficients by hand so there could be errors..
//
//		float a = (n-1)*.5f;
//		float b = 12.0f/ ( n*(n-1)*(n-1) );
//		float c = n*(n-1)*.5f;
//		float d = 1.0f / n;
//
//		gradient = ( ampMoment[i] - a * ampSum[i] ) * b;
//		offset = ( ampSum[i] - c * gradient ) * d;
//
//		if (gradient < 0.0)
//		{
//			freq[nModesFound] = (float)i / nTimeSamplesPerWindow * wavConfig.m_nFramesPerSecond;
//			damping[nModesFound] = (float)wavConfig.m_nFramesPerSecond* (-gradient) 
//									/ (float)nTimeSamplesBetweenWindows;
//
//			float p = overlapFactor * (-gradient);	// Decay amp compensation factor.
//			float a = ampcoupling[nModesFound] = expf(offset) * p / (1- expf(-p));
//			if (a > maxAmpcoupling) maxAmpcoupling = a;
//			nModesFound++;
//		}
//
//		
//	}while( maxPeaks >= minPeaks );




	// Normalize amplitudes.
	for(i=0; i<nModesFound; i++) ampcoupling[i] /= maxAmpcoupling;



	//! Need to add normaliser based on energy response to white noise excitation.
	// Should use Phya dll for the paModalRes.
	

	// Sort by field given by testFiled(i) macro.


	#define testField(I) (ampcoupling[I])

	{
		float record;
		int recordIndex;
		int i;
		int j;
		float t;

		for(i=0; i<nModesFound-1; i++) {
			record = testField(i);
			recordIndex = i;
			for(j=i+1; j<nModesFound; j++) {
				if (testField(j) > record) {
					record = testField(j);
					recordIndex = j;
				}
			}
			if (recordIndex > i){
				t = ampcoupling[i];
				ampcoupling[i] = ampcoupling[recordIndex];
				ampcoupling[recordIndex] = t;

				t = damping[i];
				damping[i] = damping[recordIndex];
				damping[recordIndex] = t;

				t = freq[i];
				freq[i] = freq[recordIndex];
				freq[recordIndex] = t;
			}
		}
	}



	i = 0;
	char outputFilename[128];

	while(argv[1][i] != 0 && argv[1][i] != '.')
	{
		outputFilename[i] = argv[1][i];
		i++;
	}

	outputFilename[i++] = '.';
	outputFilename[i++] = 'm';
	outputFilename[i++] = 'd';
	outputFilename[i++] = 0;


	FILE *fh = fopen(outputFilename, "w");


	if (fh == NULL) {
		fprintf(stderr, "\nFailed to open modal datafile for writing.\n\n");
		exit(-1);
	}



	fprintf(fh, "1.000000	1.000000	1.000000\n");
	

	for(i=0; i < nModesFound; i++)
	{
		fprintf(fh, "%f	%f	%f\n", freq[i], damping[i], ampcoupling[i]);
	}


	fclose(fh);

//getchar();
}

