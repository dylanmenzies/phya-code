//
// paModalData.cpp
//

#include <stdio.h>

#include "Resonator/paModalData.hpp"
#include "Resonator/paModalVars.hpp"
#include "System/paMemory.hpp"
#include "Utility/paRnd.hpp"




paModalData :: paModalData() {

	m_nModes = 1;	// Initially a single test mode.
	m_usingDefaultModes = true;

	//! Allocation of memory for modes should be done after reading number of modes. For now..
	m_freq = paFloatCalloc(paModal::nMaxModes);
	m_damp = paFloatCalloc(paModal::nMaxModes);
	m_amp = paFloatCalloc(paModal::nMaxModes);

	m_freq[0] = 440;	// Handy default.
	m_damp[0] = 1;
	m_amp[0] = 1;

}


paModalData :: ~paModalData() {

	paFree(m_freq);
	paFree(m_damp);
	paFree(m_amp);
}

struct AutoCloseFile
{
	FILE*	mFile;
	AutoCloseFile() : mFile(0) {}
	~AutoCloseFile() { if (mFile) fclose(mFile); }
};

int
paModalData :: read( const char* filename_or_data, bool isFilename ) {

	int err;
	paFloat freqScale;
	paFloat dampScale;
	paFloat ampScale;

	float f,d,a;


	AutoCloseFile fileHandle;
	void* data = 0;

	typedef int (*scanType)(void*,const char *,...);
	scanType scanFunc = (scanType) &fscanf;

	if( isFilename )
	{
		FILE *fh = fopen(filename_or_data, "r");
		if (fh == NULL) {
//			fprintf(stderr, "\nModal datafile %s failed to load.\n\n", filename_or_data);
			return(-1);
		}
		fileHandle.mFile = fh;
		data = fh;
	}
	else
	{
		scanFunc = (scanType) &scanf;
		data = (void*) filename_or_data;
	}

////// Load data and prepare runtime data from it.


	if ( scanFunc(data, "%f	%f	%f\n", &freqScale, &dampScale, &ampScale) != 3 )
	{
		fprintf(stderr, "\nBad scaling factors in %s.\n\n", filename_or_data);
		return(-1);
	}

	if (m_usingDefaultModes) {
		m_usingDefaultModes = false;	// Otherwise add new data to existing data.
		m_nModes = 0;
	}

	while( m_nModes < paModal::nMaxModes && (err = scanFunc(data, "%f %f %f\n", &f, &d, &a)) == 3 ) {
		m_freq[m_nModes] = f * freqScale;
		m_damp[m_nModes] = d * dampScale;
		m_amp[m_nModes] = a * ampScale;

//! Test Risset mode idea.
//m_nModes++;
//m_freq[m_nModes] = m_freq[m_nModes-1] * 1.001;   //+paRnd(int(0), int(50))/10; //  //+ paRnd(paFloat(1.0),paFloat(5.0));
//m_damp[m_nModes] = m_damp[m_nModes-1];
//m_amp[m_nModes] = m_amp[m_nModes-1];

		m_nModes++;
	}

//	if (err == 3 && m_nModes == paModal::nMaxModes)
//		fprintf(stderr, "Total mode limit of %d exceeded: Some modes lost.\n\n", paModal::nMaxModes);

	return(0);
}
