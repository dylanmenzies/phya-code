//
// AIOvars.cpp
//
// Handy place for changing global-default variables.
//

#include "CAIOoutStream.hpp"

long CAIOoutStreamConfig::nFramesPerSecond = 44100;	//22050
int CAIOoutStreamConfig::nChannels = 1;


// nStreamBufferFrames is used in blocking-write-mode to determine the size
// of writes to the device buffer. This should be 1/3 - 2/3 of the device buffer size.
// Making nStreamBufferFrames small increases the streaming overhead (more than you think..)
// nDeviceBufferFrames is the device buffer and determines the output latency.
// With audio high thread priority, can be as low as 256 @ 22050Hz -> 12ms latency.
// NB the difference between the two values gives the 'safety zone' size.

// (Large values make it easier to monitor the cpu costs of the other parts of the audio thread).
// The default is very safe, so probably want to reduce these.
int CAIOoutStreamConfig::nStreamBufferFrames = 2000; //100; //200;	
int CAIOoutStreamConfig::nDeviceBufferFrames = 3000; //300; //300;


// In a non-blocking write we set a minimum size for a 'device write' from the stream buffer.
// This limits I/O overheads and works around a directsound glitch problem.
// nMinDeviceBufferSamplesFilled sets a minimum fill level target for the device buffer.
// Making this smaller will increase the chance of an ouput underrun.

int CAIOoutStreamConfig::nMinDeviceBufferSamplesFilled = 512;
int CAIOoutStreamConfig::nMinDeviceBufferWriteSamples = 256;




