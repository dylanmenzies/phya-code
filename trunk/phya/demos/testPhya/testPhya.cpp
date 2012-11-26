
//-----------------------------------------------------------------------------------------------
//
//	testPhya.cpp
//
//	Non-interactive demonstration of Phya.
//  Simple dynamical output from a dynamics-collision engine is simulated 
//  to simplify the presentation. Also useful for benchmarking Phya.
//
//	Consists of demos of Phya Contacts followed by a demo of Phya Impacts.
//
//  Guidelines :
// 
//  1. Read the code to understand how Phya works. 
//  2. Experiment with dampings and surface cutoffs to achieve sounds that 
//     seem natural for a given scenario and resonances.
//  3. Experiment with amplitudes and limit impulses and forces to
//     minimize clipping without limiting in place.
//  4. Use a paLimiter to eliminate residual clipping, either using the limiter included
//     in paGenerate or using paLimiter in your code. 
//  5. All output is mono using paGenerate : To generate multichannel
//     output for 3D work use a multi-output callback : see the commented-out code.
//  6. See paGeomUtilsAPI.h for tools to calculate contact velocities from 
//     rigid body kinematic parameters.
//  7. Use contact damping to make collisions more realistic - length of resonance
//     will depend on what other bodies are in contact.
//  8. Skids are available as part of an paImpact, however better skid effects are by generating 
//     a short contact in addition to an impact. The contact should be updated by the 
//     physics engine to generate good profile.
//  9. Make your own modal resonance files (.md) from impulse recordings, using the command
//     line application modan. Impulses used should be as bright as possible to enable full range 
//     of frequency excitation in Phya.
//
//-----------------------------------------------------------------------------------------------

//!! Contacts need fixing.

#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <math.h>

#include "Phya.hpp"

#define N_BODIES 5


char modalDataFileName1[] = "../resource/eg.md";
char modalDataFileName2[] = "eg.md";


void
multipleOutputCallback(paRes *res, paFloat* output)
{
	//// Example multiple output callback
	//// Use userData in res to link to required info.
	//// PhysicalBody refers to an object from the dynamics library you use.
	//// velocity might be used for doppler or directional radiation effects.

	// Get 3D position, velocity and 3D sound buffer channel.

//	resUserData* d = (resUserData*)(res->getUserData());
//	paFloat* position = d->PhysicalBody->position;
//	paFloat* velocity = d->PhysicalBody->velocity;
//	paFloat* audioChannel = d->audioChannel;
//  paLimiter* limiter = d->limiter;

	// You must copy the samples pointed to by output
	// - this buffer is valid only for this call.
	// Limiters are useful! Use paLimiter to make a set of limiters.

//	limiter->tick(output);
//	Your3DSoundEncode(audioChannel, position, velocity, output)
}



void
main()
{
	//// Specify some different surfaces.
	//// FunSurfaces are derived from the general Surface class.
	//// A FunSurface takes a Fun, a stateless function describing the surface profile.
	//// A number of parameters are available to taylor the sonic behaviour of the surface as
	//// the contact dynamics vary.


	paWhiteFun* whitefun = new paWhiteFun;				// White noise
	paRndFun* rndfun = new paRndFun;					// 'Rnd' surface function.
	paGridFun* gridfun = new paGridFun;					// 'Grid' surface function.
	paWavFun* wavfun = new paWavFun;					// Surface described by a loaded audio file.
	paFunSurface* whitesurf = new paFunSurface;			// Function surface.. uses surface functions, combined with
	paFunSurface* rndsurf = new paFunSurface;			// filter model for slip/roll.
	paFunSurface* gridsurf = new paFunSurface;
	paFunSurface* wavsurf = new paFunSurface;
	
	whitesurf->setFun(whitefun);							// White noise surface texture.
	whitesurf->setContactGain(32000.0f);				// Includes gain to generate audio output level.

	// The following define the velocity to surface-filter map.
	// This rises linearly to a maximum frequency value.
	whitesurf->setCutoffFreqAtRoll(10.0f);					// Adjust the rel body vel to filter cutoff mapping.
	whitesurf->setCutoffFreqRate(1000.0f);					// Rate of change of cutoff freq with slip speed.
	whitesurf->setCutoffFreq2AtRoll(10.0f);					// Beef up rolling with optional extra filter layer.
	whitesurf->setCutoffFreq2Rate(1000.0f);
//	whitesurf->setGainAtRoll(10.0f);							// Modify volume towards rolling, simulates increased transfer.
//	whitesurf->setGainBreakSlipSpeed(0.1f);
    whitesurf->setContactDirectGain(0.0);					// We can tune the settings for each type of surface.
	whitesurf->setContactGain(70000.0);

	rndsurf->setFun(rndfun);								// Random generator with variable 'bump' frequency and width distributions.
	rndsurf->setContactGain(20000.0f);
	rndsurf->setRateAtSpeed(100.0f, 1.0f);					// Rate here means bumps per second. Speed is the contact speed for which this is given.
	rndfun->setZeroRate(1.0f);								// Rate of surface height returning to zero, relative to main rate. Higher -> more spikey.
//	rndfun->setZeroRate(-1.0f);								// Always returns to zero immediately - max spikey.
	rndsurf->setCutoffFreqAtRoll(50.0f);					// Adjust the rel body vel to filter cutoff mapping.
	rndsurf->setCutoffFreqRate(1500.0f);
	rndsurf->setCutoffFreq2AtRoll(10.0f);					// Beef up rolling with optional extra filter layer.
	rndsurf->setCutoffFreq2Rate(1000.0f);
//	rndsurf->setGainAtRoll(10.0f);							// Boosts volume towards rolling, simulates increased transfer.
//	rndsurf->setGainBreakSlipSpeed(0.1f);

	gridsurf->setFun(gridfun);								// Grid bar function.
	gridfun->setMark(0.02f);								// Width of each grid bar as fraction of bar to bar distance.
	gridsurf->setContactGain(20000.0f);
	gridsurf->setRateAtSpeed(44.0f, 1.0f);					// Rate here means bars per second.
	gridsurf->setCutoffFreqAtRoll(50.0f);					// Adjust the rel body vel to filter cutoff mapping.
	gridsurf->setCutoffFreqRate(20000.0f);

	wavsurf->setFun(wavfun);								// Function loaded from a wav file, with interpolated read back.
	wavfun->readWav("../resource/byhand.wav");				// wav file prepared using recording / synthesis / hand editing tools.
	wavfun->setInterpOff();									// Interpolation off keeps edges hard, good for specially prepared wavs.
	wavsurf->setContactGain(2.0f);					// This wav file has output gain level already, so bo extra required.
	wavsurf->setRateAtSpeed(0.04f, 1.0f);					// Rate means wav seconds per actual seconds.
	wavsurf->setCutoffFreqAtRoll(50.0f);					// Adjust the rel body vel to filter cutoff mapping.
	wavsurf->setCutoffFreqRate(2000.0f);
	wavsurf->setCutoffFreq2AtRoll(50.0f);					// Adjust the rel body vel to filter cutoff mapping.
	wavsurf->setCutoffFreq2Rate(20000.0f);
//	wavsurf->setGainAtRoll(10.0f);
//	rndsurf->setGainBreakSlipSpeed(0.1f);

	//// Create resonators and bodies, set properties.
	//// Notice how different bodies can be made by frequency scaling a single modal set.
	//// Also possible to have several bodies sharing the same resonator, for complex systems.

	paModalData* data = new paModalData;
	paModalRes* res[N_BODIES];
	paBody* body[N_BODIES];


	// Read modal data file.
	// Modal file created from a real recording, using modan.exe
	// The format is one line per mode: frequency, damping, amplitude.
	// (An e at the top indicates that it is produced by the evaluation
	// version of modan - values are encoded)
	// The first line contains factors that scale all the modes.
	// The file can be edited to select the most important modes.
	// Usually these are the loudest and/or least damped.

	if (data->read(modalDataFileName1) == -1)
		if (data->read(modalDataFileName2) == -1)
			printf("Can't read modal data file.\n");

	int i,j;

	for(i=0; i< N_BODIES; i++) {
		paModalRes *mr = res[i] = new paModalRes;
		mr->setData(data);
		mr->setQuietLevel(1.0f);		// Determines at what rms envelope level a resonator will be 
										// faded out when no longer in contact, to save cpu.
										// Make bigger to save more cpu, but possibly truncate decays notceably.
								
//		mr->setnActiveModes(10);		// Can trade detail for speed.
		mr->setAuxAmpScale(.1f);
		mr->setAuxFreqScale(0.5f + 0.1f*i);
		body[i] = new paBody;
		body[i]->setRes(mr);			// NB Possible to have several bodies using one res for efficiency.
		body[i]->setSurface(whitesurf);
	}


	//// Temporary variables

	paContact* contact = new paContact;
	paImpact* impact = new paImpact;
	paContactDynamicData contactData;
	paImpactDynamicData impactData;

	float p,v;


	//// Create pools and lists that enable efficient
	//// dynamic collision management.

	paBlock::setnMaxFrames(128);						// The size of each audio block.
	paBlock::pool.allocate(N_BODIES+5);					// Num resonators+5.
	paContact::pool.allocate(1);
	paImpact::pool.allocate(1);
	paFunSurface::contactGenPool.allocate(20);			// Each Contact and Impact takes 2 contactGens.
	paFunSurface::impactGenPool.allocate(20);			// Each Impact takes 2 impactGens.
	paRes::activeResList.allocate(N_BODIES);			// Num resonators.


//-----------------------------------------------------------------------------------------------
//	Config and start audio io, possibly using an audio thread.
//-----------------------------------------------------------------------------------------------


	paSetnFramesPerSecond(44100);


	//// Configure simple mono output stream.
	paSetnStreamBufFrames(512);		// Num of frames in single write to device.
	paSetnDeviceBufFrames(4096);		//4096 // Size of device's internal buffer. (The AIO library used is intended for demonstration, and is not optimized)
	paOpenStream();					

	//// Alternatively use output callbacks to route seperate resonator outputs to a 3d audio system.
//	paSetMultipleOutputCallback( multipleOutputCallback );
	
	paSetLimiter(0.005f, 0.015f, 0.075f);	// Set limiter parameters. Leave this out if you don't want limiting.
	paInit();			
//	paStartThread();						// Thread option, not used here to keep things simple.


//	printf("\nUsing modal data from file  %s\n\n",modalDataFileName);

//-----------------------------------------------------------------------------------------------
//	Quick impulse test
//-----------------------------------------------------------------------------------------------



	printf("First recreate impulse using maximum hardness impact..\n\n");

	(body[0]->getRes())->setAuxFreqScale(1.0f);
	(body[0]->getRes())->setAuxDampScale(1.0f);
//	(body[0]->getRes())->setAuxAmpScale(0.05f);
	(body[0]->getSurface())->setHardness(10000.0f);  // Impulse time = 1/hardness (limited).

	impact = paImpact::newImpact();
	impact->setBody1(body[0]);
	impactData.relTangentSpeedAtImpact = 0; // No skid.
	impactData.impactImpulse = 1.0;
	impact->setDynamicData(&impactData);
	for(i=0; i<1000; i++) paGenerate();


//	goto impacts;

//-----------------------------------------------------------------------------------------------
//
// Contact demonstration
//
// A new contact is created from the pool. The dynamic parameters must 
// be continually updated, the faster the better.
// 3 control speed parameters and one force parameter must be updated using
// information from each physics frame.
// NB Utility/paGeomAPI.h can be used to calculate the speeds from rigid body
// dynamics information : velocity, angular velocity, contact curvature.
// Examples are given to highlight the two extremes of contact behaviour:
// total sliding and total rolling. In general contact behaviour at any time is a mixture. 
// The contact should be deleted once the corresponding physical contact is broken.
//
//-----------------------------------------------------------------------------------------------
//
// Sliding
//
// Body1 models a small object sliding back and forth in a U shaped Body2,
// with oscillations decaying, simulating frictional loss of energy.
// Body1 is sliding over Body2 with a single contact point,
// so the contact point does not move relative to Body1,
// but it does move relative to Body2.
// At the contact point Body1 moves relative to Body2
// (A variation is to have the contact moving relative to both, simulating a wide contact area.)
// body[1], body[2] are used for clarity.
//
//-----------------------------------------------------------------------------------------------
//goto rnd;
	printf("Sliding with WhiteFun surface..\n\n");

	contact = paContact::newContact();
	contact->setBody1(body[1]);
	contact->setBody2(body[2]);
	body[1]->getRes()->setAuxDampScale(3.0);
	body[2]->getRes()->setAuxDampScale(3.0);
	body[1]->setSurface(whitesurf);
	body[2]->setSurface(whitesurf);


	contactData.speedContactRelBody1 = 0;

	for(i=0; i<2000; i++)
	{
		p = ((float)sin((float)i/30.0f));
		v = (float)exp(-(float)i/500.0f) * p;

		contactData.contactForce =  0.1f *v; //abs(p+1);			// Simulates change in normal force due to angle of 'U'.
		contactData.speedContactRelBody2 = v;		// Sign of v can matter here for some kinds of surface.
		contactData.speedBody1RelBody2 = v;	// Generally sign doesn't matter here.
		contact->setDynamicData(&contactData);

		paGenerate();							// Calculates 1 block of audio output and plays it out.
												// Use paTick() to just calculate.
	}

	contact->fadeAndDelete(); 

	for(i=0; i<1000; i++) paGenerate();	// Wait.



//-----------------------------------------------------------------------------------------------
//
// Rolling
//
// Body1 is like a cylinder rolling in a U shaped Body2.
// The contact point now moves relative both objects,
// but the relative speed between the bodies at the contact is zero.
// This causes less friction, and the excitation signals are 'damped'
//
//-----------------------------------------------------------------------------------------------



	printf("Rolling with WhiteFun surface...\n\n");


	contact = paContact::newContact();
	contact->setBody1(body[1]);
	contact->setBody2(body[2]);
	body[1]->getRes()->setAuxDampScale(3.0);
	body[2]->getRes()->setAuxDampScale(3.0);

	contactData.speedBody1RelBody2 = 0;		// Characteristic of rolling.

//	whitesurf->setBoostAtZeroSlipSpeed(10);

	for(i=0; i<2000; i++)
	{
		v = (float)exp(-(float)i/500.0f) * (float)sin((float)i/30.0f);
		contactData.contactForce =  0.1f *v;
		contactData.speedContactRelBody1 = v;
		contactData.speedContactRelBody2 = v;
		contact->setDynamicData(&contactData);

		paGenerate();				// Generates 1 block of output.
	}

	contact->fadeAndDelete(); 

	for(i=0; i<1000; i++) paGenerate();



//-----------------------------------------------------------------------------------------------
//rnd:
	printf("Sliding with RndFun surface..\n\n");

	body[1]->setSurface(rndsurf);
	body[1]->getRes()->setAuxDampScale(3.0);
	body[2]->setSurface(rndsurf);
	body[2]->getRes()->setAuxDampScale(3.0);

	contact = paContact::newContact();
	contact->setBody1(body[1]);
	contact->setBody2(body[2]);

	contactData.speedContactRelBody1 = 0;

	for(i=0; i<2000; i++)
	{

		v = (float)exp(-(float)i/500.0f) * ((float)sin((float)i/30.0f));

		contactData.contactForce = 0.1f *v;	// Simulates change in normal force due to angle of 'U'.
		contactData.speedContactRelBody2 = v;
		contactData.speedBody1RelBody2 = v;
		contact->setDynamicData(&contactData);

		paGenerate();							// Generates 1 block of output.
	}

	contact->fadeAndDelete(); 

	for(i=0; i<1000; i++) paGenerate();	// Wait.



//-----------------------------------------------------------------------------------------------


	printf("Rolling with RndFun surface..\n\n");

	body[1]->setSurface(rndsurf);
	body[1]->getRes()->setAuxDampScale(3.0);
	body[2]->setSurface(rndsurf);
	body[2]->getRes()->setAuxDampScale(3.0);

	contact = paContact::newContact();
	contact->setBody1(body[1]);
	contact->setBody2(body[2]);
	contactData.speedBody1RelBody2 = 0;

	for(i=0; i<2000; i++)
	{
		v = (float)exp(-(float)i/500.0f) * (float)sin((float)i/30.0f);
		contactData.contactForce =  0.1f *v;
		contactData.speedContactRelBody1 = v;
		contactData.speedContactRelBody2 = v;
		contact->setDynamicData(&contactData);

		paGenerate();				// Generates 1 block of output.
	}

	contact->fadeAndDelete(); 

	for(i=0; i<1000; i++) paGenerate();



//-----------------------------------------------------------------------------------------------

	printf("Sliding with particle-like RndFun surface..\n\n");

	rndfun->setZeroRate(-1.0f);					// Make surface particle-like
	rndsurf->setContactGain(80000);
//	rndsurf->setGainAtRoll(50.0f);
	rndsurf->setCutoffFreqAtRoll(50.0f);					// Adjust the rel body vel to filter cutoff mapping.
	rndsurf->setCutoffFreqRate(1500.0f);
	rndsurf->setCutoffFreq2AtRoll(50.0f);					// Beef up rolling with optional extra filter layer.
	rndsurf->setCutoffFreq2Rate(1000.0f);

	body[1]->setSurface(rndsurf);
	body[1]->getRes()->setAuxDampScale(3.0);
	body[2]->setSurface(rndsurf);
	body[2]->getRes()->setAuxDampScale(3.0);

	
	contact = paContact::newContact();
	contact->setBody1(body[1]);
	contact->setBody2(body[2]);
	contactData.speedContactRelBody1 = 0;

	for(i=0; i<2000; i++)
	{
		v = (float)exp(-(float)i/500.0f) * ((float)sin((float)i/30.0f));

		contactData.contactForce =  0.1f *v;	// Simulates change in normal force due to angle of 'U'.
		contactData.speedContactRelBody2 = v;
		contactData.speedBody1RelBody2 = v;
		contact->setDynamicData(&contactData);

		paGenerate();							// Generates 1 block of output.
	}

	contact->fadeAndDelete(); 

	for(i=0; i<1000; i++) paGenerate();	// Wait.



//-----------------------------------------------------------------------------------------------


	printf("Rolling with particle-like RndFun surface..\n\n");

	rndfun->setZeroRate(-1.0f);					// Modify to make surface particle-like.
	rndsurf->setContactGain(40000);

	body[1]->setSurface(rndsurf);
	body[1]->getRes()->setAuxDampScale(3.0f);
	body[2]->setSurface(rndsurf);
	body[2]->getRes()->setAuxDampScale(3.0f);

	contact = paContact::newContact();
	contact->setBody1(body[1]);
	contact->setBody2(body[2]);
	contactData.speedBody1RelBody2 = 0;

	for(i=0; i<2000; i++)
	{
		v = (float)exp(-(float)i/500.0f) * (float)sin((float)i/30.0f);
		contactData.contactForce =  0.1f *v;
		contactData.speedContactRelBody1 = v;
		contactData.speedContactRelBody2 = v;
		contact->setDynamicData(&contactData);

		paGenerate();				// Generates 1 block of output.
	}

	contact->fadeAndDelete(); 

	for(i=0; i<1000; i++) paGenerate();



//-----------------------------------------------------------------------------------------------


	printf("Sliding with GridFun surface..\n\n");

	body[1]->setSurface(gridsurf);
	body[1]->getRes()->setAuxDampScale(1.0f);
	body[1]->getRes()->setAuxFreqScale(1.3f);
	body[2]->setSurface(gridsurf);
	body[2]->getRes()->setAuxDampScale(1.0f);
	body[2]->getRes()->setAuxFreqScale(1.5f);

	contact = paContact::newContact();
	contact->setBody1(body[1]);
	contact->setBody2(body[2]);
	contactData.speedContactRelBody1 = 0;

	for(i=0; i<2000; i++)
	{
		v = (float)exp(-(float)i/500.0f) * ((float)sin((float)i/30.0f));

		contactData.contactForce =  0.1f *v;	// Simulates change in normal force due to angle of 'U'.
		contactData.speedContactRelBody2 = v;
		contactData.speedBody1RelBody2 = v;
		contact->setDynamicData(&contactData);

		paGenerate();							// Generates 1 block of output.
	}

	contact->fadeAndDelete(); 

	for(i=0; i<1000; i++) paGenerate();	// Wait.


//-----------------------------------------------------------------------------------------------


	printf("Rolling with GridFun surface..\n\n");

	body[1]->setSurface(gridsurf);
	body[1]->getRes()->setAuxDampScale(1.0f);
	body[1]->getRes()->setAuxFreqScale(1.3f);
	body[2]->setSurface(gridsurf);
	body[2]->getRes()->setAuxDampScale(1.0f);
	body[2]->getRes()->setAuxFreqScale(1.5f);

	contact = paContact::newContact();
	contact->setBody1(body[1]);
	contact->setBody2(body[2]);
	contactData.speedContactRelBody1 = 0;

	contactData.speedBody1RelBody2 = 0;

	for(i=0; i<2000; i++)
	{
		v = (float)exp(-(float)i/500.0f) * (float)sin((float)i/30.0f);
		contactData.contactForce =  0.1f *v;
		contactData.speedContactRelBody1 = v;
		contactData.speedContactRelBody2 = v;
		contact->setDynamicData(&contactData);

		paGenerate();				// Generates 1 block of output.
	}

	contact->fadeAndDelete(); 

	for(i=0; i<1000; i++) paGenerate();



//-----------------------------------------------------------------------------------------------


	printf("Sliding with WavFun surface..\n\n");

	body[1]->setSurface(wavsurf);
	body[1]->getRes()->setAuxDampScale(3.0f);
	body[1]->getRes()->setAuxFreqScale(1.1f);
	body[2]->setSurface(wavsurf);
	body[2]->getRes()->setAuxDampScale(3.0f);
	body[2]->getRes()->setAuxFreqScale(1.2f);

	contact = paContact::newContact();
	contact->setBody1(body[1]);
	contact->setBody2(body[2]);
	contactData.speedContactRelBody1 = 0;

	for(i=0; i<2000; i++)
	{
		v = (float)exp(-(float)i/500.0f) * ((float)sin((float)i/30.0f));

		contactData.contactForce =  0.1f *v;	// Simulates change in normal force due to angle of 'U'.
		contactData.speedContactRelBody2 = v;
		contactData.speedBody1RelBody2 = v;
		contact->setDynamicData(&contactData);

		paGenerate();							// Generates 1 block of output.
	}

	contact->fadeAndDelete(); 

	for(i=0; i<1000; i++) paGenerate();	// Wait.

//-----------------------------------------------------------------------------------------------


	printf("Rolling with WavFun surface..\n\n");

	body[1]->setSurface(wavsurf);
	body[1]->getRes()->setAuxDampScale(3.0f);
	body[1]->getRes()->setAuxFreqScale(1.1f);
	body[2]->setSurface(wavsurf);
	body[2]->getRes()->setAuxDampScale(3.0f);
	body[2]->getRes()->setAuxFreqScale(1.2f);

	contact = paContact::newContact();
	contact->setBody1(body[1]);
	contact->setBody2(body[2]);
	contactData.speedContactRelBody1 = 0;

	contactData.speedBody1RelBody2 = 0;

	for(i=0; i<2000; i++)
	{
		v = (float)exp(-(float)i/500.0f) * (float)sin((float)i/30.0f);
		contactData.contactForce =  0.1f *v;
		contactData.speedContactRelBody1 = v;
		contactData.speedContactRelBody2 = v;
		contact->setDynamicData(&contactData);

		paGenerate();				// Generates 1 block of output.
	}

	contact->fadeAndDelete(); 

	for(i=0; i<1000; i++) paGenerate();


	impacts:
//-----------------------------------------------------------------------------------------------
//
// Impact demonstration
//
// Impacts should be created when a momentary impact is detected.
// They delete themselves once finished.
// 'FunSurfaces' support momentary contact generation for skid type effects,
// but this is quite a crude effect, because this contact generator
// is not dynamically updated. A FunSurface does not skid by default.
// It is better to generate this kind of effect by with a
// dynamically updated contact following a detected impact.
//
// The damping is randomly changed to simulate object touching.
// This effect can be generated automatically when contacts are made,
// provided surface damping factors have been set.

// One body has frequency modulation added. This can be used to model object deformation,
// Which causes modes to move around.

// Impulse-dependent hardness response added, ie nonlinear impact.
// Makes sound brighter when impact is harder.


	// Add skid properties to surface:

	whitesurf->setSkidGain(0.1f);
	whitesurf->setSkidImpulseToForceRatio(0.2f);
	whitesurf->setSkidThickness(0.01f);
	whitesurf->setSkidMinTime(0.01f);
	whitesurf->setSkidMaxTime(0.05f);

// Make impacts brighter above an impulse threshold, to model non-linear surfaces. 
	whitesurf->setHardness(400.0f);
	whitesurf->setImpulseToHardnessBreakpoint(1.0f);
	whitesurf->setImpulseToHardnessScale(1000.0f);     

	for(i=0; i<N_BODIES; i++) body[i]->setSurface(whitesurf);


	printf("Nonlinear surface : increasing impulse causes increasing impact hardness and brightness..\n\n");

	paFloat impulse = 0.5f;
	for(i=0; i<10; i++) {
		impact = paImpact::newImpact();
		if (impact) 
			{
//				body[0]->getRes()->setAuxDampScale(1.5);
				impact->setBody1(body[0]);
				impact->setBody2(0);
				impactData.relTangentSpeedAtImpact = 0.0;
				impulse *= 1.3f;
				impactData.impactImpulse = impulse;
				impact->setDynamicData(&impactData);
			}
		for(j=0; j<200; j++)
			paGenerate();
	}

	printf("Impacts with randomized damping, skids and frequency modulation..\n\n");

	while(1) {

		if (paRnd(0.0f, 1.0f) < .01) 
		{
			impact = paImpact::newImpact();
			if (impact) 
			{
				int body1;
				int body2;
				
				body1 = paRnd(0, N_BODIES-1);
				impact->setBody1(body[body1]);

				body2 = paRnd(0, N_BODIES-1);
				impact->setBody2(body[body2]);

				if (paRnd(0.0f, 1.0f) < .8) 
					body[body1]->getRes()->setAuxDampScale(paRnd(0.5f, 3.0f));

				impactData.relTangentSpeedAtImpact = paRnd(0.0f, 10.0f);	// For skidding.
				impactData.relNormalSpeedAtImpact = paRnd(0.0f, 1.0f);
				impactData.impactImpulse = paRnd(0.0f,2.0f);
				impact->setDynamicData(&impactData);
			}
		}


	//// Frequency modulation
	//// Could be driven by deformable body stress.

	body[0]->getRes()->setAuxFreqScale(1.0f + 0.01f * (float)sin((float)i/10)
		+ 0.01f * (float)sin((float)i/7));
	i++;

		paGenerate();

//		Sleep(50);		// Dynamics tick. Use if running audio thread. In milliseconds

/*
		// Monitor resource useage.
	
		printf("Number of blocks being used = %d\n",
			paBlock::pool.getnActiveObjects() );
		printf("Number of contacts being used = %d\n",
			paContact::pool.getnActiveObjects() );
		printf("Number of impacts being used = %d\n",
			paImpact::pool.getnActiveObjects() );
		printf("Number of resonators being used = %d\n",
			paRes::activeResList.getnMembers() );
*/


	}


	//// Destroy resources
	
	paBlock::pool.deallocate();
	paContact::pool.deallocate();
	paImpact::pool.deallocate();
	paFunSurface::contactGenPool.deallocate();
	paFunSurface::impactGenPool.deallocate();
	paRes::activeResList.deallocate();

	delete whitefun;
	delete rndfun;
	delete gridfun;
	delete wavfun;
	delete whitesurf;
	delete rndsurf;
	delete gridsurf;
	delete wavsurf;
	delete data;

	for(i=0; i< N_BODIES; i++) {
		delete res[i];
		delete body[i];
	}

}


