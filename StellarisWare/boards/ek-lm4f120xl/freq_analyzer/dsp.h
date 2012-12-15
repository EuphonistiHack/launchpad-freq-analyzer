//*****************************************************************************
//
// dsp.h - Predefines, public functions, and globals for the digital signal
// processing portion of the code.
//
// Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
//
//*****************************************************************************

#ifndef __DSP_H__
#define __DSP_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// pre-processor macros
//
//*****************************************************************************

//
// Number of samples to capture for each FFT process.  Set to CMSIS max for
// best resolution
//
#define NUM_SAMPLES				2048

//
// Amount by which the recorded maximum power decays 15 times per second
//
#define	POWER_DECAY_FACTOR		0.999

//*****************************************************************************
//
// global variables
//
//*****************************************************************************
extern float32_t maxLEDPowers[MAX_NUMBARS];
extern float g_HzPerBin;

//*****************************************************************************
//
// public functions
//
//*****************************************************************************
extern void InitDSP(void);
extern void ProcessData(void);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __DSP_H__
