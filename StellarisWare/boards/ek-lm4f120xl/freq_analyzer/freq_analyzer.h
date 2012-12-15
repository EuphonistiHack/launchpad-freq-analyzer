//*****************************************************************************
//
// led_equalizer.h - Predefines, public functions, and globals for the
// led_equalizer main file
//
// Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
//
//*****************************************************************************

#ifndef __LED_EQUALIZER_H__
#define __LED_EQUALIZER_H__

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
// The maximum number of transfers one UDMA transaction can complete
//
#define UDMA_XFER_MAX			1024

#define DMA_METHOD_SLOW			0
#define DMA_METHOD_FAST			1

//*****************************************************************************
//
// global variables
//
//*****************************************************************************
extern volatile unsigned char g_ucDataReady;
extern volatile unsigned char g_ucDMAMethod;
extern volatile unsigned char g_ucFramesPerSec;
extern volatile unsigned char g_ucLastFramesPerSec;
extern volatile unsigned int g_uiDSPPerSec;
extern volatile unsigned int g_uiLastDSPPerSec;
extern unsigned short g_ulADCValues[NUM_SAMPLES];

//*****************************************************************************
//
// public functions
//
//*****************************************************************************
extern void InitSamplingTimer();

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __LED_EQUALIZER_H__
