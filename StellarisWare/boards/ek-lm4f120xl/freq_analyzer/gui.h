//*****************************************************************************
//
// gui.h - Predefines, public functions, and globals for the graphical user
// interface portion of the code.
//
// Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
//
//*****************************************************************************

#ifndef __GUI_H__
#define __GUI_H__

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
// Indices for which slider is which
//
#define FMAX_DISP_SLIDER	0
#define FMIN_DISP_SLIDER	1
#define FSAMP_SLIDER		2
#define NUMBARS_SLIDER		3

//
// Blugh... there's a better way to do this, but I'm lazy.  We need this macro
// defined for the extern statements on variables below, but but if we define
// it here, it's an unnecessary public variable... screw it, I'm lazy.  Just
// put it here -.-
//
#define MAX_NUMBARS				300


//*****************************************************************************
//
// global variables
//
//*****************************************************************************
extern unsigned char LEDDisplay[MAX_NUMBARS];
extern unsigned int  LEDFreqBreakpoints[MAX_NUMBARS + 1];
extern unsigned char LEDDisplayMaxes[MAX_NUMBARS];
extern unsigned char g_ucPrintDbg;
extern unsigned int g_uiNumDisplayBars;
extern unsigned int g_uiMinDisplayFreq;
extern unsigned int g_uiMaxDisplayFreq;
extern unsigned int g_uiSamplingFreq;

//*****************************************************************************
//
// Public Functions
//
//*****************************************************************************
extern void GUIinit(void);
extern void GUIUpdateDisplay(void);
extern void GUIUpdateSlider(int iSliderNum, int iSliderVal);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __GUI_H__
