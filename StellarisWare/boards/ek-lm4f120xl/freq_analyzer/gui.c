//*****************************************************************************
//
// gui.c - Contains all functions and variables needed for the human interface
// portion of the code: sliders, widgets, checkboxes, etc...
//
// Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************
//
//*****************************************************************************

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#include "grlib/grlib.h"
#include "grlib/widget.h"
#include "grlib/canvas.h"
#include "grlib/checkbox.h"
#include "grlib/container.h"
#include "grlib/pushbutton.h"
#include "grlib/slider.h"
#include "drivers/Kentec320x240x16_ssd2119_8bit.h"
#include "drivers/touch.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

#include "arm_math.h"
#include "images.h"
#include "gui.h"
#include "dsp.h"
#include "freq_analyzer.h"

//*****************************************************************************
//
// pre-processor macros
//
//*****************************************************************************

#define REFRESH_RATE			18

#define INIT_SAMPLING_FREQ		26000
#define MAX_SAMPLING_FREQ		80000
#define MIN_SAMPLING_FREQ		4000
#define INTERVAL_SAMPLING_FREQ	1000
#define INIT_NUMBARS			75
#define MIN_NUMBARS				8
#define INTERVAL_NUMBARS		1
#define INIT_DISPLAY_L_FREQ		40
#define MIN_DISPLAY_L_FREQ		1
#define MAX_DISPLAY_L_FREQ		5000
#define INTERVAL_DISPLAY_L_FREQ	10
#define INIT_DISPLAY_U_FREQ		INIT_SAMPLING_FREQ/2
#define MIN_DISPLAY_U_FREQ		1000
#define MAX_DISPLAY_U_FREQ		INIT_SAMPLING_FREQ/2
#define INTERVAL_DISPLAY_U_FREQ	500
#define INIT_DISPLAY_RAIN		0

//
// The strength of the "gravity" at which the rain accelrates downard
//
#define DISPLAY_DECAY_FACTOR	0.9

#define CHECK_RAIN			0
#define CHECK_DEBUG			1
#define CHECK_VERBOSE		2

#define RAIN_HEIGHT	1

//*****************************************************************************
//
// Forward declaration of private functions
//
//*****************************************************************************
static void OnConfigPress(tWidget *pWidget);
static void OnSliderChange(tWidget *pWidget, long lValue);
static void OnButtonPress(tWidget *pWidget);
static void OnCheckChange(tWidget *pWidget, unsigned long bSelected);
static void InitDisplayTimer(void);

//*****************************************************************************
//
// Global variables used for the display functions
//
//*****************************************************************************

//
// The character array used to update the LED display
//
unsigned char LEDDisplay[MAX_NUMBARS];

//
// An array detailing the first and last bin numbers (indexed 0-NUM_SAMPLES*2)
// that should be used for each bar
//
unsigned int  LEDFreqBreakpoints[MAX_NUMBARS + 1];

//
// An array used to keep track of the current location of each "rain drop,"
// which represents the biggest value drawn on screen for a given bar, minus
// the effects of gravity over time
//
unsigned char LEDDisplayMaxes[MAX_NUMBARS];

//
// An array used to keep track of the current "acceleration" of the falling
// "rain drop"
//
unsigned char g_pucGravity[MAX_NUMBARS];

//
// The curent state of the display.
// 	0: displaying bars
//	1: displaying config screen 1
//  2: displaying config screen 2
//
static unsigned char g_ucCfgDisplay;

//
// Flag used to inform the system that a screen refresh needs to happen as soon
// as sanely possible
//
volatile unsigned char g_ucDispRefresh;


//*****************************************************************************
//
// Graphics Library widgets
//
//*****************************************************************************
tContext sContext;

//
// The configuration button at the bottom of the screen, always present
//
RectangularButton(g_sCfgButton, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 210,
				  320, 30, PB_STYLE_OUTLINE, ClrDarkRed, ClrRed, ClrSilver, 0,
				  &g_sFontCm20, "Config", 0, 0, 0, 0, OnConfigPress);

//*****************************************************************************
//
// The first config page, which has all the checkboxes
//
//*****************************************************************************
unsigned char g_ucDispRain;
unsigned char g_ucPrintDbg;

extern tCanvasWidget g_psPanelCfg1;

tCanvasWidget g_psCheckBoxIndicators[] =
{
    CanvasStruct(&g_psPanelCfg1, g_psCheckBoxIndicators + 1, 0,
                 &g_sKentec320x240x16_SSD2119, 260, 30, 50, 42,
                 CANVAS_STYLE_IMG, 0, 0, 0, 0, 0, g_pucLightOff, 0),
    CanvasStruct(&g_psPanelCfg1, g_psCheckBoxIndicators + 2, 0,
                 &g_sKentec320x240x16_SSD2119, 260, 82, 50, 42,
                 CANVAS_STYLE_IMG, 0, 0, 0, 0, 0, g_pucLightOff, 0),
    CanvasStruct(&g_psPanelCfg1, 0, 0,
                 &g_sKentec320x240x16_SSD2119, 260, 134, 50, 42,
                 CANVAS_STYLE_IMG, 0, 0, 0, 0, 0, g_pucLightOff, 0)
};
tCheckBoxWidget g_psCheckBoxes[] =
{
    CheckBoxStruct(&g_psPanelCfg1, g_psCheckBoxes + 1, 0,
                   &g_sKentec320x240x16_SSD2119, 20, 30, 300, 42,
                   CB_STYLE_TEXT, 16, 0, ClrSilver, ClrSilver, &g_sFontCm20,
                   "Make it rain!!!", 0, OnCheckChange),
    CheckBoxStruct(&g_psPanelCfg1, g_psCheckBoxes + 2, 0,
                   &g_sKentec320x240x16_SSD2119, 20, 82, 300, 48,
                   CB_STYLE_TEXT, 16, 0, ClrSilver, ClrSilver, &g_sFontCm20,
				   "Enable Debug (UART)", 0, OnCheckChange),
    CheckBoxStruct(&g_psPanelCfg1, g_psCheckBoxIndicators, 0,
                   &g_sKentec320x240x16_SSD2119, 20, 134, 300, 42,
                   CB_STYLE_TEXT, 16, 0, ClrSilver, ClrSilver, &g_sFontCm20,
                   "Verbose Debug (UART)", 0, OnCheckChange),
};


tCanvasWidget g_psPanelCfg1 = CanvasStruct(0, 0, g_psCheckBoxes,
		&g_sKentec320x240x16_SSD2119, 0, 24, 320, 186, CANVAS_STYLE_FILL, 0, 0,
		0, 0, 0, 0, 0);

//*****************************************************************************
//
// The second config page, which has all the sliders
//
//*****************************************************************************
unsigned int g_uiNumDisplayBars;
unsigned int g_uiMinDisplayFreq;
unsigned int g_uiMaxDisplayFreq;
unsigned int g_uiSamplingFreq;
unsigned char g_ucDisplayRain;
long g_plSliderVal[4];
char g_pcSliderText[4][7];

extern tCanvasWidget g_psPanelCfg2;
extern tPushButtonWidget g_psPushButtons[];
extern tSliderWidget g_psSliders[];
extern tContainerWidget g_sUDispContainer;
extern tContainerWidget g_sLDispContainer;
extern tContainerWidget g_sFSampContainer;
extern tContainerWidget g_sNumBarsContainer;

Container(g_sUDispContainer, &g_psPanelCfg2, &g_sLDispContainer, g_psSliders,
		  &g_sKentec320x240x16_SSD2119, 0, 30, 320, 40, CTR_STYLE_FILL | CTR_STYLE_TEXT | CTR_STYLE_TEXT_CENTER,
          ClrBlack, 0, ClrSilver, &g_sFontCm16, "Max Disp Freq");
Container(g_sLDispContainer, &g_psPanelCfg2, &g_sFSampContainer, g_psSliders+1,
          &g_sKentec320x240x16_SSD2119, 0, 75, 320, 40, CTR_STYLE_FILL | CTR_STYLE_TEXT | CTR_STYLE_TEXT_CENTER,
          ClrBlack, 0, ClrSilver, &g_sFontCm16, "Min Disp Freq");
Container(g_sFSampContainer, &g_psPanelCfg2, &g_sNumBarsContainer, g_psSliders+2,
          &g_sKentec320x240x16_SSD2119, 0, 120, 320, 40, CTR_STYLE_FILL | CTR_STYLE_TEXT | CTR_STYLE_TEXT_CENTER,
          ClrBlack, 0, ClrSilver, &g_sFontCm16, "Sampling Freq");
Container(g_sNumBarsContainer, &g_psPanelCfg2, 0, g_psSliders+3,
          &g_sKentec320x240x16_SSD2119, 0, 165, 320, 40, CTR_STYLE_FILL | CTR_STYLE_TEXT | CTR_STYLE_TEXT_CENTER,
          ClrBlack, 0, ClrSilver, &g_sFontCm16, "Number of Bars");

tPushButtonWidget g_psPushButtons[] =
{
    RectangularButtonStruct(&g_sUDispContainer, g_psPushButtons+1, 0,
                            &g_sKentec320x240x16_SSD2119, 10, 45, 25, 25,
                            PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                            ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
                            &g_sFontCm20, "-", 0, 0, 0, 0, OnButtonPress),
	RectangularButtonStruct(&g_sUDispContainer, 0, 0,
							&g_sKentec320x240x16_SSD2119, 280, 45, 25, 25,
							PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
							ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
							&g_sFontCm20, "+", 0, 0, 0, 0, OnButtonPress),
	RectangularButtonStruct(&g_sLDispContainer, g_psPushButtons+3, 0,
							&g_sKentec320x240x16_SSD2119, 10, 90, 25, 25,
							PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
							ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
							&g_sFontCm20, "-", 0, 0, 0, 0, OnButtonPress),
	RectangularButtonStruct(&g_sLDispContainer, 0, 0,
							&g_sKentec320x240x16_SSD2119, 280, 90, 25, 25,
							PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
							ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
							&g_sFontCm20, "+", 0, 0, 0, 0, OnButtonPress),
	RectangularButtonStruct(&g_sFSampContainer, g_psPushButtons+5, 0,
							&g_sKentec320x240x16_SSD2119, 10, 135, 25, 25,
							PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
							ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
							&g_sFontCm20, "-", 0, 0, 0, 0, OnButtonPress),
	RectangularButtonStruct(&g_sFSampContainer, 0, 0,
							&g_sKentec320x240x16_SSD2119, 280, 135, 25, 25,
							PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
							ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
							&g_sFontCm20, "+", 0, 0, 0, 0, OnButtonPress),
	RectangularButtonStruct(&g_sNumBarsContainer, g_psPushButtons+7, 0,
							&g_sKentec320x240x16_SSD2119, 10, 180, 25, 25,
							PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
							ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
							&g_sFontCm20, "-", 0, 0, 0, 0, OnButtonPress),
	RectangularButtonStruct(&g_sNumBarsContainer, 0, 0,
							&g_sKentec320x240x16_SSD2119, 280, 180, 25, 25,
							PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
							ClrMidnightBlue, ClrBlack, ClrGray, ClrSilver,
							&g_sFontCm20, "+", 0, 0, 0, 0, OnButtonPress),
};

tSliderWidget g_psSliders[] =
{
    SliderStruct(&g_sUDispContainer, g_psPushButtons, 0,
                 &g_sKentec320x240x16_SSD2119, 50, 45, 220, 25,
                 MIN_DISPLAY_U_FREQ, MAX_DISPLAY_U_FREQ, INIT_DISPLAY_U_FREQ,
                 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                 SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrBlueViolet, ClrWhite, ClrSilver, ClrBlack, 0,
                 &g_sFontCm18, "13000", 0, 0, OnSliderChange),
    SliderStruct(&g_sLDispContainer, g_psPushButtons+2, 0,
                 &g_sKentec320x240x16_SSD2119, 50, 90, 220, 25,
                 MIN_DISPLAY_L_FREQ, MAX_DISPLAY_L_FREQ, INIT_DISPLAY_L_FREQ,
                 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                 SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrBlueViolet, ClrWhite, ClrSilver, ClrBlack, 0,
                 &g_sFontCm18, "40", 0, 0, OnSliderChange),
    SliderStruct(&g_sFSampContainer, g_psPushButtons+4, 0,
    			 &g_sKentec320x240x16_SSD2119, 50, 135, 220, 25,
    			 MIN_SAMPLING_FREQ, MAX_SAMPLING_FREQ, INIT_SAMPLING_FREQ,
				 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
				 SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
				 ClrBlueViolet, ClrWhite, ClrSilver, ClrBlack, 0,
				 &g_sFontCm18, "26000", 0, 0, OnSliderChange),
    SliderStruct(&g_sNumBarsContainer, g_psPushButtons+6, 0,
    			 &g_sKentec320x240x16_SSD2119, 50, 180, 220, 25, MIN_NUMBARS,
    			 MAX_NUMBARS, INIT_NUMBARS,
    			 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
				 SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
				 ClrBlueViolet, ClrWhite, ClrSilver, ClrBlack, 0,
				 &g_sFontCm18, "75", 0, 0, OnSliderChange),
};


tCanvasWidget g_psPanelCfg2 = CanvasStruct(0, 0, &g_sUDispContainer,
		&g_sKentec320x240x16_SSD2119, 0, 24, 320, 186, CANVAS_STYLE_FILL, 0, 0,
		0, 0, 0, 0, 0);

//*****************************************************************************
//
// Interrupt Handlers
//
//*****************************************************************************


//*****************************************************************************
//
// The interrupt handler for Timer0A.  This timer is used to keep a constant
// refresh rate for the LCD display.
//
//*****************************************************************************
void
Timer3AIntHandler(void)
{
	int i;
    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

    g_ucDispRefresh = 1;
    for(i=0;i<g_uiNumDisplayBars;i++)
    {
    	maxLEDPowers[i-1] *= POWER_DECAY_FACTOR;
    }
	TimerLoadSet(TIMER3_BASE, TIMER_A, SysCtlClockGet()/REFRESH_RATE);
	TimerEnable(TIMER3_BASE, TIMER_A);
}

//*****************************************************************************
//
// Private Functions
//
//*****************************************************************************


//*****************************************************************************
//
// Initialize the timer that will cause the LED display to refresh, Timer3,
// based on macro'd refresh rate (rate declared in frames per second)
//
//*****************************************************************************
static void
InitDisplayTimer()
{
	//
	// Set up timer3A to be the display timer, interrupting at 15 Hz
	//
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER3_BASE, TIMER_A, SysCtlClockGet()/REFRESH_RATE);
    IntEnable(INT_TIMER3A);
    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A);
}


//*****************************************************************************
//
// This function will draw a column of an image as specified by the input
// parameters.  This is necessary because trying to draw row by row does not
// yield the screen refresh rate necessary for an impressive display.
//
// param pContext: a pointer to the context in which the image is to be drawn
// param *pucImage: a pointer to the character array in which the image is
//		 stored
// param lColumn: the column of the image that is to be drawn
// param lYStart: the Y coordinate at which to start drawing the column
// param lYStop: the Y coordinate at which to stop drawing the column
//
//*****************************************************************************
static void
DrawImgColumn2(const tContext *pContext, const unsigned char *pucImage,
        long lColumn, long lYStart, long lYStop)
{
    long lBPP, lWidth, lHeight, lY;
    const unsigned char *pucPalette;
    unsigned long pulBWPalette[2];

    //
    // Get the image format from the image data.
    //
    lBPP = *pucImage++;

    //
    // Get the image width from the image data.
    //
    lWidth = *(unsigned short *)pucImage;
    pucImage += 2;

    //
    // Get the image height from the image data.
    //
    lHeight = *(unsigned short *)pucImage;
    pucImage += 2;

    //
    // Determine the color palette for the image based on the image format.
    //
    if((lBPP & 0x7f) == IMAGE_FMT_1BPP_UNCOMP)
    {
        //
        // Construct a local "black & white" palette based on the foreground
        // and background colors of the drawing context.
        //
        pulBWPalette[0] = pContext->ulBackground;
        pulBWPalette[1] = pContext->ulForeground;

        //
        // Set the palette pointer to the local "black & white" palette.
        //
        pucPalette = (unsigned char *)pulBWPalette;
    }
    else
    {
        //
        // For 4 and 8 BPP images, the palette is contained at the start of the
        // image data.
        //
        pucPalette = pucImage + 1;
        pucImage += (pucImage[0] * 3) + 4;
    }

    //
    // See if the image is compressed.
    //
    if((lBPP & 0x80))
    {
    	//
		// We don't support compression yet... Deal with it.
		//
		while(1);
    }

	//
	// The image is not compressed.
	// Skip past the data for the rows that lie above the desired region
	//
	pucImage += (((lWidth * lBPP) + 7) / 8) * lYStart;

	//
	// Set the display height
	//
	lHeight = lYStop - lYStart + 1;

	lY = lYStart;

	DpyPixelDrawMultipleVertical(lColumn, lY, lHeight, lWidth,
								 pucImage + lColumn, pucPalette);
}

//*****************************************************************************
//
// The function used to paint the equalizer bars.
//
// param ucResetDisp: Whether we are drawing a fresh display (1) or updating a
//		 previously drawn display (0).  If this is a fresh display, we need to
//		 draw the entire bar from top to bottom, as opposed to just the part of
//		 the bar that has changed since last draw.
// param pContext: the context in which the bars are to be drawn
//
//*****************************************************************************
void
OnEqPaint(unsigned char ucResetDisp, tContext *pContext)
{
    unsigned long ulIdx;
    tRectangle sRect;
    int canvasWidth, maxWidth, width;
    int Xmin, Ymax;
    unsigned int uiColIdx;
    unsigned long ulColor;
    static unsigned char pucPrevHeight[MAX_NUMBARS] = {0};

    //Todo: these should probably be macro'd out...
    Ymax = 210;
    canvasWidth = 300;
    maxWidth = 50;

    //
    // Figure out the width of each bar based on the number of pixels the
    // entire display can take up
    //
    width = canvasWidth / g_uiNumDisplayBars;
    if(width > maxWidth)
    {
    	width = maxWidth;
    }
    Xmin = 10 + (canvasWidth - (width * g_uiNumDisplayBars))/2;

    sRect.sXMin = Xmin;
    sRect.sXMax = Xmin + width - 1;

    //
    // If this is a draw on a fresh display, set all previous heights back to 0
    //
    if(ucResetDisp)
    {
    	for(ulIdx = 0; ulIdx < g_uiNumDisplayBars; ulIdx++)
    	{
    		pucPrevHeight[ulIdx] = 0;
    	}
    }

    //
    // Draw each bar
    //
    for(ulIdx = 0; ulIdx < g_uiNumDisplayBars; ulIdx++)
    {
    	if(pucPrevHeight[ulIdx] < LEDDisplay[ulIdx])
    	{
    		//
    		// If last drawn height is smaller than current, we need to draw
    		// the bar color from previous height to current height.  This way,
    		// we don't have to waste time drawing parts of the bar that are
    		// already on the screen from the last draw.
    		//

    		//
    		// Set the color to be an even gradient from blue to red
    		//
    		ulColor = ((((g_uiNumDisplayBars - ulIdx) * 255) / g_uiNumDisplayBars) << ClrBlueShift) |
    				   (((ulIdx * 255) / g_uiNumDisplayBars) << ClrRedShift);


    		if(g_ucDispRain)
    		{
				if(LEDDisplay[ulIdx] >= LEDDisplayMaxes[ulIdx])
				{
					sRect.sYMax = Ymax - pucPrevHeight[ulIdx] + RAIN_HEIGHT;
				}
				else
				{
					sRect.sYMax = Ymax - pucPrevHeight[ulIdx];
				}
    		}
    		else
    		{
    			sRect.sYMax = Ymax - pucPrevHeight[ulIdx];
    		}
    		sRect.sYMin = Ymax - LEDDisplay[ulIdx] - 1;
    		for(uiColIdx=sRect.sXMin; uiColIdx<=sRect.sXMax; uiColIdx++)
			{
    			DpyLineDrawV(pContext->pDisplay, uiColIdx, sRect.sYMin,
    						 sRect.sYMax,
    						 DpyColorTranslate(pContext->pDisplay, ulColor));
			}
    	}
    	else
    	{
    		//
    		// If last drawn height is bigger than current, we need to draw the
    		// background image from last height to current height.
    		//
    		sRect.sYMin = Ymax - pucPrevHeight[ulIdx] - 1;
    		sRect.sYMax = Ymax - LEDDisplay[ulIdx];
    		for(uiColIdx=sRect.sXMin; uiColIdx<=sRect.sXMax; uiColIdx++)
    		{
    			DrawImgColumn2(pContext, g_pucImage, uiColIdx, sRect.sYMin, sRect.sYMax);
    		}
    	}


    	if(g_ucDispRain)
    	{
			if(LEDDisplayMaxes[ulIdx] <= LEDDisplay[ulIdx])
			{
				//
				// We have a new maximum... no need for gravity calculations
				// this time, and drawing the bar (from above step) will have
				// drawn over where the raindrop was last time
				//
				LEDDisplayMaxes[ulIdx] = LEDDisplay[ulIdx];
				g_pucGravity[ulIdx] = 0;
			}
			else
			{
				//
				// No new maximum... we need to draw the background image back
				// over where the rain drop was last time
				//
				sRect.sYMax = Ymax - LEDDisplayMaxes[ulIdx] + RAIN_HEIGHT;
				sRect.sYMin = Ymax - LEDDisplayMaxes[ulIdx];

				for(uiColIdx=sRect.sXMin; uiColIdx<=sRect.sXMax; uiColIdx++)
				{
					DrawImgColumn2(pContext, g_pucImage, uiColIdx, sRect.sYMin, sRect.sYMax);
				}

				//
				// If gravity droves the last maximum below the current value,
				// then current value is new maximum
				//
				if(g_pucGravity[ulIdx] > LEDDisplayMaxes[ulIdx])
				{
					LEDDisplayMaxes[ulIdx] = LEDDisplay[ulIdx];
				}
				else
				{
					//
					// apply gravity to the raindrop
					//
					LEDDisplayMaxes[ulIdx] -= g_pucGravity[ulIdx];
					g_pucGravity[ulIdx]++;
				}
			}

			//
			// Draw the raindrop
			//
			if((LEDDisplayMaxes[ulIdx] > RAIN_HEIGHT) &&
			   (LEDDisplayMaxes[ulIdx] > LEDDisplay[ulIdx]))
			{
				sRect.sYMax = Ymax - LEDDisplayMaxes[ulIdx] + RAIN_HEIGHT;
				sRect.sYMin = Ymax - LEDDisplayMaxes[ulIdx];
				ulColor = ClrLightGrey;
				GrContextForegroundSet(pContext, ulColor);
				GrRectFill(pContext, &sRect);
			}
    	}
    	sRect.sXMin += width;
    	sRect.sXMax += width;
    	pucPrevHeight[ulIdx] = LEDDisplay[ulIdx];
    }
}

//*****************************************************************************
//
// Update the global configurable variables based on the slider values
//
//*****************************************************************************
void
UpdateGConfigs(void)
{
	g_uiMinDisplayFreq = g_plSliderVal[FMIN_DISP_SLIDER];
	g_uiMaxDisplayFreq = g_plSliderVal[FMAX_DISP_SLIDER];
	g_uiSamplingFreq = g_plSliderVal[FSAMP_SLIDER];
	g_uiNumDisplayBars = g_plSliderVal[NUMBARS_SLIDER];

	InitSamplingTimer();
	InitDSP();
}

//*****************************************************************************
//
// Function handler for when a check box is pressed
//
//*****************************************************************************
static void
OnCheckChange(tWidget *pWidget, unsigned long bSelected)
{

    if(pWidget == (tWidget *)(g_psCheckBoxes+CHECK_RAIN))
    {
    	//
    	// Handle the "enable rain" checkbox
    	//
        if(bSelected)
        {
        	g_ucDispRain = 1;
        	CanvasImageSet(g_psCheckBoxIndicators + CHECK_RAIN,
        			       g_pucLightOn);
        }
        else
        {
        	g_ucDispRain = 0;
        	CanvasImageSet(g_psCheckBoxIndicators + CHECK_RAIN,
        				   g_pucLightOff);
        }
        WidgetPaint((tWidget *)(g_psCheckBoxIndicators + CHECK_RAIN));
    }
    else if(pWidget == (tWidget *)(g_psCheckBoxes+CHECK_DEBUG))
    {
    	//
    	// Handle the "debug level one" checkbox
    	//
        if(bSelected)
        {
        	g_ucPrintDbg |= 1;
        	CanvasImageSet(g_psCheckBoxIndicators + CHECK_DEBUG,
						   g_pucLightOn);
        }
        else
        {
        	g_ucPrintDbg = 0;
        	CanvasImageSet(g_psCheckBoxIndicators + CHECK_DEBUG,
        							   g_pucLightOff);
        	CanvasImageSet(g_psCheckBoxIndicators + CHECK_VERBOSE,
        							   g_pucLightOff);
        }
        WidgetPaint((tWidget *)(g_psCheckBoxIndicators + CHECK_DEBUG));
        WidgetPaint((tWidget *)(g_psCheckBoxIndicators + CHECK_VERBOSE));
    }
    else
    {
    	//
    	// Handle the "debug level two" checkbox
    	//
    	if(bSelected)
    	{
    		g_ucPrintDbg = 3;
    		CanvasImageSet(g_psCheckBoxIndicators + CHECK_DEBUG,
						   g_pucLightOn);
    		CanvasImageSet(g_psCheckBoxIndicators + CHECK_VERBOSE,
						   g_pucLightOn);
    	}
    	else
    	{
    		g_ucPrintDbg &= (~2);
    		CanvasImageSet(g_psCheckBoxIndicators + CHECK_VERBOSE,
						   g_pucLightOff);
    	}
        WidgetPaint((tWidget *)(g_psCheckBoxIndicators + CHECK_DEBUG));
        WidgetPaint((tWidget *)(g_psCheckBoxIndicators + CHECK_VERBOSE));
    }
}

//*****************************************************************************
//
// Function handler for when the config button at the bottom is pressed
//
//*****************************************************************************
static void
OnConfigPress(tWidget *pWidget)
{

	if(g_ucCfgDisplay == 0)
	{
		//
		// We were displaying bars, so now we need to display the first config
		// screen
		//

		//
		// redraw the background image, which will erase the title text for the
		// last page
		//
		GrImageDraw(&sContext, g_pucImage, 0, 0);
		UARTprintf("\nDisplay off, show cfg1\n");

		//
		// Disable automatic screen refresh.  We'll repaint the screen using
		// widget paint functions until we get back to displaying bars
		//
		g_ucDispRefresh = 0;
		TimerDisable(TIMER3_BASE, TIMER_A);

		//
		// Update state
		//
		g_ucCfgDisplay = 1;

		//
		// Add and paint the first config screen and its title block
		//
		WidgetAdd(WIDGET_ROOT, (tWidget *)&g_psPanelCfg1);
		WidgetPaint(WIDGET_ROOT);
		GrContextFontSet(&sContext, &g_sFontCm16);
		GrContextForegroundSet(&sContext, ClrLightGrey);
		GrStringDrawCentered(&sContext, "Configuration Page 1", 20,
							 GrContextDpyWidthGet(&sContext) / 2, 10, 0);
	}
	else if(g_ucCfgDisplay == 1)
	{
		//
		// We were displaying config screen 1, now we display config screen 2
		//

		//
		// Remove the last config screen
		//
		WidgetRemove((tWidget *)&g_psPanelCfg1);

		//
		// Draw the background image again, erasing the last title block
		//
		GrImageDraw(&sContext, g_pucImage, 0, 0);
		UARTprintf("\nCfg1 to Cfg 2\n");

		//
		// Add and paint the second config screen and its title block
		//
		WidgetAdd(WIDGET_ROOT, (tWidget *)&g_psPanelCfg2);
		WidgetPaint(WIDGET_ROOT);
		GrContextFontSet(&sContext, &g_sFontCm16);
		GrContextForegroundSet(&sContext, ClrLightGrey);
	    GrStringDrawCentered(&sContext, "Configuration Page 2", 20,
	                         GrContextDpyWidthGet(&sContext) / 2, 10, 0);

	    //
	    // Update state
	    //
		g_ucCfgDisplay = 2;
	}
	else if(g_ucCfgDisplay == 2)
	{
		//
		// Go back to displaying bars
		//
		UARTprintf("Cfg2 to Display on, save changes\n");

		//
		// Remove the old config screen
		//
		WidgetRemove((tWidget *)&g_psPanelCfg2);

		//
		// update states
		//
		g_ucCfgDisplay = 0;
		g_ucDispRefresh = 2;
	}
}

//*****************************************************************************
//
// Function used to increment a slider by an even value.  For example, if we
// want a pushbutton to increment a slider by 500, and the current value is
// 1297, we want the next value to be 1500, then 2000, so on so forth
//
// param uiVal: the current value of the slider
// param iIncAmount: the increment interval (+ or -)
// param uiMax: the maximum value we can increment to
// param uiMin: the minimum value we can increment to
//
// return: the incremented value
//
//*****************************************************************************
unsigned int
uiIncrementValue(unsigned int uiVal, int iIncAmount,
				 unsigned int uiMax, unsigned int uiMin)
{
	unsigned int uiRetVal;

	uiRetVal = uiVal + iIncAmount;
	if(uiRetVal > uiMax)
	{
		return(uiMax);
	}
	else if(uiRetVal < uiMin)
	{
		return(uiMin);
	}

	uiRetVal = uiVal + iIncAmount - ((int)uiVal % iIncAmount);
	return(uiRetVal);
}


//*****************************************************************************
//
// Function handler for one of the slider + or - pushbuttons being pressed.
//
// This function gets kind of complicated, because we need to prevent the user
// from setting nonsensical values.  Max display freq can't be bigger than
// sampling frequency/2 or smaller than Min display frequency, etc...
//
//*****************************************************************************
static void
OnButtonPress(tWidget *pWidget)
{

	unsigned int uiNewConfigVal;

	//
	// Handle - and + for the max display freq
	//
	if(pWidget == (tWidget *)&g_psPushButtons[2*FMAX_DISP_SLIDER])
	{
		//
		// increment to new value
		//
		uiNewConfigVal = uiIncrementValue(g_plSliderVal[FMAX_DISP_SLIDER],
						                  -INTERVAL_DISPLAY_U_FREQ,
						                  MAX_DISPLAY_U_FREQ,
						                  MIN_DISPLAY_U_FREQ);

		if(uiNewConfigVal <= g_plSliderVal[FMIN_DISP_SLIDER])
		{
			//
			// If current value is bad, make the text go red and set the value
			// to the nearest sane value possible to what the user wants.
			//
			SliderTextColorSet(&g_psSliders[FMAX_DISP_SLIDER], ClrRed);
			SliderBackgroundTextColorSet(&g_psSliders[FMAX_DISP_SLIDER],
										 ClrRed);
			g_plSliderVal[FMAX_DISP_SLIDER] = g_plSliderVal[FMIN_DISP_SLIDER]+1;
			usprintf(g_pcSliderText[FMAX_DISP_SLIDER], "%d",
					 g_plSliderVal[FMIN_DISP_SLIDER]+1);
			SliderTextSet(&g_psSliders[FMAX_DISP_SLIDER],
						  g_pcSliderText[FMAX_DISP_SLIDER]);
			SliderValueSet(&g_psSliders[FMAX_DISP_SLIDER],
							g_plSliderVal[FMIN_DISP_SLIDER]+1);
			WidgetPaint((tWidget *)&g_psSliders[FMAX_DISP_SLIDER]);
		}
		else
		{
			//
			// If the value is good, update the slider text to the new value,
			// update the slider value to the new value, and repaint the widget
			//
			SliderTextColorSet(&g_psSliders[FMAX_DISP_SLIDER], ClrBlack);
			SliderBackgroundTextColorSet(&g_psSliders[FMAX_DISP_SLIDER],
									     ClrBlack);
			g_plSliderVal[FMAX_DISP_SLIDER] = uiNewConfigVal;
			usprintf(g_pcSliderText[FMAX_DISP_SLIDER], "%d",
					 g_plSliderVal[FMAX_DISP_SLIDER]);
			SliderTextSet(&g_psSliders[FMAX_DISP_SLIDER],
						  g_pcSliderText[FMAX_DISP_SLIDER]);
			SliderValueSet(&g_psSliders[FMAX_DISP_SLIDER],
						   g_plSliderVal[FMAX_DISP_SLIDER]);
			WidgetPaint((tWidget *)&g_psSliders[FMAX_DISP_SLIDER]);
		}
	}
	else if(pWidget == (tWidget *)&g_psPushButtons[2*FMAX_DISP_SLIDER+1])
	{
		uiNewConfigVal = uiIncrementValue(g_plSliderVal[FMAX_DISP_SLIDER],
										  INTERVAL_DISPLAY_U_FREQ,
										  MAX_DISPLAY_U_FREQ,
										  MIN_DISPLAY_U_FREQ);
		g_plSliderVal[FMAX_DISP_SLIDER] = uiNewConfigVal;
		SliderTextColorSet(&g_psSliders[FMAX_DISP_SLIDER], ClrBlack);
		SliderBackgroundTextColorSet(&g_psSliders[FMAX_DISP_SLIDER],
									 ClrBlack);
		usprintf(g_pcSliderText[FMAX_DISP_SLIDER], "%d",
				 g_plSliderVal[FMAX_DISP_SLIDER]);
		SliderTextSet(&g_psSliders[FMAX_DISP_SLIDER],
					  g_pcSliderText[FMAX_DISP_SLIDER]);
		SliderValueSet(&g_psSliders[FMAX_DISP_SLIDER],
					   g_plSliderVal[FMAX_DISP_SLIDER]);
		WidgetPaint((tWidget *)&g_psSliders[FMAX_DISP_SLIDER]);
	}

	//
	// Handle - and + for the min display frequency
	//
	if(pWidget == (tWidget *)&g_psPushButtons[2*FMIN_DISP_SLIDER])
	{
		uiNewConfigVal = uiIncrementValue(g_plSliderVal[FMIN_DISP_SLIDER],
										  -INTERVAL_DISPLAY_L_FREQ,
										  MAX_DISPLAY_L_FREQ,
										  MIN_DISPLAY_L_FREQ);
		g_plSliderVal[FMIN_DISP_SLIDER] = uiNewConfigVal;
		usprintf(g_pcSliderText[FMIN_DISP_SLIDER], "%d",
				 g_plSliderVal[FMIN_DISP_SLIDER]);
		SliderTextColorSet(&g_psSliders[FMIN_DISP_SLIDER], ClrBlack);
		SliderBackgroundTextColorSet(&g_psSliders[FMIN_DISP_SLIDER], ClrBlack);
		SliderTextSet(&g_psSliders[FMIN_DISP_SLIDER],
					  g_pcSliderText[FMIN_DISP_SLIDER]);
		SliderValueSet(&g_psSliders[FMIN_DISP_SLIDER],
					   g_plSliderVal[FMIN_DISP_SLIDER]);
		WidgetPaint((tWidget *)&g_psSliders[FMIN_DISP_SLIDER]);
	}
	else if(pWidget == (tWidget *)&g_psPushButtons[2*FMIN_DISP_SLIDER+1])
	{
		uiNewConfigVal = uiIncrementValue(g_plSliderVal[FMIN_DISP_SLIDER],
										  INTERVAL_DISPLAY_L_FREQ,
										  MAX_DISPLAY_L_FREQ,
										  MIN_DISPLAY_L_FREQ);
		if(uiNewConfigVal >= g_plSliderVal[FMAX_DISP_SLIDER])
		{
			SliderTextColorSet(&g_psSliders[FMIN_DISP_SLIDER], ClrRed);
			SliderBackgroundTextColorSet(&g_psSliders[FMIN_DISP_SLIDER],
									     ClrRed);
			g_plSliderVal[FMIN_DISP_SLIDER] = g_plSliderVal[FMAX_DISP_SLIDER]-1;
			usprintf(g_pcSliderText[FMIN_DISP_SLIDER], "%d",
					 g_plSliderVal[FMAX_DISP_SLIDER]-1);
			SliderTextSet(&g_psSliders[FMIN_DISP_SLIDER],
						  g_pcSliderText[FMIN_DISP_SLIDER]);
			SliderValueSet(&g_psSliders[FMIN_DISP_SLIDER],
							g_plSliderVal[FMAX_DISP_SLIDER]-1);
			WidgetPaint((tWidget *)&g_psSliders[FMIN_DISP_SLIDER]);
		}
		else
		{
			SliderTextColorSet(&g_psSliders[FMIN_DISP_SLIDER], ClrBlack);
			SliderBackgroundTextColorSet(&g_psSliders[FMIN_DISP_SLIDER],
									     ClrBlack);
			g_plSliderVal[FMIN_DISP_SLIDER] = uiNewConfigVal;
			usprintf(g_pcSliderText[FMIN_DISP_SLIDER], "%d",
					 g_plSliderVal[FMIN_DISP_SLIDER]);
			SliderTextSet(&g_psSliders[FMIN_DISP_SLIDER],
						  g_pcSliderText[FMIN_DISP_SLIDER]);
			SliderValueSet(&g_psSliders[FMIN_DISP_SLIDER],
						   g_plSliderVal[FMIN_DISP_SLIDER]);
			WidgetPaint((tWidget *)&g_psSliders[FMIN_DISP_SLIDER]);
		}
	}

	//
	// Handle - and + for the sampling frequency
	//
	if(pWidget == (tWidget *)&g_psPushButtons[2*FSAMP_SLIDER])
	{
		uiNewConfigVal = uiIncrementValue(g_plSliderVal[FSAMP_SLIDER],
										  -INTERVAL_SAMPLING_FREQ,
										  MAX_SAMPLING_FREQ,
										  MIN_SAMPLING_FREQ);
		if(uiNewConfigVal < 2*g_plSliderVal[FMAX_DISP_SLIDER])
		{
			SliderTextColorSet(&g_psSliders[FSAMP_SLIDER], ClrRed);
			SliderBackgroundTextColorSet(&g_psSliders[FSAMP_SLIDER], ClrRed);
			g_plSliderVal[FSAMP_SLIDER] = 2*g_plSliderVal[FMAX_DISP_SLIDER];
			usprintf(g_pcSliderText[FSAMP_SLIDER], "%d",
					 2*g_plSliderVal[FMAX_DISP_SLIDER]);
			SliderTextSet(&g_psSliders[FSAMP_SLIDER],
						  g_pcSliderText[FSAMP_SLIDER]);
			SliderValueSet(&g_psSliders[FSAMP_SLIDER],
						   2*g_plSliderVal[FMAX_DISP_SLIDER]);
			WidgetPaint((tWidget *)&g_psSliders[FSAMP_SLIDER]);
		}
		else
		{
			SliderTextColorSet(&g_psSliders[FSAMP_SLIDER], ClrBlack);
			SliderBackgroundTextColorSet(&g_psSliders[FSAMP_SLIDER], ClrBlack);
			g_plSliderVal[FSAMP_SLIDER] = uiNewConfigVal;
			usprintf(g_pcSliderText[FSAMP_SLIDER], "%d",
					 g_plSliderVal[FSAMP_SLIDER]);
			SliderTextSet(&g_psSliders[FSAMP_SLIDER],
					      g_pcSliderText[FSAMP_SLIDER]);
			SliderValueSet(&g_psSliders[FSAMP_SLIDER],
					       g_plSliderVal[FSAMP_SLIDER]);
			WidgetPaint((tWidget *)&g_psSliders[FSAMP_SLIDER]);
		}
	}
	else if(pWidget == (tWidget *)&g_psPushButtons[2*FSAMP_SLIDER+1])
	{
		uiNewConfigVal = uiIncrementValue(g_plSliderVal[FSAMP_SLIDER],
										  INTERVAL_SAMPLING_FREQ,
										  MAX_SAMPLING_FREQ,
										  MIN_SAMPLING_FREQ);
		SliderTextColorSet(&g_psSliders[FSAMP_SLIDER], ClrBlack);
		SliderBackgroundTextColorSet(&g_psSliders[FSAMP_SLIDER], ClrBlack);
		g_plSliderVal[FSAMP_SLIDER] = uiNewConfigVal;
		usprintf(g_pcSliderText[FSAMP_SLIDER], "%d",
				 g_plSliderVal[FSAMP_SLIDER]);
		SliderTextSet(&g_psSliders[FSAMP_SLIDER],
					  g_pcSliderText[FSAMP_SLIDER]);
		SliderValueSet(&g_psSliders[FSAMP_SLIDER],
				       g_plSliderVal[FSAMP_SLIDER]);
		WidgetPaint((tWidget *)&g_psSliders[FSAMP_SLIDER]);
	}

	if(pWidget == (tWidget *)&g_psPushButtons[2*NUMBARS_SLIDER])
	{
		uiNewConfigVal = uiIncrementValue(g_plSliderVal[NUMBARS_SLIDER],
										  -INTERVAL_NUMBARS,
										  MAX_NUMBARS,
										  MIN_NUMBARS);
		g_plSliderVal[NUMBARS_SLIDER] = uiNewConfigVal;
		usprintf(g_pcSliderText[NUMBARS_SLIDER], "%d",
				 g_plSliderVal[NUMBARS_SLIDER]);
		SliderTextSet(&g_psSliders[NUMBARS_SLIDER],
				      g_pcSliderText[NUMBARS_SLIDER]);
		SliderValueSet(&g_psSliders[NUMBARS_SLIDER],
					   g_plSliderVal[NUMBARS_SLIDER]);
		WidgetPaint((tWidget *)&g_psSliders[NUMBARS_SLIDER]);
	}
	else if(pWidget == (tWidget *)&g_psPushButtons[2*NUMBARS_SLIDER+1])
	{
		uiNewConfigVal = uiIncrementValue(g_plSliderVal[NUMBARS_SLIDER],
										  INTERVAL_NUMBARS,
										  MAX_NUMBARS,
										  MIN_NUMBARS);
		g_plSliderVal[NUMBARS_SLIDER] = uiNewConfigVal;
		usprintf(g_pcSliderText[NUMBARS_SLIDER], "%d",
				 g_plSliderVal[NUMBARS_SLIDER]);
		SliderTextSet(&g_psSliders[NUMBARS_SLIDER],
					  g_pcSliderText[NUMBARS_SLIDER]);
		SliderValueSet(&g_psSliders[NUMBARS_SLIDER],
					   g_plSliderVal[NUMBARS_SLIDER]);
		WidgetPaint((tWidget *)&g_psSliders[NUMBARS_SLIDER]);
	}
}

//*****************************************************************************
//
// Function handler for changing a slider value by pressing in the area of the
// slider itself
//
// This function gets kind of complicated, because we need to prevent the user
// from setting nonsensical values.  Max display freq can't be bigger than
// sampling frequency/2 or smaller than Min display frequency, etc...
//
//*****************************************************************************
static void
OnSliderChange(tWidget *pWidget, long lValue)
{

    if(pWidget == (tWidget *)&g_psSliders[FMAX_DISP_SLIDER])
	{
    	//
    	// hanlde changes to the maximum display frequency slider
    	//
    	if(lValue <= g_plSliderVal[FMIN_DISP_SLIDER])
    	{
    		SliderTextColorSet(&g_psSliders[FMAX_DISP_SLIDER], ClrRed);
			SliderBackgroundTextColorSet(&g_psSliders[FMAX_DISP_SLIDER],
										 ClrRed);
			g_plSliderVal[FMAX_DISP_SLIDER] = g_plSliderVal[FMIN_DISP_SLIDER]+1;
			usprintf(g_pcSliderText[FMAX_DISP_SLIDER], "%d",
					 g_plSliderVal[FMAX_DISP_SLIDER]);
			SliderTextSet(&g_psSliders[FMAX_DISP_SLIDER],
					      g_pcSliderText[FMAX_DISP_SLIDER]);
			WidgetPaint((tWidget *)&g_psSliders[FMAX_DISP_SLIDER]);
    	}
    	else
    	{
    		SliderTextColorSet(&g_psSliders[FMAX_DISP_SLIDER], ClrBlack);
			SliderBackgroundTextColorSet(&g_psSliders[FMAX_DISP_SLIDER],
										 ClrBlack);
			g_plSliderVal[FMAX_DISP_SLIDER] = lValue;
			usprintf(g_pcSliderText[FMAX_DISP_SLIDER], "%d", lValue);
			SliderTextSet(&g_psSliders[FMAX_DISP_SLIDER],
						  g_pcSliderText[FMAX_DISP_SLIDER]);
			WidgetPaint((tWidget *)&g_psSliders[FMAX_DISP_SLIDER]);
    	}
	}
    if(pWidget == (tWidget *)&g_psSliders[FMIN_DISP_SLIDER])
	{
    	//
		// hanlde changes to the minimum display frequency slider
		//
    	if(lValue >= g_plSliderVal[FMAX_DISP_SLIDER])
		{
			SliderTextColorSet(&g_psSliders[FMIN_DISP_SLIDER], ClrRed);
			SliderBackgroundTextColorSet(&g_psSliders[FMIN_DISP_SLIDER],
										 ClrRed);
			g_plSliderVal[FMIN_DISP_SLIDER] = g_plSliderVal[FMAX_DISP_SLIDER]-1;
			usprintf(g_pcSliderText[FMIN_DISP_SLIDER], "%d",
					 g_plSliderVal[FMIN_DISP_SLIDER]);
			SliderTextSet(&g_psSliders[FMIN_DISP_SLIDER],
						  g_pcSliderText[FMIN_DISP_SLIDER]);
			WidgetPaint((tWidget *)&g_psSliders[FMIN_DISP_SLIDER]);
		}
		else
		{
			SliderTextColorSet(&g_psSliders[FMIN_DISP_SLIDER], ClrBlack);
			SliderBackgroundTextColorSet(&g_psSliders[FMIN_DISP_SLIDER],
									     ClrBlack);
			g_plSliderVal[FMIN_DISP_SLIDER] = lValue;
			usprintf(g_pcSliderText[FMIN_DISP_SLIDER], "%d", lValue);
			SliderTextSet(&g_psSliders[FMIN_DISP_SLIDER],
						  g_pcSliderText[FMIN_DISP_SLIDER]);
			WidgetPaint((tWidget *)&g_psSliders[FMIN_DISP_SLIDER]);
		}
	}
    if(pWidget == (tWidget *)&g_psSliders[FSAMP_SLIDER])
    {
    	//
		// hanlde changes to the sampling frequency slider
		//
    	if(lValue < 2*g_plSliderVal[FMAX_DISP_SLIDER])
    	{
    		SliderTextColorSet(&g_psSliders[FSAMP_SLIDER], ClrRed);
    		SliderBackgroundTextColorSet(&g_psSliders[FSAMP_SLIDER], ClrRed);
    		g_plSliderVal[FSAMP_SLIDER] = 2*g_plSliderVal[FMAX_DISP_SLIDER];
    		usprintf(g_pcSliderText[FSAMP_SLIDER], "%d",
    				 2*g_plSliderVal[FMAX_DISP_SLIDER]);
    		SliderTextSet(&g_psSliders[FSAMP_SLIDER],
    					  g_pcSliderText[FSAMP_SLIDER]);
    		WidgetPaint((tWidget *)&g_psSliders[FSAMP_SLIDER]);
    	}
    	else
    	{
    		SliderTextColorSet(&g_psSliders[FSAMP_SLIDER], ClrBlack);
			SliderBackgroundTextColorSet(&g_psSliders[FSAMP_SLIDER], ClrBlack);
			g_plSliderVal[FSAMP_SLIDER] = lValue;
			usprintf(g_pcSliderText[FSAMP_SLIDER], "%d", lValue);
			SliderTextSet(&g_psSliders[FSAMP_SLIDER],
					      g_pcSliderText[FSAMP_SLIDER]);
			WidgetPaint((tWidget *)&g_psSliders[FSAMP_SLIDER]);
    	}

    	//
    	// Update the range of the Max Disp Freq slider based on the new
    	// sampling freq
    	//
    	SliderRangeSet(&g_psSliders[FMAX_DISP_SLIDER], MIN_DISPLAY_U_FREQ,
    			       g_plSliderVal[FSAMP_SLIDER]/2);
    	WidgetPaint((tWidget *)&g_psSliders[FMAX_DISP_SLIDER]);
	}
    if(pWidget == (tWidget *)&g_psSliders[NUMBARS_SLIDER])
	{
    	//
		// hanlde changes to the "number of bars" slider
		//
		g_plSliderVal[NUMBARS_SLIDER] = lValue;
		usprintf(g_pcSliderText[NUMBARS_SLIDER], "%d", lValue);
		SliderTextSet(&g_psSliders[NUMBARS_SLIDER],
				      g_pcSliderText[NUMBARS_SLIDER]);
		WidgetPaint((tWidget *)&g_psSliders[NUMBARS_SLIDER]);
	}
}

//*****************************************************************************
//
// Public functions
//
//*****************************************************************************

//*****************************************************************************
//
// Function used to update the display, if necessary.
//
//*****************************************************************************
void
GUIUpdateDisplay(void)
{
	if(g_ucDispRefresh)
	{
		if(g_ucDispRefresh == 2)
		{
			GrImageDraw(&sContext, g_pucImage, 0, 0);
			UpdateGConfigs();
			GrContextFontSet(&sContext, &g_sFontCm16);
			GrContextForegroundSet(&sContext, ClrLightGrey);
			GrStringDrawCentered(&sContext, "Frequency Analyzer", 19,
								 GrContextDpyWidthGet(&sContext) / 2, 10, 0);
			OnEqPaint(1, &sContext);
		}
		OnEqPaint(0, &sContext);
		g_ucFramesPerSec++;
		g_ucDispRefresh = 0;
		TimerEnable(TIMER3_BASE, TIMER_A);
	}
}

//*****************************************************************************
//
// Function used to update a slider value.
//
// param iSliderNum: the slider to change
// param iSliderVal: the value to change the slider to
//
//*****************************************************************************
void
GUIUpdateSlider(int iSliderNum, int iSliderVal)
{
	g_plSliderVal[iSliderNum] = iSliderVal;
	SliderValueSet(&g_psSliders[iSliderNum], g_plSliderVal[iSliderNum]);
}

//*****************************************************************************
//
// This function will initialize all peripherals, devices, and variables
// needed for the graphical user interface.
//
//*****************************************************************************
void
GUIinit(void)
{
	int i;

	InitDisplayTimer();

    //
    // Initialize the display driver.
    //
    Kentec320x240x16_SSD2119Init();

    //
    // Initialize the graphics context.
    //
    GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_psPanelCfg1);
    WidgetRemove((tWidget *)&g_psPanelCfg1);
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_psPanelCfg2);
    WidgetRemove((tWidget *)&g_psPanelCfg2);

	GrImageDraw(&sContext, g_pucImage, 0, 0);
	GrContextFontSet(&sContext, &g_sFontCm16);
	GrContextForegroundSet(&sContext, ClrLightGrey);
	GrStringDrawCentered(&sContext, "Frequency Analyzer", 19,
						 GrContextDpyWidthGet(&sContext) / 2, 10, 0);

	TouchScreenInit();
	TouchScreenCallbackSet(WidgetPointerMessage);

	WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sCfgButton);

	g_ucDispRain = 0;
	g_ucPrintDbg = 0;
	g_ucDisplayRain = INIT_DISPLAY_RAIN;
	g_uiSamplingFreq = INIT_SAMPLING_FREQ;
	g_uiMinDisplayFreq = INIT_DISPLAY_L_FREQ;
	g_uiMaxDisplayFreq = INIT_DISPLAY_U_FREQ;
	g_uiNumDisplayBars = INIT_NUMBARS;
	g_plSliderVal[FMIN_DISP_SLIDER] = g_uiMinDisplayFreq;
	g_plSliderVal[FMAX_DISP_SLIDER] = g_uiMaxDisplayFreq;
	g_plSliderVal[FSAMP_SLIDER] = g_uiSamplingFreq;
	g_plSliderVal[NUMBARS_SLIDER] = g_uiNumDisplayBars;


	for(i=0;i<g_uiNumDisplayBars;i++)
	{
		g_pucGravity[i] = 0;
	}
}
