//*****************************************************************************
//
// gui.c - Contains all functions and variables needed for the signal
// processing portion of the code
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

#include "inc/hw_adc.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"

#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

#include "arm_math.h"
#include "images.h"
#include "gui.h"
#include "dsp.h"
#include "freq_analyzer.h"
#include <math.h>


//*****************************************************************************
//
// Public global variables
//
//*****************************************************************************

//
// The array used to store the maximum power seen over a frequency range.
//
float32_t maxLEDPowers[MAX_NUMBARS];

//
// The range in Hertz of each frequency bin
//
float g_HzPerBin;

//*****************************************************************************
//
// Private predefines and variables used for the FFT portion of the DSP loop
//
//*****************************************************************************

//
// Flag used to determine whether to calculate forward (0) or inverse(1) fft
//
#define INVERT_FFT				0
//
// Flag used to determine if fft result will be output in standard bit order(1)
// or reversed bit order (0)
//
#define BIT_ORDER_FFT			1

//
// The array used to store the results of the fast fourier transform.  Each
// element in this array represents the power found in a frequency bin of width
// Fs / FFT_length
//
static float32_t g_fFFTResult[NUM_SAMPLES * 2];

//
// FFT and CMSIS config structures
//
arm_rfft_instance_f32 fftStructure;
arm_cfft_radix4_instance_f32 cfftStructure;

//
// Hamming window, used to prepare samples for fft and correct for the fact
// we're using an algorithm meant for a continuous, infinite signal on a
// signal that is finite and not always continuous.
//
extern float ti_hamming_window_vector[NUM_SAMPLES];


//*****************************************************************************
//
// This function will dynamically determine, based on the minimum and maximum
// display frequencies currently used by the system and the current sampling
// frequency, what display elements need to correspond to which FFT result bins
// in order for the display to have a logarithmic scale for the frequency
//
//*****************************************************************************
void
setFreqBreakpoints(void)
{
    float minVal, maxVal, numLEDs;
    float minLog, maxLog, deltaLog, tempLog;
    int binMin, binMax, binCenter, binCenterNext;
    static int freqArray[MAX_NUMBARS+1];
    int i;
    int j;
    int targetFreq;
    int upperDelta, lowerDelta;

    minVal = g_uiMinDisplayFreq;
    maxVal = g_uiMaxDisplayFreq;
    numLEDs = g_uiNumDisplayBars;

    minLog = log10f(minVal);
    maxLog = log10f(maxVal);
    deltaLog = (maxLog - minLog)/numLEDs;

    //
    // Use the minimum and maximum display frequencies, along with the
    // bandwidth of each frequency bin, to determine what the ideal frequency
    // range is for each display element
    //
    tempLog = minLog;
    for(i=0;i<numLEDs+1;i++)
    {
            freqArray[i] = pow(10, tempLog);
            tempLog += deltaLog;
    }

    //
    // Figure out which frequency bin corresponds to the cutoff frequency for
    // each display element
    //
    for(j=0;j<numLEDs+1;j++)
    {
        targetFreq = freqArray[j];
		for(i=0; i<NUM_SAMPLES/2; i++)
		{
			binMin = g_HzPerBin * i;
			binMax = g_HzPerBin * (i+1);
			binCenter = (binMin + binMax)/2;
			binMin = g_HzPerBin * (i+1);
			binMax = g_HzPerBin * (i+2);
			binCenterNext = (binMin + binMax)/2;

			if(binCenter > targetFreq)
			{
				upperDelta = binCenter - targetFreq;
				lowerDelta = targetFreq - binCenterNext;
				if(upperDelta < lowerDelta)
				{
					LEDFreqBreakpoints[j] = i;
				}
				else
				{
					LEDFreqBreakpoints[j] = i-1;
				}
				break;
			}
		}
    }

    //
    // it is very possible for our bin size to be too large to match perfectly
    // the ideal logarithmic scale.  If that's the case, make sure each bar
    // contains at least one bin, and never the same bin as the bar below it.
    for(i=1;i<numLEDs+1;i++)
    {
		if(LEDFreqBreakpoints[i] <= LEDFreqBreakpoints[i-1])
		{
			LEDFreqBreakpoints[i] = LEDFreqBreakpoints[i-1] + 1;
		}

		//
		// If the frequency corresponding to this bar is greater than our
		// desired max display frequency, then we don't have enough granularity
		// to display this little frequency over this many bars, so override
		// the user set number of bars.
		//
		if((g_HzPerBin*LEDFreqBreakpoints[i]) > g_uiMaxDisplayFreq)
		{
			g_uiNumDisplayBars = i;
			GUIUpdateSlider(NUMBARS_SLIDER, i);
			break;
		}
    }

    if(g_ucPrintDbg | 2)
    {
		UARTprintf("// \n");
		for(i=0;i<numLEDs;i++)
		{
			UARTprintf("// LED %d: %05d:%05d Hz\t%d..%d\n", i, freqArray[i],
					freqArray[i+1], LEDFreqBreakpoints[i], LEDFreqBreakpoints[i+1]);
		}
		UARTprintf("// \n\n");
		for(i=0;i<numLEDs + 1;i++)
		{
			UARTprintf("LEDFreqBreakpoints[%d] = %d;\n", i, LEDFreqBreakpoints[i]);
		}
    }

}

//*****************************************************************************
//
// Initialize the digital signal processing engine.
//
//*****************************************************************************
void
InitDSP(void)
{
	int i;

	//
	// zero out our maximum power history
	//
	for(i=0;i<g_uiNumDisplayBars;i++)
	{
		maxLEDPowers[i] = 0;
		LEDFreqBreakpoints[i] = 0;
		LEDDisplayMaxes[i] = 0;
	}

	g_HzPerBin = (float)g_uiSamplingFreq / (float)NUM_SAMPLES;

	//
	// set our frequency range breakpoints
	//
	setFreqBreakpoints();

	//
	// Determine if our sampling frequency is fast enough to handle our refresh
	// rate
	//
	if((g_uiSamplingFreq/NUM_SAMPLES) > 16 )
	{
		g_ucDMAMethod = DMA_METHOD_FAST;
	}
	else
	{
		g_ucDMAMethod = DMA_METHOD_SLOW;
	}


	//
	// Call the CMSIS real fft init function
	//
	arm_rfft_init_f32(&fftStructure, &cfftStructure, NUM_SAMPLES, INVERT_FFT, BIT_ORDER_FFT);
}

//*****************************************************************************
//
// Run the DSP calculations on the input vector.
//
// Step 1: multiply vector by hamming window
// Step 2: get fast fourier transform of samples
// Step 3: get complex power of each element in fft output
// Step 4: figure out power in each LED range of bins, compare to previously
//		   observed maximum power, and set global LED array for that LED column
//		   accordingly
// Step 5: ???
// Step 6: Profit
//
//*****************************************************************************
void
ProcessData(void)
{
	uint32_t i;
	uint32_t j;
	float32_t power;
	float32_t maxValue;
	static float32_t historicMax = 0;
	static float32_t LEDPower[MAX_NUMBARS];
	//uint32_t dummy;

	if(g_ucDMAMethod == DMA_METHOD_SLOW)
	{
		//
		// Convert the samples to floats and center around 0
		//
		for(i=0; i < NUM_SAMPLES; i++)
		{
			g_fFFTResult[i] = ((float)g_ulADCValues[i] - (float)0x800);
		}
		g_ucDataReady = 0;
	}
	else
	{
		//
		// Ugly, ugly, ugly part where we have to move the ul samples into a float
		// array because the fixed point fft functions in CMSIS seem to be not
		// working.  While we're at it, we might as well center the samples around
		// 0, as the CMSIS algorithm seems to like that.
		//
		for(i=0;i<NUM_SAMPLES;i++)
		{
			g_fFFTResult[i] = ((float)g_ulADCValues[i] - (float)0x800);// / (float)640;
		}


	}

	//
	// Multiply samples by hamming window
	//
	arm_mult_f32(g_fFFTResult, ti_hamming_window_vector, g_fFFTResult, NUM_SAMPLES);

	//
	// Calculate FFT on samples
	//
	arm_rfft_f32(&fftStructure, g_fFFTResult, g_fFFTResult);

	//
	// Calculate complex power of FFT results
	//
	arm_cmplx_mag_f32(g_fFFTResult, g_fFFTResult, NUM_SAMPLES * 2);


	if(g_ucPrintDbg)
	{
		//
		// For debug purposes, find the maximum bin
		//
		arm_max_f32(g_fFFTResult, NUM_SAMPLES, &maxValue, &i);
		if(maxValue > historicMax)
		{
			historicMax = maxValue;
		}

		UARTprintf("FPS: %2d  ", g_ucLastFramesPerSec);
		UARTprintf("DPSPS: %2d  ", g_uiLastDSPPerSec);
		UARTprintf("Peak is b/w %06d and %06d\r",
				   (int)(g_HzPerBin*i), (int)(g_HzPerBin*(i+1)));
	}
	//
	// Calculate power stored in the frequency band each LED represents
	//
	j = LEDFreqBreakpoints[0];
	for(i=1;i<g_uiNumDisplayBars + 1;i++)
	{
		//
		// Find the average power of all the LED bins in the freq array. j is
		// the index of the lowest bin used for this LED.  To get the size of
		// range to take the average of, we do highest indexed bin,
		// LEDfreqBreakPoints[i], minus the lowest indexed bin, j, + 1.
		//
		arm_mean_f32(g_fFFTResult+j, LEDFreqBreakpoints[i]-j + 1, &power);
		//arm_max_f32(g_fFFTResult+j, LEDFreqBreakpoints[i]-j+1, &power, &dummy);
		if(maxLEDPowers[i -1] < power)
		{
			maxLEDPowers[i - 1] = power;
		}
		//else
		//{
		//	  maxLEDPowers[i-1] *= POWER_DECAY_FACTOR;
		//}
		LEDPower[i - 1] = power;
		j = LEDFreqBreakpoints[i] + 1;

		//
		// Normalize currently observed power by maximum observed power for
		// this frequency range
		//
		power = LEDPower[i-1] / maxLEDPowers[i-1];

		//
		// power is now between 0 and 1, so multiply by max display power to
		// figure out how many display elements to light up
		//
		LEDDisplay[i-1] = (int)(power * 185);
	}

	if(g_ucDMAMethod == DMA_METHOD_FAST)
	{
		//
		// Clear the data ready bit and set up the next DMA transfer
		//
		g_ucDataReady = 0;
		if(NUM_SAMPLES > UDMA_XFER_MAX)
		{
			uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
								   UDMA_MODE_BASIC,
								   (void *)(ADC0_BASE + ADC_O_SSFIFO3 + (0x20 * UDMA_ARB_1)),
								   g_ulADCValues, UDMA_XFER_MAX);
		}
		else
		{
			uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
								   UDMA_MODE_BASIC,
								   (void *)(ADC0_BASE + ADC_O_SSFIFO3 + (0x20 * UDMA_ARB_1)),
								   g_ulADCValues, NUM_SAMPLES);
		}

		//
		// Enable the timer and start the sampling timer
		//
		uDMAChannelEnable(UDMA_CHANNEL_ADC3);
		TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/(g_uiSamplingFreq - 1));
		TimerEnable(TIMER0_BASE, TIMER_A);
	}
}
