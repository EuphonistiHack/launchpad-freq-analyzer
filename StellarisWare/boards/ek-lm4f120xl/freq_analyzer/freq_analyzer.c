//*****************************************************************************
//
// led_equalizer.c - A project that uses the ADC to capture analog input, the
// CMSIS DSP library to analyze that input, and Kentec 320x240 touchscreen to
// display the power of logarithmicly spaced frequency ranges.
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

#include "inc/hw_adc.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/adc.h"
#include "driverlib/cpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"

#include "grlib/grlib.h"
#include "grlib/widget.h"
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
//!
//! This project uses the following peripherals and pins:
//! UART0 for display of debug statements
//! ADC input ch0 on ADC0 Sequencer 3 for audio capture
//! ADC input ch8 and 9 on ADC1 Sequencer 3 for touchscreen capture
//! Timer 0 for the audio sampling timer
//! Timer 1 for the touchscreen sampling timer
//! Timer 2 for 1 Hz debug functionality (FPS calculation, DSPPS calculation)
//! Timer 3 for screen refresh
//! Various GPIOs for touch screen communication
//!
//*****************************************************************************


//*****************************************************************************
//
// Preprocessor statements and global variables for audio capture and signal
// processing
//
//*****************************************************************************

//
// The ADC sequencer to be used to sample the audio signal
//
#define ADC_SEQUENCER			3

//
// The maximum number of transfers one UDMA transaction can complete
//
#define UDMA_XFER_MAX			1024

//
// The size of the ping pong buffer used for the slow uDMA method
//
#define DMA_SIZE 256

//
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
//
#if defined(ewarm)
#pragma data_alignment=1024
unsigned char ucControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(ucControlTable, 1024)
unsigned char ucControlTable[1024];
#else
unsigned char ucControlTable[1024] __attribute__ ((aligned(1024)));
#endif

//
// Variables used for storing ADC values and signalling which uDMA method to
// use
//
volatile unsigned char g_ucDMAMethod;
unsigned short g_ulADCValues[NUM_SAMPLES];
unsigned short g_usDMAping[DMA_SIZE];
unsigned short g_usDMApong[DMA_SIZE];
volatile unsigned char g_ucDMApingpong;

//
// Flag from the uDMA engine signaling that data is ready to be processed
//
volatile unsigned char g_ucDataReady;

//
// The count of uDMA errors.  This value is incremented by the uDMA error
// handler.  Hopefully this will always remain 0.
//
static volatile unsigned long g_uluDMAErrCount = 0;

//
// The count of times various uDMA error conditions detected.
//
static volatile unsigned long g_ulBadPeriphIsr1;
static volatile unsigned long g_ulBadPeriphIsr2;

//*****************************************************************************
//
// Debug and analytic variables
//
//*****************************************************************************
volatile unsigned char g_ucFramesPerSec;
volatile unsigned char g_ucLastFramesPerSec;
volatile unsigned int g_uiDSPPerSec;
volatile unsigned int g_uiLastDSPPerSec;


//*****************************************************************************
//
// Interrupt Handlers
//
//*****************************************************************************

//*****************************************************************************
//
// The interrupt handler for Timer0A.  Originally, this was used to display
// debug information once per second.  Keeping this around for debug, but in
// the final system, this isn't necessary, as ADC capture is triggered at the
// HW level, alleviating the need for a timer interrupt handler.
//
//*****************************************************************************
void
Timer0AIntHandler(void)
{

    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

//*****************************************************************************
//
// The interrupt handler for Timer2A.  This timer ticks by at once per second,
// and is used to keep track of analytics for frames per second and DSP loops
// per second.
//
//*****************************************************************************

void
Timer2AIntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    g_ucLastFramesPerSec = g_ucFramesPerSec;
    g_ucFramesPerSec = 0;

    g_uiLastDSPPerSec = g_uiDSPPerSec;
    g_uiDSPPerSec = 0;

    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet());
    TimerEnable(TIMER2_BASE, TIMER_A);
}

//*****************************************************************************
//
// The interrupt handler for the ADC sequencer used to capture all audio data.
// This handler is called each time the uDMA completes a full ADC->memory copy.
// Yes, that's a bit confusing... the uDMA interrupt is only triggered when the
// uDMA behaves abnormally.  When the uDMA engine completes its transfer, it
// calls the interrupt of whatever module fed it the data, not the uDMA
// interrupt.
//
//*****************************************************************************
void
ADC3IntHandler(void)
{
	unsigned long ulStatus;
	static unsigned long uluDMACount = 0;
	static unsigned long ulDataXferd = 0;
	unsigned long ulNextuDMAXferSize = 0;
	unsigned short *pusDMABuffer;
	unsigned short *pusCopyBuffer;
	int i;

	//
	// Clear the ADC interrupt
	//`
	ADCIntClear(ADC0_BASE, ADC_SEQUENCER);

	//
	// If the channel's not done capturing, we have an error
	//
	if(uDMAChannelIsEnabled(UDMA_CHANNEL_ADC3))
	{
		g_ulBadPeriphIsr2++;
		ADCIntDisable(ADC0_BASE, ADC_SEQUENCER);
		IntPendClear(INT_ADC0SS3);
		return;
	}

	//
	// Verify count remaining is 0.
	//
	ulStatus = uDMAChannelSizeGet(UDMA_CHANNEL_ADC3);
	if(ulStatus)
	{
		g_ulBadPeriphIsr1++;
		return;
	}

	if(g_ucDMAMethod == DMA_METHOD_SLOW)
	{
		//
		// We are using the slow DMA method, meaning there are not enough
		// samples in a second to generate a new set of FFT values and still
		// have data frequent enough to refresh at 15 frames per second
		//

		//
		// when pingpong is 0, uDMA just finished transferring into ping, so next
		// we transfer into pong.
		//
		if(g_ucDMApingpong == 0)
		{
			pusDMABuffer = g_usDMApong;
			pusCopyBuffer = g_usDMAping;
			g_ucDMApingpong = 1;
		}
		else
		{
			pusDMABuffer = g_usDMAping;
			pusCopyBuffer = g_usDMApong;
			g_ucDMApingpong = 0;
		}

		//
		// Set up the next uDMA transfer
		//
		uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
							   UDMA_MODE_BASIC,
							   (void *)(ADC0_BASE + ADC_O_SSFIFO3),// + (0x20 * UDMA_ARB_1)),
							   pusDMABuffer, DMA_SIZE);
		uDMAChannelEnable(UDMA_CHANNEL_ADC3);

		IntPendClear(INT_ADC0SS3);

		//
		// Shift everything back DMA_SIZE samples
		//
		for(i=0; i < (NUM_SAMPLES - DMA_SIZE); i++)
		{
			g_ulADCValues[i] = g_ulADCValues[i + DMA_SIZE];
		}

		//
		// Copy the new samples from the copy buffer into the sample array
		//
		for(i=0; i < DMA_SIZE; i++)
		{
			g_ulADCValues[i + NUM_SAMPLES - DMA_SIZE] = pusCopyBuffer[i];
		}

		//
		// Signal that we have new data to be processed
		//
		g_ucDataReady = 1;
	}
	else
	{
		//
		// We are using the fast DMA method, meaning that we are sampling audio
		// data fast enough to calculate the FFT values in a non-continuous
		// fashion but still have have new data frequently enough to ensure
		// there is a fresh set of values for each screan draw
		//

		//
		// Disable the sampling timer
		//
		TimerDisable(TIMER0_BASE, TIMER_A);

		//
		// If our sample size can be greater than our maximum uDMA transfer
		// size, we might need to set up another uDMA transfer before signaling
		// that we are ready to process the data.
		//
		uluDMACount++;
		ulDataXferd += UDMA_XFER_MAX;

		if(NUM_SAMPLES > ulDataXferd)
		{
			//
			// Figure out how many more uDMA transfers are required to completely
			// fill our sample array, which will tell us what size we need our next
			// uDMA transfer to be
			//
			if((NUM_SAMPLES - ulDataXferd) > UDMA_XFER_MAX)
			{
				ulNextuDMAXferSize = UDMA_XFER_MAX;
			}
			else
			{
				ulNextuDMAXferSize = NUM_SAMPLES - ulDataXferd;
			}
			uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
								   UDMA_MODE_BASIC,
								   (void *)(ADC0_BASE + ADC_O_SSFIFO3 + (0x20 * UDMA_ARB_1)),
								   g_ulADCValues + (UDMA_XFER_MAX * uluDMACount),
								   ulNextuDMAXferSize);
			uDMAChannelEnable(UDMA_CHANNEL_ADC3);
			TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/(g_uiSamplingFreq - 1));
			TimerEnable(TIMER0_BASE, TIMER_A);
		}
		else
		{
			//
			// Since this transfer is now done, disable the interrupt so the
			// handler is not called any more until set up again.  Also need
			// to unpend in case it happened again since we entered this
			// handler
			//
			uluDMACount = 0;
			ulDataXferd = 0;
			ADCIntDisable(ADC0_BASE, ADC_SEQUENCER);
			IntPendClear(INT_ADC0SS3);

			//
			// Signal that we have new data to be processed
			//
			g_ucDataReady = 1;
		}
	}
}

//*****************************************************************************
//
// The interrupt handler for uDMA errors.  This interrupt will occur if the
// uDMA encounters a bus error while trying to perform a transfer.  Hopefully,
// this code will never be executed.
//
//*****************************************************************************
void
uDMAErrorHandler(void)
{
    unsigned long ulStatus;

    //
    // Check for uDMA error bit
    //
    ulStatus = uDMAErrorStatusGet();

    //
    // If there is a uDMA error, then clear the error and increment
    // the error counter.
    //
    if(ulStatus)
    {
        uDMAErrorStatusClear();
        g_uluDMAErrCount++;
    }
}


//*****************************************************************************
//
// Public Functions
//
//*****************************************************************************

//*****************************************************************************
//
// Initialize the timer that will trigger the ADC captures
//	Timer0 @ 44.6k kHz (macro'd) sampling frequency
//
//*****************************************************************************
void
InitSamplingTimer()
{
    //
    // Set up timer0A to be a one shot that interrupts at 44.6kHz
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/(g_uiSamplingFreq - 1));
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER0_BASE, TIMER_A);
}

//*****************************************************************************
//
// Private Functions
//
//*****************************************************************************

//*****************************************************************************
//
// Initialize the signal capture chain.
// ADC is initialized to capture based on the sampling timer
// uDMA is set up to transfer the samples from the ADC to the global sample
//   array each time a sample is captured by the ADC.
//
// The end result is a very efficient, 90% hardware handled process in which
// the software says "give me a sample" and the hardware interrupts a short
// time later with the data nicely arranged in a convenient location.  I'm not
// gonna lie... this is pretty awesome.
//
//*****************************************************************************
static void
InitADC3Transfer(void)
{
    unsigned int uIdx;

    g_ucDataReady = 0;

    //
    // Init buffers
    //
    for(uIdx = 0; uIdx < NUM_SAMPLES; uIdx++)
    {
    	g_ulADCValues[uIdx] = 0;
    }

    //
	// Configure and enable the uDMA controller
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	IntEnable(INT_UDMAERR);
	uDMAEnable();
	uDMAControlBaseSet(ucControlTable);
	UARTprintf("Capturing audio on ADC0 seq 3 using DMA channel %d\n",
			   UDMA_CHANNEL_ADC3 & 0xff);

    //
    // Configure the ADC to capture one sample per sampling timer tick
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);
    ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCER, ADC_TRIGGER_TIMER, 0);
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCER, 0, ADC_CTL_CH0 |
    							 ADC_CTL_IE | ADC_CTL_END);

    //
    // Enable the sequencer
    //
    ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCER);
    ADCIntEnable(ADC0_BASE, ADC_SEQUENCER);

    //
    // Configure the DMA channel
    //
    uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC3,
								UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
								UDMA_ATTR_HIGH_PRIORITY |
								UDMA_ATTR_REQMASK);
    uDMAChannelControlSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
						  UDMA_SIZE_16 | UDMA_SRC_INC_NONE |
						  UDMA_DST_INC_16 | UDMA_ARB_1);

    if(g_ucDMAMethod == DMA_METHOD_SLOW)
    {
		g_ucDMApingpong = 0;
		uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
							   UDMA_MODE_BASIC,
							   (void *)(ADC0_BASE + ADC_O_SSFIFO3 + (0x20 * UDMA_ARB_1)),
							   g_usDMAping, DMA_SIZE);
    }
    else
    {
		//
		// The uDMA engine has an upper limit of the number of transfers it can
		// complete before it must be configured for a new transfer.  As a result,
		// we need to configure the uDMA engine to transfer as many samples as
		// possible, up to its maximum amount
		//
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
    }
    //
    // Enable the DMA channel
    //
    uDMAChannelEnable(UDMA_CHANNEL_ADC3);
}

//*****************************************************************************
//
// Initialize basic stuff:
//	processor
//  UART0(debug)
//
//*****************************************************************************
void
InitBasics(void)
{
	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	//
	ROM_FPUEnable();
	ROM_FPULazyStackingEnable();

	//
	// Set the clocking to run directly from the crystal.
	//
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
					   SYSCTL_OSC_MAIN);

	//
	// Initialize the UART.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioInit(0);

	//
	// Hello!
	//
	UARTprintf("Hello, world!\n");

}

//*****************************************************************************
//
// Initialize the 1/second debug timer.
//
//*****************************************************************************
void
InitDebugTimer()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet());
    IntEnable(INT_TIMER2A);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER2_BASE, TIMER_A);
}


//*****************************************************************************
//
// The main function.  DO EVERYTHING.
//
//*****************************************************************************
int
main(void)
{

	//
	// Initialize global variables
	//
	g_ucDMApingpong = 0;

    g_ucLastFramesPerSec = 0;
    g_ucFramesPerSec = 0;
    g_uiDSPPerSec = 0;
    g_uiLastDSPPerSec = 0;

	//
	// Initialize all peripherals
	//
    InitBasics();
	GUIinit();
    InitSamplingTimer();
    InitDebugTimer();
	InitADC3Transfer();
	InitDSP();

	//
	// Once ADC3 interrupts are enabled, our capture engine will start churning
	//
	IntEnable(INT_ADC3);

    while(1)
    {
    	//
    	// TODO: Figure out how processor sleeping works and add a sleep here.
    	// We only take action after an interrupt occurs anyway, so there's no
    	// sense in sitting here polling a flag that gets set in an interrupt
    	// if we can instead just sleep until an interrupt occurs.
    	//
    	//CPUwfi();

    	//
    	// Make sure uDMA hasn't exploded
    	//
    	if(g_ulBadPeriphIsr1 || g_ulBadPeriphIsr2)
    	{
    		UARTprintf("uDMA got out of synch! ISR1: %d, ISR2: %d\n",
    				   g_ulBadPeriphIsr1, g_ulBadPeriphIsr2);
    		g_ulBadPeriphIsr1 = 0;
    		g_ulBadPeriphIsr2 = 0;
    	}

    	GUIUpdateDisplay();

    	//
    	// If we have new audio data, handle it
    	//
    	if(g_ucDataReady)
    	{
    		ProcessData();
    		g_uiDSPPerSec++;
    	}

    	//
    	// Look for button presses
    	//
		WidgetMessageQueueProcess();
    }
}
