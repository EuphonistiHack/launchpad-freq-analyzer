//*****************************************************************************
//
// led_equalizer.c - A project that uses the ADC to capture analog input, the
// CMSIS DSP library to analyze that input, and an Olimex 8x8 LED tile to
// display the power of 8 logarithmicly spaced frequency ranges.
//
// Copyright (c) Jordan Wills, aka EuphonistiHack, wills.jordan@gmail.com
// For more information, see writeup at
// http://www.euphonistihack.blogspot.com/2012/08/the-writeup.html
//
// This project was intended to run on a TI LM4F120XL on a Stellaris Launchpad
// evaluation kit.  It was developed in my free time without using company
// resources.  As such, feel free to copy and redistribute this code as you see
// fit.  I politely ask that you only use this code on Texas Instruments
// hardware, as it would be frustrating to see my hard work running on a board
// manufactured by one of my employer's competitors (I'm lookikng at you, ST
// and Freescale).  However, my polite request in no way constitutes a legally
// enforcable demand, so if you really want to use it on an ST board, I won't
// stop you.  At least give me some credit in your comments though :)
//
// This project contains ABSOLUTELY NO WARRANTY, meaning it does not even have
// the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
// PURPOSE.  If there's a bug in this code that smokes your board, sorry, but
// you're on your own.
//
//*****************************************************************************

#include <string.h>
#include "inc/hw_adc.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "utils/uartstdio.h"

#include "arm_math.h"

//*****************************************************************************
//
//!
//! This project uses the following peripherals and pins:
//! UART0 for display of debug statements
//! ADC input 0 for audio capture
//! SSI2 for communication with the Olimex booster pack
//!
//
//  Hardware conditioning circuit
//  Aout		        = PE3 - AIN0
//
//  Olimex J1
//  Pin 1 - Vcc			= J1[1] - VBUS
//  Pin 6 - SR_LATCH	= PA4 - GPIO
//  Pin 7 - SR_SCK      = PB4 - SSI2Clk
//
//  Olimex J2
//  Pin 1 - GND			= J1[2] - GND
//  Pin 7 - SR_DATA_IN  = PB7 - SSI2Tx
//
//
//
//*****************************************************************************

//*****************************************************************************
//
// Preprocessor directives
//
//*****************************************************************************

//
// The error routine that is called if the driver library encounters an error.
//
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//*****************************************************************************
//
// SSI and display macros
//
//*****************************************************************************
//
// The refresh rate of the Olimex LED display
//
#define REFRESH_RATE			1000
#define NUM_F_LEDS				8
#define NUM_P_LEDS				8
//
// Pins used for SSI interface
//
#define SSI_CLK					GPIO_PIN_4
#define SSI_TX					GPIO_PIN_7

//*****************************************************************************
//
// Audio capture and signal processing macros
//
//*****************************************************************************
//
// The ADC sequencer to be used to sample the audio signal
//
#define ADC_SEQUENCER			3
//
// audio sampling frequency = 2 * Nyquist = 2 * 22.3 kHz = 44600
//
#define SAMPLING_FREQ			44600
//
// Number of samples to capture for each FFT process.  Set to CMSIS max for
// best resolution
//
#define NUM_SAMPLES				2048
//
// Amount by which the recorded maximum power decays each time through the DSP
// engine
//
#define	POWER_DECAY_FACTOR		0.999
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
// The maximum number of transfers one UDMA transaction can complete
//
#define UDMA_XFER_MAX			1024

//*****************************************************************************
//
// Global variables
//
//*****************************************************************************
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
// The ADC storage array
//
unsigned long g_ulADCValues[NUM_SAMPLES];
float32_t g_fFFTResult[NUM_SAMPLES * 2];
float32_t maxLEDPowers[NUM_F_LEDS];

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

//
// The range in Hertz of each frequency bin
//
float g_HzPerBin;

//
// Flag from the uDMA engine signaling that data is ready to be processed
//
volatile unsigned char g_ucDataReady;

//
// The count of uDMA errors.  This value is incremented by the uDMA error
// handler.
//
static volatile unsigned long g_uluDMAErrCount = 0;

//
// The count of times various uDMA error conditions detected.
//
static volatile unsigned long g_ulBadPeriphIsr1;
static volatile unsigned long g_ulBadPeriphIsr2;

//
// The character array used to update the LED display
//
unsigned char LEDDisplay[NUM_F_LEDS];
unsigned int  LEDFreqBreakpoints[NUM_F_LEDS + 1];

//*****************************************************************************
//
// Private function declarations
//
//*****************************************************************************

//*****************************************************************************
//
// Bit-wise reverses a number, useful for sending data to LED display
//
//*****************************************************************************
unsigned char
Reverse(unsigned char ucNumber)
{
    unsigned short ucIndex;
    unsigned short ucReversedNumber = 0;
    for(ucIndex=0; ucIndex<8; ucIndex++)
    {
        ucReversedNumber = ucReversedNumber << 1;
        ucReversedNumber |= ((1 << ucIndex) & ucNumber) >> ucIndex;
    }
    return ucReversedNumber;
}

//*****************************************************************************
//
// The interrupt handler for Timer0A.  Originally, this was used to display
// debug information once per second.  Keeping this around for debug, but in
// the final system, this isn't necessary, as ADC capture is triggered at the
// HW level, alleviating the need for a software interrupt handler to trigger.
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
// The interrupt handler for Timer1A.  Used for refreshing the LED display.
//
//*****************************************************************************
void
Timer1AIntHandler(void)
{
    unsigned long ulData;
    static unsigned char ucLED = 0;

    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    ulData = (Reverse(LEDDisplay[ucLED]) << 8) + (1 << ucLED);

	//
	// Send the data using the "blocking" put function.  This function
	// will wait until there is room in the send FIFO before returning.
	// This allows you to assure that all the data you send makes it into
	// the send FIFO.
	//
	SSIDataPut(SSI2_BASE, ulData);

	//
	// Wait until SSI is done transferring all the data in the transmit FIFO
	//
	while(SSIBusy(SSI2_BASE))
	{
	}

	//
	// Hit the SSI latch, locking in the data
	//
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);

	//
	// Move on to the next display element
	//
	ucLED++;
	if(ucLED >= NUM_F_LEDS)
	{
		ucLED = 0;
	}

	//
	// Reset the timer to trigger again on the next refresh cycle
	//
	TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet()/REFRESH_RATE);
	TimerEnable(TIMER1_BASE, TIMER_A);
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

	//
	// Clear the ADC interrupt
	//
	ADCIntClear(ADC0_BASE, 3);

	//
	// Disable the sampling timer
	//
	TimerDisable(TIMER0_BASE, TIMER_A);

	//
	// If the channel's not done capturing, we have an error
	//
	if(uDMAChannelIsEnabled(UDMA_CHANNEL_ADC3))
	{
		g_ulBadPeriphIsr2++;
		ADCIntDisable(ADC0_BASE, 3);
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
							   (void *)(ADC0_BASE + ADC_O_SSFIFO3 +
									    (0x20 * UDMA_ARB_1)),
							   g_ulADCValues + (UDMA_XFER_MAX * uluDMACount),
							   ulNextuDMAXferSize);
		uDMAChannelEnable(UDMA_CHANNEL_ADC3);
		TimerLoadSet(TIMER0_BASE, TIMER_A,
					 SysCtlClockGet()/(SAMPLING_FREQ - 1));
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
		ADCIntDisable(ADC0_BASE, 3);
		IntPendClear(INT_ADC0SS3);
		g_ucDataReady = 1;
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
void
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
						  UDMA_SIZE_32 | UDMA_SRC_INC_NONE |
						  UDMA_DST_INC_32 | UDMA_ARB_1);

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
							   (void *)(ADC0_BASE + ADC_O_SSFIFO3 +
									   (0x20 * UDMA_ARB_1)),
							   g_ulADCValues, UDMA_XFER_MAX);
    }
    else
    {
    	uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
							   UDMA_MODE_BASIC,
							   (void *)(ADC0_BASE + ADC_O_SSFIFO3 +
									   (0x20 * UDMA_ARB_1)),
							   g_ulADCValues, NUM_SAMPLES);
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
	ROM_FPULazyStackingEnable();

	//
	// Set the clocking to run directly from the crystal.
	//
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
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
// Initialize the SSI interface used to communicate with the Olimex LED panel
// Originally, I was using the software SSI lib to do this, hence the HW in the
// name.
//
//*****************************************************************************
void
InitHWSSI()
{
	//
	// Initialize the pins we will need for communication with the LED Matrix
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB4_SSI2CLK);
	GPIOPinConfigure(GPIO_PB7_SSI2TX);
	GPIOPinTypeSSI(GPIO_PORTB_BASE, SSI_CLK | SSI_TX);

	//
	// Configure SSI2
	//
	SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
					   SSI_MODE_MASTER, SysCtlClockGet()/2, 16);

	//
	// Enable the SSI module.
	//
	SSIEnable(SSI2_BASE);

}

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
    // Set up timer1A to be a 32 bit timer that interrupts at 1/sec
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/(SAMPLING_FREQ - 1));
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER0_BASE, TIMER_A);
}

//*****************************************************************************
//
// Initialize the timer that will cause the LED display to refresh
//	Timer1 @ 1 kHz (macro'd) refresh rate (1 column per refresh)
//
//*****************************************************************************
void
InitDisplayTimer()
{
	//
	// Set up timer1A to be the display timer, interrupting at 15 Hz
	//
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet()/REFRESH_RATE);
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER1_BASE, TIMER_A);
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
	for(i=0;i<NUM_F_LEDS;i++)
	{
		maxLEDPowers[i] = 0;
	}

	//
	// set our frequency range breakpoints
	// TODO: we generate all of these with a separate generic algorithm; why
	// don't we just put that algorithm in here and calculate these values on
	// init?
	//

	// LED 0: 00039:00082 Hz        1..3
	// LED 1: 00082:00169 Hz        3..7
	// LED 2: 00169:00349 Hz        7..16
	// LED 3: 00349:00721 Hz        16..33
	// LED 4: 00721:01485 Hz        33..68
	// LED 5: 01485:03061 Hz        68..140
	// LED 6: 03061:06308 Hz        140..289
	// LED 7: 06308:12999 Hz        289..596
	//

	LEDFreqBreakpoints[0] = 1;
	LEDFreqBreakpoints[1] = 3;
	LEDFreqBreakpoints[2] = 7;
	LEDFreqBreakpoints[3] = 16;
	LEDFreqBreakpoints[4] = 33;
	LEDFreqBreakpoints[5] = 68;
	LEDFreqBreakpoints[6] = 140;
	LEDFreqBreakpoints[7] = 289;
	LEDFreqBreakpoints[8] = 596;

	//
	// Call the CMSIS real fft init function
	//
	arm_rfft_init_f32(&fftStructure, &cfftStructure, NUM_SAMPLES, INVERT_FFT,
					  BIT_ORDER_FFT);
	g_HzPerBin = (float)SAMPLING_FREQ / (float)NUM_SAMPLES;
}

#if 0
extern q15_t ti_sample_sine_vector[NUM_SAMPLES];
//*****************************************************************************
//
// Run the DSP calculations on the sample sine vector.  If everything is
// running properly, the debug console will tell us that we have a power spike
// at 440 Hz.  This is not compiled in the main code because the sample sine
// vector is relatively large, and I don't want to waste all that space.
//
//*****************************************************************************
void
SampleSineTest(void)
{
	unsigned long i;
	for(i=0;i<NUM_SAMPLES;i++)
	{
		//g_ulADCValues[i] = ti_sample_sine_vector[i];
		g_fFFTResult[i] = ((float)ti_sample_sine_vector[i] - (float)0x800) / (float)640;
	}

	UARTprintf("test processing using sample 440 Hz sine wave\n");

	//arm_mult_q15 (ti_sample_sine_vector, ti_hamming_window_vector, ti_sample_sine_vector, NUM_SAMPLES);
	//arm_mult_f32((float32_t *)g_ulADCValues, ti_hamming_window_vector, (float32_t *)g_ulADCValues, NUM_SAMPLES);
	arm_mult_f32(g_fFFTResult, ti_hamming_window_vector, g_fFFTResult, NUM_SAMPLES);
	UARTprintf("Done with Hamming window multiplication\n");

	UARTprintf("Calculating FFT\n");
	//arm_rfft_q15 (&fft15Structure,ti_sample_sine_vector, g_fFFT15Result);
	//arm_rfft_f32(&fftStructure, (float32_t *)g_ulADCValues, g_fFFTResult);
	arm_rfft_f32(&fftStructure, g_fFFTResult, g_fFFTResult);

	UARTprintf("FFT generated\n");

	/* Process the data through the Complex Magnitude Module for
		calculating the magnitude at each bin */
	//for(i=0;i<NUM_SAMPLES*2;i++)
	//{
	//	g_result[i] = (g_fFFT15Result[(2*i)] * g_fFFT15Result[(2*i)]) +
	//				  (g_fFFT15Result[(2*i)+1] * g_fFFT15Result[(2*i)+1]);
	//}
	//arm_cmplx_mag_squared_q15(g_fFFT15Result, g_result, NUM_SAMPLES);
	arm_cmplx_mag_f32(g_fFFTResult, g_fFFTResult, NUM_SAMPLES * 2);
	while(1);
}
#endif

//*****************************************************************************
//
// Run the DSP calculations on the input vector.
//
// Step 1: multiply vector by hamming window
// Step 2: get fast fourier transform of samples
// Step 3: get complex power of each element in fft output
// Step 4: calculate average power in each LED range of bins
// Step 5: compare to previously observed maximum power, and set global LED
//		   array for that LED column accordingly
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
	unsigned int LEDPower[NUM_P_LEDS];
	unsigned char ucDisplay;
	//uint32_t dummy;

	//
	// Ugly, ugly, ugly part where we have to move the ul samples into a float
	// array because the fixed point fft functions in CMSIS seem to be not
	// working.  While we're at it, we might as well center the samples around
	// 0, as the CMSIS algorithm seems to like that.
	//
	for(i=0;i<NUM_SAMPLES;i++)
	{
		g_fFFTResult[i] = ((float)g_ulADCValues[i] - (float)0x800);
	}

	//
	// Multiply samples by hamming window
	//
	arm_mult_f32(g_fFFTResult, ti_hamming_window_vector, g_fFFTResult,
				 NUM_SAMPLES);

	//
	// Calculate FFT on samples
	//
	arm_rfft_f32(&fftStructure, g_fFFTResult, g_fFFTResult);

	//
	// Calculate complex power of FFT results
	//
	arm_cmplx_mag_f32(g_fFFTResult, g_fFFTResult, NUM_SAMPLES * 2);

	//
	// For debug purposes, find the maximum bin
	//
	arm_max_f32(g_fFFTResult, NUM_SAMPLES, &maxValue, &i);
	if(maxValue > historicMax)
	{
		historicMax = maxValue;
	}
	UARTprintf("Peak is between %06d and %06d Hz: %06d Max: %06d\r",
			   (int)(g_HzPerBin*i), (int)(g_HzPerBin*(i+1)), (int)maxValue,
			   (int)historicMax);

	//
	// Calculate power stored in the frequency band each LED represents
	//
	j = LEDFreqBreakpoints[0];
	for(i=1;i<NUM_F_LEDS + 1;i++)
	{
		//
		// Find the average power of all the LED bins in the freq array. j is
		// the index of the lowest bin used for this LED.  To get the size of
		// range to take the average of, we do highest indexed bin,
		// LEDfreqBreakPoints[i], minus the lowest indexed bin, j, + 1.
		//
		arm_mean_f32(g_fFFTResult+j, LEDFreqBreakpoints[i]-j + 1, &power);

		//
		// It might make more sense to take the max instead of the average...
		// use whichever looks best.
		//
		//arm_max_f32(g_fFFTResult+j, LEDFreqBreakpoints[i]-j+1, &power,
		//			  &dummy);
		if(maxLEDPowers[i -1] < power)
		{
			maxLEDPowers[i - 1] = power;
		}
		else
		{
			maxLEDPowers[i-1] *= POWER_DECAY_FACTOR;
		}
		LEDPower[i - 1] = power;
		j = LEDFreqBreakpoints[i] + 1;
	}

	//
	// Update the LED Display array based on how much power is in the band for
	// each LED
	//
	for(i=0;i<NUM_F_LEDS;i++)
	{
		ucDisplay = 0;
		for(j=0;j<NUM_P_LEDS;j++)
		{
			//
			// Normalize currently observed power by maximum observed power for
			// this frequency range
			//
			power = (float)LEDPower[i] / maxLEDPowers[i];

			//
			// power is now between 0 and 1, so figure out if it's greater than
			// 1/8, 2/8, 3/8, etc
			//
			if(power >= ((float)j * 0.125))
			{
				ucDisplay |= (1 << j);
			}
			else
			{
				break;
			}
		}
		LEDDisplay[i] = ucDisplay;
	}

	//
	// Clear the data ready bit and set up the next DMA transfer
	//
	g_ucDataReady = 0;
    if(NUM_SAMPLES > UDMA_XFER_MAX)
    {
    	uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
							   UDMA_MODE_BASIC,
							   (void *)(ADC0_BASE + ADC_O_SSFIFO3 +
									   (0x20 * UDMA_ARB_1)),
							   g_ulADCValues, UDMA_XFER_MAX);
    }
    else
    {
    	uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT,
							   UDMA_MODE_BASIC,
							   (void *)(ADC0_BASE + ADC_O_SSFIFO3 +
									   (0x20 * UDMA_ARB_1)),
							   g_ulADCValues, NUM_SAMPLES);
    }

    //
    // Enable the timer and start the sampling timer
    //
	uDMAChannelEnable(UDMA_CHANNEL_ADC3);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/(SAMPLING_FREQ - 1));
	TimerEnable(TIMER0_BASE, TIMER_A);
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
	// Initialize all peripherals
	//
    InitBasics();
    InitDSP();
    InitHWSSI();
    InitSamplingTimer();
    InitDisplayTimer();
	InitADC3Transfer();

	//
	// If we just want to test the DSP stuff, uncomment this line
	//
	//SampleSineTest();

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
    		while(1);
    	}

    	//
    	// If we have new audio data, handle it
    	//
    	if(g_ucDataReady)
    	{
    		ProcessData();
    	}
    }
}
