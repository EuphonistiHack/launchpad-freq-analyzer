/******************************************************************************
 *
 * led_equalizer_sourcerygxx.ld - Scatter file for Sourcery CodeBench
 *
 * Copyright (c) 2011 Texas Instruments Incorporated.  All rights reserved.
 * TI Information - Selective Disclosure
 *
 *****************************************************************************/

/******************************************************************************
 *
 * Define the end of the heap space, which determines the beginning of the
 * stack space.
 *
 *****************************************************************************/
__cs3_heap_end = __cs3_region_end_ram - 1024;

/******************************************************************************
 *
 * Define the interrupt handlers used by the application.
 *
 *****************************************************************************/
EXTERN(ADC3IntHandler)
__cs3_isr_adc3 = ADC3IntHandler;
EXTERN(Timer0AIntHandler)
__cs3_isr_timer0a = Timer0AIntHandler;
EXTERN(Timer1AIntHandler)
__cs3_isr_timer1a = Timer1AIntHandler;
EXTERN(uDMAErrorHandler)
__cs3_isr_udmaerr = uDMAErrorHandler;
