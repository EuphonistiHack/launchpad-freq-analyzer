#******************************************************************************
#
# Makefile - Rules for building the Audio in used to display graphic eq on an olimex LED tile booster pack.
#
# Copyright (c) 2011 Texas Instruments Incorporated.  All rights reserved.
# TI Information - Selective Disclosure
#
#******************************************************************************

#
# Defines the part type that this project uses.
#
PART=LM4F120H5QR

#
# Set the processor variant.
#
VARIANT=cm4f

#
# The base directory for StellarisWare.
#
ROOT=../../..

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find source files that do not live in this directory.
#
VPATH=../../../utils

#
# Where to find header files that do not live in the source directory.
#
IPATH=..
IPATH+=../../..
IPATH+=../../../dsplib

#
# The default rule, which causes the Audio in used to display graphic eq on an olimex LED tile booster pack to be built.
#
all: ${COMPILER}
all: ${COMPILER}/led_equalizer.axf
##### INTERNAL BEGIN #####
all_comp:
	@for i in `grep COMPILER= ${ROOT}/makedefs | awk -F= '{print $$2}'`; \
	 do                                                                  \
	     make COMPILER=$${i};                                            \
	 done

#
# A rule to regenerate the Makefile if config.in has changed.
#
Makefile: config.in
	@perl ${ROOT}/scripts/buildproj.pl -s -f config.in
##### INTERNAL END #####

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}
##### INTERNAL BEGIN #####
	@rm -rf settings ${wildcard *.Bak} ${wildcard *.dep}
clean_comp:
	@for i in `grep COMPILER= ${ROOT}/makedefs | awk -F= '{print $$2}'`; \
	 do                                                                  \
	     make COMPILER=$${i} clean;                                      \
	 done

#
# Cleans out everything, including the Makefile.  Should return this directory
# to fresh from subversion status.
#
realclean:: clean
	@rm -rf ccs codered ewarm gcc rvmdk sourcerygxx ccstempws
	@rm -f ${wildcard *.Uv2} ${wildcard *.Opt}
	@rm -f ${wildcard *.uvopt} ${wildcard *.uvproj}
	@rm -f ${wildcard *.ew?} ${wildcard *.xcl}
	@rm -f ${wildcard *.sgxx}
	@rm -f ccs_proj_args.txt
	@rm -f led_equalizer_ccs.cmd
	@rm -f led_equalizer_codered.ld
	@rm -f led_equalizer.ld
	@rm -f led_equalizer_sourcerygxx.ld
	@rm -f ${wildcard *-rom.ld}
	@rm -f led_equalizer.icf
	@rm -f led_equalizer.sct
	@rm -f ${wildcard *.xml}
	@rm -f ${wildcard startup_*}
	@rm -f readme.txt
	@rm -f Makefile cr_project.xml
##### INTERNAL END #####

#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

#
# Rules for building the Audio in used to display graphic eq on an olimex LED tile booster pack.
#
${COMPILER}/led_equalizer.axf: ${COMPILER}/led_equalizer.o
${COMPILER}/led_equalizer.axf: ${COMPILER}/startup_${COMPILER}.o
${COMPILER}/led_equalizer.axf: ${COMPILER}/ti_hamming_window_vector.o
${COMPILER}/led_equalizer.axf: ${COMPILER}/ti_sample_sine_vector2.o
${COMPILER}/led_equalizer.axf: ${COMPILER}/uartstdio.o
${COMPILER}/led_equalizer.axf: ${ROOT}/driverlib/${COMPILER}-cm4f/libdriver-cm4f.a
##### INTERNAL BEGIN #####
ifeq (${COMPILER}, ccs)
${COMPILER}/led_equalizer.axf: led_equalizer_ccs.cmd
endif
SCATTERccs_led_equalizer=led_equalizer_ccs.cmd
ifeq (${COMPILER}, codered)
${COMPILER}/led_equalizer.axf: led_equalizer_codered.ld
endif
SCATTERcodered_led_equalizer=led_equalizer_codered.ld
ifeq (${COMPILER}, ewarm)
${COMPILER}/led_equalizer.axf: led_equalizer.icf
endif
SCATTERewarm_led_equalizer=led_equalizer.icf
ifeq (${COMPILER}, gcc)
##### INTERNAL END #####
${COMPILER}/led_equalizer.axf: led_equalizer.ld
##### INTERNAL BEGIN #####
endif
##### INTERNAL END #####
SCATTERgcc_led_equalizer=led_equalizer.ld
##### INTERNAL BEGIN #####
ifeq (${COMPILER}, rvmdk)
${COMPILER}/led_equalizer.axf: led_equalizer.sct
endif
SCATTERrvmdk_led_equalizer=led_equalizer.sct
ifeq (${COMPILER}, sourcerygxx)
${COMPILER}/led_equalizer.axf: led_equalizer_sourcerygxx.ld
endif
SCATTERsourcerygxx_led_equalizer=lm4f120h5qr-rom.ld -T led_equalizer_sourcerygxx.ld
##### INTERNAL END #####
ENTRY_led_equalizer=ResetISR
##### INTERNAL BEGIN #####
CFLAGSccs=-DTARGET_IS_BLIZZARD_RA1
##### INTERNAL END #####
##### INTERNAL BEGIN #####
CFLAGScodered=-DTARGET_IS_BLIZZARD_RA1
##### INTERNAL END #####
##### INTERNAL BEGIN #####
CFLAGSewarm=-DTARGET_IS_BLIZZARD_RA1
##### INTERNAL END #####
CFLAGSgcc=-DTARGET_IS_BLIZZARD_RA1
##### INTERNAL BEGIN #####
CFLAGSrvmdk=-DTARGET_IS_BLIZZARD_RA1
##### INTERNAL END #####
##### INTERNAL BEGIN #####
CFLAGSsourcerygxx=-DTARGET_IS_BLIZZARD_RA1
##### INTERNAL END #####

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
##### INTERNAL BEGIN #####
ifneq (${MAKECMDGOALS},realclean)
##### INTERNAL END #####
-include ${wildcard ${COMPILER}/*.d} __dummy__
##### INTERNAL BEGIN #####
endif
##### INTERNAL END #####
endif
