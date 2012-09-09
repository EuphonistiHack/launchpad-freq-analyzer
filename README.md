launchpad-freq-analyzer
===============================================================================

Project code for an fft based graphic equalizer to be run on a Stellaris
Launchpad.

I'm assuming that TI will follow the standard procedure for Stellaris installs
for Launchpad, which is to say that when you install StellarisWare, the file
structure will be such that you have a root (defaulted to C:/StellarisWare) in
which driverlib will be copied.  That root will also contain a  boards
directory, which in turn should contain a directory for the lm4f120.  If you do
a git clone off of the above address, you'll find a similar file structure.
You should just be able to copy the led_equalizer project into the lm4f120
board directory, and copy dsplib into the stellarisWare root.

In it's current state, the only thing I've verified working is the Code
Composer Studio build.  I have included project files that should at least be
95% good for all the other Stellaris supported environments: a makefile for you
command line GCC enthusiasts and project files for Keil uVision, IAR Embedded
Workbench, and CodeSourcery's IDE.  I'm guessing that if you tried to import
and build into any of those as is, you will hit two problems. 

For one, there's a config array that the uDMA engine uses that needs to be
stored on a 1024 byte boundary.  I have some pragmas in place to handle that
for CCS and IAR, but I haven't yet figured out the syntax for handling this in
other compilers.  Shouldn't be too hard to figure out, and hopefully I'll have
that done soon.

The second issue will be with the CMSIS library.  The current release contains
a precompiled .lib and the headers necessary to interface with it.  This was
compiled with the tms470 compiler, though, so it won't work with anything other
than CCS.  If you download CMSIS for yourself, it should come with precompiled
binaries for use with uVision, GCC, and CodeSourcery.  You'll want to add those
to dsplib, then edit your project's library include path/list to point to the
proper binary.  I am considering including these in dsplib, but I'm not really
sure what ARM's policy is on redistributing their binaries, and I'd prefer not
get sued out of existence :)
