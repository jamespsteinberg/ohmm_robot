# Hey Emacs, this is a -*- makefile -*-
#
# Open Hardware Mobile Manipulator (OHMM) AVR Monitor
#
# See README.txt for build documentation.
#
# Specialization of a default makefile giving project-specific options for a
# one-target avr-gcc project.  Ends with an include of makefile_AVR.mk, which
# is generic across projects.
#
# Derived from the WinAVR template, which is public domain.
#
################################################################################
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation; either version 2 of the License, or (at your option)
# any later version.
# 
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc., 59
# Temple Place - Suite 330, Boston, MA 02111-1307, USA.

# Subversion ID for this file.
#
# $Id: makefile 4 2013-01-17 21:53:38Z vona $

# the (relative) location of the OHMM monitor open source library
#
OHMM_SW ?= ../../ohmm-sw
OHMM_DIR ?= $(OHMM_SW)/llp/monitor

# Mandatory AVR part info, used like this:
#
# avr-gcc -mmcu=$(MCU)
# avrdude -p $(AVRDUDE_MCU)
#
# MCU = atmega128
MCU = atmega1284p
AVRDUDE_MCU = $(MCU)

# Defines a C macro giving the CPU frequency in HZ, needed by some of the
# avr-libc timing procedures.
#
# F_CPU = 14745600
F_CPU = 20000000

# Primary target basename.  Default uses parent directory name.
#
#TARGET = $(notdir $(lastword $(shell pwd)))
TARGET = ohmm

# C sources, if any (makefile_AVR.mk assumes they must end in .c).
#
SRC = $(wildcard *.c)

# Assembly sources, if any (makefile_AVR.mk assumes they must end in .S).
#
# The .S must be capital, even if the underlying filesystem is not case
# sensitive.  avr-gcc would accept .s, but the generic makefile assumes all
# assembly sources end in .S.  The difference between .s and .S is that avr-gcc
# passes only the latter through the c preprocessor.
#
ASRC = $(wildcard *.S)

# Optimization level for avr-gcc -O flag.
#
OPT = s

# Uncomment this to build a library $(TARGET).a containing all object code.
#
#MAKE_LIB = 1

# Object files to exclude from the exe target, if they exist.
#
EXE_EXCLUDE = test_%.o

# Object files to exclude from the lib target, if they exist.
#
LIB_EXCLUDE = main.o test_%.o

# Comment this out to suppress generation of the normal "executable" elf, hex,
# bin, lss, and eep products.
#
MAKE_EXE = 1

# Uncomment this to include eeprom in executable build and program.
#
# MAKE_EEP = 1

# [user@]host:path destination to publish zipfiles
#
PUBLISH_DEST = TBD

# Uncomment this to enable link-time dead code elimination.
#
# For more info read about gcc's -ffunction-sections and -Wl,--gc-sections
# options.
#
# This can be useful for both exe and lib targets; compiling a lib with
# LINK_DCE enables link-time DCE of later exe targets which link the lib.
#
LINK_DCE = 1

# Uncomment this to enable compile-time dead code elimination.
#
# For more info read about gcc's -combine and -fwhole-program options.
#
# This applies only to exe target (not lib), and (for exe) takes precedence over
# LINK_DCE.
#
# COMPILE_DCE = 1

# Debug info format, or comment out to disable debug info generation (should
# produce smaller code). Native formats for avr-gcc -g are stabs [default], or
# dwarf-2.  AVR (extended) COFF requires stabs, plus an avr-objcopy run.
#
# DBGINFO = stabs

# Set this nonzero to define the C macro DBG at compile time.  This may be used
# to enable runtime debug code, and is orthogonal to compiling with debug info
# (see DBGINFO).
#
# ?= syntax gives preference to inherited value from environment var, if any
#
DBG ?= 1

# Avrdude programmer options.
#
# ?= syntax gives preference to inherited value from environment var, if any
#
# orignal stk500 on a Linux serial port
# AVRDUDE_PROGRAMMER = stk500v2
# AVRDUDE_PORT ?= /dev/ttyS0
#
# avrisp connected to usb (-B command sets sck in us, default may be very slow)
# AVRDUDE_PROGRAMMER = avrisp2 -B 5
# AVRDUDE_PORT = usb
#
# pololu orangutan SVP on Linux
AVRDUDE_PROGRAMMER = avrisp2
AVRDUDE_PORT ?= /dev/ttyACM0

# note: you may need privileged access to the usb port, so you either have to
# run the program command as root (sudo) or you can remount the usb file system
# to give yourself permission:
#
# sudo mount -t usbfs none /proc/bus/usb -o remount,devgid=`id -g`,devmode=664
#
# A similar line could be put in your /etc/fstab to make this happen
# automatically at boot

# Uncomment the following if you do not wish a verification to be performed
# after programming the device.
#
# AVRDUDE_NO_VERIFY = -V

# Uncomment the following to enable avrdude's erase cycle counter.  Note that
# this counter needs to be initialized first using -Yn, see avrdude manual.
#
#AVRDUDE_ERASE_COUNTER = -y

# Increase avrdude verbosity level.  Please use this when submitting bug
# reports about avrdude. See <http://savannah.nongnu.org/projects/avrdude> to
# submit bug reports.
#
#AVRDUDE_VERBOSE = -v -v

# Define [FUSE_BYTE | (LFUSE_BYTE HFUSE_BYTE [EFUSE_BYTE])] and [LOCK_BYTE]
# in hex to have them programmed by the program-fuses and program-lock
# targets.
#
# The read-*fuse targets may be used to read the current fuse settings from the
# device.
#
# FUSE_BYTE =
#
# mega128 lfuse default 11100001
# mega128 gotcha: defaults to internal oscillator
# mega128 external osc >= 8MHz slow rise power 11111111
# LFUSE_BYTE = ff
#
# mega128 hfuse default 10011001 = 99
# mega128 hfuse gotcha: jtag is enabled by default
# mega128 hfuse w/o jtag 11011001 = d9
# HFUSE_BYTE = d9
#
# mega128 efuse default 11111101
# mega128 efuse gotcha: atmega123 compat mode is default, screws up port c
# mega128 efuse w/0 compat 11111111 = ff
# EFUSE_BYTE = ff
#
# LOCK_BYTE =

# Compiler flag to set the C Standard level.
#
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
#
CSTANDARD = -std=gnu99

# avr-libc provides three printf implementations.  The default, chosen by
# commenting out PRINTF_LIB, implements everything except floating point
# conversions.  Set PRINTF_LIB to PRINTF_LIB_FLOAT to enable those, at the
# expense of some code space.  To save even more code space, use
# PRINTF_LIB_MIN, which skips most format flags.
#
# PRINTF_LIB_MIN - Minimalistic printf version
# PRINTF_LIB_FLOAT - Floating point printf version (requires MATH_LIB = -lm)
#
#PRINTF_LIB = $(PRINTF_LIB_MIN)
PRINTF_LIB = $(PRINTF_LIB_FLOAT)

# Similar to printf, avr-libc provides three scanf implementations.
#
# SCANF_LIB_MIN - Minimalistic scanf version
# SCANF_LIB_FLOAT - Floating point scanf version (requires MATH_LIB = -lm)
#
#SCANF_LIB = $(SCANF_LIB_MIN)
SCANF_LIB = $(SCANF_LIB_FLOAT)

# Uncomment this to link with the standard math library (last)
#
MATH_LIB = -lm

# Add extra -D or -U options here

# Add extra -I options here
CINCS += -I . -I $(OHMM_DIR)

# Add extra -L and -l options here
EXTRA_LIBS += -L $(OHMM_DIR) -lohmm -lpololu_$(MCU)

# Add extra CFLAGS here
# CEXTRA += 

# External memory options, leave commented out for no external memory.
#
# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# used for variables (.data/.bss) and heap (malloc()).
#EXTMEMOPTS = -Wl,-Tdata=0x801100,--defsym=__heap_end=0x80ffff
#
# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# only used for heap (malloc()).
#
#EXTMEMOPTS = -Wl,--defsym=__heap_start=0x801100,--defsym=__heap_end=0x80ffff

# Now include the all the generic AVR makefile stuff.
#
include makefile_AVR.mk

show-src:
	@echo SRC: $(SRC)

show-obj:
	@echo OBJ: $(OBJ)
	@echo EXE_OBJ: $(EXE_OBJ)
	@echo LIB_OBJ: $(LIB_OBJ)
