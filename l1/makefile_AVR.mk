# Hey Emacs, this is a -*- makefile -*-
#
# Generic parts of avr-gcc makefile.  This is normally included from
# "makefile", which first defines project-specific options.
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
# $Id: makefile_AVR.mk 4 2013-01-17 21:53:38Z vona $

# these let $(COMMA) and $(SPACE) be lone arguments to make function calls
COMMA := ,
EMPTY :=
SPACE := $(EMPTY) $(EMPTY)

SVN_PATH := $(shell expr "`svn info 2>/dev/null| grep ^URL`" : ".*trunk/\(.*\)")
SVN_HOME := $(shell echo $(SVN_PATH) | sed -r s/[^/]+/../g)

# tools
CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
NM = avr-nm
AR = avr-ar
AVRDUDE = avrdude
RM = rm -f
MV = mv -f
CD = cd
SVNVERSION = svnversion
DATE = date
RSYNC = rsync

SVNVERSION_FILE := SVNVERSION.txt

SVN_VERSION_CMD := $(shell $(SVNVERSION) -n | cut -f2 -d:)
BUILD_DATE_CMD := $(shell $(DATE) +%Y-%m-%d)

ifneq ($(wildcard $(SVNVERSION_FILE)),)
SVN_VERSION := $(strip $(shell head -n 1 $(SVNVERSION_FILE)))
BUILD_DATE := $(strip $(shell tail -n 1 $(SVNVERSION_FILE)))
else
SVN_VERSION := $(SVN_VERSION_CMD)
BUILD_DATE := $(BUILD_DATE_CMD)
endif

CDBGINFO = $(if $(DBGINFO),-g$(DBGINFO),)
CDBG = $(if $(DBG),-DDBG=$(DBG),)

CWARN = -Wall #-Wstrict-prototypes
#CTUNING = -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CTUNING = -mcall-prologues
#CEXTRA = -Wa,-adhlns=$(<:.c=.lst)
#ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs 
#compile-time DCE compiler options
CDCEC = -combine -fwhole-program
#link-time DCE compiler options
CDCEL = -ffunction-sections
#link-time DCE linker options
LDCEL = -Wl,--gc-sections

CFLAGS = $(CDBGINFO) $(CDBG) $(CDEFS) $(CINCS) -O$(OPT) $(CWARN) $(CTUNING) $(CSTANDARD) -DF_CPU=$(F_CPU) -DSVN_VERSION=\"$(SVN_VERSION)\" -DBUILD_DATE=\"$(BUILD_DATE)\" $(CEXTRA) $(CPROJ)

# used for "make depend"
DEPENDFILE = makefile.depend

# Minimalistic printf version
PRINTF_LIB_MIN = -Wl,-u,vfprintf -lprintf_min
# Floating point printf version (requires MATH_LIB = -lm below)
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt
# Minimalistic scanf version
SCANF_LIB_MIN = -Wl,-u,vfscanf -lscanf_min
# Floating point + %[ scanf version (requires MATH_LIB = -lm below)
SCANF_LIB_FLOAT = -Wl,-u,vfscanf -lscanf_flt

#LDMAP = $(LDFLAGS) -Wl,-Map=$(TARGET).map,--cref
#LDTUNING = -Wl,--relax
# Bug 2596 - Wrong code: linker relaxation for AVR 
# http://sourceware.org/bugzilla/show_bug.cgi?id=2596
LDTUNING = 
LDFLAGS = $(LDTUNING) $(EXTMEMOPTS) $(LDMAP)
LDLIBS = $(PRINTF_LIB) $(SCANF_LIB) $(EXTRA_LIBS) $(MATH_LIB)

# Programming support using avrdude.
AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex
AVRDUDE_WRITE_EEPROM = -U eeprom:w:$(TARGET).eep
AVRDUDE_WRITE_FUSES = $(if $(FUSE_BYTE),-U fuse:w:0x$(FUSE_BYTE):m,$(if $(LFUSE_BYTE),-U lfuse:w:0x$(LFUSE_BYTE):m) $(if $(HFUSE_BYTE),-U hfuse:w:0x$(HFUSE_BYTE):m) $(if $(EFUSE_BYTE),-U efuse:w:0x$(EFUSE_BYTE):m))
AVRDUDE_WRITE_LOCK = $(if $(LOCK_BYTE),-U lock:w:$(LOCK_BYTE):m)
AVRDUDE_READBACK_FILE = .avrdude-readback
DUMP_AVRDUDE_READBACK_FILE = echo; echo $@; echo 76543210; xxd -b $(AVRDUDE_READBACK_FILE) | cut -d ' ' -f 2; xxd $(AVRDUDE_READBACK_FILE) | cut -d ' ' -f 2; rm $(AVRDUDE_READBACK_FILE)
AVRDUDE_BASIC = -p $(AVRDUDE_MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER)
AVRDUDE_FLAGS = $(AVRDUDE_BASIC) $(AVRDUDE_NO_VERIFY) $(AVRDUDE_VERBOSE) $(AVRDUDE_ERASE_COUNTER) 

# excludes the usual suspects
RSYNC_FLAGS := -rv --progress --exclude "**~" --exclude ".\#**" --exclude CVS --exclude .svn --exclude "*.tmp" --cvs-exclude

# Define all object files.
OBJ = $(SRC:.c=.o) $(ASRC:.S=.o) 
EXE_OBJ := $(filter-out $(EXE_EXCLUDE),$(OBJ))
LIB_OBJ := $(filter-out $(LIB_EXCLUDE),$(OBJ))

# Define all listing files.
LST = $(ASRC:.S=.lst) $(SRC:.c=.lst)

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS)
ALL_ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS)

# Default target.
all: build

build: depend $(if $(MAKE_LIB),lib,) $(if $(MAKE_EXE),elf hex bin $(if $(MAKE_EEP),eep,) lss)

# For integration with super-ninja makefile recursive builds.
package: build
package.recursive: build

elf: $(TARGET).elf
hex: $(TARGET).hex
bin: $(TARGET).bin
eep: $(TARGET).eep
lss: $(TARGET).lss 
sym: $(TARGET).sym
lib: lib$(TARGET).a

# Program the device.  
program: $(TARGET).hex $(if $(MAKE_EEP),$(TARGET).eep,)
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(if $(MAKE_EEP),$(AVRDUDE_WRITE_EEPROM),)

program-eep: $(TARGET).eep
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_EEPROM)

program-fuses: 
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FUSES)

program-lock: 
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_LOCK)

read-fuse:
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U fuse:r:$(AVRDUDE_READBACK_FILE):r
	$(DUMP_AVRDUDE_READBACK_FILE)

read-lfuse:
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U lfuse:r:$(AVRDUDE_READBACK_FILE):r
	$(DUMP_AVRDUDE_READBACK_FILE)

read-hfuse:
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U hfuse:r:$(AVRDUDE_READBACK_FILE):r
	$(DUMP_AVRDUDE_READBACK_FILE)

read-efuse:
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U efuse:r:$(AVRDUDE_READBACK_FILE):r
	$(DUMP_AVRDUDE_READBACK_FILE)

read-lock:
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U lock:r:$(AVRDUDE_READBACK_FILE):r
	$(DUMP_AVRDUDE_READBACK_FILE)

# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
--change-section-address .data-0x800000 \
--change-section-address .bss-0x800000 \
--change-section-address .noinit-0x800000 \
--change-section-address .eeprom-0x810000 

coff: $(TARGET).elf
	$(COFFCONVERT) -O coff-avr $(TARGET).elf $(TARGET).cof

extcoff: $(TARGET).elf
	$(COFFCONVERT) -O coff-ext-avr $(TARGET).elf $(TARGET).cof

.SUFFIXES: .elf .hex .eep .lss .sym .cee .bin

# Create bin file from hex file.
.hex.bin:
	$(OBJCOPY) -I ihex -O binary -R .eeprom $< $@

# Create hex file from ELF output file.
.elf.hex:
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# Create eeprom file from ELF output file.
.elf.eep:
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O elf $< $@

# Create extended listing file from ELF output file.
.elf.lss:
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
.elf.sym:
	$(NM) -n $< > $@

# Link: create ELF output file from object files.
ifneq ($(COMPILE_DCE),)
$(TARGET).elf: $(SRC)
	$(CC) $(ALL_CFLAGS) $(CDCEC) $(SRC) --output $@ $(LDFLAGS) $(LDLIBS)
else ifneq ($(LINK_DCE),)
$(TARGET).elf: $(EXE_OBJ)
	$(CC) $(ALL_CFLAGS) $(EXE_OBJ) --output $@ $(LDFLAGS) $(LDCEL)  $(LDLIBS)
else
$(TARGET).elf: $(EXE_OBJ)
	$(CC) $(ALL_CFLAGS) $(EXE_OBJ) --output $@ $(LDFLAGS) $(LDLIBS)
endif

# Archive: create .a library file from object files.
lib$(TARGET).a: $(LIB_OBJ)
	$(AR) rcs $@ $(LIB_OBJ)

# macro Expand: run the c preprocessor
.c.cee:
	$(CC) $(ALL_CFLAGS) -E $< > $@

# Compile: create object files from C source files.
ifneq ($(LINK_DCE),)
.c.o:
	$(CC) -c $(ALL_CFLAGS) $(CDCEL) $< -o $@ 
else
.c.o:
	$(CC) -c $(ALL_CFLAGS) $< -o $@ 
endif

# Compile: create assembler files from C source files.
.c.s:
	$(CC) -S $(ALL_CFLAGS) $< -o $@

# Assemble: create object files from assembler source files.
.S.o:
	$(CC) -c $(ALL_ASFLAGS) $< -o $@

# Target: clean project.
clean:
	$(RM) \
	$(TARGET).cof $(TARGET).elf $(TARGET).bin \
	$(TARGET).map $(TARGET).sym $(TARGET).lss \
	$(OBJ) $(LST) $(SRC:.c=.s) $(SRC:.c=.d) $(SRC:.c=.cee) \
	makefile.depend

realclean: clean
	$(RM) $(TARGET).hex $(TARGET).bin $(TARGET).eep lib$(TARGET).a $(DEPENDFILE)

depend:
	$(CC) -M -mmcu=$(MCU) $(CDEFS) $(CINCS) $(SRC) $(ASRC) > $(DEPENDFILE)

.PHONY:	all build package package.recursive elf hex eep lss sym program coff extcoff clean realclean depend

# use this target as a prereq to force another target
.PHONY: FORCE
FORCE:

# use -include so that build does not fail if dependfile is not present
-include $(DEPENDFILE)

# zipfile targets
ZBN = $(TARGET)
ZBN_NEWEST := $(ZBN)-newest
ZBN_PAT := $(ZBN)-R*
ZBN_SUBST := $(ZBN)-R$(SVN_VERSION)_$(BUILD_DATE)
ZIP_DIR := .$(ZBN).tmp

zip:
	mkdir $(ZIP_DIR)
	mkdir $(ZIP_DIR)/$(ZBN_SUBST)
	$(RSYNC) $(RSYNC_FLAGS) --exclude "*.zip" * $(ZIP_DIR)/$(ZBN_SUBST)
	$(CD) $(ZIP_DIR) && zip -rp $(ZBN_SUBST).zip $(ZBN_SUBST)
	mv $(ZIP_DIR)/$(ZBN_SUBST).zip .
	ln -s $(ZBN_SUBST).zip $(ZBN_NEWEST).zip

zip-clean:
	$(RM) -r $(ZIP_DIR) $(ZBN_PAT).zip $(ZBN_NEWEST).zip

publish: $(ZBN_SUBST).zip
	@echo publishing to $(PUBLISH_DEST)
	$(RSYNC) $(ZBN_SUBST).zip $(PUBLISH_DEST)
	$(RSYNC) --links $(ZBN_NEWEST).zip $(PUBLISH_DEST)

$(SVNVERSION_FILE): FORCE
	@echo "$(SVN_VERSION_CMD)" > $@
	@echo "$(BUILD_DATE_CMD)" >> $@

show-svnversion: FORCE
	@echo "$(SVN_VERSION) (cmd: $(SVN_VERSION_CMD))"
	@echo "$(BUILD_DATE) (cmd: $(BUILD_DATE_CMD))"

show-svn-paths:
	@echo SVN_PATH: $(SVN_PATH)
	@echo SVN_HOME: $(SVN_HOME)

.PHONY: zip zip-clean publish show-svnversion show-svn-paths

# prevent make from always trying to remake makefiles
makefile makefile_AVR.mk: ;
