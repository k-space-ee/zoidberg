###############################################################################
#
# Simple Makefile for Arduino MEGA 2560 C projects
#
###############################################################################

CC = avr-gcc
OBJCOPY = avr-objcopy
AVRDUDE = avrdude
CODE_FORMATTER = tooling/format-code.sh
AVRSIZE = avr-size

BOARD = atmega2560

# Use shell command export to define it
# Example: export ARDUINO=/dev/ttyACM0
DEVICE = $(ARDUINO)

# Build artifacts
BINDIR = bin
TARGET = $(BINDIR)/$(BOARD)-user-code.ihx
ELF = $(BINDIR)/$(BOARD)-user-code.elf

# Source files. wildard "uses" all .c files in src directory
SRCDIR = src
BUILD_LIBS_DIR = lib
SRC = $(wildcard $(SRCDIR)/*.c $(BUILD_LIBS_DIR)/*/*.c)

# Define object files from .c files defined above
OBJ=$(SRC:.c=.o)

# Compiler flags
# Note that those beginning with -D are acctually pre-processor macros
# -Wall ... -Wfatal-errors All possible warning options
# -Os Optimize code. The special option -Os is meant to turn on all -O2
# optimizations that are not expected to increase code size.
# -std=c11 use C11 standard
CFLAGS =	-Wall \
			-Wextra \
			-Wpedantic \
			-Wformat \
			-pedantic-errors \
			-Werror \
			-Wfatal-errors \
			-Os \
			-flto \
			-fdata-sections \
			-ffunction-sections \
			-mmcu=$(BOARD) \
			-DF_CPU=16000000UL \
			-DGIT_DESCR=\"$(shell git describe --abbrev=6 --dirty --always --tags --long)\" \
			-DDEBUG=$(DEBUG) \
			-ffreestanding \
			-std=gnu11

# Linker flags
LDFLAGS =	-mmcu=$(BOARD) \
			-flto \
			-Wl,-gc-sections

OBJCOPYARGS =	-O ihex \
				-R .eeprom

# FIXME Find out why some Arduinos require -D to write code
AVRDUDEARGS =	-p $(BOARD) \
				-c wiring \
				-F \
				-P $(DEVICE) \
				-b 115200 \
				-D

AVRSIZEARGS =	-C \
				--mcu=$(BOARD)

all: $(ELF) $(TARGET)

%.o : %.c
	$(CC) -c $(CFLAGS) -o $*.o $<

$(ELF): $(OBJ)
	$(CC) $(LDFLAGS) $^ -o $@

$(TARGET):
	$(OBJCOPY) $(OBJCOPYARGS) $(ELF) $(TARGET)

clean:
#Do not remove .placeholder in BINDIR
	find $(BINDIR) -type f -not -name '.placeholder' -print0 | xargs -0 rm -f --
	rm -f $(SRCDIR)/*.o
	rm -fr $(BUILD_LIBS_DIR)/*/*.o

install:
	@if [ ! -c "$(ARDUINO)" ]; then \
		echo -e "\n\nEnvironment variable ARDUINO is \"$(ARDUINO)\" and that is invalid."\
				"\nDid you do \"export ARDUINO=/dev/ttyACM0\" before running make install?"\
				"\nAlso make sure that ARDUINO env var points to a valid tty device\n\n"; \
		exit 1;\
	fi

	$(AVRDUDE) $(AVRDUDEARGS) -U flash:w:$(TARGET)

format:
	$(CODE_FORMATTER) $(SRCDIR)/*.c

size:
	$(AVRSIZE) $(AVRSIZEARGS) $(ELF)

.PHONY: clean install format size
