#makefile for Lego Power Functions Receiver

TARGET=LPFR
export CPU=20000000UL
export DEVICE = atmega328p
export AVRDUDE_DEVICE = m328p
PORT_SERIAL = /dev/ttyUSB0
AVRDUDE=avrdude
AVRDUDE_TOOL=avrisp2

OBJ2HEX=avr-objcopy
OBJDUMP=avr-objdump
CC=avr-gcc
CXX=avr-g++
AVRSIZE=avr-size


LIBPATH=-Llib/libMWAVR
LIBS=-lmwavr
INCLUDE=-Ilib/libMWAVR

CFLAGS=-g -Wall -mcall-prologues -mmcu=$(DEVICE) -Os -DF_CPU=$(CPU) $(INCLUDE) -ffunction-sections -fno-threadsafe-statics -fno-inline-small-functions
CXXFLAGS=-g -Wall -mcall-prologues -mmcu=$(DEVICE) -Os -DF_CPU=$(CPU) $(INCLUDE) -ffunction-sections -fno-threadsafe-statics -fno-inline-small-functions
LDFLAGS=-Wl,-gc-sections -Wl,-Map=$(TARGET).map $(LIBPATH) $(LIBS) -Wl,-relax


OBJECT_FILES=src/*.cc lib/IRremote/*.cpp

$(TARGET): all

Debug: all
Release: all uploadserial

Program: uploadserial

uploadserial:
	$(AVRDUDE) -p $(AVRDUDE_DEVICE) -c arduino -b 57600 -P $(PORT_SERIAL) -U flash:w:$(TARGET).hex -q

all: $(TARGET).hex \
	dumpinfo

dumpinfo:
	$(OBJDUMP) -h -S $(TARGET).elf > $(TARGET).dump
	$(AVRSIZE) $(TARGET).elf

$(TARGET).hex: $(TARGET).elf
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

$(TARGET).elf: $(OBJECT_FILES)
	$(CC) $(CFLAGS) $(OBJECT_FILES) $(LDFLAGS) -o $@

otherlibs:
	cd lib/libMWAVR && $(MAKE) CPU=$(CPU) DEVICE=$(DEVICE)



clean:
	rm -f *.o *.hex *.elf *.hex *.obj *.map *.dump *.info
cleanDebug: clean
cleanRelease: clean




