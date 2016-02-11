### DISCLAIMER
### This is an example Makefile and it MUST be configured to suit your needs.
### For detailled explanations about all the avalaible options,
### please refer to https://github.com/sudar/Arduino-Makefile/blob/master/arduino-mk-vars.md

### PROJECT_DIR
### This is the path to where you have created/cloned your project
PROJECT_DIR       = /home/mathew/Development/PowerFunctions

### AVR_GCC_VERSION
### Check if the version is equal or higher than 4.9
AVR_GCC_VERSION  := $(shell expr `avr-gcc -dumpversion | cut -f1` \>= 4.9)

### ARDMK_DIR
### Path to the Arduino-Makefile directory.
ARDMK_DIR         = $(PROJECT_DIR)/Arduino-Makefile

### ARDUINO_DIR
### Path to the Arduino application and ressources directory.
ARDUINO_DIR       = /home/mathew/Applications/arduino

### USER_LIB_PATH
### Path to where the your project's libraries are stored.
USER_LIB_PATH     :=  $(realpath $(PROJECT_DIR)/lib)

### BOARD_TAG & BOARD_SUB
### It must be set to the board you are currently using. (i.e uno, mega, etc.)
### For the Arduino Uno, only BOARD_TAG is mandatory and BOARD_SUB can be equal to anything
### For the Arduino Mega2560, BOARD_SUB is also needed
BOARD_TAG         = uno2
#BOARD_SUB         = atmega2560

### MONITOR_BAUDRATE
### It must be set to Serial baudrate value you are using.
MONITOR_BAUDRATE  = 57600

### AVR_TOOLS_DIR
### Path to the AVR tools directory such as avr-gcc, avr-g++, etc.
AVR_TOOLS_DIR     = /usr

### AVRDDUDE
### Path to avrdude directory.
AVRDDUDE          = /usr/bin/avrdude
AVRDUDE_ARD_BAUDRATE=57600
### CFLAGS_STD
CFLAGS_STD        = -std=gnu11 -ffunction-sections -fno-threadsafe-statics -fno-inline-small-functions -mcall-prologues -Os

### CXXFLAGS_STD
CXXFLAGS_STD      = -std=gnu++11 -ffunction-sections -fno-threadsafe-statics -fno-inline-small-functions -mcall-prologues -Os

### CPPFLAGS
### Flags you might want to set for debugging purpose. Comment to stop.
CXXFLAGS         =  -Wall  -ffunction-sections -fno-threadsafe-statics -fno-inline-small-functions -mcall-prologues -Os

### If avr-gcc -v is higher than 4.9, activate coloring of the output
ifeq "$(AVR_GCC_VERSION)" "1"
    CXXFLAGS += -fdiagnostics-color
endif

### MONITOR_PORT
### The port your board is connected to. Using an '*' tries all the ports and finds the right one.
MONITOR_PORT      = /dev/ttyUSB0

### don't touch this
CURRENT_DIR       = $(shell basename $(CURDIR))

### OBJDIR
### This is were you put the binaries you just compile using 'make'
CURRENT_DIR       = $(shell basename $(CURDIR))
OBJDIR            = $(PROJECT_DIR)/bin/$(CURRENT_DIR)/$(BOARD_TAG)

AVRDUDE_OPTS = -q -V -D

### path to Arduino.mk, inside the ARDMK_DIR, don't touch.
include $(ARDMK_DIR)/Arduino.mk

