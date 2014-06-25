# Name: Makefile
# Author: Brad Quick
# Copyright: Copyright 2013 Brad Quick
# License: Released under GPL 3 or any later

# This is a prototype Makefile. Modify it according to your needs.
# You should at least check the settings for
# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# OBJECTS ...... The object files created from your source files. This list is
#                usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#                uploading to the AVR and the interface where this hardware
#                is connected.
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.

#DEVICE     = atmega2560
CLOCK      = 16000000
#PROGRAMMER = -c stk500v2 -P/dev/cu.SLAB_USBtoUART
#PROGRAMMER = -c arduino -P/dev/cu.SLAB_USBtoUART
OBJECTS    = multifly.o lib_usb.o rx.o serial.o output.o gyro.o accelerometer.o imu.o baro.o checkboxes.o compass.o autotune.o aircraft.o pilotcontrol.o autopilot.o uncrashable.o eeprom.o gps.o navigation.o vectors.o lib_pwm.o lib_digitalio.o lib_timers.o lib_i2c.o lib_serial.o lib_fp.o
# to run at 8mhz:
#FUSES      = -U hfuse:w:0xdf:m -U lfuse:w:0xe2:m

# Tune the lines below only if you know what you are doing:

#AVRDUDE = avrdude -v -v -v -v $(PROGRAMMER) -p $(DEVICE)
AVRDUDE = avrdude -v $(PROGRAMMER) -p $(DEVICE) 
LIBRARYPATH = './'
COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -I "$(PROJECT_DIR)" -I $(LIBRARYPATH)

# install: is called for the make and burn target
# symbolic targets:
all:
	$(COMPILE) -c multifly.cpp -o multifly.o
	$(COMPILE) -c rx.cpp -o rx.o
	$(COMPILE) -c serial.cpp -o serial.o
	$(COMPILE) -c output.cpp -o output.o
	$(COMPILE) -c gyro.cpp -o gyro.o
	$(COMPILE) -c accelerometer.cpp -o accelerometer.o
	$(COMPILE) -c imu.cpp -o imu.o
	$(COMPILE) -c baro.cpp -o baro.o
	$(COMPILE) -c checkboxes.cpp -o checkboxes.o
	$(COMPILE) -c compass.cpp -o compass.o
	$(COMPILE) -c eeprom.cpp -o eeprom.o
	$(COMPILE) -c gps.cpp -o gps.o
	$(COMPILE) -c navigation.cpp -o navigation.o
	$(COMPILE) -c vectors.cpp -o vectors.o
	$(COMPILE) -c pilotcontrol.cpp -o pilotcontrol.o
	$(COMPILE) -c autotune.cpp -o autotune.o
	$(COMPILE) -c autopilot.cpp -o autopilot.o
	$(COMPILE) -c aircraft.cpp -o aircraft.o
	$(COMPILE) -c uncrashable.cpp -o uncrashable.o
	$(COMPILE) -c $(LIBRARYPATH)lib_pwm.cpp -o lib_pwm.o
#	$(COMPILE) -c $(LIBRARYPATH)analogio.cpp -o analogio.o
	$(COMPILE) -c $(LIBRARYPATH)lib_digitalio.cpp -o lib_digitalio.o
	$(COMPILE) -c $(LIBRARYPATH)lib_timers.cpp -o lib_timers.o
	$(COMPILE) -c $(LIBRARYPATH)lib_i2c.cpp -o lib_i2c.o
	$(COMPILE) -c $(LIBRARYPATH)lib_serial.cpp -o lib_serial.o
	$(COMPILE) -c $(LIBRARYPATH)lib_fp.cpp -o lib_fp.o
	$(COMPILE) -c $(LIBRARYPATH)lib_usb.cpp -o lib_usb.o
	$(COMPILE) -o multifly.elf -Wl,--gc-sections $(OBJECTS)
	avr-size multifly.elf
	rm -f multifly.hex
	avr-objcopy -j .text -j .data -O ihex multifly.elf multifly.hex
	
#all:	multifly.hex

.cpp.o:
	$(COMPILE) -c $< -o $@

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
#   python resetleonardo.py /dev/cu.usbmodem1421
	$(AVRDUDE) -U flash:w:multifly.hex:i
#avrdude.conf -v -v -v -v -patmega2560 -cstk500v2 -P/dev/cu.SLAB_USBtoUART -Uflash:w:/var/folders/qn/5h0p0ppc8xjczf001s7ppjqr0000gn/T/build5197281368797617405.tmp/MultiWii_2_1_brad.cpp.hex:i 


#fuse:
#	$(AVRDUDE) $(FUSES)

# Xcode uses the Makefile targets "", "clean" and "install"
install: flash

# if you use a bootloader, change the command below appropriately:
load: all
	bootloadHID multifly.hex

clean:
	rm -f multifly.hex multifly.elf $(OBJECTS)

# file targets:
multifly.elf: $(OBJECTS)
	echo here
	$(COMPILE) -o multifly.elf $(OBJECTS)

multifly.hex: multifly.elf
	rm -f multifly.hex
	avr-objcopy -j .text -j .data -O ihex multifly.elf multifly.hex
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	multifly.elf
	avr-objdump -d multifly.elf

cpp:
	$(COMPILE) -E multifly.c
