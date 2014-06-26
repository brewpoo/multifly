#Multifly

##Introduction

Multifly is an open source software alternative for MultiWii-compatible flight controllers.  This software is based on [BradWii](https://github.com/bradquick/bradwii) which is a complete rewrite/rework of the official MultiWii codebase written by Brad Quick.  Here are Brad's comments regarding his hardwork on the BradWii project:

>> BradWii is open source multi-copter software.  The name BradWii comes from the fact that many concepts and some code were borrowed from Multi-Wii.  It's also a play on the original developer's name, Bradley (Call me Brad).  Though based on Multi-Wii, BradWii is pretty much a complete re-write.  BradWii is intended to be a platform on which other projects can be built.

## Goals
The primary goals of the Multifly software are:

- Stable, easy to configure, and easy to tune flight controller

- Straight forward code base that is understandable and modular

- MultiWii Protocol (MWP) compatible, supporting serial and bluetooth
	- MultiWii Config Tool
 	- Android/iPhone apps
 	- MinimOSD w/KV Team OSD Firmware)
  
- Pin compatible with each MultiWii supported board
	- Try Multifly without having to rewire anything
  
- Control what features make it into the code on your board
	- Easy to opt out of software features (minimize footprint)

##Features

- Currently supports the following aircraft configurations:
	- Quad X
	- Tricopter
  	- Will be adding more configurations soon
  
- Currently works with the following Hobby King boards:
	- Multi-Wii Pro 2.0
	- Multi-Wii 328p
	- Multi-Wii NanoWii
	- Will be adding more boards soon

- Extensive use of fixed point math means the code is faster and more accurate 
	- Each value has 32 bit precision and the loop times are frequently under 2 milliseconds
	- All integration and filtering is time based, which means that constant loop times aren't critical
	- The result is stable flight

- Three pilot flight control modes
  - Full Acro (Acro)
  - Semi-Acro (Horizon)
  - Level (Angle)

- Optional flight modes (may be combined with pilot flight mode)
  - Position hold (GPS position)
  - Mag Hold (Heading lock)
  - Altitude hold (Barometer)
  - Return to Home
  - Throttle assist - adjust throttle based on aircraft pitch/roll
  - Uncrashable (safe mode) - Takes over when you get below critical attitude or get too far away
  
- Auto PID tuning

- Standard or DSM2 Satellite receivers

- The project doesn't rely on any Arduino libraries which means that it can be easily developed on other development systems.

- Code compiles from XCode or Arduino

## Planned Features

- Autopilot
  - Multiple 3D waypoints
  - Potential support for MAVlink

- Autoland
  - Controlled vertical descent
  - Auto disarm on landing
  
- Advanced Light Controls
  - Single-wire based LED pixels and strip
  - Landing and navigating configurations

##Quick Setup Guide

1. Download the Arduino Software.
2. Download the BradWii project.
3. Download a Multi-Wii config program for your comptuer.
4. Edit the config.h file to match your particular setup.
5. Upload the software to your control board.
6. Connect to the board with the Multi-Wii config program.
7. Calibrate the Accelerometer.
8. Calibrate the Magnetometer (if your board has one)
9. Set the checkboxes that select what the aux switches do.  Make sure one arms the board.  Set up another aux switch for autotune.
10. Write the settings to the board.
11. Before installing a battery, while still connected via Multi-Wii Config, use your transmitter to arm the board.  Apply throttle and other inputs and verify that the motor outputs react as expected.
12. Calibrate your ESC's:
	1. Remove the props
	2. Edit config.h and remove the two slashes from in front of the ESC_CALIB_CANNOT_FLY
	3. Re-upload the software into the control board.
	4. Unplug the board from the computer.  Plug in the battery.  Wait for the ESC's to stop making their noises.
	5. Unplug the battery.
	6. Put the two slashes back in front of ESC_CALIB_CANNOT_FLY
	7. Re-upload the software into the control board.	
13. Remove the props, install the battery and test the operation.
14. Fly it.
15. While hovering, switch the autotune aux switch.  Let it oscillate for about 30 secconds (more is better).  Switch autotune off and back on again and let it oscillate again for 30 seconds.  Land, disarm, switch autotune back on and off again.
16. Use Multi-Wii Config to set your aux switches to do other cool stuff.
17. Fly
