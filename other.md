##More Detailed Operating Instructions

###Config

Brad-Wii is compatible with Multi-Wii config programs.  Use any of these programs to configure the software once it is installed.  Note that writes to eeprom won't take place if the aircraft is armed.  When you hit the "Write" button, confirm that the updating of the config program pauses for a second or so.  This indicates that the eeprom write is being performed.

###Calibrating

Set the aircraft on a level surface. Use the config program to calibrate the accelerometer.  This will also calibrate the gyro.  Calibration will only work when disarmed.

If you are using a compass (magnetometer), also calibrate the compass.  Click the calibrate mag button on the config program and then rotate the aircraft in all directions for 30 seconds. For best calibration, first figure out which direction the magnetic field points (highest mag values in the config program).  In most places, it points down into the earth at a steep angle.  While calibrating, make sure both ends of each axis get pointed in this direction during the 30 second period.  When the config program starts updating again, the calibration is done.  Yaw the aircraft 360 degrees and verify the compass feedback on the config program.  If it doesn't look right, repeat the calibration.

###The LED

The LED stays lit when the signals from the gyros and accelerometers indicate that the aircraft is stable.  If you move the aircraft around by hand, you should notice the led going off when it's accelerating. When hovering, the LED should be lit most of the time.  If it's off more than it's on during hover, then you probably have too much vibration from the motors feeding back into the control board.

When using a GPS, the LED won't light until 5 satellites have been acquired.

###Arming

Arming currently needs to be done using Aux switches.  By default it arms when Aux1 goes high.  It will only arm and disarm when the throttle is low.  When armed, the props should all spin at a low rate.

###Flight Modes

The software defaults to Level mode, where the angle of the transmitter stick corresponds directly to the angle of the aircraft.

Acro mode can be turned on via Aux switches when setup in the config program. In acro mode, the angle of the transmitter stick corresponds with the aircraft's rate of rotation.

Semi Acro mode can also be set via Aux switches.  Semi Acro mode is a mix between acro mode and Level mode.  This the easiest mode to do flips without crashing.

Acro rotation rates can be set using Multi-Wii Config.

Maximum tilt angles (when in level mode) and rotation rates (when in acro mode) have dual rates.  The high rates (or they could be lower rates if desired) can be activated by aux switches when setup using the Multi-Wii Config program.  One preferred setup is to have high angles active when aux2 is switched off, and to have low angles, high rates, and semi acro mode active when aux2 is switched on.

###Autotuning

To use autotuning, use the Multi-Wii Config program to assign autotuning to one of your Aux switches.

Hover your aircraft as still as possible and flip the the aux switch to turn on autotuning.  The aircraft should start to roll back and forth.  If you can, let it do so for 30 seconds or longer.  While it's doing this, try to use only pitch and yaw to bring keep the aircraft from flying away.

Flip the aux switch off, then back on again.  This time it should start rocking in the pitch direction.  Again, try not to use pitch control while it's doing this.  After 30 seconds or longer, turn the switch off and test it.  You can use the switch as many times as you want.  It will alternate between tuning roll and tuning pitch.

If you want the new settings to be saved to eeprom, land, then dis-arm, then switch the autotune aux switch on while dis-armed.  This will cause a write to eeprom.

###Throttle Helper

Throttle helper mode automatically applies extra throttle when the aircraft is banked.  Without throttle helper activated, when you bank, you lose vertical lift, so if you don't apply extra throttle, you lose altitude.  Throttle helper applies the extra throttle for you so that you don't lose altitude.

###Altitude Hold

If you have a barometer, altitude hold keeps the aircraft at a constant altitude by varying the throttle setting automatically.  Throttle helper is automatically activated when altitude hold is activated in order to minimize altitude fluctuation due to banking of the aircraft (flying it around).

###Position Hold

This holds the aircraft at the GPS location where position hold was activated. Note that altitude hold isn't automatically activated with position hold, so if you also want altitude hold, you need to set it also.

###Return To Home

When return to home is activated, the aircraft will return to the home position, which will be where it was armed.  Note that altitude hold isn't automatically activated with return to home, so if you also want altitude hold, you need to set it also.

###Uncrashability Mode

Uncrashability (somewhat experimental) attempts to make the aircraft uncrashable by taking over the controls when danger approaches.  It uses the barometer to determine altitude and optionally uses the gps to determine location.  The home location and altitude are set when uncrashability is activated, so it should be activated when the aircraft is at a safe altitude.  If the software anticipates that the altitude will go below the home altitude, it will automatically bring the aircraft to level, then apply throttle to get it back to the home level.  If it goes more than a pre-determined distance from the home location, it will take over the control of the aircraft until it gets back in bounds.



