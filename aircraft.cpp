/*
 Copyright 2014 Jon Lochner
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "aircraft.h"
#include "multifly.h"
#include "lib_fp.h"

extern globalstruct global;
extern settingsstruct settings;

#define SERVO_DIR(s) settings.servo[s].direction
#define SERVO_MID(s) settings.servo[s].middle

void compute_mix(fixedpointnum throttle, fixedpointnum pid[]) {
#if (AIRCRAFT_CONFIGURATION==QUADX || AIRCRAFT_CONFIGURATION==VTAIL4)
    global.motor[QUADX_REAR_RIGHT_MOTOR] =throttle-pid[ROLL_INDEX]+pid[PITCH_INDEX]-pid[YAW_INDEX];
    global.motor[QUADX_FRONT_RIGHT_MOTOR]=throttle-pid[ROLL_INDEX]-pid[PITCH_INDEX]+pid[YAW_INDEX];
    global.motor[QUADX_REAR_LEFT_MOTOR]  =throttle+pid[ROLL_INDEX]+pid[PITCH_INDEX]+pid[YAW_INDEX];
    global.motor[QUADX_FRONT_LEFT_MOTOR] =throttle+pid[ROLL_INDEX]-pid[PITCH_INDEX]-pid[YAW_INDEX];
#elif (AIRCRAFT_CONFIGURATION==QUADP)
    global.motor[QUADP_RIGHT_MOTOR] =throttle-pid[ROLL_INDEX]-pid[YAW_INDEX];
    global.motor[QUADP_FRONT_MOTOR] =throttle-pid[PITCH_INDEX]-pid[YAW_INDEX];
    global.motor[QUADP_REAR_MOTOR]  =throttle+pid[PITCH_INDEX]-pid[YAW_INDEX];
    global.motor[QUADP_LEFT_MOTOR]  =throttle+pid[ROLL_INDEX]+pid[YAW_INDEX];
#elif (AIRCRAFT_CONFIGURATION==TRI)
    global.motor[TRI_REAR_MOTOR] =throttle+lib_fp_multiply(FIXEDPOINT_FOUR_THIRDS,pid[PITCH_INDEX]);
    global.motor[TRI_LEFT_MOTOR] =throttle-pid[ROLL_INDEX]-lib_fp_multiply(FIXEDPOINT_TWO_THIRDS,pid[PITCH_INDEX]);
    global.motor[TRI_RIGHT_MOTOR]=throttle=pid[ROLL_INDEX]-lib_fp_multiply(FIXEDPOINT_TWO_THIRDS,pid[PITCH_INDEX]);
    global.servo[TRI_REAR_SERVO] =SERVO_DIR(TRI_REAR_SERVO)*pid[YAW_INDEX]+SERVO_MID(TRI_REAR_SERVO);
#elif (AIRCRAFT_CONFIGURATION==BI)
    global.motor[BI_LEFT_MOTOR] =throttle+pid[ROLL_INDEX];
    global.motor[BI_RIGHT_MOTOR]=throttle-pid[ROLL_INDEX];
    global.servo[BI_LEFT_SERVO] =(SERVO_DIR(BI_LEFT_SERVO)*pid[YAW_INDEX])-(SERVO_DIR(BI_LEFT_SERVO)*pid[PITCH_INDEX])+SERVO_MID(BI_LEFT_SERVO);
    global.servo[BI_RIGHT_SERVO]=(SERVO_DIR(BI_RIGHT_SERVO)*pid[YAW_INDEX])-(SERVO_DIR(BI_LEFT_SERVO)*pid[PITCH_INDEX])+SERVO_MID(BI_RIGHT_SERVO);
#else
    #error "Must define mix for aircraft"
#endif
}
