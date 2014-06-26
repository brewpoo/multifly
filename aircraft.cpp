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

#include <stdlib.h>
#include "aircraft.h"
#include "multifly.h"
#include "lib_fp.h"

extern globalstruct global;
extern settingsstruct settings;

#define SERVO_DIR(s) FIXEDPOINTCONSTANT(abs(settings.servo[s].rate)/settings.servo[s].rate)
#define SERVO_MID(s) FIXEDPOINTCONSTANT(settings.servo[s].middle)

#define MOTOR_MIX(X,Y,Z) throttle+lib_fp_multiply(pid[ROLL_INDEX],FIXEDPOINTCONSTANT(X)) \
                               +lib_fp_multiply(pid[PITCH_INDEX],FIXEDPOINTCONSTANT(Y))\
                               +lib_fp_multiply(pid[YAW_INDEX],FIXEDPOINTCONSTANT(Z))

void compute_mix(fixedpointnum throttle, fixedpointnum pid[]) {
#if (AIRCRAFT_CONFIGURATION==QUADX || AIRCRAFT_CONFIGURATION==VTAIL4)
    global.motor[QUADX_REAR_RIGHT_MOTOR] = MOTOR_MIX(-1,+1,-1);
    global.motor[QUADX_FRONT_RIGHT_MOTOR]= MOTOR_MIX(-1,-1,+1);
    global.motor[QUADX_REAR_LEFT_MOTOR]  = MOTOR_MIX(+1,+1,+1);
    global.motor[QUADX_FRONT_LEFT_MOTOR] = MOTOR_MIX(+1,-1,-1);
#elif (AIRCRAFT_CONFIGURATION==QUADP)
    global.motor[QUADP_REAR_MOTOR]  = MOTOR_MIX( 0,+1,-1);
    global.motor[QUADP_RIGHT_MOTOR] = MOTOR_MIX(-1, 0,+1);
    global.motor[QUADP_LEFT_MOTOR]  = MOTOR_MIX(+1, 0,+1);
    global.motor[QUADP_FRONT_MOTOR] = MOTOR_MIX( 0,-1,-1);
#elif (AIRCRAFT_CONFIGURATION==TRI)
    global.motor[TRI_REAR_MOTOR] = MOTOR_MIX( 0, +4/3, 0);
    global.motor[TRI_LEFT_MOTOR] = MOTOR_MIX(+1, -2/3, 0);
    global.motor[TRI_RIGHT_MOTOR]= MOTOR_MIX(-1, -2/3, 0);
    global.servo[TRI_REAR_SERVO] = lib_fp_multiply(SERVO_DIR(TRI_REAR_SERVO),pid[YAW_INDEX])+SERVO_MID(TRI_REAR_SERVO);
#elif (AIRCRAFT_CONFIGURATION==BI)    
    global.motor[BI_LEFT_MOTOR] = MOTOR_MIX(+1, 0, 0);
    global.motor[BI_RIGHT_MOTOR]= MOTOR_MIX(-1, 0, 0);
    global.servo[BI_LEFT_SERVO] = lib_fp_multiply(SERVO_DIR(BI_LEFT_SERVO),pid[YAW_INDEX])-lib_fp_multiply(SERVO_DIR(BI_LEFT_SERVO),pid[PITCH_INDEX])+SERVO_MID(BI_LEFT_SERVO);
    global.servo[BI_RIGHT_SERVO]=lib_fp_multiply(SERVO_DIR(BI_RIGHT_SERVO),pid[YAW_INDEX])-lib_fp_multiply(SERVO_DIR(BI_LEFT_SERVO),pid[PITCH_INDEX])+SERVO_MID(BI_RIGHT_SERVO);
#else
    #error "Must define mix for aircraft"
#endif
}
