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

#include "options.h"
#include "config.h"
#include "lib_fp.h"

// set aircraft type dependant defines here
#if (AIRCRAFT_CONFIGURATION==QUADX || AIRCRAFT_CONFIGURATION==VTAIL4)
    #define NUM_MOTORS 4
    #define QUADX_REAR_RIGHT_MOTOR 0
    #define MOTOR_0_CHANNEL D3_PWM
    #define MOTOR_0_PIN     D3_PIN

    #define QUADX_FRONT_RIGHT_MOTOR 1
    #define MOTOR_1_CHANNEL D5_PWM
    #define MOTOR_1_PIN     D5_PIN

    #define QUADX_REAR_LEFT_MOTOR 2
    #define MOTOR_2_CHANNEL D6_PWM
    #define MOTOR_2_PIN     D6_PIN

    #define QUADX_FRONT_LEFT_MOTOR 3
    #define MOTOR_3_CHANNEL D2_PWM
    #define MOTOR_3_PIN     D2_PIN

    #define NUM_SERVOS 0
#elif (AIRCRAFT_CONFIGURATION==QUADP)
    #define NUM_MOTORS 4
    #define QUADP_RIGHT_MOTOR 0
    #define MOTOR_0_CHANNEL D5_PWM
    #define MOTOR_0_PIN     D5_PIN

    #define QUADP_FRONT_MOTOR 1
    #define MOTOR_1_CHANNEL D2_PWM
    #define MOTOR_1_PIN     D2_PIN

    #define QUADP_REAR_MOTOR 2
    #define MOTOR_2_CHANNEL D3_PWM
    #define MOTOR_2_PIN     D3_PIN

    #define QUADP_LEFT_MOTOR 3
    #define MOTOR_3_CHANNEL D6_PWM
    #define MOTOR_3_PIN     D6_PIN

    #define NUM_SERVOS 0
#elif (AIRCRAFT_CONFIGURATION==TRI)
    #define NUM_MOTORS 3
    #define TRI_REAR_MOTOR 0
    #define MOTOR_0_CHANNEL D3_PWM
    #define MOTOR_0_PIN     D3_PIN

    #define TRI_LEFT_MOTOR 1
    #define MOTOR_1_CHANNEL D6_PWM
    #define MOTOR_1_PIN     D6_PIN

    #define TRI_RIGHT_MOTOR 2
    #define MOTOR_2_CHANNEL D5_PWM
    #define MOTOR_2_PIN     D5_PIN

    #define NUM_SERVOS 1
    #define TRI_REAR_SERVO 0
    #define SERVO_0_CHANNEL D2_PWM
    #define SERVO_0_PIN     D2_PIN
#elif (AIRCRAFT_CONFIGURATION==BI)
    #define NUM_MOTORS 2
    #define BI_LEFT_MOTOR 0
    #define MOTOR_0_CHANNEL D3_PWM
    #define MOTOR_0_PIN     D3_PIN

    #define BI_RIGHT_MOTOR 1
    #define MOTOR_1_CHANNEL D5_PWM
    #define MOTOR_1_PIN     D5_PIN

    #define NUM_SERVOS 2
    #define BI_LEFT_SERVO 0
    #define SERVO_0_CHANNEL D6_PWM
    #define SERVO_0_PIN     D6_PIN

    #define BI_RIGHT_SERVO 1
    #define SERVO_1_CHANNEL D2_PWM
    #define SERVO_1_PIN     D2_PIN
#else
  #error "Need to define an aircraft type"
#endif

void compute_mix(fixedpointnum throttle, fixedpointnum pid[]);

/*

#elif defined( Y4 )
motor[0] = PIDMIX(+0,+1,-1);   //REAR_1 CW
motor[1] = PIDMIX(-1,-1, 0); //FRONT_R CCW
motor[2] = PIDMIX(+0,+1,+1);   //REAR_2 CCW
motor[3] = PIDMIX(+1,-1, 0); //FRONT_L CW
#elif defined( Y6 )
motor[0] = PIDMIX(+0,+4/3,+1); //REAR
motor[1] = PIDMIX(-1,-2/3,-1); //RIGHT
motor[2] = PIDMIX(+1,-2/3,-1); //LEFT
motor[3] = PIDMIX(+0,+4/3,-1); //UNDER_REAR
motor[4] = PIDMIX(-1,-2/3,+1); //UNDER_RIGHT
motor[5] = PIDMIX(+1,-2/3,+1); //UNDER_LEFT
#elif defined( HEX6 )
motor[0] = PIDMIX(-7/8,+1/2,+1); //REAR_R
motor[1] = PIDMIX(-7/8,-1/2,-1); //FRONT_R
motor[2] = PIDMIX(+7/8,+1/2,+1); //REAR_L
motor[3] = PIDMIX(+7/8,-1/2,-1); //FRONT_L
motor[4] = PIDMIX(+0  ,-1  ,+1); //FRONT
motor[5] = PIDMIX(+0  ,+1  ,-1); //REAR
#elif defined( HEX6X )
motor[0] = PIDMIX(-1/2,+7/8,+1); //REAR_R
motor[1] = PIDMIX(-1/2,-7/8,+1); //FRONT_R
motor[2] = PIDMIX(+1/2,+7/8,-1); //REAR_L
motor[3] = PIDMIX(+1/2,-7/8,-1); //FRONT_L
motor[4] = PIDMIX(-1  ,+0  ,-1); //RIGHT
motor[5] = PIDMIX(+1  ,+0  ,+1); //LEFT
#elif defined( HEX6H )
motor[0] = PIDMIX(-1,+1,-1); //REAR_R
motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
motor[2] = PIDMIX(+ 1,+1,+1); //REAR_L
motor[3] = PIDMIX(+ 1,-1,-1); //FRONT_L
motor[4] = PIDMIX(0 ,0 ,0); //RIGHT
motor[5] = PIDMIX(0 ,0 ,0); //LEFT
#elif defined( OCTOX8 )
motor[0] = PIDMIX(-1,+1,-1); //REAR_R
motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
motor[2] = PIDMIX(+1,+1,+1); //REAR_L
motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
motor[4] = PIDMIX(-1,+1,+1); //UNDER_REAR_R
motor[5] = PIDMIX(-1,-1,-1); //UNDER_FRONT_R
motor[6] = PIDMIX(+1,+1,-1); //UNDER_REAR_L
motor[7] = PIDMIX(+1,-1,+1); //UNDER_FRONT_L
#elif defined( OCTOFLATP )
motor[0] = PIDMIX(+7/10,-7/10,+1); //FRONT_L
motor[1] = PIDMIX(-7/10,-7/10,+1); //FRONT_R
motor[2] = PIDMIX(-7/10,+7/10,+1); //REAR_R
motor[3] = PIDMIX(+7/10,+7/10,+1); //REAR_L
motor[4] = PIDMIX(+0   ,-1   ,-1); //FRONT
motor[5] = PIDMIX(-1   ,+0   ,-1); //RIGHT
motor[6] = PIDMIX(+0   ,+1   ,-1); //REAR
motor[7] = PIDMIX(+1   ,+0   ,-1); //LEFT
#elif defined( OCTOFLATX )
motor[0] = PIDMIX(+1  ,-1/2,+1); //MIDFRONT_L
motor[1] = PIDMIX(-1/2,-1  ,+1); //FRONT_R
motor[2] = PIDMIX(-1  ,+1/2,+1); //MIDREAR_R
motor[3] = PIDMIX(+1/2,+1  ,+1); //REAR_L
motor[4] = PIDMIX(+1/2,-1  ,-1); //FRONT_L
motor[5] = PIDMIX(-1  ,-1/2,-1); //MIDFRONT_R
motor[6] = PIDMIX(-1/2,+1  ,-1); //REAR_R
motor[7] = PIDMIX(+1  ,+1/2,-1); //MIDREAR_L
#elif defined( VTAIL4 )
motor[0] = PIDMIX(+0,+1, +1); //REAR_R
motor[1] = PIDMIX(-1, -1, +0); //FRONT_R
motor[2] = PIDMIX(+0,+1, -1); //REAR_L
motor[3] = PIDMIX(+1, -1, -0); //FRONT_L
 */
