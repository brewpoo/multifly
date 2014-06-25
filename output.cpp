/* 
Copyright 2013 Brad Quick

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

#include <avr/io.h>
#include "output.h"
#include "lib_pwm.h"
#include "projectsettings.h"
#include "multifly.h"
#include "lib_timers.h"

extern globalstruct global;

#ifdef DC_MOTORS
   // for dc motors, we reduce the top so that we can switch at 8khz
   #define TOPMOTORCOUNT16BIT 0x3FF
   #define TOPMOTORCOUNT11BIT 0x3FF
   #define PRESCALER11BIT PWM411BITPRESCALER1
#else
   // for speed controllers, we switch at about 490Hz
   #define TOPMOTORCOUNT16BIT 0x3FFF
   #define TOPMOTORCOUNT11BIT 0x7FF // top is smaller, but we use a bigger prescaller so the cycle time is correct
   #define PRESCALER11BIT PWM411BITPRESCALER16
#endif

void init_outputs() {
#ifdef USEPWM1
    lib_pwm_init1(PWM1PHASECORRECTMODE,PWM1NORMALOUTPUTA | PWM1NORMALOUTPUTB,PWM1PRESCALER1,TOPMOTORCOUNT16BIT); // TOP to 16383
#endif
#ifdef USEPWM2
    lib_pwm_init2(PWM2FASTMODE,PWM2NORMALOUTPUTA | PWM2NORMALOUTPUTB,PWM2PRESCALER64);
#endif
#ifdef USEPWM3
    lib_pwm_init3(PWM3PHASECORRECTMODE,PWM3NORMALOUTPUTA | PWM3NORMALOUTPUTB | PWM3NORMALOUTPUTC,PWM3PRESCALER1,TOPMOTORCOUNT16BIT); // TOP to 16383
#endif
#ifdef USEPWM4
    lib_pwm_init4(PWM4PHASECORRECTMODE,PWM4NORMALOUTPUTA | PWM4NORMALOUTPUTB | PWM4NORMALOUTPUTC,PWM4PRESCALER1,TOPMOTORCOUNT16BIT); // TOP to 16383
#endif
#ifdef USEPWM411BIT
    lib_pwm_init4(PWM411BITPHASECORRECTMODE,/*PWM411BITNORMALOUTPUTA | PWM411BITNORMALOUTPUTB |*/ PWM411BITNORMALOUTPUTD,PRESCALER11BIT,TOPMOTORCOUNT11BIT); // TOP to 2047
#endif
#ifdef USEPWM5
    lib_pwm_init5(PWM5PHASECORRECTMODE,PWM5NORMALOUTPUTA | PWM5NORMALOUTPUTB | PWM5NORMALOUTPUTC,PWM5PRESCALER1,TOPMOTORCOUNT16BIT); // TOP to 16383
#endif

    lib_digitalio_initpin(MOTOR_0_PIN, DIGITALOUTPUT);
    lib_digitalio_initpin(MOTOR_1_PIN, DIGITALOUTPUT);
    lib_digitalio_initpin(MOTOR_2_PIN, DIGITALOUTPUT);
#if (NUM_MOTORS>3)
    lib_digitalio_initpin(MOTOR_3_PIN, DIGITALOUTPUT);
#endif
#if (NUM_MOTORS>4)
    lib_digitalio_initpin(MOTOR_4_PIN, DIGITALOUTPUT);
    lib_digitalio_initpin(MOTOR_5_PIN, DIGITALOUTPUT);
#endif
   
#if (NUM_MOTORS>0)
    set_all_motor_outputs(MIN_MOTOR_OUTPUT);
#endif
    
#if (NUM_SERVOS>0)
    set_all_servo_outputs(SERVO_MIDPOINT);
#endif
   
 /********  special version of MultiWii to calibrate all attached ESCs ************/
#if defined(ESC_CALIB_CANNOT_FLY)
    set_all_motor_outputs(ESC_CALIB_HIGH);
    lib_timers_delaymilliseconds(3000);
    set_all_motor_outputs(ESC_CALIB_LOW);

    while (1) {
        lib_timers_delaymilliseconds(500);
        lib_digitalio_setoutput(LED1_OUTPUT, 0);
        lib_timers_delaymilliseconds(500);
        lib_digitalio_setoutput(LED1_OUTPUT, 1);
    }
#endif
}

void set_motor_output(unsigned char motornum, unsigned char motorchannel,fixedpointnum fpvalue) {
    // set the output of a motor
    // convert from fixedpoint 0 to 1 into int 1000 to 2000
    int value=1000+((fpvalue*1000L)>>FIXEDPOINTSHIFT);
   
    if (value<ARMED_MIN_MOTOR_OUTPUT) value=ARMED_MIN_MOTOR_OUTPUT;
    if (value>MAX_MOTOR_OUTPUT) value=MAX_MOTOR_OUTPUT;
    set_output(motorchannel,value);
   
    global.motorOutputValue[motornum]=value;
}
   
void set_all_motor_outputs(int value) {
    for (int x=0;x<NUM_MOTORS;++x)
        global.motorOutputValue[x]=value;

    set_output(MOTOR_0_CHANNEL,value);
    set_output(MOTOR_1_CHANNEL,value);
    set_output(MOTOR_2_CHANNEL,value);
#if (NUM_MOTORS>3)
    set_output(MOTOR_3_CHANNEL,value);
#endif
#if (NUM_MOTORS>4)
    set_output(MOTOR_4_CHANNEL,value);
    set_output(MOTOR_5_CHANNEL,value);
#endif
#if (NUM_MOTORS>6)
    set_output(MOTOR_6_CHANNEL,value);
    set_output(MOTOR_7_CHANNEL,value);
#endif
}

void set_all_servo_outputs(int value) {
#if (NUM_SERVOS>0)
    for (int x=0;x<NUM_SERVOS;++x)
        global.servoOutputValue[x]=value;
    
    set_output(SERVO_0_CHANNEL,value);
#if (NUM_SERVOS>1)
    set_output(SERVO_1_CHANNEL,value);
#endif
#endif
}

void set_output(unsigned char outputchannel, unsigned int value) {
    // value is from 1000 to 2000
    unsigned char timernum=outputchannel & 0xF0;
    unsigned char timerchannel=outputchannel & 0x0F;
   
    unsigned int pwmvalue;

#if defined (DC_MOTORS)
    pwmvalue=value-1000;
#elif !defined(EXT_MOTOR_RANGE)
    pwmvalue=value<<3;
#else
    pwmvalue=((value<<4) - 16000) + 128;
#endif

#ifdef USEPWM1
    if (timernum==OUTPUT_TIMER1) {
        if (timerchannel==OUTPUT_CHANNELA) lib_pwm_setduty1A(pwmvalue);
        else if (timerchannel==OUTPUT_CHANNELB) lib_pwm_setduty1B(pwmvalue);
        return;
    }
#endif
#ifdef USEPWM2
    if (timernum==OUTPUT_TIMER2) {
        // timer 2 only has an 8 bit range
#ifndef EXT_MOTOR_RANGE 
        pwmvalue=value>>3;
#else
        pwmvalue=((value>>2) - 250) + 2;
#endif
        if (timerchannel==OUTPUT_CHANNELA) lib_pwm_setduty2A(pwmvalue);
        else if (timerchannel==OUTPUT_CHANNELB) lib_pwm_setduty2B(pwmvalue);
        return;
    }
#endif
#ifdef USEPWM3
    if (timernum==OUTPUT_TIMER3) {
        if (timerchannel==OUTPUT_CHANNELA) lib_pwm_setduty3A(pwmvalue);
        else if (timerchannel==OUTPUT_CHANNELB) lib_pwm_setduty3B(pwmvalue);
        else if (timerchannel==OUTPUT_CHANNELC) lib_pwm_setduty3C(pwmvalue);
        return;
    }
#endif
#ifdef USEPWM4
    if (timernum==OUTPUT_TIMER4) {
        if (timerchannel==OUTPUT_CHANNELA) lib_pwm_setduty4A(pwmvalue);
        else if (timerchannel==OUTPUT_CHANNELB) lib_pwm_setduty4B(pwmvalue);
        else if (timerchannel==OUTPUT_CHANNELC) lib_pwm_setduty4C(pwmvalue);
        return;
    }
#endif
#ifdef USEPWM411BIT
    if (timernum==OUTPUT_TIMER4) {
      #ifdef DC_MOTORS
        // the value is already set
      #elif !defined(EXT_MOTOR_RANGE)
        pwmvalue=value;
      #else
        pwmvalue=((value-1000)<<1)+16;
      #endif

        if (timerchannel==OUTPUT_CHANNELA) lib_pwm_setduty4A(pwmvalue);
        else if (timerchannel==OUTPUT_CHANNELB) lib_pwm_setduty4B(pwmvalue);
        else if (timerchannel==OUTPUT_CHANNELD) lib_pwm_setduty4D(pwmvalue);
        return;
    }
#endif

#ifdef USEPWM5
    if (timernum==OUTPUT_TIMER5) {
        if (timerchannel==OUTPUT_CHANNELA) lib_pwm_setduty5A(pwmvalue);
        else if (timerchannel==OUTPUT_CHANNELB) lib_pwm_setduty5B(pwmvalue);
        else if (timerchannel==OUTPUT_CHANNELC) lib_pwm_setduty5C(pwmvalue);
        return;
    }
#endif
}

void write_servo_outputs() {
    // Store the computed numbers and write out to the channel
#if (NUM_SERVOS>0)
    for (int x=0;x<NUM_SERVOS;++x)
        global.servoOutputValue[x]=global.servo[x];
    
    set_output(SERVO_0_CHANNEL,global.servo[0]);
#if (NUM_SERVOS>1)
    set_output(SERVO_1_CHANNEL,global.servo[1]);
#endif
#endif
}

void write_motor_outputs() {
    // Store the computed numbers and write out to the channel
    for (int x=0;x<NUM_MOTORS;++x)
        global.motorOutputValue[x]=global.motor[x];
    
    set_output(MOTOR_0_CHANNEL,global.motor[0]);
    set_output(MOTOR_1_CHANNEL,global.motor[1]);
    set_output(MOTOR_2_CHANNEL,global.motor[2]);
#if (NUM_MOTORS>3)
    set_output(MOTOR_3_CHANNEL,global.motor[3]);
#endif
#if (NUM_MOTORS>4)
    set_output(MOTOR_4_CHANNEL,global.motor[4]);
    set_output(MOTOR_5_CHANNEL,global.motor[5]);
#endif
#if (NUM_MOTORS>6)
    set_output(MOTOR_6_CHANNEL,global.motor[6]);
    set_output(MOTOR_7_CHANNEL,global.motor[7]);
#endif
}
