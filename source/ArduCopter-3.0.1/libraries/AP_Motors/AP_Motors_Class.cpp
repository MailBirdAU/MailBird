// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       AP_Motors.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 */

#include "AP_Motors_Class.h"
#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;

// parameters for the motor class
const AP_Param::GroupInfo AP_Motors::var_info[] PROGMEM = {
    // 0 was used by TB_RATIO

    // @Param: TCRV_ENABLE
    // @DisplayName: Thrust Curve Enable
    // @Description: Controls whether a curve is used to linearize the thrust produced by the motors
    // @Values: 0:Disabled,1:Enable
    AP_GROUPINFO("TCRV_ENABLE", 1, AP_Motors, _throttle_curve_enabled, THROTTLE_CURVE_ENABLED),

    // @Param: TCRV_MIDPCT
    // @DisplayName: Thrust Curve mid-point percentage
    // @Description: Set the pwm position that produces half the maximum thrust of the motors
    // @Range: 20 80
    AP_GROUPINFO("TCRV_MIDPCT", 2, AP_Motors, _throttle_curve_mid, THROTTLE_CURVE_MID_THRUST),

    // @Param: TCRV_MAXPCT
    // @DisplayName: Thrust Curve max thrust percentage
    // @Description: Set to the lowest pwm position that produces the maximum thrust of the motors.  Most motors produce maximum thrust below the maximum pwm value that they accept.
    // @Range: 20 80
    AP_GROUPINFO("TCRV_MAXPCT", 3, AP_Motors, _throttle_curve_max, THROTTLE_CURVE_MAX_THRUST),

    AP_GROUPEND
};

// Constructor
AP_Motors::AP_Motors( RC_Channel* rc_roll, RC_Channel* rc_pitch, RC_Channel* rc_throttle, RC_Channel* rc_yaw, uint16_t speed_hz ) :
    _rc_roll(rc_roll),
    _rc_pitch(rc_pitch),
    _rc_throttle(rc_throttle),
    _rc_yaw(rc_yaw),
    _speed_hz(speed_hz),
    _armed(false),
    _frame_orientation(0),
    _min_throttle(AP_MOTORS_DEFAULT_MIN_THROTTLE),
    _max_throttle(AP_MOTORS_DEFAULT_MAX_THROTTLE),
    _hover_out(AP_MOTORS_DEFAULT_MID_THROTTLE)
{
    uint8_t i;

    AP_Param::setup_object_defaults(this, var_info);

    // initialise motor map
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
        set_motor_to_channel_map(APM1_MOTOR_TO_CHANNEL_MAP);
#else
        set_motor_to_channel_map(APM2_MOTOR_TO_CHANNEL_MAP);
#endif

    // clear output arrays
    for(i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        motor_out[i] = 0;
    }
};

// init
void AP_Motors::Init()
{
    // set-up throttle curve - motors classes will decide whether to use it based on _throttle_curve_enabled parameter
    setup_throttle_curve();
};

// set_min_throttle - sets the minimum throttle that will be sent to the engines when they're not off (i.e. to prevents issues with some motors spinning and some not at very low throttle)
void AP_Motors::set_min_throttle(uint16_t min_throttle)
{
    _min_throttle = (float)min_throttle * (_rc_throttle->radio_max - _rc_throttle->radio_min) / 1000.0f;
}

// set_mid_throttle - sets the mid throttle which is close to the hover throttle of the copter
// this is used to limit the amount that the stability patch will increase the throttle to give more room for roll, pitch and yaw control
void AP_Motors::set_mid_throttle(uint16_t mid_throttle)
{
    _hover_out = _rc_throttle->radio_min + (float)(_rc_throttle->radio_max - _rc_throttle->radio_min) * mid_throttle / 1000.0f;
}

// throttle_pass_through - passes pilot's throttle input directly to all motors - dangerous but used for initialising ESCs
void AP_Motors::throttle_pass_through()
{
    if (armed()) {
        // send the pilot's input directly to each enabled motor
        for (int16_t i=0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                hal.rcout->write(_motor_to_channel_map[i], _rc_throttle->radio_in);
            }
        }
    }
}

// setup_throttle_curve - used to linearlise thrust output by motors
// returns true if set up successfully
bool AP_Motors::setup_throttle_curve()
{
    int16_t min_pwm = _rc_throttle->radio_min;
    int16_t max_pwm = _rc_throttle->radio_max;
	int16_t mid_throttle_pwm = (max_pwm + min_pwm) / 2;
    int16_t mid_thrust_pwm = min_pwm + (float)(max_pwm - min_pwm) * ((float)_throttle_curve_mid/100.0f);
    int16_t max_thrust_pwm = min_pwm + (float)(max_pwm - min_pwm) * ((float)_throttle_curve_max/100.0f);
    bool retval = true;

    // some basic checks that the curve is valid
    if( mid_thrust_pwm >= (min_pwm+_min_throttle) && mid_thrust_pwm <= max_pwm && max_thrust_pwm >= mid_thrust_pwm && max_thrust_pwm <= max_pwm ) {
        // clear curve
        _throttle_curve.clear();

        // curve initialisation
        retval &= _throttle_curve.add_point(min_pwm, min_pwm);
        retval &= _throttle_curve.add_point(min_pwm+_min_throttle, min_pwm+_min_throttle);
        retval &= _throttle_curve.add_point(mid_throttle_pwm, mid_thrust_pwm);
        retval &= _throttle_curve.add_point(max_pwm, max_thrust_pwm);

        // return success
        return retval;
    }else{
        retval = false;
    }

    // disable throttle curve if not set-up corrrectly
    if( !retval ) {
        _throttle_curve_enabled = false;
        hal.console->println_P(PSTR("AP_Motors: failed to create throttle curve"));
    }

    return retval;
}
