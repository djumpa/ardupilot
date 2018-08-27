/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
*       AP_MotorsTri.cpp - ArduCopter motors library
*       Code by RandyMackay. DIYDrones.com
*
*/
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_MotorsBi.h"

extern const AP_HAL::HAL& hal;

// init
void AP_MotorsBi::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
	add_motor_num(AP_MOTORS_MOT_1);
	add_motor_num(AP_MOTORS_MOT_2);


	// set update rate for the 3 motors (but not the servo on channel 7)
	set_update_rate(_speed_hz);

	// set the motor_enabled flag so that the ESCs can be calibrated like other frame types
	motor_enabled[AP_MOTORS_MOT_1] = true;
	motor_enabled[AP_MOTORS_MOT_2] = true;


	// find the yaw servo
	_yaw_servo = SRV_Channels::get_channel_for(SRV_Channel::k_motor3, AP_MOTORS_CH_SERVO_BI_LEFT);
	_right_servo = SRV_Channels::get_channel_for(SRV_Channel::k_motor4, AP_MOTORS_CH_SERVO_BI_RIGHT);

	if (!_yaw_servo) {
		gcs().send_text(MAV_SEVERITY_ERROR, "MotorsBi: unable to setup yaw channel");
		// don't set initialised_ok
		//return;
	}

	// allow mapping of motor7
	add_motor_num(AP_MOTORS_CH_SERVO_BI_LEFT);
	add_motor_num(AP_MOTORS_CH_SERVO_BI_RIGHT);

	// record successful initialisation if what we setup was the desired frame_class
	_flags.initialised_ok = (frame_class == MOTOR_FRAME_BI);
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsBi::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
	_flags.initialised_ok = (frame_class == MOTOR_FRAME_BI);
}

// set update rate to motors - a value in hertz
void AP_MotorsBi::set_update_rate(uint16_t speed_hz)
{
	// record requested speed
	_speed_hz = speed_hz;

	// set update rate for the 3 motors (but not the servo on channel 7)
	uint32_t mask =
		1U << AP_MOTORS_MOT_1 |
		1U << AP_MOTORS_MOT_2;
	rc_set_freq(mask, _speed_hz);
}

void AP_MotorsBi::output_to_motors()
{
	switch (_spool_mode) {
	case SHUT_DOWN:
		// sends minimum values out to the motors
		hal.rcout->cork();
		rc_write(AP_MOTORS_MOT_1, get_pwm_output_min());
		rc_write(AP_MOTORS_MOT_2, get_pwm_output_min());
		rc_write(AP_MOTORS_CH_SERVO_BI_LEFT, _yaw_servo->get_trim());
		rc_write(AP_MOTORS_CH_SERVO_BI_RIGHT, _yaw_servo->get_trim());
		hal.rcout->push();
		break;
	case SPIN_WHEN_ARMED:
		// sends output to motors when armed but not flying
		hal.rcout->cork();
		rc_write(AP_MOTORS_MOT_1, calc_spin_up_to_pwm());
		rc_write(AP_MOTORS_MOT_2, calc_spin_up_to_pwm());
		rc_write(AP_MOTORS_CH_SERVO_BI_LEFT, _yaw_servo->get_trim());
		rc_write(AP_MOTORS_CH_SERVO_BI_RIGHT, _yaw_servo->get_trim());
		hal.rcout->push();
		break;
	case SPOOL_UP:
	case THROTTLE_UNLIMITED:
	case SPOOL_DOWN:
		// set motor output based on thrust requests
		hal.rcout->cork();
		rc_write(AP_MOTORS_MOT_1, calc_thrust_to_pwm(_thrust_left));
		rc_write(AP_MOTORS_MOT_2, calc_thrust_to_pwm(_thrust_right));
		rc_write(AP_MOTORS_CH_SERVO_BI_LEFT, _angle_left);
		rc_write(AP_MOTORS_CH_SERVO_BI_RIGHT, _angle_right);
		hal.rcout->push();
		break;
	}
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsBi::get_motor_mask()
{
	// bi copter uses channels 1,2,3 and 4
	uint16_t motor_mask = (1U << AP_MOTORS_MOT_1) |
		                  (1U << AP_MOTORS_MOT_2) |
		                  (1U << AP_MOTORS_CH_SERVO_BI_LEFT) |
		                  (1U << AP_MOTORS_CH_SERVO_BI_RIGHT);

    uint16_t mask = rc_map_mask(motor_mask);

    // add parent's mask
    mask |= AP_MotorsMulticopter::get_motor_mask();

    return mask;
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsBi::output_armed_stabilizing()
{
	float   roll_thrust;                // roll thrust input value, +/- 1.0
	float   pitch_thrust;               // pitch thrust input value, +/- 1.0
	float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
	float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
	float _left_reverse;
	float _right_reverse;
	float _scale_servo;
	//float   throttle_thrust_best_rpy;   // throttle providing maximum roll, pitch and yaw range without climbing
	//float   rpy_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
	//float   rpy_low = 0.0f;             // lowest motor value
	//float   rpy_high = 0.0f;            // highest motor value
	//float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

										// sanity check YAW_SV_ANGLE parameter value to avoid divide by zero
	_yaw_servo_angle_max_deg = constrain_float(_yaw_servo_angle_max_deg, AP_MOTORS_TRI_SERVO_RANGE_DEG_MIN, AP_MOTORS_TRI_SERVO_RANGE_DEG_MAX);

	// apply voltage and air pressure compensation
	roll_thrust = _roll_in * get_compensation_gain();
	pitch_thrust = _pitch_in * get_compensation_gain();
	yaw_thrust = _yaw_in * get_compensation_gain();
	throttle_thrust = get_throttle() * get_compensation_gain();






	// calculate angle of yaw pivot
	//_pivot_angle = safe_asin(yaw_thrust);
	//if (fabsf(_pivot_angle) > radians(_yaw_servo_angle_max_deg)) {
	//	limit.yaw = true;
	//	_pivot_angle = constrain_float(_pivot_angle, -radians(_yaw_servo_angle_max_deg), radians(_yaw_servo_angle_max_deg));
	//}

	//float pivot_thrust_max = cosf(_pivot_angle);
	//float thrust_max = 1.0f;

	// sanity check throttle is above zero and below current limited throttle
	if (throttle_thrust <= 0.0f) {
		throttle_thrust = 0.0f;
		limit.throttle_lower = true;
	}
	if (throttle_thrust >= _throttle_thrust_max) {
		throttle_thrust = _throttle_thrust_max;
		limit.throttle_upper = true;
	}

	_throttle_avg_max = constrain_float(_throttle_avg_max, throttle_thrust, _throttle_thrust_max);

	// The following mix may be offer less coupling between axis but needs testing
	//_thrust_right = roll_thrust * -0.5f + pitch_thrust * 1.0f;
	//_thrust_left = roll_thrust * 0.5f + pitch_thrust * 1.0f;
	//_thrust_rear = 0;
	if (_yaw_servo->get_reversed()) {
			_left_reverse = -1.0f;
	}
	else{
		  _left_reverse = 1.0f;
	}


	if (_right_servo->get_reversed()) {
			_right_reverse = -1.0f;
	}
	else{
		  _right_reverse = 1.0f;
	}

	_scale_servo=abs(yaw_thrust)+abs(pitch_thrust);
  if(_scale_servo<=1){
	_scale_servo=1;
  }


	_thrust_right = -roll_thrust  + throttle_thrust;
	_thrust_left = roll_thrust  + throttle_thrust;


	_angle_left = (-pitch_thrust - yaw_thrust)*(_yaw_servo->get_output_max()-_yaw_servo->get_trim())*_left_reverse/_scale_servo + _yaw_servo->get_trim();
	_angle_right =(-pitch_thrust + yaw_thrust)*(_right_servo->get_output_max()-_right_servo->get_trim())*_right_reverse/_scale_servo +_right_servo->get_trim();


	// constrain all outputs to 0.0f to 1.0f
	// test code should be run with these lines commented out as they should not do anything
	_thrust_right = constrain_float(_thrust_right, 0.0f, 1.0f);
	_thrust_left = constrain_float(_thrust_left, 0.0f, 1.0f);

}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsBi::output_test_seq(uint8_t motor_seq, int16_t pwm)
{
	// exit immediately if not armed
	if (!armed()) {
		return;
	}

	// output to motors and servos
	switch (motor_seq) {
	case 1:
		// front right motor
		rc_write(AP_MOTORS_MOT_1, pwm);
		break;
	case 2:
		// back motor
		rc_write(AP_MOTORS_MOT_2, pwm);
		break;
	case 3:
		// back servo
		rc_write(AP_MOTORS_CH_SERVO_BI_LEFT, pwm);
		break;
	case 4:
		// front left motor
		rc_write(AP_MOTORS_CH_SERVO_BI_RIGHT, pwm);
		break;
	default:
		// do nothing
		break;
	}
}



/*
call vehicle supplied thrust compensation if set. This allows for
vehicle specific thrust compensation for motor arrangements such as
the forward motors tilting
*/
void AP_MotorsBi::thrust_compensation(void)
{
	if (_thrust_compensation_callback) {
		// convert 3 thrust values into an array indexed by motor number
		float thrust[4]{ _thrust_right, _thrust_left, 0, 0 };

		// apply vehicle supplied compensation function
		_thrust_compensation_callback(thrust, 4);

		// extract compensated thrust values
		_thrust_right = thrust[0];
		_thrust_left = thrust[1];

	}
}
