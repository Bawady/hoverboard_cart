#ifndef CART_STATE_HPP
#define CART_STATE_HPP

#include <stdint.h>
#include <Arduino.h>

#include "cart_state.hpp"

const uint16_t EBRAKE_SPEED_THRES = 10;

typedef struct{
	 uint16_t start;
	 int16_t  cmd1;
	 int16_t  cmd2;
	 int16_t  speedR_meas;
	 int16_t  speedL_meas;
	 int16_t  batVoltage;
	 int16_t  boardTemp;
	 uint16_t cmdLed;
	 uint16_t checksum;
} SerialFeedback;

typedef enum{
	WEB=0,
	THROTTLE=1
} InputMode;

typedef enum {
	FRONT_WHEEL=0,
	REAR_WHEEL=1,
	ALL_WHEEL=2
} DriveMode;

typedef enum {
	FRONT=0,
	REAR=1
} Axis;

typedef struct {
	int16_t speed;
	int16_t rpm_left;
	int16_t rpm_right;
	uint8_t idx;
	uint16_t bufStartFrame;
	byte *p;
	byte incomingByte;
	byte incomingBytePrev;
	SerialFeedback Feedback;
	SerialFeedback NewFeedback;
} AxisState;

typedef struct {
	int16_t speed_set;
	int16_t speed_actual;
	AxisState front_axis;
	AxisState rear_axis;
	InputMode input_mode;
	DriveMode drive_mode;
	unsigned long last_tol;
} CartState;

String cart_state_to_json(CartState& state);
String axis_state_to_json(AxisState& state);
AxisState get_axis_state(CartState& state, Axis axis);

#endif // CART_STATE_HPP
