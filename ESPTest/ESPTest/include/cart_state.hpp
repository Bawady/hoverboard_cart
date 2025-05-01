#ifndef CART_STATE_HPP
#define CART_STATE_HPP

#include <stdint.h>
#include <Arduino.h>

typedef struct {
	int16_t speed_set;
	int16_t speed_actual;
} CartState;

String cart_state_to_json(CartState& state);

#endif // CART_STATE_HPP
