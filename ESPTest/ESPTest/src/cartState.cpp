#include "cart_state.hpp"

String cart_state_to_json(CartState& state) {
    return "{\"speed_set\": " + String(state.speed_set) + ", \"speed_actual\": " + String(state.speed_actual) + "}";
}