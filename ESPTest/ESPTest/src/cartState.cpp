#include "cart_state.hpp"

String cart_state_to_json(CartState& state) {
    return "{\"speed_set\": " + String(state.speed_set) + ", \"speed_actual\": " + String(state.speed_actual) + ", \"axes\": [" + axis_state_to_json(state.front_axis) + "," + axis_state_to_json(state.rear_axis) + "]}";
}

String axis_state_to_json(AxisState& state) {
    return "{\"left_rpm\": " + String(state.rpm_left) + ", \"right_rpm\": " + String(state.rpm_right) + "}";
}

AxisState get_axis_state(CartState& state, Axis axis) {
    if (axis == FRONT) {
        return state.front_axis;
    }
    else {
        return state.rear_axis;
    }

}