#include "web_ctrl.hpp"
#include "web_assets.hpp"
#include "cart_state.hpp"
#include "configure.hpp"


void handleRoot(WebServer& server, HardwareSerial& serial) {
	serial.println("Index page request");
	server.send_P(200, "text/html", html);
}

void handleSet(WebServer& server, CartState& state) {
	if (server.hasArg("value")) {
		int16_t speed = (int16_t) server.arg("value").toInt();
		state.speed_set = constrain(speed, -SPEED_MAX_TEST, SPEED_MAX_TEST);
	}
	server.send(204, "text/plain", "");
}

void handleStateRequest(WebServer& server, HardwareSerial& serial, CartState& state) {
    String json = cart_state_to_json(state);
    server.send(200, "application/json", json);
}