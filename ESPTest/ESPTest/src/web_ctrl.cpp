#include "web_ctrl.hpp"
#include "web_assets.hpp"
#include "cart_state.hpp"
#include "configure.hpp"
#include "wifi.hpp"

void handle_root(WebServer& server, HardwareSerial& serial) {
	serial.println("Index page request");
	server.send_P(200, "text/html", html);
}

void handle_set(WebServer& server, HardwareSerial& serial, CartState& state) {
	if (server.hasArg("value")) {
		int16_t speed = (int16_t) server.arg("value").toInt();
		state.speed_set = constrain(speed, -SPEED_MAX_TEST, SPEED_MAX_TEST);
	}
	server.send(204, "text/plain", "");
}

void handle_set_mode(WebServer& server, HardwareSerial& serial, CartState& state) {
	if (server.hasArg("drive")) {
//		InputMode input = (InputMode) server.arg("control").toInt();
		DriveMode drive = (DriveMode) server.arg("drive").toInt();
//		state.input_mode = input;
		state.drive_mode = drive;
	}
	server.send(204, "text/plain", "");
}

void handle_state_request(WebServer& server, HardwareSerial& serial, CartState& state) {
    String json = cart_state_to_json(state);
	state.last_tol = millis();
    server.send(200, "application/json", json);
}

void handle_save_ap(WebServer& server, HardwareSerial& serial) {
	if (server.hasArg("ssid") && server.hasArg("password")) {
		serial.println("Saving access point " + server.arg("ssid") + " " + server.arg("password"));
		save_ap_and_reboot(server.arg("ssid"), server.arg("password"));
	}
	else {
		serial.println("Invalid http request to save_ap. 'ssid' and/or 'password' missing");
		server.send(400, "text/plain", "Bad Request, expected 'ssid' and 'password'");
	}
}

void handle_get_networks(WebServer& server, HardwareSerial& serial) {
	serial.println("Get visible networks");
	std::vector<WiFiNetwork> networks = get_visible_wifis();
	serial.println("Got them");
	String json = "[";

	for (size_t i = 0; i < networks.size(); ++i) {
	  json += "{\"ssid\":\"" + networks[i].ssid + "\",\"rssi\":" + String(networks[i].rssi) + "}";
	  if (i < networks.size() - 1) json += ",";
	}
	json += "]";
	server.send(200, "application/json", json);
}

void register_web_handles(WebServer& server, HardwareSerial& serial, CartState& cart_state) {
	server.on("/", HTTP_GET, [ & ]() {
	    handle_root(server, serial);
	});
	server.on("/set", HTTP_GET, [ & ]() {
	    handle_set(server, serial, cart_state);
	});
	server.on("/set_mode", HTTP_GET, [ & ]() {
	    handle_set_mode(server, serial, cart_state);
	});
	server.on("/save_ap", HTTP_POST, [ & ]() {
	    handle_save_ap(server, serial);
	});
	server.on("/get_networks", HTTP_GET, [ & ]() {
	    handle_get_networks(server, serial);
	});
	server.on("/get_state", HTTP_GET, [ & ]() {
	    handle_state_request(server, serial, cart_state);
	});
	server.onNotFound([&]() {
	  serial.println("Not Found: " + server.uri());
	  server.send(404, "text/plain", "Not found");
	});
}