#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>

#include "web_ctrl.hpp"
#include "web_assets.hpp"
#include "cart_state.hpp"
#include "configure.hpp"
#include "wifi.hpp"


void handle_root(AsyncWebServerRequest *request, HardwareSerial& serial) {
	serial.println("Index page request");
    request->send(SPIFFS, "/index.html", "text/html");
}

// Handle the set request (change drive mode) asynchronously
void handle_set(AsyncWebServerRequest *request, HardwareSerial& serial, CartState& state) {
	if (request->hasArg("drive")) {
		DriveMode drive = (DriveMode) request->arg("drive").toInt();
		// Only change the drive mode when the cart is standing still / at low speed
		if (drive != state.drive_mode && state.speed_set < EBRAKE_SPEED_THRES && state.speed_set < EBRAKE_SPEED_THRES) {
			state.drive_mode = drive;
		}
	}
	if (request->hasArg("baby")) {
		bool baby_mode = (bool) request->arg("baby").toInt();
		// Only change the drive mode when the cart is standing still / at low speed
		if (baby_mode != state.baby_mode && state.speed_set < BABY_MODE_SPEED) {
			state.baby_mode = baby_mode;
		}
	}
	request->send(204, "text/plain", "");
}

// Handle the state request asynchronously
void handle_state_request(AsyncWebServerRequest *request, HardwareSerial& serial, CartState& state) {
	String json = cart_state_to_json(state);
	state.last_tol = millis();
	request->send(200, "application/json", json);
}

// Handle save access point information asynchronously
void handle_save_ap(AsyncWebServerRequest *request, HardwareSerial& serial) {
	if (request->hasArg("ssid") && request->hasArg("password")) {
		serial.println("Saving access point " + request->arg("ssid") + " " + request->arg("password"));
		save_ap_and_reboot(request->arg("ssid"), request->arg("password"));
		request->send(200, "text/html", "Saved AP condif. Rebooting now.");
    	delay(2000);
    	ESP.restart();
	}
	else {
		serial.println("Invalid HTTP request to save_ap. 'ssid' and/or 'password' missing");
		request->send(400, "text/plain", "Bad Request, expected 'ssid' and 'password'");
	}
}

// Handle the get networks request asynchronously
void handle_get_networks(AsyncWebServerRequest *request, HardwareSerial& serial) {
	serial.println("get_networks request");
	std::vector<WiFiNetwork> networks = get_visible_wifis();
	String json = "[";

	for (size_t i = 0; i < networks.size(); ++i) {
	  json += "{\"ssid\":\"" + networks[i].ssid + "\",\"rssi\":" + String(networks[i].rssi) + "}";
	  if (i < networks.size() - 1) json += ",";
	}
	json += "]";
	request->send(200, "application/json", json);
}

// Register the asynchronous web handlers
void register_web_handles(AsyncWebServer& server, HardwareSerial& serial, CartState& cart_state) {
	// Root page handler (serving the index.html file)
	server.on("/", HTTP_GET, [ & ](AsyncWebServerRequest *request){
		handle_root(request, serial);
	});
	server.on("/index.html", HTTP_GET, [ & ](AsyncWebServerRequest *request){
		handle_root(request, serial);
	});
	
	// JavaScript file handlers
	server.on("/raphael.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
	    request->send(SPIFFS, "/raphael.min.js", "application/javascript");
	});

	server.on("/justgage.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
	    request->send(SPIFFS, "/justgage.min.js", "application/javascript");
	});
	
	// Set drive mode handler
	server.on("/set", HTTP_GET, [ & ](AsyncWebServerRequest *request) {
	    handle_set(request, serial, cart_state);
	});
	
	// Save AP handler
	server.on("/save_ap", HTTP_POST, [ & ](AsyncWebServerRequest *request) {
	    handle_save_ap(request, serial);
	});
	
	// Get available networks handler
	server.on("/get_networks", HTTP_GET, [ & ](AsyncWebServerRequest *request) {
	    handle_get_networks(request, serial);
	});
	
	// Get cart state handler
	server.on("/get_state", HTTP_GET, [ & ](AsyncWebServerRequest *request) {
	    handle_state_request(request, serial, cart_state);
	});
	
	// 404 handler for unknown requests
	server.onNotFound([&](AsyncWebServerRequest *request) {
	    serial.println("Not Found: " + request->url());
	    request->send(404, "text/plain", "Not found");
	});
}
