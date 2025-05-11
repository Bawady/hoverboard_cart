#include <Arduino.h>
#include <WiFi.h>     
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Preferences.h>

#include "wifi.hpp"
#include "web_ctrl.hpp"
#include "pin_defs.hpp"
#include "configure.hpp"
#include "cart_state.hpp"


extern AsyncWebServer server;

CartState cart_state = {.speed_set = 0, .speed_actual = 0, .input_mode = THROTTLE, .drive_mode = ALL_WHEEL};


void setup() {
	pinMode(THROTTLE_GRIP, INPUT);

	Serial.begin(SERIAL_BAUD);
	// Front axis
	Serial1.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD1, TXD1);
	Serial2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD2, TXD2);
	// Rear axis
//	Serial1.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD2, TXD2);
//	Serial2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD1, TXD1);

	delay(1000);

	// Send sign of life to board such that it keeps quiet at the beginning while ESP tries to connect
	// Nicer would be to have the respective loop here and send the SOL during the loop, or pass a handle to the connect function
	send_hover_command(Serial1, 0, 0);
	send_hover_command(Serial2, 0, 0);

	if (!connect_to_saved_wifi(Serial)) {
		start_ap_mode(Serial);
	}

	if(!SPIFFS.begin()){
		Serial.println("SPIFFS Mounting failed");
		return;
	}

	register_web_handles(server, Serial, cart_state);
	server.begin();

	Serial.printf("Hover control online\n");
	cart_state.last_tol = millis();
}

void loop(void) { 
	unsigned long time_now = millis();
	uint16_t th1=0, th2=0, th3=0;

	read_axis_serial(Serial1, Serial, cart_state.front_axis);
	read_axis_serial(Serial2, Serial, cart_state.rear_axis);

	th1 = analogRead(THROTTLE_GRIP);

	if (time_now % SPEED_UPDATE_MS == 0) {
		int16_t speed;
		int16_t speed_front, speed_rear;
		uint16_t top_speed = cart_state.baby_mode ? BABY_MODE_SPEED : 4095;
		uint16_t th = (th1 + th2 + th3) / 3;

		speed = th >= 920 ? map(th, 920, 3150, 0, top_speed) : 0;

		// The e-brake applies automatically beyond a certain threshold -> set above threshold for single axis mode
		// However: Doesn't the control board actively try to control to the set speed? I.e., will the inactive axis not simply work against the active one?
		speed_front = speed;
		speed_rear = speed;
		if (cart_state.drive_mode == FRONT_WHEEL) {
			speed_front = speed;
			speed_rear = speed >= EBRAKE_SPEED_THRES ? EBRAKE_SPEED_THRES : 0;
		}
		else if (cart_state.drive_mode == REAR_WHEEL) {
			speed_rear = speed;
			speed_front = speed >= EBRAKE_SPEED_THRES ? EBRAKE_SPEED_THRES : 0;
		}
		cart_state.speed_set = speed;
		send_hover_command(Serial1, speed_front, 0);
		send_hover_command(Serial2, speed_rear, 0);
	}
	th3 = th2;
	th2 = th1;
}
