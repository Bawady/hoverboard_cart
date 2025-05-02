#include <Arduino.h>
#include <WiFi.h>     
#include <WebServer.h>
#include <Preferences.h>

#include "wifi.hpp"
#include "web_ctrl.hpp"
#include "pin_defs.hpp"
#include "configure.hpp"
#include "cart_state.hpp"


// Global variables
//uint8_t idx = 0;
//uint16_t bufStartFrame;                 
//byte *p;                                
//byte incomingByte;
//byte incomingBytePrev;
uint16_t th_min = 4095, th_max=0, th;

unsigned long iTimeSend = 0;

CartState cart_state = {.speed_set = 0, .speed_actual = 0, .input_mode = THROTTLE, .drive_mode = ALL_WHEEL};

typedef struct{
	 uint16_t start;
	 int16_t  steer;
	 int16_t  speed;
	 uint16_t checksum;
} SerialCommand;
SerialCommand Command;

extern WebServer server;

void Send(int16_t uSteer, int16_t uSpeed, HardwareSerial& serial) {
	Command.start    = (uint16_t) START_FRAME;
	Command.steer    = (int16_t)  uSteer;
	Command.speed    = (int16_t)  uSpeed;
	Command.checksum = (uint16_t) (Command.start ^ Command.steer ^ Command.speed);

	serial.write((uint8_t *) &Command, sizeof(Command)); 
}

void setup() {
//	pinMode(ONBOARD_LED, OUTPUT);
	pinMode(THROTTLE_GRIP, INPUT);

	Serial.begin(SERIAL_BAUD);
	// Front axis
	Serial1.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD1, TXD1);
	// Rear axis
	Serial2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD2, TXD2);

	delay(1000);

	Send(0, 0, Serial1);
	Send(0, 0, Serial2);

	if (!connect_to_saved_wifi(Serial)) {
		start_ap_mode(Serial);
	}

	register_web_handles(server, Serial, cart_state);
	server.begin();

	Serial.printf("Hover control online\n");
	cart_state.last_tol = millis();
}

void Receive(HardwareSerial& hover_serial, HardwareSerial& debug_serial, AxisState& axis_state) {
 // Check for new data availability in the Serial buffer
		if (hover_serial.available()) {
				axis_state.incomingByte 	= hover_serial.read();                                   // Read the incoming byte
				axis_state.bufStartFrame	= ((uint16_t)(axis_state.incomingByte) << 8) | axis_state.incomingBytePrev;       // Construct the start frame
		}
		else {
				return;
		}

		// Copy received data
		if (axis_state.bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
				axis_state.p       = (byte *)&axis_state.NewFeedback;
				*(axis_state.p)++    = axis_state.incomingBytePrev;
				*(axis_state.p)++    = axis_state.incomingByte;
				axis_state.idx     = 2;	
		} else if (axis_state.idx >= 2 && axis_state.idx < sizeof(SerialFeedback)) {  // Save the new received data
				*(axis_state.p)++    = axis_state.incomingByte; 
				axis_state.idx++;
		}	
		
		// Check if we reached the end of the package
		if (axis_state.idx == sizeof(SerialFeedback)) {
				uint16_t checksum;
				checksum = (uint16_t)(axis_state.NewFeedback.start ^ axis_state.NewFeedback.cmd1 ^ axis_state.NewFeedback.cmd2 ^ axis_state.NewFeedback.speedR_meas ^ axis_state.NewFeedback.speedL_meas
														^ axis_state.NewFeedback.batVoltage ^ axis_state.NewFeedback.boardTemp ^ axis_state.NewFeedback.cmdLed);

				// Check validity of the new data
				if (axis_state.NewFeedback.start == START_FRAME && checksum == axis_state.NewFeedback.checksum) {
						// Copy the new data
						memcpy(&axis_state.Feedback, &axis_state.NewFeedback, sizeof(SerialFeedback));

						axis_state.rpm_left = (int16_t) axis_state.Feedback.speedL_meas;
						axis_state.rpm_right = (int16_t) axis_state.Feedback.speedR_meas;
						// Print data to built-in Serial
//						debug_serial.print(" 1: ");  Serial.print(Feedback.cmd1);
//						debug_serial.print(" 2: ");  Serial.print(Feedback.cmd2);
//						debug_serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
//						debug_serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
//						debug_serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
//						debug_serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
//						debug_serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
				}
//				else {
//					debug_serial.println("Non-valid data skipped");
//				}
				axis_state.idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
		}

		// Update previous states
		axis_state.incomingBytePrev = axis_state.incomingByte;
}

void loop(void) { 
	unsigned long time_now = millis();
	server.handleClient();

	Receive(Serial1, Serial, cart_state.front_axis);
	Receive(Serial2, Serial, cart_state.rear_axis);

//	if (time_now % 2000 == 0) {
//		Serial.printf("%d\n", cart_state.speed);
//	}

//	digitalWrite(ONBOARD_LED, (time_now % 2000) < 500);
	if (time_now % 200 == 0) {
		int16_t speed;
		int16_t speed_front, speed_rear;
		th = analogRead(THROTTLE_GRIP);

//		if (cart_state.input_mode == THROTTLE) {
			speed = th >= 920 ? map(th, 920, 3150, 0, 4095) : 0;
//		}
//		else {
//			speed = cart_state.speed_set;
//			Serial.println(speed);
//		}

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
		Send(0, speed_front, Serial1);
		Send(0, speed_rear, Serial2);
	}
}
