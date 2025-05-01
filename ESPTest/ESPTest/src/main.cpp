#include <Arduino.h>
#include <WiFi.h>     
#include <WebServer.h>

#include "web_ctrl.hpp"
#include "pin_defs.hpp"
#include "configure.hpp"
#include "cart_state.hpp"


static const char *ap_ssid = "ESP32_Hotspot";
static const char *ap_password = "12345678";
static const unsigned port = 80;

// Global variables
uint8_t idx = 0;                        
uint16_t bufStartFrame;                 
byte *p;                                
byte incomingByte;
byte incomingBytePrev;
uint16_t th_min = 4095, th_max=0, th;

unsigned long iTimeSend = 0;

CartState cart_state = {.speed_set = 0, .speed_actual = 42};

typedef struct{
	 uint16_t start;
	 int16_t  steer;
	 int16_t  speed;
	 uint16_t checksum;
} SerialCommand;
SerialCommand Command;

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
SerialFeedback Feedback;
SerialFeedback NewFeedback;

WebServer server(port);

void setup() {
	Serial.begin(SERIAL_BAUD);
	Serial2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD2, TXD2);


	WiFi.softAP(ap_ssid, ap_password);

	server.on("/", HTTP_GET, [ & ]() {
	    handleRoot(server, Serial);
	});

	server.on("/set", HTTP_GET, [ & ]() {
	    handleSet(server, cart_state);
	});

	server.on("/cartstate", HTTP_GET, [ & ]() {
	    handleStateRequest(server, Serial, cart_state);
	});

	server.begin();

	pinMode(ONBOARD_LED, OUTPUT);
	pinMode(THROTTLE_GRIP, INPUT);

	Serial.printf("Hover control online: %s\n", WiFi.softAPIP().toString().c_str());

	delay(1000);
}

void Send(int16_t uSteer, int16_t uSpeed) {
	Command.start    = (uint16_t) START_FRAME;
	Command.steer    = (int16_t)  uSteer;
	Command.speed    = (int16_t)  uSpeed;
	Command.checksum = (uint16_t) (Command.start ^ Command.steer ^ Command.speed);

	Serial2.write((uint8_t *) &Command, sizeof(Command)); 
}

void Receive() {
 // Check for new data availability in the Serial buffer
		if (Serial2.available()) {
				incomingByte 	  = Serial2.read();                                   // Read the incoming byte
				bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
		}
		else {
				return;
		}

		// Copy received data
		if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
				p       = (byte *)&NewFeedback;
				*p++    = incomingBytePrev;
				*p++    = incomingByte;
				idx     = 2;	
		} else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
				*p++    = incomingByte; 
				idx++;
		}	
		
		// Check if we reached the end of the package
		if (idx == sizeof(SerialFeedback)) {
				uint16_t checksum;
				checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
														^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

				// Check validity of the new data
				if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
						// Copy the new data
						memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

						// Print data to built-in Serial
						Serial.print(" 1: ");  Serial.print(Feedback.cmd1);
						Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
						Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
						Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
						Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
						Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
						Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
				} else {
					Serial.println("Non-valid data skipped");
				}
				idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
		}

		// Update previous states
		incomingBytePrev = incomingByte;
}

void loop(void) { 
	unsigned long time_now = millis();
	server.handleClient();

	Receive();

//	if (time_now % 2000 == 0) {
//		Serial.printf("%d\n", cart_state.speed);
//	}

	digitalWrite(ONBOARD_LED, (time_now % 2000) < 500);
	if (time_now % 50 == 0) {
		th = analogRead(THROTTLE_GRIP);
		Send(0, map(th, 900, 3150, 0, 4095));
	}
}
