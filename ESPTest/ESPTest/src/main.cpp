#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing
#define SPEED_STEP          20          // [-] Speed step
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

unsigned long iTimeSend = 0;
int iTest = 0;
int iStep = SPEED_STEP;

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

const char *ssid = "ESP8266_Hotspot";
const char *password = "12345678";

ESP8266WebServer server(80);


void handleRoot() {
  String html = "<html><head><title>ESP8266 Control</title></head><body>";
  html += "<h2>ESP8266 Web Slider Control</h2>";
  html += "<input type='range' min='-300' max='300' value='" + String(iTest) + "' id='slider' oninput='updateValue(this.value)'><br>";
  html += "<p>Value: <span id='value'>" + String(iTest) + "</span></p>";
  html += "<script>function updateValue(val){document.getElementById('value').innerText=val;fetch('/set?value='+val)}</script>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleSet() {
  if (server.hasArg("value")) {
      iTest = server.arg("value").toInt();
  }
  iTest = iTest >= SPEED_MAX_TEST ? SPEED_MAX_TEST : iTest;
  iTest = iTest <= -SPEED_MAX_TEST ? -SPEED_MAX_TEST : iTest;
  server.send(204, "text/plain", "");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  WiFi.softAP(ssid, password);
    
  server.on("/", HTTP_GET, handleRoot);
  server.on("/set", HTTP_GET, handleSet);

    
  server.begin();

  pinMode(LED_BUILTIN, OUTPUT);
}

void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  Serial.write((uint8_t *) &Command, sizeof(Command)); 
}

void Receive()
{
    // Check for new data availability in the Serial buffer
    if (Serial.available()) {
        incomingByte 	  = Serial.read();                                   // Read the incoming byte
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
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

void loop(void)
{ 
  unsigned long timeNow = millis();
  server.handleClient();

  // Check for new received data
  Receive();

  // Send commands
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  Send(0, iTest);

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
}