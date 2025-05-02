#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>

#include "wifi.hpp"

Preferences preferences;
WebServer server(PORT);

bool connect_to_saved_wifi(HardwareSerial& serial){
  preferences.begin("wifi", true);
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("password", "");
  preferences.end();

  if (ssid == "") return false;

  serial.print("Attempting to connect to " + ssid);
  WiFi.begin(ssid.c_str(), password.c_str());
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    serial.print(".");
    delay(500);
  }
  wl_status_t statusCode = WiFi.status();
  if (statusCode == WL_CONNECTED) {
      serial.printf("Connected: ");
      serial.println(WiFi.localIP());
      return true;
  }
  else{
      serial.printf("Timeout! Error Code: %d\n", statusCode);
      return false;
  }
}

void start_ap_mode(HardwareSerial& serial) {
    Serial.println("Starting AP");
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    IPAddress IP = WiFi.softAPIP();
    Serial.println("IP: " + IP.toString());
}

std::vector<WiFiNetwork> get_visible_wifis() {
    std::vector<WiFiNetwork> scannedNetworks;
    int n = WiFi.scanNetworks();

    for (int i = 0; i < n; ++i) {
      WiFiNetwork net;
      net.ssid = WiFi.SSID(i);
      net.rssi = WiFi.RSSI(i);
      scannedNetworks.push_back(net);
    }
    return scannedNetworks;
}

void save_ap_and_reboot(String ssid, String password) {
    preferences.begin("wifi", false);
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.end();
    server.send(200, "text/html", "Saved AP condif. Rebooting now.");
    delay(1000);
    ESP.restart();
}