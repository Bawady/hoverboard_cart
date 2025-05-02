#ifndef WIFI_HPP
#define WIFI_HPP

#define AP_SSID "ESP32_Hotspot"
#define AP_PASSWORD "12345678"
#define PORT 80

#include <Preferences.h>
#include <WiFi.h>


struct WiFiNetwork {
	String ssid;
	int32_t rssi;
};

bool connect_to_saved_wifi(HardwareSerial& serial);
void start_ap_mode(HardwareSerial& serial);
void save_ap_and_reboot(String ssid, String password);
std::vector<WiFiNetwork> get_visible_wifis();


#endif // WIFI_HPP