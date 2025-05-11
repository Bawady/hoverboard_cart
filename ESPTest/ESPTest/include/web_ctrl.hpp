#ifndef WEB_CTRL_HPP
#define WEB_CTRL_HPP

#include <ESPAsyncWebServer.h>
#include <HardwareSerial.h>
#include <Arduino.h>

#include "cart_state.hpp"

void register_web_handles(AsyncWebServer& server, HardwareSerial& serial, CartState& cart_state);

#endif // WEB_CTRL_HPP