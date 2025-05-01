#ifndef WEB_CTRL_HPP
#define WEB_CTRL_HPP

#include <WebServer.h>
#include <HardwareSerial.h>
#include <Arduino.h>

#include "cart_state.hpp"

void handleRoot(WebServer& server, HardwareSerial& serial);

void handleSet(WebServer& server, CartState& state);

void handleStateRequest(WebServer& server, HardwareSerial& serial, CartState& state);

#endif // WEB_CTRL_HPP