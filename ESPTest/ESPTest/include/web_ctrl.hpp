#ifndef WEB_CTRL_HPP
#define WEB_CTRL_HPP

#include <WebServer.h>
#include <HardwareSerial.h>
#include <Arduino.h>

#include "cart_state.hpp"

void register_web_handles(WebServer& server, HardwareSerial& serial, CartState& cart_state);

#endif // WEB_CTRL_HPP