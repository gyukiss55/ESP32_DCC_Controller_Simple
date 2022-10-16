
/*
* DCCWebServer.h
*/

#pragma once

#include <Arduino.h>
#include <string>

void setupDCCWebServer();
void loopDCCWebServer(std::string& command);
