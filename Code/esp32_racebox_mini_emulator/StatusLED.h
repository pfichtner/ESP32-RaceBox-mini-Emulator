#ifndef STATUSLED_H
#define STATUSLED_H

#include <Arduino.h>

// --- LED States ---
#define STATE_BOOT          0
#define STATE_GNSS_SEARCH   1
#define STATE_GNSS_2D       2
#define STATE_GNSS_3D       3
#define STATE_BLE_CONNECTED 4
#define STATE_ERROR         5

extern uint8_t currentState;

void ledSetup();
void updateLed();
void setLedColor(uint32_t color);

#endif
