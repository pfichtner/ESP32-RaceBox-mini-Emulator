#include <Adafruit_NeoPixel.h>
#include "StatusLED.h"

// --- NeoPixel Config ---
#define LED_PIN    8
#define LED_COUNT  1
#define LED_BRIGHTNESS 25
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- Color Definitions ---
#define OFF     led.Color(0, 0, 0)
#define RED     led.Color(255, 0, 0)
#define GREEN   led.Color(0, 255, 0)
#define ORANGE  led.Color(255, 200, 0)
#define BLUE    led.Color(0, 0, 255)

// --- LED Timing Variables ---
static unsigned long lastUpdateTime = 0;
static bool ledOn = false;
uint8_t currentState = STATE_BOOT;

// --- Initialization ---
void ledSetup() {
    led.begin();
    led.setBrightness(LED_BRIGHTNESS);
    led.show();
}

// --- Helper ---
void setLedColor(uint32_t color) {
    led.setPixelColor(0, color);
    led.show();
}

// --- LED Update ---
void updateLed() {
    unsigned long now = millis();
    switch (currentState) {

        case STATE_BOOT:
            setLedColor(OFF); // permanently off
            break;

        case STATE_GNSS_SEARCH:
            // slow orange blinking (1 Hz)
            if (now - lastUpdateTime > 500) {
                ledOn = !ledOn;
                setLedColor(ledOn ? ORANGE : OFF);
                lastUpdateTime = now;
            }
            break;

        case STATE_GNSS_2D:
            // fast green blinking (4 Hz)
            if (now - lastUpdateTime > 125) {
                ledOn = !ledOn;
                setLedColor(ledOn ? GREEN : OFF);
                lastUpdateTime = now;
            }
            break;

        case STATE_GNSS_3D:
            // slow green blinking (1 Hz)
            if (now - lastUpdateTime > 500) {
                ledOn = !ledOn;
                setLedColor(ledOn ? GREEN : OFF);
                lastUpdateTime = now;
            }
            break;

        case STATE_BLE_CONNECTED:
            // blue flash every 5 seconds
            if (now - lastUpdateTime > 5000) {
                setLedColor(BLUE);
                lastUpdateTime = now;
            } else if (now - lastUpdateTime > 50) { // brief flash duration 50 ms
                setLedColor(OFF);
            }
            break;

        case STATE_ERROR:
            // fast red blinking (8 Hz)
            if (now - lastUpdateTime > 625) {
                ledOn = !ledOn;
                setLedColor(ledOn ? RED : OFF);
                lastUpdateTime = now;
            }
            break;

        default:
            setLedColor(OFF);
            break;
    }
}
