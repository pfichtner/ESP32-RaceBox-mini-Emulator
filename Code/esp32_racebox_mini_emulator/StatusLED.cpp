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

// --- Shared state ---
volatile uint8_t currentLedState = STATE_BOOT;

// --- LED task ---
void ledTask(void *pvParameters) {
    bool ledOn = false;
    unsigned long lastUpdateTime = 0;

    while (true) {
        unsigned long now = millis();

        switch (currentLedState) {

            case STATE_BOOT:
                led.setPixelColor(0, OFF);
                led.show();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;

            case STATE_GNSS_SEARCH:
                if (now - lastUpdateTime > 500) { // 1 Hz
                    ledOn = !ledOn;
                    led.setPixelColor(0, ledOn ? ORANGE : OFF);
                    led.show();
                    lastUpdateTime = now;
                }
                vTaskDelay(50 / portTICK_PERIOD_MS);
                break;

            case STATE_GNSS_2D:
                if (now - lastUpdateTime > 125) { // 4 Hz
                    ledOn = !ledOn;
                    led.setPixelColor(0, ledOn ? GREEN : OFF);
                    led.show();
                    lastUpdateTime = now;
                }
                vTaskDelay(20 / portTICK_PERIOD_MS);
                break;

            case STATE_GNSS_3D:
                if (now - lastUpdateTime > 500) { // 1 Hz
                    ledOn = !ledOn;
                    led.setPixelColor(0, ledOn ? GREEN : OFF);
                    led.show();
                    lastUpdateTime = now;
                }
                vTaskDelay(50 / portTICK_PERIOD_MS);
                break;

            case STATE_BLE_CONNECTED:
                if (now - lastUpdateTime > 5000) {
                    led.setPixelColor(0, BLUE);
                    led.show();
                    lastUpdateTime = now;
                } else if (now - lastUpdateTime > 50) { // brief flash
                    led.setPixelColor(0, OFF);
                    led.show();
                }
                vTaskDelay(20 / portTICK_PERIOD_MS);
                break;

            case STATE_ERROR:
                if (now - lastUpdateTime > 125) { // 8 Hz
                    ledOn = !ledOn;
                    led.setPixelColor(0, ledOn ? RED : OFF);
                    led.show();
                    lastUpdateTime = now;
                }
                vTaskDelay(20 / portTICK_PERIOD_MS);
                break;

            default:
                led.setPixelColor(0, OFF);
                led.show();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;
        }
    }
}

// --- Initialization ---
void ledSetup() {
    led.begin();
    led.setBrightness(LED_BRIGHTNESS);
    led.show();

    xTaskCreate(
        ledTask,        // Task function
        "LED Task",     // Name
        4096,           // Stack size
        NULL,           // Parameters
        1,              // Priority
        NULL            // Task handle
    );
}

// --- Update LED state ---
void setLedState(uint8_t state) {
    currentLedState = state;
}
