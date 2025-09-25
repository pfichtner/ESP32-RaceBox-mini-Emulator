#include "Config.h"
#include <ArduinoJson.h>
#include "FS.h"
#include "SPIFFS.h"

#define GNSS_CONFIG_PATH "/gnss_config.json"

// --- Enable GNSS constellations ---
// The specific constellations available depend on your u-blox module 
// and how many you can turn on depend on your u-blox module 
// Common ones are GPS, Galileo, GLONASS, BeiDou, QZSS, SBAS.
// check this out for which constellations to enable https://app.qzss.go.jp/GNSSView/gnssview.html
GnssConfig gnssConfig = { true, true, false, false, false, false }; // defaults

// --- Initialization ---
void configSetup() {
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ SPIFFS Mount Failed");
  }
  loadGnssConfig(gnssConfig);
}

void saveGnssConfig(const GnssConfig& config) {
  File f = SPIFFS.open(GNSS_CONFIG_PATH, FILE_WRITE);
  if (!f) {
    Serial.println("❌ Failed to open GNSS config for writing");
    return;
  }
  StaticJsonDocument<256> doc;
  doc["gps"] = config.gps;
  doc["galileo"] = config.galileo;
  doc["glonass"] = config.glonass;
  doc["beidou"] = config.beidou;
  doc["qzss"] = config.qzss;
  doc["sbas"] = config.sbas;
  if (serializeJson(doc, f) == 0) {
    Serial.println("❌ Failed to write GNSS config JSON");
  } else {
    Serial.println("💾 GNSS config saved");
  }
  f.close();
}

void loadGnssConfig(GnssConfig& config) {
  if (!SPIFFS.exists(GNSS_CONFIG_PATH)) {
    Serial.println("⚠️ No GNSS config, using defaults");
    return;
  }
  File f = SPIFFS.open(GNSS_CONFIG_PATH, FILE_READ);
  if (!f) {
    Serial.println("❌ Failed to open GNSS config for reading");
    return;
  }
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, f);
  if (err) {
    Serial.printf("❌ Failed to parse GNSS config: %s\n", err.c_str());
  } else {
    config.gps = doc["gps"] | true;
    config.galileo = doc["galileo"] | true;
    config.glonass = doc["glonass"] | false;
    config.beidou = doc["beidou"] | false;
    config.qzss = doc["qzss"] | false;
    config.sbas = doc["sbas"] | false;
    Serial.println("📂 GNSS config loaded");
  }
  f.close();
}
