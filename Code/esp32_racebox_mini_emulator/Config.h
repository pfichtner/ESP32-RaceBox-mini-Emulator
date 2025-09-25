#pragma once
#include <Arduino.h>

// Struct definition
struct GnssConfig {
  bool gps;
  bool galileo;
  bool glonass;
  bool beidou;
  bool qzss;
  bool sbas;
};

// Global instance (declared as extern here, defined in Config.cpp)
extern GnssConfig gnssConfig;

// Functions
void configSetup();
void saveGnssConfig(const GnssConfig& config);
void loadGnssConfig(GnssConfig& config);
