// NullMPUAdapter.h
#include "SensorInterface.h"
#include <cstring>

class NullMPUAdapter final : public SensorInterface {
public:
  bool begin() override {
#if defined(ARDUINO)
      Serial.println("⚠️ No MPU defined, using NullMPUAdapter");      
#endif
    return true;
  }

protected:
  bool readRaw(SensorData& data) override {
    memset(&data, 0, sizeof(data));
    return true;
  }
};
