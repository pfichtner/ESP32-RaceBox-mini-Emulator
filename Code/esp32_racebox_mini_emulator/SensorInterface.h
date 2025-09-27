// SensorInterface.h
#pragma once

struct SensorData {
  float ax, ay, az;
  float gx, gy, gz;

  // Flip all axes if board is mounted overhead (180° rotation)
  void flipOverhead() {
    ax = -ax; ay = -ay; az = -az;
    gx = -gx; gy = -gy; gz = -gz;
  }
};

class SensorInterface {
public:
  virtual bool begin() = 0;
  virtual bool read(SensorData& data) = 0;
  virtual ~SensorInterface() {}
};

