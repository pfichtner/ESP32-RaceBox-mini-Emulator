// Mpu6050Adapter.h
#include <Adafruit_MPU6050.h>
#include "SensorInterface.h"

class Mpu6050Adapter : public SensorInterface {
  Adafruit_MPU6050 mpu;
  TwoWire* wire;
  int addr;
public:
  // Default ctor uses &Wire and the default I2C address
  Mpu6050Adapter(TwoWire* wire = &Wire, int addr = MPU6050_I2CADDR_DEFAULT)
    : wire(wire), addr(addr) {}
public:
  bool begin() override {
    if (!mpu.begin()) return false;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    return true;
  }

  bool read(SensorData& data) override {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    data.ax = a.acceleration.x;
    data.ay = a.acceleration.y;
    data.az = a.acceleration.z;
    data.gx = g.gyro.x;
    data.gy = g.gyro.y;
    data.gz = g.gyro.z;
    return true;
  }
};

