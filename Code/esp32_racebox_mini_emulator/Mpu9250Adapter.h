// Mpu9250Adapter.h
#include "eigen.h"
#include "units.h"
using namespace bfs;
#include "mpu9250.h"
#include "SensorInterface.h"

class Mpu9250Adapter : public SensorInterface {
  bfs::Mpu9250 mpu;
public:
  Mpu9250Adapter(TwoWire* wire, bfs::Mpu9250::I2cAddr addr = bfs::Mpu9250::I2C_ADDR_PRIM)
    : mpu(wire, addr) {}

  bool begin() override {
    if (mpu.Begin() != 0) return false;
    mpu.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_8G);
    mpu.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_500DPS);
    mpu.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_20HZ);
    return true;
  }

protected:
  bool readRaw(SensorData& data) override {
    if (!mpu.Read()) return false;
    data.ax = mpu.accel_mps2()[0];
    data.ay = mpu.accel_mps2()[1];
    data.az = mpu.accel_mps2()[2];
    data.gx = mpu.gyro_radps()[0];
    data.gy = mpu.gyro_radps()[1];
    data.gz = mpu.gyro_radps()[2];
    return true;
  }
};

