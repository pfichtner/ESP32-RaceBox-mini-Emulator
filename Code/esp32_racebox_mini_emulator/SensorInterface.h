// SensorInterface.h
#include "BoardConfig.h"
#pragma once

struct SensorData {
  float ax, ay, az;
  float gx, gy, gz;

  /**
  * Apply Exponential Moving Average (EMA) smoothing to the sensor values.
  *
  * This function updates the current SensorData values using the EMA formula:
  *    y[n] = α * x[n] + (1 - α) * y[n-1]
  *
  * where:
  *   - x[n] is the new input reading (raw sensor data)
  *   - y[n] is the filtered output (this SensorData instance)
  *   - α is the smoothing factor (0..1)
  *
  * Higher α values (closer to 1.0) make the output follow the raw data more closely
  * (less smoothing). Lower α values make the output smoother but slower to respond.
  *
  * accelAlpha: smoothing factor for accelerometer data (ax/ay/az)
  * gyroAlpha:  smoothing factor for gyro data (gx/gy/gz)
  */
  void applyEMA(const SensorData& input, float accelAlpha, float gyroAlpha) {
      ax = accelAlpha * input.ax + (1.0f - accelAlpha) * ax;
      ay = accelAlpha * input.ay + (1.0f - accelAlpha) * ay;
      az = accelAlpha * input.az + (1.0f - accelAlpha) * az;

      gx = gyroAlpha * input.gx + (1.0f - gyroAlpha) * gx;
      gy = gyroAlpha * input.gy + (1.0f - gyroAlpha) * gy;
      gz = gyroAlpha * input.gz + (1.0f - gyroAlpha) * gz;
  }

  /**
  * Flip IMU axes if the sensor board is mounted upside down.
  *
  * This applies a 180° rotation by negating all accelerometer and gyro axes.
  * Use when the IMU is mounted overhead (upside down) to keep coordinate system
  * consistent with the rest of the firmware.
  */
  void flipOverhead() {
    ax = -ax; ay = -ay; az = -az;
    gx = -gx; gy = -gy; gz = -gz;
  }
};

class SensorInterface {
public:
  virtual bool begin() = 0;

  virtual bool read(SensorData& data) {
    if (!readRaw(data)) return false;
#ifdef IMU_MOUNTED_OVERHEAD
    data.flipOverhead();
#endif
    return true;
  }
  virtual ~SensorInterface() {}

protected:
  virtual bool readRaw(SensorData& data) = 0;
};
