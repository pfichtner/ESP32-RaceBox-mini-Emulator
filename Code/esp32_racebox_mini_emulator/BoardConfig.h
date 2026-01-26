#pragma once

// --- GPS Configuration ---
constexpr int GPS_RX_PIN = 23;
constexpr int GPS_TX_PIN = 24;
constexpr int GPS_BAUD = 115200;
constexpr int FACTORY_GPS_BAUD = 9600;
constexpr int MAX_NAVIGATION_RATE = 25;
// #define GPS_SERIAL_PACKET_RATE_DEBUG

// Which sensor to use
#define USE_MPU6050
// #define USE_MPU9250

// --- MPU6050/MPU9250 Configuration ---
constexpr int MPU_SDA_PIN = 4;
constexpr int MPU_SCL_PIN = 5;

// #define IMU_MOUNTED_OVERHEAD // board is mounted overhead (upside down)
