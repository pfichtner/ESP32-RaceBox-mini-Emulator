#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <NimBLEDevice.h>
#include "SensorInterface.h"
#include "Config.h"
#include "StatusLED.h"

// --- GPS Configuration ---
#define GPS_RX_PIN 23
#define GPS_TX_PIN 24
#define GPS_BAUD 115200
#define FACTORY_GPS_BAUD 9600
#define MAX_NAVIGATION_RATE 25
// #define GPS_SERIAL_PACKET_RATE_DEBUG

// Which sensor to use
#define USE_MPU6050
// #define USE_MPU9250

// --- MPU6050/MPU9250 Configuration ---
#define MPU_SDA_PIN 4
#define MPU_SCL_PIN 5
// if board is mounted overhead
#define IMU_MOUNTED_OVERHEAD

SFE_UBLOX_GNSS myGNSS;
#ifdef CONFIG_IDF_TARGET_ESP32H2
HardwareSerial GPS_Serial(1); // UART1
#else
HardwareSerial GPS_Serial(2); // UART2
#endif

constexpr const char rawDeviceName[] = "RaceBox Mini 0123456789";
constexpr unsigned long MAX_ALLOWED = 3999999999UL;

constexpr int cstrlen(const char* s) {
    int len = 0;
    while (s[len] != '\0') ++len;
    return len;
}

constexpr bool startsWith(const char* s, const char* prefix) {
    for (int i = 0; prefix[i] != '\0'; ++i)
        if (s[i] != prefix[i]) return false;
    return true;
}

constexpr unsigned long parseSuffix(const char* name) {
    if (!startsWith(name, "RaceBox Mini ")) return 0;

    int len = cstrlen(name);
    if (len < 10) return 0;

    unsigned long val = 0;
    for (int i = len - 10; i < len; ++i) {
        char c = name[i];
        if (c < '0' || c > '9') return 0;
        val = val * 10 + (c - '0');
    }
    return val;
}

static_assert(parseSuffix(rawDeviceName) <= MAX_ALLOWED, "ERROR: RaceBox Mini number cannot exceed 3999999999");
static_assert(parseSuffix(rawDeviceName) != 0, "ERROR: Invalid RaceBox Mini device name");
constexpr const char* deviceName = rawDeviceName;

#if defined(USE_MPU6050)
  #include "Mpu6050Adapter.h"
  SensorInterface* sensor = new Mpu6050Adapter(&Wire);
#elif defined(USE_MPU9250)
  #include "Mpu9250Adapter.h"
  SensorInterface* sensor = new Mpu9250Adapter(&Wire);
#else
  #error "You must define either USE_MPU6050 or USE_MPU9250"
#endif

const unsigned long accelSampleInterval = 10; // 10ms = 100Hz

// --- Smoothing Configuration ---
// alpha = 1.0: No filtering (raw data)
// alpha = 0.5: 50% current reading, 50% previous (moderate)
// alpha = 0.8: Very snappy, just kills high-frequency "buzz"
float accelAlpha = 0.8;
float gyroAlpha = 0.8;
// Storage for the filtered values
SensorData filtered = {
    NAN, NAN, NAN,
    NAN, NAN, NAN
};

// --- BLE Configuration ---
const char* const RACEBOX_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
const char* const RACEBOX_CHARACTERISTIC_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
const char* const RACEBOX_CHARACTERISTIC_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
const char* const RACEBOX_CHARACTERISTIC_GNSS_UUID = "6E400004-B5A3-F393-E0A9-E50E24DCCA9E";

NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pCharacteristicTx = nullptr;
NimBLECharacteristic* pCharacteristicRx = nullptr;
NimBLECharacteristic* pCharacteristicGnss = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// --- Packet Timing ---
unsigned long lastPacketSendTime = 0;
const unsigned long PACKET_SEND_INTERVAL_MS = 1000 / 25;
unsigned long lastGpsRateCheckTime = 0;
unsigned int gpsUpdateCount = 0;
const unsigned long GPS_RATE_REPORT_INTERVAL_MS = 5000;
unsigned int gnssUpdateCount = 0;

void applyGnssConfig() {
  auto applyGnss = [&](bool enabled, sfe_ublox_gnss_ids_e id, const char* name) {
      const char* state = enabled ? "enabled" : "disabled";
      const char* icon  = enabled ? "✅" : "🚫";
      if (myGNSS.enableGNSS(enabled, id)) {
          Serial.printf("%s %s %s\n", icon, name, state);
      } else {
          Serial.printf("❌ Failed to %s %s\n", state, name);
      }
  };

  applyGnss(gnssConfig.gps,     SFE_UBLOX_GNSS_ID_GPS,     "GPS");
  applyGnss(gnssConfig.galileo, SFE_UBLOX_GNSS_ID_GALILEO, "Galileo");
  applyGnss(gnssConfig.glonass, SFE_UBLOX_GNSS_ID_GLONASS, "GLONASS");
  applyGnss(gnssConfig.beidou,  SFE_UBLOX_GNSS_ID_BEIDOU,  "BeiDou");
  applyGnss(gnssConfig.sbas,    SFE_UBLOX_GNSS_ID_SBAS,    "SBAS");
  applyGnss(gnssConfig.qzss,    SFE_UBLOX_GNSS_ID_QZSS,    "QZSS");
}

// --- BLE Callbacks ---
class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    deviceConnected = true;
    uint16_t mtu = pServer->getPeerMTU(connInfo.getConnHandle());
    Serial.printf("✅ BLE Client connected, negotiated MTU = %d\n", mtu);
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    deviceConnected = false;
    Serial.println("❌ BLE Client disconnected");
  }
};

class MyCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
public:
  void onWrite(NimBLECharacteristic* pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (!rxValue.empty()) {
      Serial.print("📨 Received BLE command: ");
      for (uint8_t c : rxValue)
        Serial.printf("0x%02X ", c);
      Serial.println();
    }
  }
};

class GnssCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
public:
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.size() != sizeof(GnssConfig)) {
            Serial.println("❌ Invalid GNSS config length");
            return;
        }

        // Copy received bytes into gnssConfig
        memcpy(&gnssConfig, value.data(), sizeof(GnssConfig));
        Serial.println("📨 GNSS config updated via BLE");

        // Apply immediately
        applyGnssConfig();

        // Persist to SPIFFS
        saveGnssConfig(gnssConfig);

        // Send back updated GNSS config to BLE client
        if (pCharacteristic->getProperties() & NIMBLE_PROPERTY::NOTIFY) {
            pCharacteristic->setValue((uint8_t*)&gnssConfig, sizeof(GnssConfig));
            pCharacteristic->notify();
            Serial.println("📤 GNSS config sent back via BLE notification");
        }
    }

    void onRead(NimBLECharacteristic* pCharacteristic) {
        pCharacteristic->setValue((uint8_t*)&gnssConfig, sizeof(GnssConfig));
        Serial.println("📤 GNSS config read via BLE");
    }
};

// --- UBX Packet Construction Helpers ---
template <typename T>
void writeLittleEndian(uint8_t* buffer, int offset, T value) {
  memcpy(buffer + offset, &value, sizeof(T));
}

void calculateChecksum(uint8_t* payload, uint16_t len, uint8_t cls, uint8_t id, uint8_t* ckA, uint8_t* ckB) {
  *ckA = *ckB = 0;
  *ckA += cls; *ckB += *ckA;
  *ckA += id; *ckB += *ckA;
  *ckA += len & 0xFF; *ckB += *ckA;
  *ckA += len >> 8; *ckB += *ckA;
  for (uint16_t i = 0; i < len; i++) {
    *ckA += payload[i];
    *ckB += *ckA;
  }
}

// Initialize GNSS at a given baud, retrying endlessly
void waitForGNSS(int baud) {
    GPS_Serial.begin(baud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(500);

    while (!myGNSS.begin(GPS_Serial)) {
        Serial.printf("GNSS not detected at %d baud! Retrying...\n", baud);
        setLedState(STATE_ERROR);
        delay(500);
    }

    Serial.printf("GNSS detected at %d baud!\n", baud);
}

void resetGpsBaudRate() {
    Serial.println("Attempting to set correct factory baud rate...");
    waitForGNSS(FACTORY_GPS_BAUD);
    delay(500);

    // Switch baud rate
    Serial.printf("Setting baud rate to %d baud...\n", GPS_BAUD);
    myGNSS.setSerialRate(GPS_BAUD);
    Serial.printf("Baud rate changed to %d baud\n", GPS_BAUD);

    GPS_Serial.end();
    delay(100);

    // Re-initialize at new baud
    waitForGNSS(GPS_BAUD);

    Serial.println("Saving configuration to Flash.");
    myGNSS.saveConfiguration();
    GPS_Serial.end();
}


void setup() {
  Serial.begin(115200);
  ledSetup();
  configSetup();
  setLedState(STATE_BOOT);
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);

  if (!sensor->begin()) {
    Serial.println("❌ Failed to init mpu sensor");
    setLedState(STATE_ERROR);
    while (1) delay(100);
  }
  SensorData data;
  if (sensor->read(data)) {
#ifdef IMU_MOUNTED_OVERHEAD
    // Flip all axes if board is mounted overhead (180° rotation)
    data.flipOverhead();
#endif

    filtered = data;
  }

  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  if (!myGNSS.begin(GPS_Serial)) {
    Serial.println("❌ GNSS not detected. Attempting to configure.");
    GPS_Serial.end();
    resetGpsBaudRate();
    GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  }

  // Set GNSS output to PVT only
  myGNSS.setAutoPVT(true);
  myGNSS.setDynamicModel(DYN_MODEL_AUTOMOTIVE);
    // --- Configure GPS update rate to MAX_NAVIGATION_RATE Hz ---
  if (myGNSS.setNavigationFrequency(MAX_NAVIGATION_RATE)) {
  Serial.printf("✅ GPS update rate set to %d Hz.\n",MAX_NAVIGATION_RATE );
  } else {
    Serial.println("❌ Failed to set GPS update rate.");
  }

  // --- GNSS Constellation Setup ---

  applyGnssConfig();

  // --- BLE Setup ---
  // Do NOT call setMTU() on ESP32-H2 — it crashes. Other ESP32 variants support larger MTU.
#ifndef CONFIG_IDF_TARGET_ESP32H2
  // Request a larger MTU to fit an 88-byte packet + headers in one go
  NimBLEDevice::setMTU(128);
#endif
  NimBLEDevice::init(deviceName);
  NimBLEDevice::setPower(4);  // Medium BLE transmit power (≈ -3 to 0 dBm) to reduce RF contention and power draw on ESP32-H2
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  NimBLEService* pService = pServer->createService(RACEBOX_SERVICE_UUID);
  pCharacteristicTx = pService->createCharacteristic(RACEBOX_CHARACTERISTIC_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  pCharacteristicRx = pService->createCharacteristic(RACEBOX_CHARACTERISTIC_RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pCharacteristicRx->setCallbacks(new MyCharacteristicCallbacks());

  pCharacteristicGnss = pService->createCharacteristic(RACEBOX_CHARACTERISTIC_GNSS_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  pCharacteristicGnss->setCallbacks(new GnssCharacteristicCallbacks());

  // Model
  NimBLECharacteristic* pModel = pService->createCharacteristic("00002A24-0000-1000-8000-00805F9B34FB", NIMBLE_PROPERTY::READ);
  pModel->setValue("RaceBox Mini");
  // Serial number (last 10 digits of device name)
  NimBLECharacteristic* pSerial = pService->createCharacteristic("00002A25-0000-1000-8000-00805F9B34FB", NIMBLE_PROPERTY::READ);
  pSerial->setValue((strlen(deviceName) >= 10) ? deviceName + strlen(deviceName) - 10 : "0000000000");
  // Firmware revision
  NimBLECharacteristic* pFirm = pService->createCharacteristic("00002A26-0000-1000-8000-00805F9B34FB", NIMBLE_PROPERTY::READ);
  pFirm->setValue("3.3");
  // Hardware revision
  NimBLECharacteristic* pHardware = pService->createCharacteristic("00002A27-0000-1000-8000-00805F9B34FB", NIMBLE_PROPERTY::READ);
  pHardware->setValue("1");
  // Manufacturer
  NimBLECharacteristic* pManufacturer = pService->createCharacteristic("00002A29-0000-1000-8000-00805F9B34FB", NIMBLE_PROPERTY::READ);
  pManufacturer->setValue("RaceBox");

  pService->start();
  NimBLEDevice::setDeviceName(deviceName);

  NimBLEAdvertisementData advData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP); // 0x01
  advData.addServiceUUID(NimBLEUUID(RACEBOX_SERVICE_UUID));           // 0x07
 // Advertise Device Information Service as well to help official apps discover the device
  advData.addServiceUUID("0000180a-0000-1000-8000-00805f9b34fb");
  advData.addTxPower();                                               // 0x0A

  NimBLEAdvertisementData scanRespData;
  scanRespData.setName(deviceName); // 0x09: Complete Local Name
  scanRespData.addData({ 0x05, 0x12, 0x20, 0x00, 0x40, 0x00 }); // 0x12

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setAdvertisementData(advData);
  pAdvertising->setScanResponseData(scanRespData);

  pAdvertising->start();
  Serial.println("📡 BLE advertising started.");

  lastGpsRateCheckTime = millis();
}

void loop() {
  // default to searching while no fix
  uint8_t state = STATE_GNSS_SEARCH;
  myGNSS.checkUblox(); // Required to keep GNSS data flowing

  static unsigned long lastAccelReadMs = 0;
  // Update accelrometer readings at fixed interval
  if (millis() - lastAccelReadMs >= accelSampleInterval) {
    lastAccelReadMs += accelSampleInterval; // Strict timing grid

    SensorData data;
    if (sensor->read(data)) {
#ifdef IMU_MOUNTED_OVERHEAD
      // Flip all axes if board is mounted overhead (180° rotation)
      data.flipOverhead();
#endif

      // Apply Exponential Moving Average (Complementary Filter logic)
      if (isnan(filtered.ax)) {
          filtered = data;
      } else {
          filtered.applyEMA(data, accelAlpha, gyroAlpha);
      }

    }
  }

  if (myGNSS.getPVT()) {
    static uint32_t lastITOW = 0;
    uint32_t currentITOW = myGNSS.packetUBXNAVPVT->data.iTOW;

    // Update LED based on GNSS fix type
    uint8_t fix = myGNSS.packetUBXNAVPVT->data.fixType;
    if (fix == 2) {
        state = STATE_GNSS_2D;
    } else if (fix >= 3) {
        state = STATE_GNSS_3D;
    }

    if (currentITOW != lastITOW) {
      lastITOW = currentITOW;
      gnssUpdateCount++;

      if ((deviceConnected && myGNSS.packetUBXNAVPVT != NULL) && (millis() - lastPacketSendTime >= PACKET_SEND_INTERVAL_MS)) {
        lastPacketSendTime = millis();
        gpsUpdateCount++;

        uint8_t payload[80] = {0};
        uint8_t packet[88] = {0};

        // Access data directly from myGNSS.packetUBXNAVPVT->data
        writeLittleEndian(payload, 0, myGNSS.packetUBXNAVPVT->data.iTOW);
        writeLittleEndian(payload, 4, myGNSS.packetUBXNAVPVT->data.year);
        writeLittleEndian(payload, 6, myGNSS.packetUBXNAVPVT->data.month);
        writeLittleEndian(payload, 7, myGNSS.packetUBXNAVPVT->data.day);
        writeLittleEndian(payload, 8, myGNSS.packetUBXNAVPVT->data.hour);
        writeLittleEndian(payload, 9, myGNSS.packetUBXNAVPVT->data.min);
        writeLittleEndian(payload, 10, myGNSS.packetUBXNAVPVT->data.sec);

        // Offset 11: Validity Flags (RaceBox Protocol) 
        uint8_t raceboxValidityFlags = 0;
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validDate) raceboxValidityFlags |= (1 << 0); // Bit 0: valid date 
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validTime) raceboxValidityFlags |= (1 << 1); // Bit 1: valid time 
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.fullyResolved) raceboxValidityFlags |= (1 << 2); // Bit 2: fully resolved 
        writeLittleEndian(payload, 11, raceboxValidityFlags);

        // Offset 12: Time Accuracy (RaceBox Protocol) 
        writeLittleEndian(payload, 12, myGNSS.packetUBXNAVPVT->data.tAcc);

        // Offset 16: Nanoseconds (RaceBox Protocol) 
        writeLittleEndian(payload, 16, myGNSS.packetUBXNAVPVT->data.nano);

        // Offset 20: Fix Status (RaceBox Protocol) 
        writeLittleEndian(payload, 20, myGNSS.packetUBXNAVPVT->data.fixType);

        // Offset 21: Fix Status Flags (RaceBox Protocol)
        uint8_t fixStatusFlagsRacebox = 0;

        if (myGNSS.packetUBXNAVPVT->data.fixType == 3) {
            fixStatusFlagsRacebox |= (1 << 0); // Bit 0: valid fix
        }

        // Add this line to set Bit 5 for valid heading using getHeadVehValid()
        if (myGNSS.getHeadVehValid()) { // Use the confirmed function to check for valid heading
            fixStatusFlagsRacebox |= (1 << 5); // Bit 5: valid heading (as per RaceBox Protocol)
        }
        writeLittleEndian(payload, 21, fixStatusFlagsRacebox);

        // Offset 22: Date/Time Flags (RaceBox Protocol) 
        uint8_t raceboxDateTimeFlags = 0;
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validTime) raceboxDateTimeFlags |= (1 << 5); // Available confirmation of Date/Time Validity
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validDate) raceboxDateTimeFlags |= (1 << 6); // Confirmed UTC Date Validity
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validTime && myGNSS.packetUBXNAVPVT->data.valid.bits.fullyResolved) raceboxDateTimeFlags |= (1 << 7); // Confirmed UTC Time Validity
        writeLittleEndian(payload, 22, raceboxDateTimeFlags);

        // Offset 23: Number of SVs (RaceBox Protocol) 
        writeLittleEndian(payload, 23, myGNSS.packetUBXNAVPVT->data.numSV);

        // Remaining fields, mostly direct mappings from u-blox data
        writeLittleEndian(payload, 24, myGNSS.packetUBXNAVPVT->data.lon);
        writeLittleEndian(payload, 28, myGNSS.packetUBXNAVPVT->data.lat);
        writeLittleEndian(payload, 32, myGNSS.packetUBXNAVPVT->data.height);
        writeLittleEndian(payload, 36, myGNSS.packetUBXNAVPVT->data.hMSL);

        writeLittleEndian(payload, 40, myGNSS.packetUBXNAVPVT->data.hAcc);
        writeLittleEndian(payload, 44, myGNSS.packetUBXNAVPVT->data.vAcc);
        writeLittleEndian(payload, 48, myGNSS.packetUBXNAVPVT->data.gSpeed);
        writeLittleEndian(payload, 52, myGNSS.packetUBXNAVPVT->data.headMot);
        writeLittleEndian(payload, 56, myGNSS.packetUBXNAVPVT->data.sAcc);
        writeLittleEndian(payload, 60, myGNSS.packetUBXNAVPVT->data.headAcc);

        writeLittleEndian(payload, 64, myGNSS.packetUBXNAVPVT->data.pDOP);

        // Offset 66: Lat/Lon Flags (RaceBox Protocol) 
        constexpr uint8_t INVALID_COORDS = 1 << 0;
        uint8_t latLonFlags = (myGNSS.packetUBXNAVPVT->data.fixType < 2) ? INVALID_COORDS : 0;
        writeLittleEndian(payload, 66, latLonFlags);

        // Offset 67: Battery status (1 byte) - report 100%
        writeLittleEndian(payload, 67, (uint8_t) 100);

        writeLittleEndian(payload, 68, ms2_to_millig(filtered.ax));
        writeLittleEndian(payload, 70, ms2_to_millig(filtered.ay));
        writeLittleEndian(payload, 72, ms2_to_millig(filtered.az));
        writeLittleEndian(payload, 74, radps_to_cdegps(filtered.gx));
        writeLittleEndian(payload, 76, radps_to_cdegps(filtered.gy));
        writeLittleEndian(payload, 78, radps_to_cdegps(filtered.gz));


        // Wrap in UBX (standard RaceBox header and checksum)
        packet[0] = 0xB5;
        packet[1] = 0x62;
        packet[2] = 0xFF; // Message Class: RaceBox Data Message 
        packet[3] = 0x01; // Message ID: RaceBox Data Message 
        packet[4] = 80;   // Payload size 
        packet[5] = 0;
        memcpy(packet + 6, payload, 80);
        uint8_t ckA, ckB;
        // Use the correct Class (0xFF) and ID (0x01) for checksum calculation as per RaceBox protocol 
        calculateChecksum(payload, 80, 0xFF, 0x01, &ckA, &ckB);
        packet[86] = ckA;
        packet[87] = ckB;

        pCharacteristicTx->setValue(packet, 88);
        pCharacteristicTx->notify();
      }
    }

    // Report packet send rate
    const unsigned long now = millis();
    if ((now - lastGpsRateCheckTime) >= GPS_RATE_REPORT_INTERVAL_MS) {
      float bleRate = gpsUpdateCount / (GPS_RATE_REPORT_INTERVAL_MS / 1000.0);
      float gnssRate = gnssUpdateCount / (GPS_RATE_REPORT_INTERVAL_MS / 1000.0);
#ifdef GPS_SERIAL_PACKET_RATE_DEBUG
      Serial.printf("BLE Packet Rate: %.2f Hz | GNSS Update Rate: %.2f Hz\n", bleRate, gnssRate);
#endif
      gpsUpdateCount = 0;
      gnssUpdateCount = 0;
      lastGpsRateCheckTime = now;
    }

    if (!deviceConnected && oldDeviceConnected) {
      delay(500);
      pServer->startAdvertising();
      oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
    }
  }
  if (deviceConnected) {
    state = STATE_BLE_CONNECTED;
  }
  setLedState(state);
}

static inline int16_t ms2_to_millig(float a) {
    return (int16_t)(a * (1000.0f / 9.80665f));
}

static inline int16_t radps_to_cdegps(float r) {
    return (int16_t)(r * (180.0f / M_PI * 100.0f));
}
