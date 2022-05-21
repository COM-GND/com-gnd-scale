#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// https://github.com/nkolban/ESP32_BLE_Arduino
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <ADS126X.h>
#include <definitions/ADS126X_hardware.h> // board specific commands

#include <Smoothed.h>

#include "scale.h"

// https://btprodspecificationrefs.blob.core.windows.net/assigned-values/16-bit%20UUID%20Numbers%20Document.pdf
/**
 * GATT Characteristic and Object Type 0x2A6D Pressure (spec says unit is pascals)
 * GATT Unit 0x2780 bar
 * GATT Unit 0x272F Celsius temperature (degree Celsius)
 * GATT Characteristic and Object Type 0x2A1C Temperature Measurement
 * GATT Characteristic and Object Type 0x2A1D Temperature Type
 */
#define SERVICE_UUID "8fc1ceca-b162-4401-9607-c8ac21383e4e"
#define PRESSURE_SENSOR_CHAR_ID "c14f18ef-4797-439e-a54f-498ba680291d" // BT read-only characteristic for pressure sensor value in bars
#define PRESSURE_TARGET_CHAR_ID "34c242f1-8b5f-4d99-8238-4538eb0b5764" // BT read/write characteristic for target pressure in bars
#define PUMP_POWER_CHAR_ID "d8ad3645-50ad-4f7a-a79d-af0a59469455"      // BT read-only characteristic for pumps power level
#define TEMP_SENSOR_CHAR_ID 0x2A1C                                     // BT read-only characteristic for boiler external temperature
#define FLOW_SENSOR_CHAR_ID "f5ec47c3-b240-49e8-a7c8-1b5fe5537cde"     // BT read-only characteristic for in-flow rate
#define BARS_UNIT_ID "2780"

Scale comGndScale;

/**
 * Pins
 * set-up for ESP32 Devkit-c
 * see: https://circuits4you.com/2018/12/31/esp32-wroom32-devkit-analog-read-example/)
 */

/**
 * SPI - ESP32-devkit-c standard outputs for VSPI
 * ESP32    ADS1262           ESP32 VSPI Std Pin
 * ==============================================
 * SPID =   MOSI = data out - VSPI uses GPIO23
 * SPIQ =   MISO = data in  - VSPI uses GPIO19
 * VSPIWP = START           - VSPI uses GPIO22
 * CK =     CLK             - VSPI uses GPIO18
 * CS =     CS              - VSPI uses GPIO5
 */
// const int spiCs = 5;           // SPI CS pin
// const int spiStart = 22;       // SPI Start pin
// const int esp32AdcRdyPin = 17; // ADS126X RDY pin (14) -> ESP32 io pin (active low)

/**
 * HSFPAR303A force sensor
 * Pin 1 vdd Exc + -> ADC AIN4 (REF+)
 * pin 2 v1  Out + -> ADC AIN0
 * pin 3 v2  Out - -> ADC AIN1
 * pin 4 gnd Exc - -> AD AIN5 (REF-)
 */

float lastGrams = 0;
float grams = 0;
Smoothed<float> voltageSmoother;

/**
 * Bluetooth Globals
 */
bool deviceConnected = false;
bool oldDeviceConnected = false;
BLEServer *pServer = NULL;
BLECharacteristic *pPressureSensorBLEChar = NULL;
BLECharacteristic *pPressureTargetBLEChar = NULL;
BLECharacteristic *pPumpPowerBLEChar = NULL;
BLECharacteristic *pTemperatureSensorBLEChar = NULL;
BLECharacteristic *pFlowSensorBLEChar = NULL;

// https://github.com/nkolban/ESP32_BLE_Arduino/blob/master/examples/BLE_notify/BLE_notify.ino
class ComGndServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    Serial.println("BLE Server connected");
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    Serial.println("BLE Server Disconnected");
    deviceConnected = false;
  }
};
// https://learn.sparkfun.com/tutorials/esp32-thing-plus-hookup-guide/arduino-example-esp32-ble
class PressureSensorBLECharCallbacks : public BLECharacteristicCallbacks
{
  void onRead(BLECharacteristic *pCharacteristic)
  {
  }
};

void bleNotifyTask(void *params)
{
  for (;;)
  {
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

/**
 * Main Setup
 */
void setup()
{

  Serial.begin(115200);
  Serial.println("Setup start");
  Serial.begin(115200);

  // flow sensor support 100k and 400k freq
  // I2C.begin(i2cSda, i2cScl, 100000);

  BLEDevice::init("COM-GND Scale");
  Serial.println("BLE Device Initialized");
  pServer = BLEDevice::createServer();
  Serial.println("BLE Server Initialized");
  BLEService *pService = pServer->createService(SERVICE_UUID);
  Serial.println("BLE Service Initialized");
  pServer->setCallbacks(new ComGndServerCallbacks());
  Serial.println("BLE Server Callback Initialized");
  pService->start();

  Serial.println("BLE Service Started");

  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  Serial.println("BLE BLEAdvertising Initialized");

  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  Serial.println("BLE BLEAdvertising Setup Complete");

  BLEDevice::startAdvertising();
  Serial.println("BLE Advertizing Started");
  Serial.println("BLE Setup Complete");

  // pinMode(8, OUTPUT);
  Serial.begin(115200);
  // digitalWrite(8, HIGH);

  comGndScale.begin();
}

void handleCli()
{
  if (Serial.available())
  {
    char input = Serial.read();
    if (input == 'r')
    {
      Serial.println("Reseting Calibration");
      comGndScale.resetCellsCal();
      Serial.println("End Reseting Calibration");
    }
    // Calibrate several times with the weight at each corner to try and
    // find the max and min values for each cell.
    if (input == 'c')
    {
      Serial.println("Start Calibration");

      comGndScale.calCells();

      Serial.println("End Calibration");
    }

    if (input == 't')
    {
      Serial.println("Start Tare");

      comGndScale.tare();

      Serial.println("End Tare");
    }
  }
}

/**
 * Main Loop
 */
void loop()
{
  signed long int outputCode = 0;
  const int samples = 1;

  float weightG = comGndScale.readGrams();

  handleCli();
}
