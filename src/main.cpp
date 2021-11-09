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
// ADC Configs
// #define ADC_VREF 3.86 // external voltage reference
// #define ADC_GAIN 32   // the gain

// ALPS HSFPARx003 Configs
// #define ALPS_SENSITIVITY 3.7                               // mV/V/N
// #define ALPS_CAPACITY 8                                    // newtons
// #define ALPS_FULLSCALE_RANGE_MV ADC_VREF *ALPS_SENSITIVITY // mV at 8 newtons

// #define ADC_FSR_MV ALPS_FULLSCALE_RANGE_MV *ADC_GAIN // mV - The ADC's fullscale range

// ads1262 PC_ADS1262; // class
// ADS126X adc;              // ADS126X class
// const int adcAin2Pin = 2; // ADS126X pin AIN2, positive input
// const int adcAin3Pin = 3; // ADS126X pin AIN3, negative input
// const int adcAin4Pin = 4; // ADS126X pin AIN4, positive input
// const int adcAin5Pin = 5; // ADS126X pin AIN5, negative input
// const int adcAin6Pin = 6; // ADS126X pin AIN4, positive input
// const int adcAin7Pin = 7; // ADS126X pin AIN5, negative input
// const int adcAin8Pin = 8; // ADS126X pin AIN4, positive input
// const int adcAin9Pin = 9; // ADS126X pin AIN5, negative input

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
 * CS =     VSPICSO         - VSPI uses GPIO5
 */
// const int spiCs = 5;           // SPI CS pin
// const int spiStart = 22;       // SPI Start pin
// const int esp32AdcRdyPin = 17; // ADS126X RDY pin (14) -> ESP32 io pin (active low)

// // Reference Voltage
// const float adcVRef = 5.08;
// // Gain
// const float adcPga = 32.0;
// // ADC fullscale range
// const double adcFsr = (double)((double)adcVRef / (double)adcPga);

/**
* HSFPAR303A force sensor 
* Pin 1 vdd Exc + -> ADC AIN4 (REF+)
* pin 2 v1  Out + -> ADC AIN0
* pin 3 v2  Out - -> ADC AIN1
* pin 4 gnd Exc - -> AD AIN5 (REF-)
*/

// const float numberOfCells = 1.0; // number of load cells wired in parallel per adc channel
// const float cellSensitivity = 3.7;
// const float cellMaxLoad = 8.0;
// const float totalMaxLoad = cellMaxLoad * numberOfCells;
// const float cellFsr = cellSensitivity * adcVRef * adcPga;

float lastGrams = 0;
float grams = 0;
Smoothed<float> voltageSmoother;

//const double adcResolution = (double)((double)adcFsr / (double)(pow(2, 31)));

// signed long int adcOffset = 0;

// float readADC(byte inpsel);

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
//https://learn.sparkfun.com/tutorials/esp32-thing-plus-hookup-guide/arduino-example-esp32-ble
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

// /**
//  * Setup ADC registers for internal temperature reading
//  */
// void setupAdcTemperatureRead()
// {
//   // prepare registers to read temperature - see data sheet 9.3.4
//   // disale chop mode
//   adc.setChopMode(ADS126X_CHOP_0);
//   // set gain to 1
//   adc.setGain(ADS126X_GAIN_1);
// }

// float readAdcTemperature()
// {
//   signed long int outputCode = adc.readADC1(ADS126X_TEMP, ADS126X_TEMP);
//   delay(250);
//   outputCode = adc.readADC1(ADS126X_TEMP, ADS126X_TEMP);
//   // convert to celsius - see data sheet 9.3.4 (Equation 9)
//   float celsius = (float)((double)(outputCode - 122400.0) / 420.0) + 25.0;
//   Serial.println("C: " + String(celsius, 4) + " raw: " + String(outputCode));

//   return celsius;
// }

// void setupAdcLoadCellRead()
// {
//   adc.enableStatus();
//   adc.setChopMode(ADS126X_CHOP_1);
//   adc.setGain(ADS126X_GAIN_32);
// }

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

  // initalize the  data ready and chip select pins:
  // pinMode(ADS1262_DRDY_PIN, INPUT);   //data ready input line
  // pinMode(ADS1262_CS_PIN, OUTPUT);    //chip enable output line
  // pinMode(ADS1262_START_PIN, OUTPUT); // start
  // pinMode(ADS1262_PWDN_PIN, OUTPUT);  // Power down output
  // pinMode(16, OUTPUT); // ADC1262 PWDN Pin
  // digitalWrite(16, HIGH);
  // digitalWrite(8, LOW);

  // pinMode(esp32AdcRdyPin, INPUT_PULLUP); // adc data ready (DRDY) pin (active low)

  // voltageSmoother.begin(SMOOTHED_EXPONENTIAL, 80);

  // adc.begin(spiCs);
  // adc.setStartPin(spiStart);

  // Internal 2.5 V is needed for internal temp sensor and to calibrate the external ref.
  // adc.enableInternalReference();

  // We use the excitation voltage as the reference for Ratiometric Measurement
  // GNDA is connected to AIN5, +5A is connected to AIN4
  // adc.setReference(ADS126X_REF_NEG_AIN1, ADS126X_REF_POS_AIN0);
  // adc.setContinuousMode();
  // adc.enableStatus();
  // // Set Sample Rate to 60 SPS
  // adc.setRate(ADS126X_RATE_400);
  // adc.setFilter(ADS126X_SINC4);
  // adc.setRate(ADS126X_RATE_20);
  // adc.setFilter(ADS126X_FIR);
  // // Enable Chop mode - note: chop mode disables offset callibration registers (SFOCAL1 and SYOCAL1)
  // uint8_t chopRegBefore = adc.readRegister(ADS126X_MODE0);
  // adc.setChopMode(ADS126X_CHOP_1);
  // uint8_t chopRegAfter = adc.readRegister(ADS126X_MODE0);
  // Serial.print("chop: ");
  // Serial.print(chopRegBefore, BIN);
  // Serial.print(" -> ");
  // Serial.print(chopRegAfter, BIN);
  // Serial.println("");

  // uint8_t gainRegBefore = adc.readRegister(ADS126X_MODE2);
  // adc.setGain(ADS126X_GAIN_32);
  // uint8_t gainRegAfter = adc.readRegister(ADS126X_MODE2);
  // Serial.print("gain: ");
  // Serial.print(gainRegBefore, BIN);
  // Serial.print(" -> ");
  // Serial.print(gainRegAfter, BIN);
  // Serial.println("");
  // adc.enablePGA();
  // adc.startADC1();

  // setupAdcTemperatureRead();
  // delay(500);
  // float celsius = readAdcTemperature();
  // delay(250);
  // setupAdcLoadCellRead();
  // delay(250);
  // adc.readADC1(adcAin2Pin, adcAin3Pin);
  // delay(250);
  // adc.readADC1(adcAin2Pin, adcAin3Pin);
  // delay(2000);
  // tare
  // double offsetTotal = 0;
  // for (uint8_t i = 0; i < 10; i++)
  // {
  //   offsetTotal += adc.readADC1(adcAin2Pin, adcAin3Pin); // read the voltage
  //   Serial.println(" Offset total: " + String(offsetTotal));

  //   delay(100);
  // }

  // adcOffset = (double)round(offsetTotal / 10);

  // adc.setRate(ADS126X_RATE_60);
  // adc.setFilter(ADS126X_SINC3);

  // Serial.println("Temp C: " + String(celsius));
  // Serial.println("ADC Res: " + String(adcResolution, 6));

  // Serial.println("ADC Offset: " + String((float)(adcOffset * adcResolution)) + " raw: " + String(adcOffset) + " Offset total: " + String(offsetTotal));
  //xTaskCreate(bleNotifyTask, "bleNotify", 5000, NULL, 1, NULL);
}

// float readAdcV(int ainPos, int ainNeg)
// {

//   adc.REGISTER.INPMUX.bit.MUXN = ainNeg;
//   adc.REGISTER.INPMUX.bit.MUXP = ainPos;
//   adc.writeRegister(ADS126X_INPMUX); // replace on ads126x

//   adc.startADC1();
//   while (digitalRead(esp32AdcRdyPin) == HIGH)
//   {
//     // wait
//   }
//   adc.stopADC1();
//   signed long int outputCode = 0;
//   // while (outputCode == 0 || outputCode == -1)
//   // {
//   outputCode = adc.readADC1(ainPos, ainNeg); // read the voltage
//   //}
//   float voltage = (float)((adcResolution) * (double)(outputCode));
//   // Serial.print(" outputCode: " + String(outputCode));

//   return voltage;
// }

// float tareCell(int ainPos, int ainNeg)
// {
//   adc.stopADC1();
//   adc.setRate(ADS126X_RATE_2_5);
//   double offsetTotal = 0;
//   for (uint8_t i = 0; i < 5; i++)
//   {
//     offsetTotal += readAdcV(ainPos, ainNeg); // read the voltage
//     Serial.println(" Offset total: " + String(offsetTotal));
//     delay(100);
//   }
//   adcOffset = (double)round(offsetTotal / 5);

//   return adcOffset;
// }

// float vToN(float volts)
// {
//   const float newtonsPerMv = (cellFsr / cellMaxLoad) * numberOfCells;
//   float newtons = volts * newtonsPerMv;
//   return newtons;
// }

// float vToG(float volts)
// {
//   float newtons = vToN(volts);
//   float grams = newtons * 101.97162;
//   return grams;
// }
/**
 * Main Loop
 */
void loop()
{
  signed long int outputCode = 0;
  const int samples = 1;

  // const float newtonsPerMv = (cellFsr / cellMaxLoad) * numberOfCells;

  float weightG = comGndScale.readGrams();

  // ideal newtons per mv is 952 per cell (based on data sheet).
  // that can be divided by the number of cells to get the ideal conversion.

  //const float vAt10g = 0.0004182; // the voltage reading at 10g (excluding the offset)
  //const float newtonsPerMv = .0980665 / vAt10g;
  // float voltage = 0.0;
  // float newtons = 0.0;

  // float cell0V = readAdcV(adcAin2Pin, adcAin3Pin);
  // float cell0G = vToG(cell0V);
  // float cell1V = readAdcV(adcAin4Pin, adcAin5Pin);
  // float cell1G = vToG(cell1V);
  // float cell2V = readAdcV(adcAin6Pin, adcAin7Pin);
  // float cell2G = vToG(cell2V);
  // float cell3V = readAdcV(adcAin8Pin, adcAin9Pin);
  // float cell3G = vToG(cell3V);
  // Serial.println("0: " + String(cell0G, 5) + " 1: " + String(cell1G, 5) + " 2: " + String(cell2G, 5) + " 3: " + String(cell3G, 5));

  // wait for Data Ready Pin to go low
  // if (digitalRead(esp32AdcRdyPin) == LOW)

  // outputCode = adc.readADC1(adcAin2Pin, adcAin3Pin); // read the voltage
  // // lastADC1Status returns 1 if reading has returned new data (see data sheet 9.4.6)
  // if (adc.lastADC1Status())
  // {

  //   //outputCode -= adcOffset;

  //   voltage = (float)((adcResolution)*outputCode);

  //   voltageSmoother.add(voltage);
  //   float smoothVoltage = voltageSmoother.get();

  //   newtons = smoothVoltage * newtonsPerMv;
  //   grams = newtons * 101.97162;

  //   Serial.print("g: " + String(grams, 2) + " n: " + String(newtons, 6) + " v:" + String(smoothVoltage, 16) + " raw: " + String(outputCode) + " b: "); // send voltage through serial
  //   Serial.println(outputCode, BIN);
  // }
}
