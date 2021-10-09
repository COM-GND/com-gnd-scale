#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// https://github.com/nkolban/ESP32_BLE_Arduino
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "ads1262.h"

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

// ADC Configs
#define PGA 1
#define VREF 2.50 // internal voltage reference
#define VFSR VREF / PGA
#define FSR (((long int)1 << 23) - 1)

ads1262 PC_ADS1262; // class

// HSFPAR303A force sensor
/*
Pin 1 vdd
pin 2 v1
pin 3 v2
pin 4 gnd
*/

float volt_V = 0;
float volt_mV = 0;
volatile int i;
volatile char SPI_RX_Buff[10];
volatile long ads1262_rx_Data[10];
volatile static int SPI_RX_Buff_Count = 0;
volatile char *SPI_RX_Buff_Ptr;
volatile int Responsebyte = false;
volatile signed long sads1262Count = 0;
volatile signed long uads1262Count = 0;
double resolution;

float readADC(byte inpsel);

/** 
 * Pins 
 * set-up for ESP32 Devkit-c
 * see: https://circuits4you.com/2018/12/31/esp32-wroom32-devkit-analog-read-example/)
 */

// I2C bus
// SDA / SCL Require Pull-up resistors to VDD
// Typical pullup value is 4.7k for 5V
// note that the 5V <-> 3.3v level shifter has 10K pull-up resistor installed
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html
// https://learn.sparkfun.com/tutorials/bi-directional-logic-level-converter-hookup-guide
// https://www.arduino.cc/en/reference/wire

const unsigned char i2cSda = 23; // i2c data line
const unsigned char i2cScl = 22; // i2c clock line

/**
 * I2C Globals
 */
TwoWire I2C = TwoWire(0);

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

float readADC(byte inpsel)
{
  volatile int i, data;

  //PC_ADS1262.ads1262_Reg_Write(INPMUX, inpsel); //Ch 1 enabled, gain 6, connected to electrode in
  //Serial.println(INPMUXP_AIN2 | INPMUXN_AIN3, BIN);
  // PC_ADS1262.ads1262_Reg_Write(INPMUX, 0);
  //delay(100);

  while (1)
  {
    Serial.println("Reading");
    if ((digitalRead(ADS1262_DRDY_PIN)) == LOW) // monitor Data ready(DRDY pin)
    {
      SPI_RX_Buff_Ptr = PC_ADS1262.ads1262_Read_Data(); // read 6 bytes conversion register
      Responsebyte = true;
    }

    if (Responsebyte == true)
    {
      // Serial.println("Raw Data");
      for (i = 0; i < 5; i++)
      {
        SPI_RX_Buff[SPI_RX_Buff_Count] = *(SPI_RX_Buff_Ptr + i);
        // Serial.println((unsigned char)SPI_RX_Buff[SPI_RX_Buff_Count]);
        SPI_RX_Buff_Count++;
      }
      Responsebyte = false;
    }

    if (SPI_RX_Buff_Count >= 5)
    {
      ads1262_rx_Data[0] = (unsigned char)SPI_RX_Buff[1]; // read 4 bytes adc count
      ads1262_rx_Data[1] = (unsigned char)SPI_RX_Buff[2];
      ads1262_rx_Data[2] = (unsigned char)SPI_RX_Buff[3];
      ads1262_rx_Data[3] = (unsigned char)SPI_RX_Buff[4];
      uads1262Count = (signed long)(((unsigned long)ads1262_rx_Data[0] << 24) | ((unsigned long)ads1262_rx_Data[1] << 16) | (ads1262_rx_Data[2] << 8) | ads1262_rx_Data[3]); //get the raw 32-bit adc count out by shifting
      sads1262Count = (signed long)(uads1262Count);                                                                                                                          // get signed value
      resolution = (double)((double)VREF / pow(2, 31));                                                                                                                      //resolution= Vref/(2^n-1) , Vref=2.5, n=no of bits
      // Serial.print(resolution, 15);
      volt_V = (resolution) * (float)sads1262Count; // voltage = resolution * adc count
      volt_mV = volt_V * 1000;                      // voltage in mV
      SPI_RX_Buff_Count = 0;
      return volt_mV;
    }
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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
  // void onWrite(BLECharacteristic *pCharacteristic)
  // {
  //   std::string value = pCharacteristic->getValue();

  //   if (value.length() > 0)
  //   {
  //     Serial.print("received value: ");
  //     for (int i = 0; i < value.length(); i++)
  //       Serial.print(value[i]);

  //     Serial.println();
  //   }
  // pCharacteristic->setValue("received " + value);
  //}
};

void bleNotifyTask(void *params)
{
  for (;;)
  {
    // if (blePressureSensorNotifyFlag)
    // {
    //   pPressureSensorBLEChar->notify();
    //   blePressureSensorNotifyFlag = false;
    // }

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
  I2C.begin(i2cSda, i2cScl, 100000);

  BLEDevice::init("COM-GND Scale");
  Serial.println("BLE Device Initialized");
  pServer = BLEDevice::createServer();
  Serial.println("BLE Server Initialized");
  BLEService *pService = pServer->createService(SERVICE_UUID);
  Serial.println("BLE Service Initialized");
  pServer->setCallbacks(new ComGndServerCallbacks());
  Serial.println("BLE Server Callback Initialized");

  // https://www.arduino.cc/en/Reference/ArduinoBLEBLECharacteristicBLECharacteristic
  // pPressureSensorBLEChar = pService->createCharacteristic(
  //     PRESSURE_SENSOR_CHAR_ID,
  //     BLECharacteristic::PROPERTY_READ |
  //         BLECharacteristic::PROPERTY_WRITE |
  //         BLECharacteristic::PROPERTY_NOTIFY |
  //         BLECharacteristic::PROPERTY_INDICATE);
  // Serial.println("BLE Pressure Sensor Characteristic Created");
  // // pPressureSensorBLEChar->setCallbacks(new PressureSensorBLECharCallbacks());
  // Serial.println("BLE Pressure Sensor Characteristic Callback Initialized");
  // pPressureSensorBLEChar->addDescriptor(new BLE2902());

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

  pinMode(8, OUTPUT);
  Serial.begin(115200);
  digitalWrite(8, HIGH);

  // initalize the  data ready and chip select pins:
  pinMode(ADS1262_DRDY_PIN, INPUT);   //data ready input line
  pinMode(ADS1262_CS_PIN, OUTPUT);    //chip enable output line
  pinMode(ADS1262_START_PIN, OUTPUT); // start
  pinMode(ADS1262_PWDN_PIN, OUTPUT);  // Power down output

  digitalWrite(8, LOW);
  PC_ADS1262.ads1262_Init(); // initialise ads1262

  //xTaskCreate(bleNotifyTask, "bleNotify", 5000, NULL, 1, NULL);
}

/**
 * Main Loop
 */
void loop()
{
  float pwr;
  byte channel;
  channel = 1;
  //pwr=abs(readADC(0x0a+channel*16));
  pwr = readADC(0 + channel * 16);
  Serial.print("Data: ");
  Serial.printf("%0.6f", pwr);
  Serial.println(" mV");
  delay(200);
}
