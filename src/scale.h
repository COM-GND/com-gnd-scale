
#ifndef SCALE_H
#define SCALE_H
#include <ADS126X.h>
#include <Smoothed.h>

class Scale
{
public:
    ADS126X adc;

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
    int spiCsPin = 5;     // SPI CS pin
    int spiStartPin = 22; // SPI Start pin
    int rdryPin = 17;     // ADS126X RDY pin (14) -> ESP32 io pin (active low)
    float adcVRef = 3.86; // Reference Voltage
    float adcPga = 32.0;  // Gain
    // ADC fullscale range
    double adcFsr = (double)((double)adcVRef / (double)adcPga);

    const float gPerN = 101.97162;
    const float nPerG = 1.0 / gPerN;

    Smoothed<float> gramsSmoother;

    // const float g100InN =
    /**
    * HSFPAR303A force sensor 
    * Pin 1 vdd Exc + -> ADC AIN4 (REF+)
    * pin 2 v1  Out + -> ADC AIN0
    * pin 3 v2  Out - -> ADC AIN1
    * pin 4 gnd Exc - -> AD AIN5 (REF-)
    */

    const float numberOfCells = 1.0; // number of load cells wired in parallel per adc channel
    const float cellSensitivity = 3.7;
    const float cellMaxLoad = 8.0;
    const float totalMaxLoad = cellMaxLoad * numberOfCells;
    const float cellFsr = cellSensitivity * adcVRef * adcPga;

    double adcResolution = (double)((double)adcFsr / (double)(pow(2, 31)));

    const int adcPosRefPin = ADS126X_REF_POS_AIN0;
    const int adcNegRefPin = ADS126X_REF_NEG_AIN1;

    const int cell0AdcPosPin = 2; // ADS126X pin AIN2, positive input
    const int cell0AdcNegPin = 3; // ADS126X pin AIN3, negative input
    const int cell1AdcPosPin = 4; // ADS126X pin AIN4, positive input
    const int cell1AdcNegPin = 5; // ADS126X pin AIN5, negative input
    const int cell2AdcPosPin = 6; // ADS126X pin AIN4, positive input
    const int cell2AdcNegPin = 7; // ADS126X pin AIN5, negative input
    const int cell3AdcPosPin = 8; // ADS126X pin AIN4, positive input
    const int cell3AdcNegPin = 9; // ADS126X pin AIN5, negative input

    struct loadCell
    {
        int posPin;        // the cell's positive V pin number
        int negPin;        // the cell's negative V pin number
        float vOffset;     // the 0g offset volate
        float ref100gVMax; // the max V when calibrating with 100g reference weight
        float ref100gVMin; // the max V when calibrating with 100g reference weight
        float nPerMv;      // the calibrated newtons per mV
        double raw;        // the raw adc value
        float v;           // voltage
        float g;           // grams
        float n;           // newtons
    };
    const float defaultNPerMv = (cellFsr / cellMaxLoad) * numberOfCells;

    loadCell loadCells[4];

    Scale();
    ~Scale();
    void begin();
    float readGrams();
    void updateLoadCellData(loadCell &loadCellData, uint8_t samples);
    float readAdcV(uint8_t, uint8_t, uint8_t);
    void tareCell(loadCell *loadCellData);
    void resetCellsCal(); // clear calibration values
    void calCell(loadCell *loadCellData);
    void calCells();
    float vToN(float volts, loadCell loadCellData);
    float vToG(float volts, loadCell loadCellData);
};

#endif