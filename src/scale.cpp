#include <Arduino.h>
#include "scale.h"

Scale::Scale()
{
    // rdryPin = _rdryPin;
    // spiCsPin = _spiCsPin;
    // spiStartPin = _spiStartPin;
    loadCells[0].posPin = cell0AdcPosPin;
    loadCells[0].negPin = cell0AdcNegPin;
    loadCells[0].nPerMv = (cellFsr / cellMaxLoad) * numberOfCells;
    loadCells[1].posPin = cell1AdcPosPin;
    loadCells[1].negPin = cell1AdcNegPin;
    loadCells[1].nPerMv = (cellFsr / cellMaxLoad) * numberOfCells;
    loadCells[2].posPin = cell2AdcPosPin;
    loadCells[2].negPin = cell2AdcNegPin;
    loadCells[2].nPerMv = (cellFsr / cellMaxLoad) * numberOfCells;
    loadCells[3].posPin = cell3AdcPosPin;
    loadCells[3].negPin = cell3AdcNegPin;
    loadCells[3].nPerMv = (cellFsr / cellMaxLoad) * numberOfCells;
}

Scale::~Scale()
{
}

void Scale::begin()
{
    pinMode(rdryPin, INPUT_PULLUP); // adc data ready (RDRY) pin (active low)
    adc.begin(spiCsPin);
    adc.enableInternalReference();
    adc.setReference(ADS126X_REF_NEG_AIN1, ADS126X_REF_POS_AIN0);
    adc.setContinuousMode();
    adc.setFilter(ADS126X_SINC4);
    adc.setChopMode(ADS126X_CHOP_1);
    adc.setGain(ADS126X_GAIN_32);
    adc.enablePGA();
    delay(500);
    tareCell(&loadCells[0]);
    tareCell(&loadCells[1]);
    tareCell(&loadCells[2]);
    tareCell(&loadCells[3]);
    Serial.println("Offsets: " + String(loadCells[0].vOffset) + " , " + String(loadCells[1].vOffset) + " , " + String(loadCells[2].vOffset) + " , " + String(loadCells[3].vOffset));
    adc.setRate(ADS126X_RATE_400);
}

float Scale::readGrams()
{
    float avgG = 0;
    for (int i = 0; i < 4; i++)
    {
        updateLoadCellData(loadCells[i]);
    }
    for (int i = 0; i < 4; i++)
    {
        avgG += loadCells[i].g;
    }

    // the weight is the average of the 4 cells.
    avgG /= 4.0;

    Serial.println("g: " + String(avgG) + " : " + String(loadCells[0].g, 6) + " , " + String(loadCells[1].g, 6) + " , " + String(loadCells[2].g, 6) + " , " + String(loadCells[3].g, 6));

    return avgG;
}

void Scale::updateLoadCellData(loadCell &loadCellData)
{
    float v = readAdcV(loadCellData.posPin, loadCellData.negPin);
    float g = vToG(v, loadCellData);
    float n = vToN(v, loadCellData);
    loadCellData.v = v;
    loadCellData.g = g;
    loadCellData.n = n;
}

float Scale::readAdcV(int ainPos, int ainNeg)
{

    adc.REGISTER.INPMUX.bit.MUXN = ainNeg;
    adc.REGISTER.INPMUX.bit.MUXP = ainPos;
    adc.writeRegister(ADS126X_INPMUX); // replace on ads126x

    adc.startADC1();
    while (digitalRead(rdryPin) == HIGH)
    {
        // wait
    }
    adc.stopADC1();
    signed long int outputCode = 0;

    outputCode = adc.readADC1(ainPos, ainNeg); // read the voltage

    float voltage = (float)((adcResolution) * (double)(outputCode));
    // Serial.print(" outputCode: " + String(outputCode));

    return voltage;
}

void Scale::calCells()
{
    for (int i = 0; i < 4; i++)
    {
        calCell(&loadCells[i]);
    }
}

void Scale::calCell(loadCell *loadCellData)
{
    Serial.println(" Calibrating cell on pins: pos " + String(loadCellData->posPin) + ", neg " + String(loadCellData->negPin));
    adc.stopADC1();
    adc.setRate(ADS126X_RATE_2_5);
    double vTotal = 0;
    for (uint8_t i = 0; i < 2; i++)
    {
        float v = readAdcV(loadCellData->posPin, loadCellData->negPin); // read the voltage
        vTotal += v;
        Serial.println(" v: " + String(v, 6));
    }
    // this is max v using a 100g reference weight
    loadCellData->ref100gVMax = (float)(vTotal / 2.0) - loadCellData->vOffset;

    float mVPerG = 100.0 / loadCellData->ref100gVMax;
    // 100g in newtons = 100 * nPerG = .980665
    float mvPerN = (100.0 * nPerG) / loadCellData->ref100gVMax;
    // find the newtonsPerMv
    // 100g = 100.0 / 101.97162 newtons

    loadCellData->nPerMv = mvPerN;

    Serial.println(" v avg: " +
                   String(loadCellData->ref100gVMax, 6) +
                   " mVPerG: " + String(mVPerG, 6) +
                   " mvPerN: " + String(mvPerN, 6) +
                   " nPerMv: " + String(loadCellData->nPerMv, 6));

    adc.stopADC1();
    adc.setRate(ADS126X_RATE_400);
}

void Scale::tareCell(loadCell *loadCellData)
{
    adc.stopADC1();
    adc.setRate(ADS126X_RATE_2_5);
    double offsetTotal = 0;
    for (uint8_t i = 0; i < 2; i++)
    {
        float offset = readAdcV(loadCellData->posPin, loadCellData->negPin); // read the voltage
        offsetTotal += offset;
        Serial.println(" Offset: " + String(offset, 6));
    }
    loadCellData->vOffset = (float)(offsetTotal / 2.0);
    Serial.println(" Offset avg: " + String(loadCellData->vOffset, 6));
}

float Scale::vToN(float volts, loadCell loadCellData)
{
    // 57.128
    const float newtonsPerMv = (cellFsr / cellMaxLoad) * numberOfCells;
    // TODO use a calibrated conversion factor;
    float newtons = (volts - loadCellData.vOffset) * loadCellData.nPerMv;
    return newtons;
}

float Scale::vToG(float volts, loadCell cellConfig)
{
    float newtons = vToN(volts, cellConfig);
    float grams = newtons * gPerN;
    return grams;
}