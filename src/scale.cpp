#include <Arduino.h>
#include "scale.h"

Scale::Scale()
{
    // rdryPin = _rdryPin;
    // spiCsPin = _spiCsPin;
    // spiStartPin = _spiStartPin;
    loadCells[0].posPin = cell0AdcPosPin;
    loadCells[0].negPin = cell0AdcNegPin;
    loadCells[1].posPin = cell1AdcPosPin;
    loadCells[1].negPin = cell1AdcNegPin;
    loadCells[2].posPin = cell2AdcPosPin;
    loadCells[2].negPin = cell2AdcNegPin;
    loadCells[3].posPin = cell3AdcPosPin;
    loadCells[3].negPin = cell3AdcNegPin;
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

    // avgG /= 4;

    Serial.println("g: " + String(loadCells[0].g, 6) + " , " + String(loadCells[1].g, 6) + " , " + String(loadCells[2].g, 6) + " , " + String(loadCells[3].g, 6));

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

float Scale::tareCell(loadCell *loadCellData)
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
    const float newtonsPerMv = (cellFsr / cellMaxLoad) * numberOfCells;
    // TODO use a calibrated conversion factor;
    float newtons = (volts - loadCellData.vOffset) * newtonsPerMv;
    return newtons;
}

float Scale::vToG(float volts, loadCell cellConfig)
{
    float newtons = vToN(volts, cellConfig);
    float grams = newtons * 101.97162;
    return grams;
}