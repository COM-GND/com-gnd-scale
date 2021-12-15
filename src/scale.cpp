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

    // // Reset FSCAL coefficients
    // adc.REGISTER.FSCAL0.bit.FSCAL = 0x00;
    // adc.REGISTER.FSCAL1.bit.FSCAL = 0x00;
    // adc.REGISTER.FSCAL2.bit.FSCAL = 0x00;
    // adc.writeRegister(ADS126X_FSCAL0);
    // adc.writeRegister(ADS126X_FSCAL1);
    // adc.writeRegister(ADS126X_FSCAL2);

    // // Reset OFCAL coefficients
    // adc.REGISTER.OFCAL0.bit.OFC = 0x00;
    // adc.REGISTER.OFCAL1.bit.OFC = 0x00;
    // adc.REGISTER.OFCAL2.bit.OFC = 0x00;
    // adc.writeRegister(ADS126X_OFCAL0);
    // adc.writeRegister(ADS126X_OFCAL1);
    // adc.writeRegister(ADS126X_OFCAL2);

    adc.setContinuousMode();
    adc.setFilter(ADS126X_SINC1);
    adc.setChopMode(ADS126X_CHOP_1);
    adc.setGain(ADS126X_GAIN_32);
    adc.enablePGA();
    delay(500);
    tareCell(&loadCells[0]);
    tareCell(&loadCells[1]);
    tareCell(&loadCells[2]);
    tareCell(&loadCells[3]);
    Serial.println(
        "Offsets: " + String(loadCells[0].vOffset) +
        " , " + String(loadCells[1].vOffset) +
        " , " + String(loadCells[2].vOffset) +
        " , " + String(loadCells[3].vOffset));
    adc.setRate(ADS126X_RATE_400);
    gramsSmoother.begin(SMOOTHED_AVERAGE, 10);
}

float Scale::readGrams()
{
    float avgG = 0;
    unsigned long readStartTimestamp = millis();
    for (int i = 0; i < 4; i++)
    {
        updateLoadCellData(loadCells[i], 2);
    }
    for (int i = 0; i < 4; i++)
    {
        avgG += loadCells[i].g;
    }

    float temp = readTemperature();

    // the weight is the average of the 4 cells.
    avgG /= 4.0;

    unsigned long readEndTimestamp = millis();
    unsigned long readTime = readEndTimestamp - readStartTimestamp;

    // estimate SNR
    gramsSmoother.add(avgG);
    float smoothedG = gramsSmoother.get();
    float noise = abs(smoothedG - avgG);
    float snr = log10f(avgG) - log10f(noise);

    Serial.println("t: " + String(readTime) +
                   " c " + String(temp, 2) +
                   " sps " + String(1000.0 / readTime) +
                   " g: " + String(avgG) +
                   " avg: " + String(smoothedG) +
                   " noise: " + String(noise, 4) +
                   " snr: " + String(snr, 4) +
                   " : " + String(loadCells[0].g, 6) +
                   " , " + String(loadCells[1].g, 6) +
                   " , " + String(loadCells[2].g, 6) +
                   " , " + String(loadCells[3].g, 6));

    return avgG;
}

void Scale::updateLoadCellData(loadCell &loadCellData, uint8_t samples = 1)
{
    float v = readAdcV(loadCellData.posPin, loadCellData.negPin, samples);
    float g = vToG(v, loadCellData);
    float n = vToN(v, loadCellData);
    loadCellData.v = v;
    loadCellData.g = g;
    loadCellData.n = n;
}

float Scale::readAdcV(uint8_t ainPos, uint8_t ainNeg, uint8_t samples = 1)
{
    // Preset the pin registers while adc is stopped.
    // This a patch fro ads126x library to preven it from changing registers
    // while the adc is started.
    adc.REGISTER.INPMUX.bit.MUXN = ainNeg;
    adc.REGISTER.INPMUX.bit.MUXP = ainPos;
    adc.writeRegister(ADS126X_INPMUX); // replace on ads126x
    signed long int outputCode = 0;
    adc.startADC1();
    for (int i = 0; i < samples; i++)
    {
        while (digitalRead(rdryPin) == HIGH)
        {
            // wait
        }
        outputCode += adc.readADC1(ainPos, ainNeg); // read the voltage
    }

    adc.stopADC1();

    outputCode = (long int)(round((float)(outputCode) / (float)samples));
    float voltage = (float)((adcResolution) * (double)(outputCode));
    // Serial.print(" outputCode: " + String(outputCode));

    return voltage;
}

void Scale::resetCellsCal()
{
    for (int i = 0; i < 4; i++)
    {
        loadCells[i].ref100gVMax = 0;
        loadCells[i].ref100gVMin = 0;
    }
}
void Scale::calCells()
{
    for (int i = 0; i < 4; i++)
    {
        calCell(&loadCells[i]);
    }
}

/**
 * Each cell requires calibration. The 100g reference Reading of each cell changes
 * according to the position of the weight relative to the cell. Multiple
 * readings at different weight positions are needed to try to capture the absolute
 * max and min ref values. 
 */
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

    // this is the v using a 100g reference weight
    float ref100g = (float)(vTotal / 2.0) - loadCellData->vOffset;

    // update cells ref 100 max reading
    if (ref100g > loadCellData->ref100gVMax)
    {
        loadCellData->ref100gVMax = ref100g;
    }

    // update cells ref 100 min reading
    if (loadCellData->ref100gVMin == 0 || ref100g < loadCellData->ref100gVMin)
    {
        loadCellData->ref100gVMin = ref100g;
    }

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

void Scale::tare()
{
    tareCell(&loadCells[0]);
    tareCell(&loadCells[1]);
    tareCell(&loadCells[2]);
    tareCell(&loadCells[3]);
}
void Scale::tareCell(loadCell *loadCellData)
{
    adc.stopADC1();
    adc.setRate(ADS126X_RATE_2_5);
    double offsetTotal = 0;
    uint8_t samples = 1;

    for (uint8_t i = 0; i < samples; i++)
    {
        float offset = readAdcV(loadCellData->posPin, loadCellData->negPin); // read the voltage
        offsetTotal += offset;
        Serial.println(" Offset: " + String(offset, 6));
    }
    loadCellData->vOffset = (float)(offsetTotal / (float)samples);
    Serial.println(" Offset avg: " + String(loadCellData->vOffset, 6));
}

float Scale::readTemperature()
{
    // prepare registers to read temperature - see data sheet 9.3.4
    // disable chop and set gain to 1;
    adc.setChopMode(ADS126X_CHOP_0);
    adc.setGain(ADS126X_GAIN_1);

    // select temperature input mux
    adc.REGISTER.INPMUX.bit.MUXN = ADS126X_TEMP;
    adc.REGISTER.INPMUX.bit.MUXP = ADS126X_TEMP;
    adc.writeRegister(ADS126X_INPMUX); // replace on ads126x

    // select internal ref
    adc.setReference(ADS126X_REF_NEG_INT, ADS126X_REF_POS_INT);

    adc.writeRegister(ADS126X_INPMUX); // replace on ads126x

    adc.startADC1();
    while (digitalRead(rdryPin) == HIGH)
    {
        // wait
    }
    signed long int outputCode = adc.readADC1(ADS126X_TEMP, ADS126X_TEMP);

    adc.stopADC1();
    // convert to celsius - see data sheet 9.3.4 (Equation 9)
    // float celsius = (float)(((double)outputCode - 122400.0) / 420.0) + 25.0;
    double tempReading = (double)25 + ((((double)outputCode * (double)0.00116415321) - (double)122400) / (double)420);

    // Serial.println("C: " + String(celsius, 4) + " raw: " + String(outputCode));

    // return to settings for reading adc inputs
    adc.setChopMode(ADS126X_CHOP_1);
    adc.setGain(ADS126X_GAIN_32);
    adc.setReference(ADS126X_REF_NEG_AIN1, ADS126X_REF_POS_AIN0);

    return (float)tempReading;
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