/*
  ADS1262.h - Library for ADS1262 Shield Arduino Firmwar.
  Created by Protocentral, December 27, 2013.
  Released into the public domain.
*/

#include <Arduino.h>
#include <SPI.h>
#include "ads1262.h"

SPIClass vspi = SPIClass(VSPI);

char *ads1262::ads1262_Read_Data()
{

    static char SPI_Dummy_Buff[6];

    digitalWrite(ADS1262_CS_PIN, LOW);

    for (int i = 0; i < 6; ++i)
    {
        SPI_Dummy_Buff[i] = vspi.transfer(CONFIG_SPI_MASTER_DUMMY);
    }

    digitalWrite(ADS1262_CS_PIN, HIGH);

    return SPI_Dummy_Buff;
}

void ads1262::ads1262_Init()
{

    // start the SPI library:
    vspi.begin();
    vspi.setBitOrder(MSBFIRST);
    //CPOL = 0, CPHA = 1
    vspi.setDataMode(SPI_MODE1);
    // Selecting 1Mhz clock for SPI
    vspi.setClockDivider(SPI_CLOCK_DIV8); // DIV16

    ads1262_Reset();
    delay(100);
    // ads1262_Disable_Start();
    // ads1262_Enable_Start();

    ads1262_Hard_Stop();
    // ads1262_Start_Data_Conv_Command();
    // ads1262_Soft_Stop();
    delay(50);
    //  ads1262_Stop_Read_Data_Continuous();					// SDATAC command
    delay(300);

    /**
     * Setup Register settings - see S 9.6 (p88) in spec sheet
     */

    ads1262_Reg_Write(POWER, 0x11); // Reset default, Level shift V Disabled, Internal Ref Enabled
    delay(10);
    ads1262_Reg_Write(INTERFACE, 0x05); // Serial Timout Disabled, Status Byte Enabled, Checksum byte enabled in 'checksum mode' during conversion data read-back
    delay(10);
    ads1262_Reg_Write(MODE0, 0x00); // Normal Mux Polarity, Continuous ADC conversion, Input chop and IDAC rotation disabled, no conversion delay
    delay(10);
    ads1262_Reg_Write(MODE1, 0x80); // FIR Filter, Sensor Bias on to ADC1, Sensor Bias Pullup mode, No Sensor Bias
    delay(10);
    ads1262_Reg_Write(MODE2, ADC1_PGA_ENABLED | ADC1_GAIN_32 | ADC1_DR_1200); // PGA Enabled, PGA Gain, Date Rate (SPS)
    //ads1262_Reg_Write(MODE2, 0x06); // PGA Enabled, 1V/V gain, 60 SPS
    delay(10);
    ads1262_Reg_Write(INPMUX, INPMUXP_AIN0 | INPMUXN_AIN1); // AIN0 Positive Input Multiplexer, AIN1 Negative Input Multiplexer,
    delay(10);
    ads1262_Reg_Write(OFCAL0, 0x00); // 0 Offset Calibration
    delay(10);
    ads1262_Reg_Write(OFCAL1, 0x00); // 0 Offset Calibration
    delay(10);
    ads1262_Reg_Write(OFCAL2, 0x00); // 0 Offset Calibration
    delay(10);
    ads1262_Reg_Write(FSCAL0, 0x00); // 0 Fullscale Calibration
    delay(10);
    ads1262_Reg_Write(FSCAL1, 0x00); // 0 Fullscale Calibration
    delay(10);
    ads1262_Reg_Write(FSCAL2, 0x40); // 0x40 Fullscale Calibration
    delay(10);

    // ads1262_Reg_Write(IDACMUX, 0xBB); // IDAC2 Output Multiplexer No Connection, IDAC1 Output Multiplexer No Connection
    // delay(10);
    // ads1262_Reg_Write(IDACMAG, 0x00); // IDAC2 Current Magnitude off, IDAC1 Current Magnitude off
    // delay(10);
    // ads1262_Reg_Write(REFMUX, 0x00); // Ref positive Input: 2.5 Internval V ref, Ref negative Input: 2.5 Internval V ref
    delay(10);
    ads1262_Reg_Write(TDACP, 0x00); // TDACP: no connection, MAGP Output: unset
    delay(10);
    ads1262_Reg_Write(TDACN, 0x00); // TDACN Output: No connection, MAGN Output: unser
    delay(10);
    ads1262_Reg_Write(GPIOCON, 0x00); // All GPIO: not connected
    delay(10);
    ads1262_Reg_Write(GPIODIR, 0x00); // All GPIO Dir: output
    delay(10);
    ads1262_Reg_Write(GPIODAT, 0x00); // All GPIO (read mode, defaults)
    delay(10);

    //adc2 is only available on ADS1263

    ads1262_Reg_Write(ADC2CFG, ADC2_GAIN2_16); // ADC2 Data Rate: 10 SPS, ADC2 Ref input: Internal 2.5V ref, ADC2 Gain: 16 V/V
    ads1262_Reg_Write(ADC2CFG, 0x00);          // ADC2 Data Rate: 10 SPS, ADC2 Ref input: Internal 2.5V ref, ADC2 Gain: 1 V/V
    delay(10);
    ads1262_Reg_Write(ADC2MUX, 0x01); // ADC2 Pos Input Mx: AIN0, ADC2 Neg, Input Mx: AIN1
    delay(10);
    ads1262_Reg_Write(ADC2OFC0, 0x00); // ADC2 0 Offset Calibration
    delay(10);
    ads1262_Reg_Write(ADC2OFC1, 0x00); // ADC2 0 Offset Calibration
    delay(10);
    ads1262_Reg_Write(ADC2FSC0, 0x00); // ADC2 0 Fullscal Calibration
    delay(10);
    ads1262_Reg_Write(ADC2FSC1, 0x40); // ADC2 0x40 Fullscal Calibration
    delay(10);

    ads1262_Start_Read_Data_Continuous();
    delay(10);
    ads1262_Enable_Start();
}

void ads1262::ads1262_Reset()
{
    digitalWrite(ADS1262_PWDN_PIN, HIGH);
    delay(100); // Wait 100 mSec
    digitalWrite(ADS1262_PWDN_PIN, LOW);
    delay(100);
    digitalWrite(ADS1262_PWDN_PIN, HIGH);
    delay(100);
}

void ads1262::ads1262_Disable_Start()
{
    digitalWrite(ADS1262_START_PIN, LOW);
    delay(20);
}

void ads1262::ads1262_Enable_Start()
{
    digitalWrite(ADS1262_START_PIN, HIGH);
    delay(20);
}

void ads1262::ads1262_Hard_Stop(void)
{
    digitalWrite(ADS1262_START_PIN, LOW);
    delay(100);
}

void ads1262::ads1262_Start_Data_Conv_Command(void)
{
    ads1262_SPI_Command_Data(START); // Send 0x08 to the ADS1x9x
}

void ads1262::ads1262_Soft_Stop(void)
{
    ads1262_SPI_Command_Data(STOP); // Send 0x0A to the ADS1x9x
}

void ads1262::ads1262_Start_Read_Data_Continuous(void)
{
    //ads1262_SPI_Command_Data(RDATAC);					// Send 0x10 to the ADS1x9x
}

void ads1262::ads1262_Stop_Read_Data_Continuous(void)
{
    //ads1262_SPI_Command_Data(SDATAC);					// Send 0x11 to the ADS1x9x
}

void ads1262::ads1262_SPI_Command_Data(unsigned char data_in)
{
    byte data[1];
    //data[0] = data_in;
    digitalWrite(ADS1262_CS_PIN, LOW);
    delay(2);
    digitalWrite(ADS1262_CS_PIN, HIGH);
    delay(2);
    digitalWrite(ADS1262_CS_PIN, LOW);
    delay(2);
    vspi.transfer(data_in);
    delay(2);
    digitalWrite(ADS1262_CS_PIN, HIGH);
}

//Sends a write command to SCP1000
void ads1262::ads1262_Reg_Write(unsigned char READ_WRITE_ADDRESS, unsigned char DATA)
{
    Serial.print("Write ADDR: ");
    Serial.print(READ_WRITE_ADDRESS, BIN);
    Serial.print(" DATA: ");
    Serial.print(DATA, BIN);
    Serial.println("");

    // now combine the register address and the command into one byte:
    byte dataToSend = READ_WRITE_ADDRESS | WREG;

    digitalWrite(ADS1262_CS_PIN, LOW);
    delay(2);
    digitalWrite(ADS1262_CS_PIN, HIGH);
    delay(2);
    // take the chip select low to select the device:
    digitalWrite(ADS1262_CS_PIN, LOW);
    delay(2);
    vspi.transfer(dataToSend); //Send register location
    vspi.transfer(0x00);       //number of registers to write. 0 = one register
    vspi.transfer(DATA);       //Send value to record into register

    delay(2);
    // take the chip select high to de-select:
    digitalWrite(ADS1262_CS_PIN, HIGH);

    // confirm write
    ads1262_Reg_Read(READ_WRITE_ADDRESS);
}

void ads1262::ads1262_Reg_Read(unsigned char READ_WRITE_ADDRESS)
{

    // now combine the register address and the command into one byte:
    byte dataToSend = READ_WRITE_ADDRESS | RREG;

    digitalWrite(ADS1262_CS_PIN, LOW);
    delay(2);
    digitalWrite(ADS1262_CS_PIN, HIGH);
    delay(2);
    // take the chip select low to select the device:
    digitalWrite(ADS1262_CS_PIN, LOW);
    vspi.transfer(dataToSend);                               // register address
    vspi.transfer(0x00);                                     //number of registers to read. 0 = one register
    uint8_t regVal = vspi.transfer(CONFIG_SPI_MASTER_DUMMY); //number of register to wr

    Serial.print("Read Reg: ");
    Serial.print(READ_WRITE_ADDRESS, BIN);
    Serial.print(": ");
    Serial.print(regVal, BIN);
    Serial.println("");

    delay(2);
    // take the chip select high to de-select:
    digitalWrite(ADS1262_CS_PIN, HIGH);
}