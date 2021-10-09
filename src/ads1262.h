/*
  ADS1262.h - Library for ADS1262 Shield Arduino Firmwar.
  Created by Protocentral, December 27, 2013.
  Released into the public domain.
*/

#ifndef ads1262_h
#define ads1262_h

#include "Arduino.h"

#define CONFIG_SPI_MASTER_DUMMY 0xFF

// Register Read Commands
#define RREG 0x20; //Read n nnnn registers starting at address r rrrr \
                   //first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)
#define WREG 0x40; //Write n nnnn registers starting at address r rrrr \
                   //first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)

#define START 0x08  //Start/restart (synchronize) conversions
#define STOP 0x0A   //Stop conversion
#define RDATAC 0x10 //Enable Read Data Conti6nuous mode.

//This mode is the default mode at power-up.
#define SDATAC 0x11 //Stop Read Data Continuously mode
#define RDATA 0x12  //Read data by command; supports multiple read back.

//Pin declartion the other you need are controlled by the SPI library
const int ADS1262_DRDY_PIN = 17;
const int ADS1262_CS_PIN = 5;
const int ADS1262_START_PIN = 23;
const int ADS1262_PWDN_PIN = 16;

//register address - see section 9.6 of the data sheet
#define POWER 0x01     // RESET, VBIAS, INTREF
#define INTERFACE 0x02 // TIMEOUT, STATUS, CRC
#define MODE0 0x03     // REFREV, RUNMODE, CHOP, DELAY,
#define MODE1 0x04     // FILTER, SBADC, SBPOL, SBMAG,
#define MODE2 0x05     // PGA BYPASS, GAIN, DR (data rate)
#define INPMUX 0x06    // MUXP, MUXN
#define OFCAL0 0x07
#define OFCAL1 0x08
#define OFCAL2 0x09
#define FSCAL0 0x0A
#define FSCAL1 0x0B
#define FSCAL2 0x0C
#define IDACMUX 0x0D
#define IDACMAG 0x0E
#define REFMUX 0x0F
#define TDACP 0x10
#define TDACN 0x11
#define GPIOCON 0x12
#define GPIODIR 0x13
#define GPIODAT 0x14
#define ADC2CFG 0x15
#define ADC2MUX 0x16
#define ADC2OFC0 0x17
#define ADC2OFC1 0x18
#define ADC2FSC0 0x19
#define ADC2FSC1 0x1A

// Config bits

// ADC1 Mode1 Register
// - ADC1 Digital Filter
#define ADC1_FILTER_SYNC1 0b00000000 // Sync1 Mode (default)
#define ADC1_FILTER_SYNC2 0b00100000 // Sync2 Mode
#define ADC1_FILTER_SYNC3 0b01000000 // Sync3 Mode
#define ADC1_FILTER_SYNC4 0b01100000 // Sync4 Mode
#define ADC1_FILTER_FIR 0b10000000   // FIR Mode
// - ADC1 Sensor Bias ADC Connect
#define ADC1_SBADC_ADC1 0b00000000 // Sensor bias connected to ADC1 mux out (default)
#define ADC1_SBADC_ADC2 0b00010000 // Sensor bias connected to ADC2 mux out
// - ADC1 Sensor Bias Polarity - Selects the sensor bias for pull-up or pull-down
#define ADC1_SBPOL_UP 0b00000000   // Sensor bias pull-up mode (AINP pulled high, AINN pulled low) (default)
#define ADC1_SBPOL_DOWN 0b00001000 // Sensor bias pull-up mode (AINP pulled high, AINN pulled low) (default)
// - ADC1 Sensor Bias Magnitude - Selects the sensor bias current magnitude or the bias resistor
#define ADC1_SBMAG_0 0b00000000   // No sensor bias current or resistor (default)
#define ADC1_SBMAG_0_5 0b00000001 // 0.5-μA sensor bias current
#define ADC1_SBMAG_2 0b00000010   // 2-μA sensor bias current
#define ADC1_SBMAG_10 0b00000011  // 10-μA sensor bias current
#define ADC1_SBMAG_50 0b00000100  // 50-μA sensor bias current
#define ADC1_SBMAG_200 0b00000101 // 200-μA sensor bias current
#define ADC1_SBMAG_RES 0b00000110 // 10M-Ohm Resistor

// ADC1 Mode2 Register (data sheet 9.6.6 p.92)
// - ADC1 PGA bypass Mode
#define ADC1_PGA_ENABLED 0b00000000  // PGA enabled (default)
#define ADC1_PGA_DISABLED 0b10000000 // PGA bypassed
// - ADC1 PGA Gain
#define ADC1_GAIN_1 0b00000000  // 1 V/V
#define ADC1_GAIN_2 0b00010000  // 2 V/V
#define ADC1_GAIN_4 0b00100000  // 4 V/V
#define ADC1_GAIN_8 0b00110000  // 8 V/V
#define ADC1_GAIN_16 0b01000000 // 16 V/V
#define ADC1_GAIN_32 0b01010000 // 32 V/V
// - ADC1 Data Rate (SPS)
#define ADC1_DR_2_5 0b00000000   // 2.5 SPS
#define ADC1_DR_5 0b00000001     // 5 SPS
#define ADC1_DR_10 0b00000010    // 10 SPS
#define ADC1_DR_16 0b00000011    // 16 SPS
#define ADC1_DR_20 0b00000100    // 20 SPS
#define ADC1_DR_50 0b00000101    // 50 SPS
#define ADC1_DR_60 0b00000110    // 60 SPS
#define ADC1_DR_100 0b00000111   // 100 SPS
#define ADC1_DR_400 0b00001000   // 400 SPS
#define ADC1_DR_1200 0b00001001  // 1200 SPS
#define ADC1_DR_2400 0b00001010  // 2400 SPS
#define ADC1_DR_4800 0b00001011  // 4800 SPS
#define ADC1_DR_7200 0b00001100  // 7200 SPS
#define ADC1_DR_14400 0b00001101 // 14400 SPS
#define ADC1_DR_19200 0b00001110 // 19200 SPS
#define ADC1_DR_38400 0b00001111 // 38400 SPS

// Input Multiplexer Register (INPMUX) (Data shet 9.6.7 p94)
// - Positive Input Multiplexer
#define INPMUXP_AIN0 0b00000000        // AINO (default)
#define INPMUXP_AIN1 0b00010000        // AIN1
#define INPMUXP_AIN2 0b00100000        // AIN2
#define INPMUXP_AIN3 0b00110000        // AIN3
#define INPMUXP_AIN4 0b01000000        // AIN4
#define INPMUXP_AIN5 0b01010000        // AIN5
#define INPMUXP_AIN6 0b01100000        // AIN6
#define INPMUXP_AIN7 0b01110000        // AIN7
#define INPMUXP_AIN8 0b10000000        // AIN8
#define INPMUXP_AIN9 0b10010000        // AIN9
#define INPMUXP_AINCOM 0b10100000      // AINCOM
#define INPMUXP_TEMP 0b10110000        // Temperature sensor monitor positive
#define INPMUXP_ANALOG_PWR 0b11000000  // Analog power supply monitor positive
#define INPMUXP_DIGITAL_PWR 0b11010000 // Digital power supply monitor positive
#define INPMUXP_TDAC 0b11100000        // TDAC test signal positive
#define INPMUXP_FLOAT 0b11110000       // Float (open connection)
// - Negative Input Multiplexer
#define INPMUXN_AIN0 0b00000000        // AINO (default)
#define INPMUXN_AIN1 0b00000001        // AIN1
#define INPMUXN_AIN2 0b00000010        // AIN2
#define INPMUXN_AIN3 0b00000011        // AIN3
#define INPMUXN_AIN4 0b00000100        // AIN4
#define INPMUXN_AIN5 0b00000101        // AIN5
#define INPMUXN_AIN6 0b00000110        // AIN6
#define INPMUXN_AIN7 0b00000111        // AIN7
#define INPMUXN_AIN8 0b00001000        // AIN8
#define INPMUXN_AIN9 0b00001001        // AIN9
#define INPMUXN_AINCOM 0b00001010      // AINCOM
#define INPMUXN_TEMP 0b00001011        // Temperature sensor monitor positive
#define INPMUXN_ANALOG_PWR 0b00001100  // Analog power supply monitor positive
#define INPMUXN_DIGITAL_PWR 0b00001101 // Digital power supply monitor positive
#define INPMUXN_TDAC 0b00001110        // TDAC test signal positive
#define INPMUXN_FLOAT 0b00001111       // Float (open connection)

// ADC2 Config Register -- only available on ADS1263
// - ADC2 Data Rate (SPS - Samples Per Second)
#define ADC2_DR_10 0b00000000  // 10 SPS
#define ADC2_DR_100 0b01000000 // 100 SPS
#define ADC2_DR_400 0b10000000 // 400 SPS
#define ADC2_DR_800 0b11000000 // 800 SPS
// - ADC2 Reference Input
#define ADC2_REF_2_5V 0b00000000     // Internal 2.5 V reference, positive and negative (default)
#define ADC2_REF_AIN_0_1 0b00001000  // External AIN0 and AIN1 pin pairs as positive and negative
#define ADC2_REF_AIN_2_3 0b00010000  // External AIN2 and AIN3 pin pairs as positive and negative
#define ADC2_REF_AIN_4_5 0b00011000  // External AIN4 and AIN5 pin pairs as positive andnegative
#define ADC2_REF_INTERNAL 0b00100000 // Internal VAVDD and VAVSS
// - ADC2 Gain
#define ADC2_GAIN2_1 0b00000000   // 1 V/V
#define ADC2_GAIN2_2 0b00000001   // 2 V/V
#define ADC2_GAIN2_4 0b00000010   // 4 V/V
#define ADC2_GAIN2_8 0b00000011   // 8 V/V
#define ADC2_GAIN2_16 0b00000100  // 16 V/V
#define ADC2_GAIN2_32 0b00000101  // 32 V/V
#define ADC2_GAIN2_64 0b00000110  // 64 V/V
#define ADC2_GAIN2_128 0b00000111 // 128 V/V

class ads1262
{
public:
  static void ads1262_Init(void);
  static void ads1262_Reset(void);
  static void ads1262_Reg_Write(unsigned char READ_WRITE_ADDRESS, unsigned char DATA);
  static void ads1262_Reg_Read(unsigned char READ_WRITE_ADDRESS);
  static void ads1262_SPI_Command_Data(unsigned char data_in);
  static void ads1262_Disable_Start(void);
  static void ads1262_Enable_Start(void);
  static void ads1262_Hard_Stop(void);
  static void ads1262_Start_Data_Conv_Command(void);
  static void ads1262_Soft_Stop(void);
  static void ads1262_Start_Read_Data_Continuous(void);
  static void ads1262_Stop_Read_Data_Continuous(void);
  static char *ads1262_Read_Data(void);
};

#endif
