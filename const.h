/* const.h
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */
 
#ifndef CONST_H
#define CONST_H

//I2C addresses
const uint8_t ACCEL_I2C_ADDRESS = (0x1D) << 1; //accelerometer
const uint8_t MAG_I2C_ADDRESS = (0x0E) << 1; //magnetometer
const uint8_t PWR_SENS_SUPPLY_I2C_ADDRESS = (0x44) << 1; // power supply power sensor
const uint8_t PWR_SENS_BATTERY_I2C_ADDRESS = (0x40) << 1; //battery power sensor
const uint8_t PWR_SENS_LOAD_I2C_ADDRESS = (0x41) << 1; //load power sensor
const uint8_t FUEL_GAUGE_I2C_ADDRESS = 0x6C; //battery fuel gauge sensor


/* Serial transmission data format
 * 
 * [HEADER BYTE][DATA_0 BYTE (MOST SIGNIFICANT][DATA_1 BYTE (LEAST SIGNIFICANT)]
 * If a sensor only has 8-bit data, the most significant byte is used, while the least significant byte is 0.
 * All sensors send data in the 3-byte cadence.
 * For example, to send accelerometer data:
 * 0x 10 XX XX 11 YY YY 12 ZZ ZZ
 * To send ADC_0:
 * 0x 50 DD
 * To indicate critical error:
 * 0x FF FF FF (last 5 bytes are don't care)
 */
 
/* Serial reception data format
 * [SENSOR_CHAR_CODE][ENABLE_CHAR_CODE][FLOATING_SAMPLE_PERIOD_CHAR_CODE]
 * For example, to enable accelerometer sampling at 50 Hz:
 * ae0.02
 * To enable ADC0 at 300 Hz:
 * 0e0.00333
 * To disable ADC0:
 * 0d<DON'T CARE, MUST BE NON-NEGATIVE FLOAT>
 */
 
 

/* Header codes
 * 
 * HEADER[7:0]
 * 
 * HEADER[7:4] -- sensor ID
 * HEADER[3:0] -- sensor channel (if applicable)
 * 
 * Sensor IDs
 * 0x0 -- b0000 -- Reserved
 * 0x1 -- b0001 -- Accelerometer
 * 0x2 -- b0010 -- Magnetometer
 * 0x3 -- b0011 -- Light sensor
 * 0x4 -- b0100 -- Touch sensor
 * 0x5 -- b0101 -- ADC
 * 0x6 -- b0110 -- Reserved
 * 0x7 -- b0111 -- Reserved
 * 0x8 -- b1000 -- Reserved
 * 0x9 -- b1001 -- Reserved
 * 0xA -- b1010 -- Reserved
 * 0xB -- b1011 -- Reserved
 * 0xC -- b1100 -- Reserved
 * 0xD -- b1101 -- Reserved
 * 0xE -- b1110 -- Reserved
 * 0xF -- b1111 -- Critical Error Condition
 *
 * Sensor channels
 *
 * 0x0 -- b0000 -- Default or X or 0
 * 0x1 -- b0001 -- Y or 1 (if applicable)
 * 0x2 -- b0010 -- Z or 2 (if applicable)
 * 0x3 -- b0011 -- 3 (if applicable)
 * 0x4 -- b0100 -- 4 (if applicable)
 * 0x5 -- b0101 -- 5 (if applicable)
 * 0x6 -- b0110 -- 6 (if applicable)
 */
const uint8_t ACCEL_X = 0x10;
const uint8_t ACCEL_Y = 0x11;
const uint8_t ACCEL_Z = 0x12;
const uint8_t MAG_X = 0x20;
const uint8_t MAG_Y = 0x21;
const uint8_t MAG_Z = 0x22;
const uint8_t LIGHT = 0x30;
const uint8_t TOUCH = 0x40;
const uint8_t ADC_0 = 0x50;
//const uint8_t CRITICAL_ERROR = 0xFF;


#endif