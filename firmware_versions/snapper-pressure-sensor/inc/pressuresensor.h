/****************************************************************************
 * pressuresensor.h
 * SnapperGPS
 * Jonas Beuchert
 * June 2023
 * Functions to use an MS5837-30BA a pressure and temperature sensor via I2C
 *****************************************************************************/

#ifndef __PRESSURESENSOR_H
#define __PRESSURESENSOR_H


uint8_t Accelerometer_whoAmI();

bool Accelerometer_isNewDataAvailable();

void Accelerometer_readXYZ(int16_t *x, int16_t *y, int16_t *z);

void Accelerometer_selectDataRate(uint16_t dataRate);

void Accelerometer_enableLowPowerMode();

void Accelerometer_disableLowPowerMode();

void Accelerometer_selectFifoModeBypass();

void Accelerometer_selectFifoModeFifo();

void Accelerometer_selectFifoModeStream();

void Accelerometer_selectFifoModeStreamToFifo();

void Accelerometer_enableHighResolutionOutputMode();

void Accelerometer_disableHighResolutionOutputMode();

void Accelerometer_enableBlockDataUpdate();

void Accelerometer_disableBlockDataUpdate();

void Accelerometer_selectScale(uint8_t g);

void Accelerometer_readCtrlReg(uint8_t *ctrlReg);

// Enable I2C for LIS3DH
void Accelerometer_enableInterface();

// Disable I2C for LIS3DH
void Accelerometer_disableInterface();

bool Accelerometer_isAvailable();

#endif /* __PRESSURESENSOR_H */
