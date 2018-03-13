#ifndef LED_H
#define LED_H

#include <VL53L0X.h>
#include <Wire.h>
#include <Arduino.h>

extern VL53L0X sensor;
extern VL53L0X sensor2;
extern uint16_t range1;
extern uint16_t range2;


void setupLED();
void updateLED();
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);

#endif
