#include "LED.h"

VL53L0X sensor;
VL53L0X sensor2;
uint16_t range1 = 0;
uint16_t range2 = 0;
int LED1 = D3;
int LED2 = D4;

void setupLED(){
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
	digitalWrite(LED1, LOW);
	digitalWrite(LED2, LOW);
	
	digitalWrite(LED1, HIGH);
	sensor.init(true);
	delay(100);
	sensor.setAddress((uint8_t)22);
	
	digitalWrite(LED2, HIGH);
	sensor2.init(true);
	delay(100);
	sensor2.setAddress((uint8_t)25);
	
	Serial.println ("I2C scanner. Scanning ...");
	byte counter = 0;

	for (byte i = 1; i < 120; i++)
	{
		Wire.beginTransmission (i);
		if (Wire.endTransmission () == 0)
		{
			Serial.print ("Found address: ");
			Serial.print (i, DEC);
			Serial.print (" (0x");
			Serial.print (i, HEX);
			Serial.println (")");
			counter++;
			delay (1);  // maybe unneeded?
		} // end of good response
	} // end of for loop
	Serial.println ("Done.");
	Serial.print ("Found ");
	Serial.print (counter, DEC);
	Serial.println (" device(s).");

	delay(2000);
}

void updateLED(){
	range1 = sensor.readRangeSingleMillimeters();
	if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

	range2 = sensor2.readRangeSingleMillimeters();
	if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
}

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
	// Set register address
	Wire.beginTransmission(Address);
	Wire.write(Register);
	Wire.endTransmission();

	// Read Nbytes
	Wire.requestFrom(Address, Nbytes); 
	uint8_t index=0;
	while (Wire.available())
	Data[index++]=Wire.read();
}

void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
	// Set register address
	Wire.beginTransmission(Address);
	Wire.write(Register);
	Wire.write(Data);
	Wire.endTransmission();
}
