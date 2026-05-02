/*!
	@file    I2C_Normal.ino
	@brief   arduino C++ bmp390 library test file, basic use, normal mode , I2C hardware
	@details bmp390 is a digital pressure sensor with temperature measurement capabilities.
           Connection ESP32 I2C SCLK = D22 , SDATA = D21
           Normal mode comprises an automated
           perpetual cycling between an active measurement period and an inactive standby period.
  @author  Gavin lyons at LionTron Systems
*/

#include "bmp390_ltsm.hpp"

// I2C setup
uint8_t  I2C_ADDRESS    =  0x76;  // I2C address: 0x76 SDO connected to GND, 0x77 SDO connected to VCC
uint32_t I2C_CLOCK_RATE =  25000; // I2C Clock rate in hertz
BMP390_Sensor bmp390(I2C_ADDRESS, &Wire, I2C_CLOCK_RATE);
// Local Pressure , Replace with today's QNH from forecast [hPa]
#define LOCAL_PRESSURE  1025.25   

void setup() {
  Serialinit();
	SensorInit();
}

void loop()
{
	PrintSensorInfo();
	Serial.println("-- END --");
	while (1){};
}

void PrintSensorInfo(void)
{
	uint16_t counter = 0;
	char SerialBuffer[100];
	while (counter < 30)
	{
		snprintf(SerialBuffer, sizeof(SerialBuffer), "Test Count  : %u", counter);
		Serial.println(SerialBuffer);
		// Test 1 Temperature
		snprintf(SerialBuffer, sizeof(SerialBuffer), "Temp OS     : %u",
			static_cast<uint8_t>(bmp390.readOversampling(BMP390_Sensor::DataType_e::Temperature)));
		Serial.println(SerialBuffer);
		snprintf(SerialBuffer, sizeof(SerialBuffer), "Temperature : %.2f [C]",
			bmp390.readTemperature());
		Serial.println(SerialBuffer);
		// Test 2 Pressure
		snprintf(SerialBuffer, sizeof(SerialBuffer), "Press OS    : %u",
			static_cast<uint8_t>(bmp390.readOversampling(BMP390_Sensor::DataType_e::Pressure)));
		Serial.println(SerialBuffer);
		snprintf(SerialBuffer, sizeof(SerialBuffer), "Pressure    : %.2f [hPa]",
			bmp390.readPressure(BMP390_Sensor::PressureUnit_e::hPa));
		Serial.println(SerialBuffer);
		// Test 3 Altitude
		snprintf(SerialBuffer, sizeof(SerialBuffer), "Altitude    : %.2f [m]",
			bmp390.readAltitude(LOCAL_PRESSURE));
		Serial.println(SerialBuffer);
		Serial.println();

		delay(5000);
		counter++;
	}
}

void SensorInit(void)
{
	Serial.println("--- START Normal I2C ---");
	while (!bmp390.InitSensor()) {
		delay(3000);
		Serial.println("- BMP390 sensor not connected  -");
	}
	// === optional I2C connection check ===
	// while (bmp390.CheckConnectionI2C() < 0) {
	// 	Serial.println("Failed to connect to BMP390 sensor");
	// 	delay(3000);
	// }
	uint8_t chipID = 0;
	chipID = bmp390.readForChipID();
	Serial.print("CHIP ID: ");
	Serial.println(chipID, HEX);  // 0x60 for BMP390
	delay(1000);
}

//Function to setup serial called from setup
void Serialinit() {
  Serial.begin(38400);
  delay(1000);
  Serial.println("--Comms UP BMP390--");
}