/*!
	@file    SPI_Normal.ino
	@brief   arduino C++ bmp390 library test file, basic use, normal mode , SPI hardware
	@details bmp390 is a digital pressure sensor with temperature measurement capabilities.
           Connection for example ESP32: SCK-18 MOSI-23 MISO-19 CS-15(any gpio you want) INT-(n/a this example)
           Normal mode comprises an automated
           perpetual cycling between an active measurement period and an inactive standby period.
  @author  Gavin lyons at LionTron Systems
*/

#include "bmp390_ltsm.hpp"

// SPI 
uint8_t chipSelectPin = 15;
uint32_t SPIBaudRate = 50000;
// Sensor instance
BMP390_Sensor bmp390(chipSelectPin, SPIBaudRate);
// Local Pressure , Replace with today's QNH from forecast [hPa]
#define LOCAL_PRESSURE  1025.25   

void setup() {
	Serialinit();
	InitSensor();
	PrintChipID();
}

void loop()
{
	PrintSensorInfo();
	Serial.println("--- END ---");
	while (1){}; // halt 
}

//Function to setup serial called from setup
void Serialinit() {
  Serial.begin(38400);
  delay(1000);
  Serial.println("--Comms UP BMP390--");
}

// Function to print start Message.
void InitSensor(void)
{
	Serial.println("--- START Normal SPI---");
	while (!bmp390.InitSensor()) {
		delay(3000);
		Serial.println("- BMP390 sensor not connected  -");
	}
}

void PrintChipID(void)
{
	uint8_t chipID = 0;
	chipID = bmp390.readForChipID();
	Serial.print("CHIP ID: ");
	Serial.println(chipID, HEX);  // 0x60 for BMP390
	delay(1000);
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