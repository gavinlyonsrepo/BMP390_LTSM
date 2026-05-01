/*!
	@file    I2C_Forced.ino
	@brief   Arduino bmp390 library test file, basic use, forced mode, I2C hardware.
	@details bmp390 is a digital pressure sensor with temperature measurement capabilities.
		In forced mode the sensor takes a single measurement on demand and
		returns to sleep automatically. takeForcedMeasurement() blocks until
		the STATUS register confirms both results are ready.
*/

#include "bmp390_ltsm.hpp"

// I2C setup
uint8_t  I2C_ADDRESS    =  0x76; // I2C address: 0x76 SDO connected to GND, 0x77 SDO connected to VCC
uint8_t  I2C_SDA_PIN    =  21;   // I2C Data pin
uint8_t  I2C_SCK_PIN    =  22;   // I2C clock pin
uint32_t I2C_CLOCK_RATE = 50000; // I2C Clock rate in hertz
BMP390_Sensor bmp390(I2C_ADDRESS, &Wire, I2C_SCK_PIN, I2C_SDA_PIN, I2C_CLOCK_RATE);
// Local Pressure , Replace with today's QNH from forecast [hPa]
#define LOCAL_PRESSURE  1025.25   

void setup() {
  Serialinit();
	SensorInit();
}

void loop()
{
	ForcedModeSetup();
	PrintSensorInfo();
	Serial.println("--- End ---");
	while (1){};
}

void PrintSensorInfo(void)
{
	uint16_t counter = 0;
	while (counter < 60)
	{
		Serial.print("Test Count: ");
		Serial.println(counter);

		if (bmp390.takeForcedMeasurement())
		{
			Serial.print("Temperature : ");
			Serial.print(bmp390.getTemperature(), 2);
			Serial.println(" [C]");
			Serial.print("Pressure    : ");
			Serial.print(bmp390.getPressure(BMP390_Sensor::PressureUnit_e::hPa), 2);
			Serial.println(" [hPa]");
			Serial.print("Altitude    : ");
			Serial.print(bmp390.readAltitude(LOCAL_PRESSURE), 2);
			Serial.println(" [m]");
		}
		else
		{
			Serial.println("takeForcedMeasurement failed - check power mode.");
		}

		Serial.println();
		delay(3000);
		counter++;
	}
}

void ForcedModeSetup(void)
{
	// Switch to forced mode: InitSensor() leaves the sensor in normal mode
	if (bmp390.setPowerMode(BMP390_Sensor::PowerMode_e::Forced))
	{
		Serial.println("Power mode set to Forced.");
	}else{
		Serial.println("Failed to set power mode to Forced.");
	}

	Serial.print("Power mode   : ");
	Serial.println(static_cast<uint8_t>(bmp390.readPowerMode()));

	// Optional: increase oversampling for a one-shot measurement
	bmp390.setOversampling(BMP390_Sensor::DataType_e::Temperature, BMP390_Sensor::sensorSampling_e::Sampling_X16);
	bmp390.setOversampling(BMP390_Sensor::DataType_e::Pressure,    BMP390_Sensor::sensorSampling_e::Sampling_X8);
	bmp390.setIIRFilter(BMP390_Sensor::Filter_e::Filter_X3);

	Serial.print("Temp OS      : ");
	Serial.println(static_cast<uint8_t>(
		bmp390.readOversampling(BMP390_Sensor::DataType_e::Temperature)));

	Serial.print("Pressure OS  : ");
	Serial.println(static_cast<uint8_t>(
		bmp390.readOversampling(BMP390_Sensor::DataType_e::Pressure)));

	Serial.print("IIR filter   : ");
	Serial.println(static_cast<uint8_t>(bmp390.readIIRFilter()));
	Serial.println();
}

void SensorInit(void)
{
	Serial.println("--- START Forced I2C ---");
	while (!bmp390.InitSensor()) {
		delay(3000);
		Serial.println("- BMP390 sensor not connected  -");
	}

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