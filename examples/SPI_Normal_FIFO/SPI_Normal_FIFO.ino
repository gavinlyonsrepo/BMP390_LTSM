/*!
	@file    SPI_Normal_FIFO.ino
	@brief   Arduino C++ BMP390 library test file, FIFO + interrupt, SPI hardware.
	@details Demonstrates the BMP390 FIFO and INT pin in Normal mode.
	         The sensor collects pressure + temperature frames into its 512-byte FIFO.
	         The FIFO watermark interrupt fires when the fill level reaches the threshold.
	         The sketch polls the INT pin and reads all available frames when it goes
	         high, then prints them. This avoids polling the sensor every loop iteration
	         and is the recommended low-power pattern.
	         Wiring (ESP32):
	           SPI  : SCK-18 MOSI-23 MISO-19 CS-15(any gpio you want)
	           INT  : BMP390 INT pin → GPIO4
	           Note : INT is push-pull active-high — no pull-up resistor needed.
*/

#include "bmp390_ltsm.hpp"

// SPI 
uint8_t chipSelectPin = 15;
uint32_t SPIBaudRate = 50000;
// Sensor instance
BMP390_Sensor bmp390(chipSelectPin, SPIBaudRate);
// ---- INT pin ----
#define INT_PIN  4
// ---- Application settings ----
#define LOCAL_PRESSURE  1025.25   // Replace with today's QNH from forecast [hPa]

// ---- FIFO configuration ----
// Frame size breakdown (pressure + temperature frame, datasheet Table 15):
//   1 header byte + 3 temperature bytes + 3 pressure bytes = 7 bytes per frame
// ODR_1_5_Hz = odr_sel 0x07 = 25/16 Hz = 1.5625 Hz → one frame every ~640 ms
// Watermark calculation:
// FIFO_WATERMARK (35 bytes) / 7 bytes per frame = 5 frames
//   Interrupt fires every: 5 frames × 640 ms = ~3,200 ms (~3.2 seconds)
#define FIFO_WATERMARK  35
// MAX_FRAMES must be >= (FIFO_WATERMARK / 7) to avoid discarding frames.
//   512 bytes / 7 bytes per frame = 73 frames maximum before FIFO is full
// Set to 80 to safely cover the full FIFO (73 frames max) with margin.
#define MAX_FRAMES  80
// Number of watermark events to collect before stopping
#define MAX_BATCHES  20

// ---- State variables ----
static uint32_t batchCount = 0;
static char     SerialBuffer[120];

void setup()
{
	Serialinit();
	SetupInterruptPin();
	InitSensor();
}


void loop()
{
	// ---- Poll INT pin ----
	// In a production sketch replace this with an ISR via attachInterrupt().
	if (digitalRead(INT_PIN) == LOW)
	{
		return;  // INT not asserted — nothing to do yet
	}else{
		SensorFIFORead();
	}
}
// Print parsed FIFO frames to Serial
void PrintFrames(BMP390_Sensor::FifoFrame_t* frames, uint8_t count)
{
	for (uint8_t f = 0; f < count; f++)
	{
		if (frames[f].hasTemperature && frames[f].hasPressure)
		{
			double altM = bmp390.readAltitude(LOCAL_PRESSURE);
			snprintf(SerialBuffer, sizeof(SerialBuffer),
				"  [%2u] T=%.2f C  P=%.2f hPa  Alt=%.1f m",
				f,
				frames[f].temperature,
				frames[f].pressure / 100.0,  // Pa → hPa
				altM);
			Serial.println(SerialBuffer);
		}
		else if (frames[f].hasTemperature)
		{
			snprintf(SerialBuffer, sizeof(SerialBuffer),
				"  [%2u] T=%.2f C  (no pressure)",
				f, frames[f].temperature);
			Serial.println(SerialBuffer);
		}
		else if (frames[f].hasPressure)
		{
			snprintf(SerialBuffer, sizeof(SerialBuffer),
				"  [%2u] P=%.2f hPa  (no temperature)",
				f, frames[f].pressure / 100.0);
			Serial.println(SerialBuffer);
		}
		else if (frames[f].hasSensortime)
		{
			snprintf(SerialBuffer, sizeof(SerialBuffer),
				"  [%2u] Sensortime=%lu",
				f, (unsigned long)frames[f].sensortime);
			Serial.println(SerialBuffer);
		}
	}
}


// Serial initialisation — called from setup(), 
void Serialinit()
{
	Serial.begin(38400);
	delay(1000);
	Serial.println("--Comms UP BMP390 I2C FIFO--");
}

// INT pin initialisation
void SetupInterruptPin()
{
	pinMode(INT_PIN, INPUT);  // push-pull active-high — no pull needed
}

	// One-time sensor initialisation 
void InitSensor(void)
{
		Serial.println("-START BMP390 I2C FIFO + Interrupt-");
		while (!bmp390.InitSensor())
		{
			delay(3000);
			Serial.println("BMP390 sensor not connected");
		}
		uint8_t chipID = bmp390.readForChipID();
		Serial.print("Chip ID      : 0x");
		Serial.println(chipID, HEX);  // Expect 0x60 for BMP390
		snprintf(SerialBuffer, sizeof(SerialBuffer),
			"Power mode   : %u",
			static_cast<uint8_t>(bmp390.readPowerMode()));
		Serial.println(SerialBuffer);
		// ---- Configure interrupt pin ----
		// Push-pull, active-high, non-latched, FIFO watermark source enabled.
		// drdy and ffull are left disabled for this example.
		bmp390.configureInterrupt(
			BMP390_Sensor::IntOutputMode_e::PushPull,
			BMP390_Sensor::IntLevel_e::ActiveHigh,
			BMP390_Sensor::IntLatch_e::NonLatched,
			false,  // drdy_en  — not used here
			false,  // ffull_en — not used here
			true    // fwtm_en  — fire when watermark is reached
		);
		Serial.println("INT configured: push-pull, active-high, watermark source");
		// ---- ODR and watermark ----
		// ODR_1_5_Hz: odr_sel=0x07, period=640ms (datasheet Table 45: 25/16 Hz)
		bmp390.setODR(BMP390_Sensor::ODR_e::ODR_1_5_Hz);
		bmp390.setFifoWatermark(FIFO_WATERMARK);
		snprintf(SerialBuffer, sizeof(SerialBuffer),
			"FIFO watermark : %u bytes (%u frames, fires every ~%.1f s)",
			FIFO_WATERMARK,
			FIFO_WATERMARK / 7,
			(FIFO_WATERMARK / 7) * 0.64f);
		Serial.println(SerialBuffer);
		// ---- Enable FIFO (press + temp, unfiltered, no subsampling) ----
		bmp390.enableFifo(
			true,                                        // pressEn
			true,                                        // tempEn
			false,                                       // timeEn — skip sensortime frames
			BMP390_Sensor::FifoOverflow_e::Streaming,    // overwrite oldest on full
			BMP390_Sensor::FifoDataSelect_e::Unfiltered,
			BMP390_Sensor::FifoSubsampling_e::Div1
		);
		Serial.println("FIFO enabled: press+temp, unfiltered, no subsampling");
		Serial.println();
}

void SensorFIFORead(void)
{
		// ---- INT is HIGH: watermark (or full) condition met ----
	uint16_t fillLevel = bmp390.getFifoLength();
	snprintf(SerialBuffer, sizeof(SerialBuffer),
		"=== Batch %lu — FIFO fill: %u bytes ===",
		(unsigned long)batchCount, fillLevel);
	Serial.println(SerialBuffer);
	// Read INT_STATUS to clear the interrupt and de-assert the INT pin.
	BMP390_Sensor::IntStatus_t status = bmp390.readIntStatus();
	snprintf(SerialBuffer, sizeof(SerialBuffer),
		"INT_STATUS: watermark=%u  full=%u  drdy=%u",
		status.fifoWatermark, status.fifoFull, status.dataReady);
	Serial.println(SerialBuffer);
	// Parse all available frames from the FIFO.
	BMP390_Sensor::FifoFrame_t frames[MAX_FRAMES];
	uint8_t count = bmp390.readFifoFrames(frames, MAX_FRAMES);
	snprintf(SerialBuffer, sizeof(SerialBuffer), "Frames parsed: %u", count);
	Serial.println(SerialBuffer);
	PrintFrames(frames, count);
	Serial.println();
	batchCount++;
	// ---- Done — clean up and halt ----
	if (batchCount >= MAX_BATCHES)
	{
		bmp390.disableFifo();
		bmp390.flushFifo();
		Serial.println("--- END ---");
		while (1) {}   // halt
	}
}