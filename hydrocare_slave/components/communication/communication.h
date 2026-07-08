#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include <inttypes.h>
#include "measurement.h"
#include "driver/spi_slave.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
// ============ ADDRESS-BASED R/W PROTOCOL ============
// Command byte format: [R/W bit (7) | 7-bit address (6-0)]
#define PROTO_CMD_READ  0x80        // R/W=1 (bit 7 set)
#define PROTO_CMD_WRITE 0x00        // R/W=0 (bit 7 clear)
#define PROTO_ADDR_MASK 0x7F        // Lower 7 bits = address

// Address space
#define ADDR_SENSOR_DATA  0x00      // Read: 20480 bytes sensor packet
#define ADDR_STATUS       0x01      // Read: 1 byte status
#define ADDR_CTRL         0x02      // Write: 1 byte control (0x01=trigger, 0x02=lock)
#define ADDR_IR_LED       0x03      // Write: 1 byte (0x00=off, 0x01=on)
#define ADDR_BRIGHTNESS   0x04      // Write: 1 byte (0-100)
#define ADDR_AMB_LIGHT    0x05      // Read: 1 byte ambient light

// Control register values
#define CTRL_TRIGGER_MEASUREMENT 0x01
#define CTRL_LOCK_BUFFERS 0x02
#define CTRL_UNLOCK_BUFFERS 0x03

// Slave status bits
#define STATUS_MEASURING 0x01       // Measurement in progress
#define STATUS_MEASURED 0x02        // Measurement complete (awaiting LOCK)
#define STATUS_LOCKED 0x04          // Buffers locked and ready for bulk read

// Constants
#define SPI_BUFFER_SIZE 12288       // 12KB - handles ~12kB packet (400 samples) + header + margin
#define RING_BUFFER_SIZE 1000       // 0.5 seconds of 2kHz data

// ============ DATA STRUCTURES ============
// Sensor data packet structure with high-speed samples (~16.6KB)
typedef struct __attribute__((packed)) {
  // Metadata
  uint16_t sequence;              // Packet sequence number
  uint16_t ambientLight;          // Ambient light value (instantaneous)
  float temperature;              // Temperature in °C (average)
  float humidity;                 // Humidity % (average)
  int16_t accelX, accelY, accelZ; // IMU accel (most recent single sample)
  int16_t gyroX, gyroY, gyroZ;    // IMU gyro (unused, zeros)
  uint32_t timestamp_ms;          // System uptime when measurement triggered
  uint8_t status;                 // Status flags
  uint16_t accelSampleCount;      // Number of accel/mic samples in this packet (400)

  // High-speed samples (2kHz sampling over 0.2 second measurement window)
  int16_t accelX_samples[400];    // 400 accel X samples @ 2kHz = 0.2 seconds
  int16_t accelY_samples[400];    // 400 accel Y samples @ 2kHz = 0.2 seconds
  int16_t accelZ_samples[400];    // 400 accel Z samples @ 2kHz = 0.2 seconds
  uint16_t microphoneSamples[400];// 400 microphone samples @ 2kHz = 0.2 seconds

  // Slow sensor frames (camera data)
  uint16_t rgbFrame[4096];        // RGB565 64x64 (8192 bytes)
  uint16_t irFrame[192];          // IR thermal 16x12 (384 bytes)
} SensorDataPacket;


// State machine - simplified for new protocol
typedef enum {
  STATE_IDLE = 0,           // No measurement active
  STATE_MEASURING = 1,      // Measurement in progress
  STATE_READY_TRANSFER = 2, // Measurement complete, buffers locked, ready to send
  STATE_ERROR = 3
} SlaveState;

// ============ FUNCTION DECLARATIONS ============
void initSPIComm();
void receiveCommand();
void startSpiCommandHandlerTask();
void startMeasurementTask();          // Start background measurement collector task
void startHighSpeedSamplerTask();     // Start 2kHz accel+mic sampler task
void setup_timer();                    // Setup timer for periodic tasks
void initIRSamplerTask();               // Start background IR sampler task
void initBMESamplerTask();
void startLis3dhSamplerTask();
#endif