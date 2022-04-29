/*
 * LSM6.h
 *
 *  Created on: Nov 25, 2021
 *      Author: mand2
 */

#ifndef INC_LSM6_H_
#define INC_LSM6_H_

#include "main.h"

// LSM6DS33 addresses //
#define DS33_SA0_HIGH_ADDRESS 0b1101011
#define DS33_SA0_LOW_ADDRESS  0b1101010
#define DS33_WHO_ID    0x69		//0b1101001 - register WHO_AM_I value for DS33

// Default timeout and error defines
#define ERROR 0
#define DEFAULT_TIMEOUT 1000

// Register addresses //
#define      FUNC_CFG_ACCESS    0x01
#define      FIFO_CTRL1         0x06
#define   	 FIFO_CTRL2         0x07
#define      FIFO_CTRL3         0x08
#define      FIFO_CTRL4         0x09
#define      FIFO_CTRL5         0x0A
#define      ORIENT_CFG_G       0x0B

#define      INT1_CTRL          0x0D
#define      INT2_CTRL          0x0E
#define      WHO_AM_I           0x0F
#define      CTRL1_XL           0x10
#define      CTRL2_G            0x11
#define      CTRL3_C            0x12
#define      CTRL4_C            0x13
#define      CTRL5_C            0x14
#define      CTRL6_C            0x15
#define      CTRL7_G            0x16
#define      CTRL8_XL           0x17
#define      CTRL9_XL           0x18
#define      CTRL10_C           0x19

#define      WAKE_UP_SRC        0x1B
#define      TAP_SRC            0x1C
#define      D6D_SRC            0x1D
#define      STATUS_REG         0x1E

#define      OUT_TEMP_L         0x20
#define      OUT_TEMP_H         0x21
#define      OUTX_L_G           0x22
#define      OUTX_H_G           0x23
#define      OUTY_L_G           0x24
#define      OUTY_H_G           0x25
#define      OUTZ_L_G           0x26
#define      OUTZ_H_G           0x27
#define      OUTX_L_XL          0x28
#define      OUTX_H_XL          0x29
#define      OUTY_L_XL          0x2A
#define      OUTY_H_XL          0x2B
#define      OUTZ_L_XL          0x2C
#define      OUTZ_H_XL          0x2D

#define      FIFO_STATUS1       0x3A
#define      FIFO_STATUS2       0x3B
#define      FIFO_STATUS3       0x3C
#define      FIFO_STATUS4       0x3D
#define      FIFO_DATA_OUT_L    0x3E
#define      FIFO_DATA_OUT_H    0x3F
#define      TIMESTAMP0_REG     0x40
#define      TIMESTAMP1_REG     0x41
#define      TIMESTAMP2_REG     0x42

#define      STEP_TIMESTAMP_L   0x49
#define      STEP_TIMESTAMP_H   0x4A
#define      STEP_COUNTER_L     0x4B
#define      STEP_COUNTER_H     0x4C

#define      FUNC_SRC           0x53

#define      TAP_CFG            0x58
#define      TAP_THS_6D         0x59
#define      INT_DUR2           0x5A
#define      WAKE_UP_THS        0x5B
#define      WAKE_UP_DUR        0x5C
#define      FREE_FALL          0x5D
#define      MD1_CFG            0x5E
#define      MD2_CFG            0x5F

// Structure to memorize coordinates of a vector
typedef struct vector
{
	double x;
	double y;
	double z;
} Vector_t;

// Used to manually choose device type or detect device automatically
typedef enum deviceType {device_DS33, device_autoDetect} deviceType;

// State of the SA0 pin.
typedef enum sa0State {sa0_low, sa0_high, sa0_autoDetect} sa0State;

// Structure of basic LSM6
typedef struct lsm6
{
	I2C_HandleTypeDef *i2c;	// i2c handler
	deviceType _device; 	// chip type
	uint8_t address;		// address of the device
	uint16_t io_timeout;	// i2c timeout
	bool did_timeout;		// did timeout occur on i2c?
	bool did_error;			// did error occur on i2c?

	Vector_t accelerometer; // accelerometer vector
	Vector_t gyroscope;		// gyroscope vector
} LSM6_t;

// Initialize extended function - detecting device using deviceType and sa0State (definition above)
bool LSM6_InitEx(I2C_HandleTypeDef *i2c, LSM6_t *LSM6, deviceType device, sa0State sa0);
// Initialize function - default DS33 device and SA0 pin connected to GND
//bool LSM6_Init(I2C_HandleTypeDef *i2c, LSM6_t *LSM6) { return LSM6_InitEx(i2c, LSM6, device_DS33, sa0_low);}
// Returns chip id if communication is established, return 0 if not
uint8_t testReg(LSM6_t *LSM6, uint8_t address, uint8_t reg);
// Writes 8-bits
void writeReg(LSM6_t *LSM6, uint8_t reg, uint8_t value);
// Reads 8-bits
uint8_t readReg(LSM6_t *LSM6, uint8_t reg);
// Starts gyroscope and accelerometer
void enableDefault(LSM6_t *LSM6);
// Reads accelerometer data
uint8_t readAcc(LSM6_t *LSM6);
// Reads gyroscope data
uint8_t readGyro(LSM6_t *LSM6);
// Main function to read LSM6 data - using both functions above
uint8_t LSM6_Read(LSM6_t *LSM6);
// Did a timeout occur in readAcc(), readGyro(), or read() since the last call to timeoutOccurred()?
bool timeoutOccurred(LSM6_t *LSM6);
void setTimeout(LSM6_t *LSM6, uint16_t timeout);
uint16_t getTimeout(LSM6_t *LSM6);
// Did an error occur in readAcc(), readGyro(), or read() since the last call to errorOccurred()?
bool errorOccured(LSM6_t *LSM6);
// Vector's basic functions
/*
int16_t vector_dot(Vector_t *vector_a, Vector_t *vector_b);
void vector_cross(Vector_t *vector_a, Vector_t *vector_b, Vector_t *vector_out);
void vector_normalize(Vector_t *vector);
*/
#endif /* INC_LSM6_H_ */
