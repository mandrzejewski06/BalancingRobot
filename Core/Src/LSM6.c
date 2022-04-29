/*
 * LSM6.c
 *
 *  Created on: Nov 25, 2021
 *      Author: mand2
 */

#include "LSM6.h"
#include "main.h"
#include "math.h"

bool LSM6_InitEx(I2C_HandleTypeDef *i2c, LSM6_t *LSM6, deviceType device, sa0State sa0)
{
	LSM6->i2c = i2c;
	LSM6->_device = device;
	LSM6->io_timeout = DEFAULT_TIMEOUT;
	LSM6->did_timeout = false;
	LSM6->did_error = false;

	// perform auto-detection unless device type and SA0 state were both specified
	if (device == device_autoDetect || sa0 == sa0_autoDetect)
	{
		// check for LSM6DS33 if device is unidentified or was specified to be this type
		if (device == device_autoDetect || device == device_DS33)
		{
			// check SA0 high address unless SA0 was specified to be low
			if (sa0 != sa0_low && testReg(LSM6, DS33_SA0_HIGH_ADDRESS, (uint8_t) WHO_AM_I) == DS33_WHO_ID)
			{
				sa0 = sa0_high;
				if (device == device_autoDetect) { device = device_DS33; }
			}
			// check SA0 low address unless SA0 was specified to be high
			else if (sa0 != sa0_high && testReg(LSM6, DS33_SA0_LOW_ADDRESS, (uint8_t) WHO_AM_I) == DS33_WHO_ID)
			{
				sa0 = sa0_low;
				if (device == device_autoDetect) { device = device_DS33; }
			}
		}

		// make sure device and SA0 were successfully detected; otherwise, indicate failure
		if (sa0 == sa0_autoDetect)
		{
			return false;
		}
	}

	LSM6->_device = device;

	switch (device)
	{
		case device_DS33:
			LSM6->address = (sa0 == sa0_high) ? DS33_SA0_HIGH_ADDRESS : DS33_SA0_LOW_ADDRESS;
			break;
		case device_autoDetect:
			return false;
	}

	return true;
}

// Returns chip id if communication is established, return 0 if not
uint8_t testReg(LSM6_t *LSM6, uint8_t address, uint8_t reg)
{
	uint8_t Value;

	if (HAL_OK == HAL_I2C_Mem_Read(LSM6->i2c, (address)<<1, reg, 1, &Value, 1, LSM6->io_timeout))
	{
		return Value;
	}
	else
	{
		return false;
	}
}

// Did a timeout occur in readAcc(), readGyro(), or read() since the last call to timeoutOccurred()?
bool timeoutOccurred(LSM6_t *LSM6)
{
	bool tmp = LSM6->did_timeout;
	LSM6->did_timeout = false;
	return tmp;
}
// Did an error occur in readAcc(), readGyro(), or read() since the last call to errorOccurred()?
bool errorOccured(LSM6_t *LSM6)
{
	bool tmp = LSM6->did_error;
	LSM6->did_error = false;
	return tmp;
}

void setTimeout(LSM6_t *LSM6, uint16_t timeout)
{
	LSM6->io_timeout = timeout;
}

uint16_t getTimeout(LSM6_t *LSM6)
{
	return LSM6->io_timeout;
}

void enableDefault(LSM6_t *LSM6)
{
	if (LSM6->_device == device_DS33)
	{
	// Accelerometer
	// ODR = 0110 (416 Hz (high performance)); FS_XL = 00 (+/-2 g full scale)
	  writeReg(LSM6, CTRL9_XL, 0x38);	// Accelerometer X, Y, Z axes enabled
	  writeReg(LSM6, CTRL1_XL, 0x30);	// Accelerometer = 52hz
	  writeReg(LSM6, INT1_CTRL, 0x01);	// Accelerometer data ready interrupt on INT1

	// Gyroscope
	// ODR = 0110 (416 Hz (high performance)); FS_XL = 00 (245 dps)
	  writeReg(LSM6, CTRL10_C, 0x38);	// Gyroscope X, Y, Z axes enabled
	  writeReg(LSM6, CTRL2_G, 0x30);	// Gyroscope = 52hz
	  writeReg(LSM6, INT2_CTRL, 0x02);	// Gyroscope data ready interrupt on INT2

	// Common
	// 0x04 = 0b00000100
	// IF_INC = 1 (automatically increment register address)
	// writeReg(CTRL3_C, 0x04);
	}
}

// Write 8-bits
void writeReg(LSM6_t *LSM6, uint8_t reg, uint8_t value)
{
	uint8_t status;

	// Write 8-bits
	status = HAL_I2C_Mem_Write(LSM6->i2c, (LSM6->address<<1), reg, I2C_MEMADD_SIZE_8BIT, &value, I2C_MEMADD_SIZE_8BIT, LSM6->io_timeout);

	// check for errors
	if(status == HAL_BUSY)
	{
		LSM6->did_timeout = true;
	}
	if(status == HAL_ERROR)
	{
		LSM6->did_error = true;
	}
}

// Read 8-bits
uint8_t readReg(LSM6_t *LSM6, uint8_t reg)
{
	uint8_t value, status;

	// Read 8 bits
	status = HAL_I2C_Mem_Read(LSM6->i2c, (LSM6->address<<1), reg, I2C_MEMADD_SIZE_8BIT, &value, I2C_MEMADD_SIZE_8BIT, LSM6->io_timeout);

	// check for errors
	if(status == HAL_BUSY)
	{
		LSM6->did_timeout = true;
		return 0;
	}
	if(status == HAL_ERROR)
	{
		LSM6->did_error = true;
		return 0;
	}

	return value;
}

uint8_t readAcc(LSM6_t *LSM6)
{
	// read output registers
	uint8_t xla = readReg(LSM6, OUTX_L_XL);
	uint8_t xha = readReg(LSM6, OUTX_H_XL);
	uint8_t yla = readReg(LSM6, OUTY_L_XL);
	uint8_t yha = readReg(LSM6, OUTY_H_XL);
	uint8_t zla = readReg(LSM6, OUTZ_L_XL);
	uint8_t zha = readReg(LSM6, OUTZ_H_XL);

	// check for errors
	if (timeoutOccurred(LSM6) == true)
	{
		return HAL_BUSY;
	}
	if (errorOccured(LSM6) == true)
	{
		return HAL_ERROR;
	}

	// combine high and low bytes
	LSM6->accelerometer.x = (int16_t)(xha << 8 | xla);
	LSM6->accelerometer.y = (int16_t)(yha << 8 | yla);
	LSM6->accelerometer.z = (int16_t)(zha << 8 | zla);

	return HAL_OK;
}

uint8_t readGyro(LSM6_t *LSM6)
{
	// read output registers
	uint8_t xlg = readReg(LSM6, OUTX_L_G);
	uint8_t xhg = readReg(LSM6, OUTX_H_G);
	uint8_t ylg = readReg(LSM6, OUTY_L_G);
	uint8_t yhg = readReg(LSM6, OUTY_H_G);
	uint8_t zlg = readReg(LSM6, OUTZ_L_G);
	uint8_t zhg = readReg(LSM6, OUTZ_H_G);

	// check for errors
	if (timeoutOccurred(LSM6) == true)
	{
		return HAL_BUSY;
	}
	if (errorOccured(LSM6) == true)
	{
		return HAL_ERROR;
	}

	// combine high and low bytes
	LSM6->gyroscope.x = (int16_t)(xhg << 8 | xlg);
	LSM6->gyroscope.y = (int16_t)(yhg << 8 | ylg);
	LSM6->gyroscope.z = (int16_t)(zhg << 8 | zlg);

	return HAL_OK;
}

// Main function to read LSM6 data
uint8_t LSM6_Read(LSM6_t *LSM6)
{
	uint8_t status;

	// Read accelerometer data
	if ((status = readAcc(LSM6)) != HAL_OK)
	{
		return status;
	}

	// Read gyroscope data
	if ((status = readGyro(LSM6)) != HAL_OK)
	{
		return status;
	}

	return HAL_OK;
}
/*
// Vector's basic functions
int16_t vector_dot(Vector_t *vector_a, Vector_t *vector_b)
{
	return (vector_a->x * vector_b->x) + (vector_a->y * vector_b->y) + (vector_a->z * vector_b->z);
}

void vector_cross(Vector_t *vector_a, Vector_t *vector_b, Vector_t *vector_out)
{
	vector_out->x = (vector_a->y * vector_b->z) - (vector_a->z * vector_b->y);
	vector_out->y = (vector_a->z * vector_b->x) - (vector_a->x * vector_b->z);
	vector_out->z = (vector_a->x * vector_b->y) - (vector_a->y * vector_b->x);
}

void vector_normalize(Vector_t *vector)
{
	float mag = sqrt(vector_dot(vector, vector));

	vector->x = (vector->x / mag);
	vector->y = (vector->y / mag);
	vector->z = (vector->z / mag);
}
*/
