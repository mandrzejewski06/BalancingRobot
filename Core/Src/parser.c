/*
 * parser.c
 *
 *  Created on: Apr 25, 2022
 *      Author: mand2
 */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "parser.h"

#define BUFFER_SIZE 32

static void (*Parser_PID_Print)(uint8_t flag, float value);
static void (*Parser_IMU_Print)(uint8_t flag, float value);
static void (*Parser_Motors_Print)(uint16_t flag, uint16_t value);
static void (*Parser_Other_Print)(uint8_t flag, float value);

static Parser_StatusTypeDef (*Parser_ReceiveLine)(char* message);

static char BufferReceive[BUFFER_SIZE];

void Parser_Register_PIDPrintCallback(void *callback)
{
	Parser_PID_Print = callback;
}

void Parser_Register_IMUPrintCallback(void *callback)
{
	Parser_IMU_Print = callback;
}

void Parser_Register_MotorsPrintCallback(void *callback)
{
	Parser_Motors_Print = callback;
}

void Parser_Register_OtherPrintCallback(void *callback)
{
	Parser_Other_Print = callback;
}

void Parser_Register_ReceiveLineCallback(void *callback)
{
	Parser_ReceiveLine = callback;
}

static Parser_StatusTypeDef Parser_GetFloatValue(float *value)
{
	char *ParsePointer = strtok(NULL, "\0");
	uint8_t i = 0;

	if(strlen(ParsePointer) == 0) return PARSER_READ;
	if(ParsePointer[0] == '-') i++;

	for( ; ParsePointer[i] != 0; i++)
	{
		if((ParsePointer[i] < '0' || ParsePointer[i] > '9') && (ParsePointer[i] != '.'))
		{
			return PARSER_ERROR;
		}
	}

	*value = atof(ParsePointer);

	return PARSER_SET;
}

static Parser_StatusTypeDef Parser_GetIntValue(uint16_t *value)
{
	char *ParsePointer = strtok(NULL, "\0");

	if(strlen(ParsePointer) == 0) return PARSER_READ;

	for(uint8_t i = 0; ParsePointer[i] != 0; i++)
	{
		if((ParsePointer[i] < '0' || ParsePointer[i] > '9'))
		{
			return PARSER_ERROR;
		}
	}

	*value = atoi(ParsePointer);

	return PARSER_SET;
}


static Parser_StatusTypeDef Parser_ParseOther(void)
{
	Parser_StatusTypeDef status;
	uint16_t flag;
	float value = 0;
	char *ParsePointer = strtok(NULL, "=?");

	if(strlen(ParsePointer) <= 0) return PARSER_ERROR;	// check if string exists

	if(strcmp(ParsePointer, "BMX") == 0)
	{
		flag = PARSER_OTH_BAT_MAXV_FLAG;
	}
	else if(strcmp(ParsePointer, "BMN") == 0)
	{
		flag = PARSER_OTH_BAT_MINV_FLAG;
	}
	else if(strcmp(ParsePointer, "BL") == 0)
	{
		flag = PARSER_OTH_BAT_LEVL_FLAG;
	}
	else return PARSER_ERROR;

	status = Parser_GetFloatValue(&value);

	if(status == PARSER_ERROR) return PARSER_ERROR;
	else if(status == PARSER_SET) flag |= PARSER_OTH_SET_FLAG;

	Parser_Other_Print(flag, value);
	return PARSER_OK;
}


static Parser_StatusTypeDef Parser_ParseMotors(void)
{
	Parser_StatusTypeDef status;
	uint16_t flag;
	uint16_t value = 0;
	char *ParsePointer = strtok(NULL, "=?");

	if(strlen(ParsePointer) <= 0) return PARSER_ERROR;	// check if string exists

	if(strcmp(ParsePointer, "FWD") == 0)
	{
		flag = PARSER_MOT_FRWD_FLAG;
	}
	else if(strcmp(ParsePointer, "BCK") == 0)
	{
		flag = PARSER_MOT_BKWD_FLAG;
	}
	else if(strcmp(ParsePointer, "LFT") == 0)
	{
		flag = PARSER_MOT_LEFT_FLAG;
	}
	else if(strcmp(ParsePointer, "RGT") == 0)
	{
		flag = PARSER_MOT_RGHT_FLAG;
	}
	else if(strcmp(ParsePointer, "MST") == 0)
	{
		flag = PARSER_MOT_MICROSTEP_FLAG;
	}
	else if(strcmp(ParsePointer, "MF") == 0)
	{
		flag = PARSER_MOT_MAXFREQ_FLAG;
	}
	else if(strcmp(ParsePointer, "MSP") == 0)
	{
		flag = PARSER_MOT_MAXSPED_FLAG;
	}
	else if(strcmp(ParsePointer, "BLK") == 0)
	{
		flag = PARSER_MOT_BLOCK_FLAG;
	}
	else if(strcmp(ParsePointer, "SPD") == 0)
	{
		flag = PARSER_MOT_SPEED_FLAG;
	}
	else if(strcmp(ParsePointer, "ST") == 0)
	{
		flag = PARSER_MOT_STATE_FLAG;
	}
	else return PARSER_ERROR;

	status = Parser_GetIntValue(&value);

	if(status == PARSER_ERROR) return PARSER_ERROR;
	else if(status == PARSER_SET) flag |= PARSER_MOT_SET_FLAG;

	Parser_Motors_Print(flag, value);
	return PARSER_OK;
}

static Parser_StatusTypeDef Parser_ParseIMU(void)
{
	Parser_StatusTypeDef status;
	uint16_t flag;
	float value = 0;
	char *ParsePointer = strtok(NULL, "=?");

	if(strlen(ParsePointer) <= 0) return PARSER_ERROR;	// check if string exists

	if(strcmp(ParsePointer, "GX") == 0)
	{
		flag = PARSER_IMU_GX_OFFSET_FLAG;
	}
	else if(strcmp(ParsePointer, "AY") == 0)
	{
		flag = PARSER_IMU_AY_OFFSET_FLAG;
	}
	else if(strcmp(ParsePointer, "AZ") == 0)
	{
		flag = PARSER_IMU_AZ_OFFSET_FLAG;
	}
	else if(strcmp(ParsePointer, "ANG") == 0)
	{
		flag = PARSER_IMU_ANGLE_FLAG;
	}
	else if(strcmp(ParsePointer, "CB") == 0)
	{
		flag = PARSER_IMU_CALIBRATION_FLAG;
	}
	else return PARSER_ERROR;

	status = Parser_GetFloatValue(&value);

	if(status == PARSER_ERROR) return PARSER_ERROR;
	else if(status == PARSER_SET) flag |= PARSER_IMU_SET_FLAG;

	Parser_IMU_Print(flag, value);
	return PARSER_OK;
}

static Parser_StatusTypeDef Parser_ParsePID(void)
{
	Parser_StatusTypeDef status;
	uint16_t flag;
	float value;
	char *ParsePointer = strtok(NULL, "=?");

	if(strlen(ParsePointer) <= 0) return PARSER_ERROR;	// check if string exists

	if(strcmp(ParsePointer, "KP") == 0)
	{
		flag = PARSER_PID_KP_FLAG;
	}
	else if(strcmp(ParsePointer, "KI") == 0)
	{
		flag = PARSER_PID_KI_FLAG;
	}
	else if(strcmp(ParsePointer, "KD") == 0)
	{
		flag = PARSER_PID_KD_FLAG;
	}
	else return PARSER_ERROR;

	status = Parser_GetFloatValue(&value);

	if(status == PARSER_ERROR) return PARSER_ERROR;
	else if(status == PARSER_SET) flag |= PARSER_PID_SET_FLAG;
	else if(status == PARSER_READ)
		status = PARSER_OK;
	Parser_PID_Print(flag, value);
	return PARSER_OK;
}

Parser_StatusTypeDef Parser_ParseLine(void)
{
	char *ParsePointer;
	Parser_StatusTypeDef status;

	if(!Parser_ReceiveLine(BufferReceive))
	{
		ParsePointer = strtok(BufferReceive, "+");
		if(strcmp(ParsePointer, "P") == 0)
		{
			status = Parser_ParsePID();
		}
		else if(strcmp(ParsePointer, "I") == 0)
		{
			status = Parser_ParseIMU();
		}
		else if(strcmp(ParsePointer, "M") == 0)
		{
			status = Parser_ParseMotors();
		}
		else if(strcmp(ParsePointer, "O") == 0)
		{
			status = Parser_ParseOther();
		}
		else
		{
			return PARSER_ERROR;
		}
		return status;
	}
	return PARSER_ERROR;
}
