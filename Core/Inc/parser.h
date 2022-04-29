/*
 * parser.h
 *
 *  Created on: Apr 25, 2022
 *      Author: mand2
 */

#ifndef INC_PARSER_H_
#define INC_PARSER_H_

#include <stdint.h>

#define PARSER_PID_KP_FLAG 			0x01	// 0b0000 0001
#define PARSER_PID_KI_FLAG			0x02
#define PARSER_PID_KD_FLAG			0x04
#define PARSER_PID_SET_FLAG			0x80

#define PARSER_IMU_GX_OFFSET_FLAG	0x01
#define PARSER_IMU_AY_OFFSET_FLAG	0x02
#define PARSER_IMU_AZ_OFFSET_FLAG	0x04
#define PARSER_IMU_ANGLE_FLAG		0x08
#define PARSER_IMU_CALIBRATION_FLAG	0x10
#define PARSER_IMU_SET_FLAG			0x80

#define PARSER_MOT_FRWD_FLAG		0x01
#define PARSER_MOT_BKWD_FLAG		0x02
#define PARSER_MOT_LEFT_FLAG		0x04
#define PARSER_MOT_RGHT_FLAG		0x08
#define PARSER_MOT_MICROSTEP_FLAG	0x10
#define PARSER_MOT_MAXFREQ_FLAG		0x20
#define PARSER_MOT_MAXSPED_FLAG		0x40
#define PARSER_MOT_BLOCK_FLAG		0x80
#define PARSER_MOT_SPEED_FLAG		0x100
#define PARSER_MOT_STATE_FLAG		0x200
#define PARSER_MOT_SET_FLAG			0x800

#define PARSER_OTH_BAT_LEVL_FLAG	0x01
#define PARSER_OTH_BAT_MINV_FLAG	0x02
#define PARSER_OTH_BAT_MAXV_FLAG	0x04
#define PARSER_OTH_SET_FLAG			0x80

typedef enum {PARSER_OK = 0, PARSER_READ, PARSER_SET, PARSER_ERROR} Parser_StatusTypeDef;

void Parser_Register_PIDPrintCallback(void *callback);
void Parser_Register_IMUPrintCallback(void *callback);
void Parser_Register_MotorsPrintCallback(void *callback);
void Parser_Register_OtherPrintCallback(void *callback);
void Parser_Register_ReceiveLineCallback(void *callback);

Parser_StatusTypeDef Parser_ParseLine(void);

#endif /* INC_PARSER_H_ */
