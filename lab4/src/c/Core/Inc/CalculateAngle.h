/*
 * CalculateAngle.h
 *
 *  Created on: Dec 28, 2023
 *      Author: fevzi
 */

#ifndef INC_CALCULATEANGLE_H_
#define INC_CALCULATEANGLE_H_

#include "MPU6050.h"
#include <math.h>
#define RADIAN_TO_DEGREE 180/3.141592f

typedef struct _Angle{
	float acc_roll;
	float acc_pitch;
	float acc_yaw;

	float gyro_roll;
	float gyro_pitch;
	float gyro_yaw;

	float ComFilt_roll;
	float ComFilt_pitch;
	float ComFilt_yaw;
}Struct_Angle;


extern Struct_Angle Angle;

void CalculateAccAngle(Struct_Angle* Angle, Struct_MPU6050* MPU6050);
void CalculateGyroAngle(Struct_Angle* Angle, Struct_MPU6050* MPU6050);
void CalculateCompliFilter(Struct_Angle* Angle, Struct_MPU6050* MPU6050);


#endif /* INC_CALCULATEANGLE_H_ */
