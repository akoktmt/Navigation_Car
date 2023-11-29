/*
 * Heading_Measurement.h
 *
 *  Created on: Nov 29, 2023
 *      Author: win 10
 */

#ifndef HEADING_MEASUREMENT_H_
#define HEADING_MEASUREMENT_H_
#include "EKF.h"
typedef struct{
	float Yaw;
	float Covariane;
}Heading;
void Heading_Init(Heading*Heading);
void Heading_HandleMeasurement(EKF *EKF, Input *Input, Heading *Heading);
#endif /* HEADING_MEASUREMENT_H_ */
