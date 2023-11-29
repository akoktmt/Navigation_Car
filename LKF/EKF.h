/*
 * EKF.h
 *
 *  Created on: Nov 11, 2023
 *      Author: win 10
 */

#ifndef EKF_H_
#define EKF_H_

#define LENGTH_REAR 10
#define LENGTH_CAR 20
#define SIZE 6
#define NUMBEROFMODLE 6
#define PI 3.141592

typedef struct{
	float FirPx;
	float FirPy;
	float FirVelx;
	float FirVely;
	float FirHea;
	float FirStee;

	float NexPx;
	float NexPy;
	float NexVelx;
	float NexVely;
	float NexHea;
	float NexStee;

	float CovPx;
	float CovPy;
	float CovVelx;
	float CovVely;
	float CovHea;
	float CovStee;

	float Prediction_CovarianceNex[NUMBEROFMODLE][NUMBEROFMODLE];
	float Prediction_CovarianceFir[NUMBEROFMODLE][NUMBEROFMODLE];
}EKF;

typedef struct {
	float Velx;
	float Vely;
	float Velz;
	float Stee;
	float Head;
	float Accx;
	float Accy;
	float Accz;
	float Time;
}Input;

typedef struct{
	float AngleBeta;
	float Roll;
	float Pitch;
	float Yaw;
}Angle;

void EKF_Init(EKF *EKF,Input *Input);
void EKF_PredictionStep(EKF *EKF, Angle *Angle, Input *Input);

#endif /* EKF_H_ */


