/*
 * GPS_Measurement.c
 *
 *  Created on: Nov 29, 2023
 *      Author: win 10
 */
#include "GPS_Measurement.h"
#include <string.h>
void GPS_Init(GPS *GPS){

	memset(GPS->GPSCovariance,0,sizeof(GPS->GPSCovariance));
	memset(GPS->GPSGetPosition,0,sizeof(GPS->GPSGetPosition));
	memset(GPS->GPS_Model,0,sizeof(GPS->GPS_Model));

	GPS->GPSCovariance[0][0]=0; //input fromsensor
	GPS->GPSCovariance[1][1]=0; //input fromsensor

	GPS->GPSGetPosition[0]=106.802343; //input fromsensor

	GPS->GPSGetPosition[1]=10.869826;//input fromsensor

	GPS->GPS_Model[0][0]=1;
	GPS->GPS_Model[1][1]=1;

}

void EFK_GPSHandleMeasurement(GPS *GPS, EKF *EKF ){
	float GPSCovariance[2][2];
	float GPSGetPos[2];
	float Error[2];
	float InovationCovariance[2][2];
	float KalmanGain[6][6];

	GPSCovariance[0][0]=GPS->GPSCovariance[0][0];
	GPSCovariance[1][1]=GPS->GPSCovariance[1][1];

	GPSGetPos[0]=GPS->GPSGetPosition[0];
	GPSGetPos[1]=GPS->GPSGetPosition[1];

	//Error
	Error[0]=GPSGetPos[0] - EKF->NexPx;
	Error[1]=GPSGetPos[1] - EKF->NexPy;
	//InovationCovarian
	InovationCovariance[0][0]=EKF->Prediction_CovarianceNex[0][0] + GPSCovariance[0][0];
	InovationCovariance[1][1]=EKF->Prediction_CovarianceNex[1][1]+  GPSCovariance[1][1];
	//KalmanGain
	KalmanGain[0][0]= EKF->Prediction_CovarianceNex[0][0]/(InovationCovariance[0][0]);
	KalmanGain[1][1]=EKF->Prediction_CovarianceNex[1][1]/(InovationCovariance[1][1]);
	//Update State
	EKF->FirPx = EKF->NexPx + (KalmanGain[0][0] * Error[0]);
	EKF->FirPy = EKF->NexPy + (KalmanGain[1][1] * Error[1]);
	//Update Covariance
	EKF->Prediction_CovarianceFir[0][0]= (GPSCovariance[0][0] / (InovationCovariance[0][0]))*EKF->Prediction_CovarianceNex[0][0];
	EKF->Prediction_CovarianceFir[1][1]= (GPSCovariance[1][1] / (InovationCovariance[0][0]))*EKF->Prediction_CovarianceNex[1][1];

}
