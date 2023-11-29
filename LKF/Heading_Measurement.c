/*
 * Heading_Measurement.c
 *
 *  Created on: Nov 29, 2023
 *      Author: win 10
 */
#include "EKFmath.h"
#include <math.h>
#include "Heading_Measurement.h"
#include <stdint.h>
#include <string.h>
void Heading_Init(Heading *Heading){
	Heading->Covariane=0;
	Heading->Yaw=0;
}

void Heading_HandleMeasurement(EKF *EKF, Input *Input, Heading *Heading){
	float 	 Jacobian[6];
	float 	 Error;
	float InovationCovariance;
	float KalmanGain[6];
	float KalvsJaco[6][6];
	float Identity[6][6];
	float Result[6][6];
	memset(Identity,0,sizeof(Identity));

	Identity[0][0]=1;
	Identity[1][1]=1;
	Identity[2][2]=1;
	Identity[3][3]=1;
	Identity[4][4]=1;
	Identity[5][5]=1;

			 Jacobian[0]=0;
			 Jacobian[1]=0;
			 Jacobian[2]=0;
			 Jacobian[3]=(Input->Time*tan(EKF->FirStee)*fabs(EKF->FirStee))/(EKF->FirStee*sqrt(EKF->FirStee*EKF->FirStee+LENGTH_REAR*LENGTH_REAR*tan(EKF->FirStee)*tan(EKF->FirStee)));
			 Jacobian[4]=1;
			 Jacobian[5]=(EKF->FirVely*LENGTH_CAR*LENGTH_CAR*LENGTH_CAR*Input->Time*(1/cos(EKF->FirStee)*1/cos(EKF->FirStee)))/(pow(LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(EKF->FirStee)*tan(EKF->FirStee),1.5)*fabs(EKF->FirStee));
	Error= Heading->Yaw - EKF->NexHea;

	InovationCovariance= (EKF->Prediction_CovarianceNex[0][0]* Jacobian[0]* Jacobian[0]) + (EKF->Prediction_CovarianceNex[3][3]* Jacobian[3]* Jacobian[3]) + (EKF->Prediction_CovarianceNex[4][4]* Jacobian[4]* Jacobian[4]) +(EKF->Prediction_CovarianceNex[1][1]* Jacobian[1]* Jacobian[1])+(EKF->Prediction_CovarianceNex[5][5]* Jacobian[5]* Jacobian[5]) + (EKF->Prediction_CovarianceNex[2][2]* Jacobian[2]* Jacobian[2])+ Heading->Covariane;

	KalmanGain[0]= (EKF->Prediction_CovarianceNex[0][0]* Jacobian[0]/InovationCovariance);
	KalmanGain[1]= (EKF->Prediction_CovarianceNex[1][1]* Jacobian[1]/InovationCovariance);
	KalmanGain[2]= (EKF->Prediction_CovarianceNex[2][2]* Jacobian[2]/InovationCovariance);
	KalmanGain[3]= (EKF->Prediction_CovarianceNex[3][3]* Jacobian[3]/InovationCovariance);
	KalmanGain[4]= (EKF->Prediction_CovarianceNex[4][4]* Jacobian[4]/InovationCovariance);
	KalmanGain[5]= (EKF->Prediction_CovarianceNex[5][5]* Jacobian[5]/InovationCovariance);

	EKF->FirPx = EKF->NexPx + KalmanGain[0] * Error;
	EKF->FirPy = EKF->NexPy + KalmanGain[1] * Error;
	EKF->FirVelx = EKF->NexVelx + KalmanGain[2] * Error;
	EKF->FirVely = EKF->NexVely + KalmanGain[3] * Error;
	EKF->FirHea = EKF->NexHea + KalmanGain[4] * Error;
	EKF->FirStee = EKF->NexStee + KalmanGain[5] * Error;

	for (int i = 0; i < 6; ++i) {
	    for (int j = 0; j < 6; ++j) {
	        KalvsJaco[i][j] = KalmanGain[i] * Jacobian[j];
	    }
	}

	subtractMatrices(Identity, KalvsJaco, Result, 6, 6);
	multiplyMatrices(Result,EKF->Prediction_CovarianceNex,EKF->Prediction_CovarianceFir,6,6);
}
