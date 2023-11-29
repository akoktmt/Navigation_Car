/*
 * EKF.c
 *
 *  Created on: Nov 11, 2023
 *      Author: win 10
 */
#include <EKF.h>
#include "EKFmath.h"
#include <stdint.h>
#include <string.h>

void EKF_Init(EKF *EKF,Input *Input)
{
//First Step
	EKF->FirPx= 106.802343;  //input
	EKF->FirPy=10.869826; //input
	EKF->FirVelx=0; //input
 	EKF->CovVely=0; //input
	EKF->FirHea=-PI/6; //input
	EKF->FirStee=PI/6; //input
//Next Step
	EKF->NexPx=0;
	EKF->NexPy=0;
	EKF->NexVelx=0;
	EKF->NexVely=0;
	EKF->NexHea=0;
	EKF->NexStee=0;
//Covariance
	EKF->CovPx=0; //input
	EKF->CovPy=0; //input
	EKF->CovVelx=0;//input
	EKF->CovVely=0;//input
	EKF->CovHea=0; //input
	EKF->CovStee=0; //input
//FirstIput
	Input->Accx=0.2;
	Input->Accy=0.2;
	Input->Accz=0.2;
	Input->Time=0.1; //input
//Covariance
	memset(EKF->Prediction_CovarianceNex,0,sizeof(EKF->Prediction_CovarianceNex));
	memset(EKF->Prediction_CovarianceFir,0,sizeof(EKF->Prediction_CovarianceFir));
	EKF->Prediction_CovarianceFir[0][0]= EKF->CovPx;
	EKF->Prediction_CovarianceFir[1][1]= EKF->CovPy;
	EKF->Prediction_CovarianceFir[2][2]= EKF->CovVelx;
	EKF->Prediction_CovarianceFir[3][3]= EKF->CovVely;
	EKF->Prediction_CovarianceFir[4][4]= EKF->CovHea;
	EKF->Prediction_CovarianceFir[5][5]= EKF->CovStee;
}

void EKF_PredictionStep(EKF *EKF, Angle *Angle, Input *Input){
// Prediction State
	float a_hx;
	float a_hy;

	 a_hx = Input->Accx * cos(Angle->Pitch) + Input->Accy * sin(Angle->Roll) * sin(Angle->Pitch) + Input->Accz * cos(Angle->Roll) * sin(Angle->Pitch);
	 a_hy = Input->Accy * cos(Angle->Roll) - Input->Accz * sin(Angle->Roll);

	 Angle->AngleBeta= (atan(LENGTH_REAR*tan(EKF->FirStee))/LENGTH_CAR);

	 EKF->NexPx= EKF->FirPx + (EKF->FirVelx * cos(Angle->AngleBeta+EKF->FirHea))*Input->Time;

	 EKF->NexPy= EKF->FirPy + (EKF->FirVely * sin(Angle->AngleBeta +EKF->FirHea))*Input->Time;

	 EKF->NexVelx= EKF->FirVelx + a_hx *Input->Time;

	 EKF->NexVely= EKF->FirVely + a_hy *Input->Time;

	 EKF->NexHea= EKF->FirHea + ((EKF->FirVely*tan(EKF->FirStee)*cos(Angle->AngleBeta))/LENGTH_CAR)*Input->Time;

	 EKF->NexStee= EKF->FirStee + Input->Stee*Input->Time;
// Prediction Covariance
	 float Mul_Result[6][6];
	 float Trans_Result[6][6];
	 float Jacobian[6][6];

	 memset(Trans_Result,0,sizeof(Trans_Result));
	 memset(Jacobian,0,sizeof(Jacobian));

	 Jacobian[0][0]=1;
	 Jacobian[0][1]=0;
	 Jacobian[0][2]=Input->Time*cos(EKF->FirHea+atan((LENGTH_REAR*tan(EKF->FirStee))/LENGTH_CAR));
	 Jacobian[0][3]=0;
	 Jacobian[0][4]=-EKF->FirVelx*Input->Time*sin(EKF->FirHea+atan(LENGTH_REAR*tan(EKF->FirStee)/LENGTH_CAR));
	 Jacobian[0][5]=-(EKF->FirVelx *LENGTH_CAR * Input->Time *sin(EKF->FirHea+atan(LENGTH_REAR*tan(EKF->FirStee)/LENGTH_CAR))*1/cos(EKF->FirStee)*cos(EKF->FirStee))/LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(EKF->FirStee)*tan(EKF->FirStee);

	 Jacobian[1][0]=0;
	 Jacobian[1][1]=1;
	 Jacobian[1][2]=0;
	 Jacobian[1][3]=Input->Time*sin(EKF->FirHea+atan((LENGTH_REAR*tan(EKF->FirStee))/LENGTH_CAR));
	 Jacobian[1][4]=EKF->FirVely*sin(EKF->FirHea+atan(LENGTH_REAR*tan(EKF->FirStee)/LENGTH_CAR));
	 Jacobian[1][5]=(EKF->FirVely *LENGTH_CAR * Input->Time *cos(EKF->FirHea+atan(LENGTH_REAR*tan(EKF->FirStee)/LENGTH_CAR))*1/cos(EKF->FirStee)*cos(EKF->FirStee))/LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(EKF->FirStee)*tan(EKF->FirStee);

	 Jacobian[2][0]=0;
	 Jacobian[2][1]=0;
	 Jacobian[2][2]=1;
	 Jacobian[2][3]=0;
	 Jacobian[2][4]=0;
	 Jacobian[2][5]=0;

	 Jacobian[3][0]=0;
	 Jacobian[3][1]=0;
	 Jacobian[3][2]=0;
	 Jacobian[3][3]=1;
	 Jacobian[3][4]=0;
	 Jacobian[3][5]=0;

	 Jacobian[4][0]=0;
	 Jacobian[4][1]=0;
	 Jacobian[4][2]=0;
	 Jacobian[4][3]=(Input->Time*tan(EKF->FirStee)*fabs(EKF->FirStee))/(EKF->FirStee*sqrt(EKF->FirStee*EKF->FirStee+LENGTH_REAR*LENGTH_REAR*tan(EKF->FirStee)*tan(EKF->FirStee)));
	 Jacobian[4][4]=1;
	 Jacobian[4][5]=(EKF->FirVely*LENGTH_CAR*LENGTH_CAR*LENGTH_CAR*Input->Time*(1/cos(EKF->FirStee)*1/cos(EKF->FirStee)))/(pow(LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(EKF->FirStee)*tan(EKF->FirStee),1.5)*fabs(EKF->FirStee));

	 Jacobian[5][0]=0;
	 Jacobian[5][1]=0;
	 Jacobian[5][2]=0;
	 Jacobian[5][3]=0;
	 Jacobian[5][4]=0;
	 Jacobian[5][5]=1;

	 multiplyMatrices(Jacobian,EKF->Prediction_CovarianceFir, Mul_Result,6,6);
	 transposeMatrix(Jacobian, Trans_Result,6,6);
	 multiplyMatrices(Mul_Result,Trans_Result,EKF->Prediction_CovarianceNex,6,6);
}
