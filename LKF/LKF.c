/*
 * LKF.c
 *
 *  Created on: Nov 11, 2023
 *      Author: win 10
 */
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "LKF.h"

void multiplyMatrices(float firstMatrix[SIZE][SIZE], float secondMatrix[SIZE][SIZE], float result[SIZE][SIZE], int row, int col) {
    // Thực hiện phép nhân
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            result[i][j] = 0;
            for (int k = 0; k < col; ++k) {
                result[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
            }
        }
    }
}

void transposeMatrix(float matrix[SIZE][SIZE], float result[SIZE][SIZE], int row, int col) {
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            result[j][i] = matrix[i][j];
        }
    }
}

void EKF_Init(LKF *LKF,Input *Input)
{
//First Step
	LKF->FriPx= 106.802343;  //input
	LKF->FriPy=10.869826; //input
	LKF->FriVelx=0; //input
 	LKF->CovVely=0; //input
	LKF->FriHea=-PI/6; //input
	LKF->FriStee=PI/6; //input
//Next Step
	LKF->NexPx=0;
	LKF->NexPy=0;
	LKF->NexVelx=0;
	LKF->NexVely=0;
	LKF->NexHea=0;
	LKF->NexStee=0;
//Covariance
	LKF->CovPx=0; //input
	LKF->CovPy=0; //input
	LKF->CovVelx=0;//input
	LKF->CovVely=0;//input
	LKF->CovHea=0; //input
	LKF->CovStee=0; //input
//FirstIput
	Input->Accx=0.2;
	Input->Accy=0.2;
	Input->Accz=0.2;
	Input->Time=0.1; //input
//Covariance
	memset(LKF->Prediction_CovarianceNex,0,sizeof(LKF->Prediction_CovarianceNex));
	memset(LKF->Prediction_CovarianceFri,0,sizeof(LKF->Prediction_CovarianceFri));
	LKF->Prediction_CovarianceFri[0][0]= LKF->CovPx;
	LKF->Prediction_CovarianceFri[1][1]= LKF->CovPy;
	LKF->Prediction_CovarianceFri[2][2]= LKF->CovVelx;
	LKF->Prediction_CovarianceFri[3][3]= LKF->CovVely;
	LKF->Prediction_CovarianceFri[4][4]= LKF->CovHea;
	LKF->Prediction_CovarianceFri[5][5]= LKF->CovStee;
}

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
void EKF_PredictionStep(LKF *LKF, Angle *Angle, Input *Input){
// Prediction State
	float a_hx;
	float a_hy;

	 a_hx = Input->Accx * cos(Angle->Pitch) + Input->Accy * sin(Angle->Roll) * sin(Angle->Pitch) + Input->Accz * cos(Angle->Roll) * sin(Angle->Pitch);
	 a_hy = Input->Accy * cos(Angle->Roll) - Input->Accz * sin(Angle->Roll);

	 Angle->AngleBeta= (atan(LENGTH_REAR*tan(LKF->FriStee))/LENGTH_CAR);

	 LKF->NexPx= LKF->FriPx + (LKF->FriVelx * cos(Angle->AngleBeta+LKF->FriHea))*Input->Time;

	 LKF->NexPy= LKF->FriPy + (LKF->FriVely * sin(Angle->AngleBeta +LKF->FriHea))*Input->Time;

	 LKF->NexVelx= LKF->FriVelx + a_hx *Input->Time;

	 LKF->NexVely= LKF->FriVely + a_hy *Input->Time;

	 LKF->NexHea= LKF->FriHea + ((LKF->FriVely*tan(LKF->FriStee)*cos(Angle->AngleBeta))/LENGTH_CAR)*Input->Time;

	 LKF->NexStee= LKF->FriStee + Input->Stee*Input->Time;
// Prediction Covariance
	 float Mul_Result[6][6];
	 float Trans_Result[6][6];
	 float Jacobian[6][6];

	 memset(Trans_Result,0,sizeof(Trans_Result));
	 memset(Jacobian,0,sizeof(Jacobian));

	 Jacobian[0][0]=1;
	 Jacobian[0][1]=0;
	 Jacobian[0][2]=Input->Time*cos(LKF->FriHea+atan((LENGTH_REAR*tan(LKF->FriStee))/LENGTH_CAR));
	 Jacobian[0][3]=0;
	 Jacobian[0][4]=-LKF->FriVelx*Input->Time*sin(LKF->FriHea+atan(LENGTH_REAR*tan(LKF->FriStee)/LENGTH_CAR));
	 Jacobian[0][5]=-(LKF->FriVelx *LENGTH_CAR * Input->Time *sin(LKF->FriHea+atan(LENGTH_REAR*tan(LKF->FriStee)/LENGTH_CAR))*1/cos(LKF->FriStee)*cos(LKF->FriStee))/LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee);

	 Jacobian[1][0]=0;
	 Jacobian[1][1]=1;
	 Jacobian[1][2]=0;
	 Jacobian[1][3]=Input->Time*sin(LKF->FriHea+atan((LENGTH_REAR*tan(LKF->FriStee))/LENGTH_CAR));
	 Jacobian[1][4]=LKF->FriVely*sin(LKF->FriHea+atan(LENGTH_REAR*tan(LKF->FriStee)/LENGTH_CAR));
	 Jacobian[1][5]=(LKF->FriVely *LENGTH_CAR * Input->Time *cos(LKF->FriHea+atan(LENGTH_REAR*tan(LKF->FriStee)/LENGTH_CAR))*1/cos(LKF->FriStee)*cos(LKF->FriStee))/LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee);

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
	 Jacobian[4][3]=(Input->Time*tan(LKF->FriStee)*fabs(LKF->FriStee))/(LKF->FriStee*sqrt(LKF->FriStee*LKF->FriStee+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee)));
	 Jacobian[4][4]=1;
	 Jacobian[4][5]=(LKF->FriVely*LENGTH_CAR*LENGTH_CAR*LENGTH_CAR*Input->Time*(1/cos(LKF->FriStee)*1/cos(LKF->FriStee)))/(pow(LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee),1.5)*fabs(LKF->FriStee));

	 Jacobian[5][0]=0;
	 Jacobian[5][1]=0;
	 Jacobian[5][2]=0;
	 Jacobian[5][3]=0;
	 Jacobian[5][4]=0;
	 Jacobian[5][5]=1;

	 multiplyMatrices(Jacobian,LKF->Prediction_CovarianceFri, Mul_Result,6,6);
	 transposeMatrix(Jacobian, Trans_Result,6,6);
	 multiplyMatrices(Mul_Result,Trans_Result,LKF->Prediction_CovarianceNex,6,6);


	 // Covaricaace result

}
void EFK_GPSHandleMeasurement(GPS *GPS, LKF *LKF ){
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
	Error[0]=GPSGetPos[0] - LKF->NexPx;
	Error[1]=GPSGetPos[1] - LKF->NexPy;
	//InovationCovarian
	InovationCovariance[0][0]=LKF->Prediction_CovarianceNex[0][0] + GPSCovariance[0][0];
	InovationCovariance[1][1]=LKF->Prediction_CovarianceNex[1][1]+  GPSCovariance[1][1];
	//KalmanGain
	KalmanGain[0][0]= LKF->Prediction_CovarianceNex[0][0]/(InovationCovariance[0][0]);
	KalmanGain[1][1]=LKF->Prediction_CovarianceNex[1][1]/(InovationCovariance[1][1]);
	//Update State
	LKF->FriPx = LKF->NexPx + (KalmanGain[0][0] * Error[0]);
	LKF->FriPy = LKF->NexPy + (KalmanGain[1][1] * Error[1]);
	//Update Covariance
	LKF->Prediction_CovarianceFri[0][0]= (GPSCovariance[0][0] / (InovationCovariance[0][0]))*LKF->Prediction_CovarianceNex[0][0];
	LKF->Prediction_CovarianceFri[1][1]= (GPSCovariance[1][1] / (InovationCovariance[0][0]))*LKF->Prediction_CovarianceNex[1][1];

}

void Heading_HandleMeasurement(LKF *LKF, Input *Input, Heading *Heading){
	float 	 Jacobian[6];
	float 	 Error;
	float InovationCovariance;
	float KalmanGain[6];

			 Jacobian[0]=0;
			 Jacobian[1]=0;
			 Jacobian[2]=0;
			 Jacobian[3]=(Input->Time*tan(LKF->FriStee)*fabs(LKF->FriStee))/(LKF->FriStee*sqrt(LKF->FriStee*LKF->FriStee+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee)));
			 Jacobian[4]=1;
			 Jacobian[5]=(LKF->FriVely*LENGTH_CAR*LENGTH_CAR*LENGTH_CAR*Input->Time*(1/cos(LKF->FriStee)*1/cos(LKF->FriStee)))/(pow(LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee),1.5)*fabs(LKF->FriStee));
	Error= Heading->Yaw - LKF->NexHea;

	InovationCovariance= (LKF->Prediction_CovarianceNex[0][0]* Jacobian[0]* Jacobian[0]) + (LKF->Prediction_CovarianceNex[3][3]* Jacobian[3]* Jacobian[3]) + (LKF->Prediction_CovarianceNex[4][4]* Jacobian[4]* Jacobian[4]) +(LKF->Prediction_CovarianceNex[1][1]* Jacobian[1]* Jacobian[1])+(LKF->Prediction_CovarianceNex[5][5]* Jacobian[5]* Jacobian[5]) + (LKF->Prediction_CovarianceNex[2][2]* Jacobian[2]* Jacobian[2])+ Heading->Covariane;

	KalmanGain[0]= (LKF->Prediction_CovarianceNex[0][0]* Jacobian[0]/InovationCovariance);
	KalmanGain[1]= (LKF->Prediction_CovarianceNex[1][1]* Jacobian[1]/InovationCovariance);
	KalmanGain[2]= (LKF->Prediction_CovarianceNex[2][2]* Jacobian[2]/InovationCovariance);
	KalmanGain[3]= (LKF->Prediction_CovarianceNex[3][3]* Jacobian[3]/InovationCovariance);
	KalmanGain[4]= (LKF->Prediction_CovarianceNex[4][4]* Jacobian[4]/InovationCovariance);
	KalmanGain[5]= (LKF->Prediction_CovarianceNex[5][5]* Jacobian[5]/InovationCovariance);

	LKF->FriPx = LKF->NexPx + KalmanGain[0] * Error;
	LKF->FriPy = LKF->NexPy + KalmanGain[1] * Error;
	LKF->FriVelx = LKF->NexVelx + KalmanGain[2] * Error;
	LKF->FriVely = LKF->NexVely + KalmanGain[3] * Error;
	LKF->FriHea = LKF->NexHea + KalmanGain[4] * Error;
	LKF->FriStee = LKF->NexStee + KalmanGain[5] * Error;
}



//void Heading_HandleMeasurement(LKF *LKF, Input *Input, Heading *Heading){
//	float Jacobian[6][6];
//	float JacobianT[6][6];
//	float JacobianIn[6][6];
//	float Inovation;
//	float Measurement[6][6];
//	float InovationCov[6][6];
//	float InovationCovInv[6][6];
//	float InovationResul[6][6];
//	float KalmanGain[6][6];
//	float KalmanGainb1[6][6];
//	float resultb1[6][6];
//	float resultb2[6][6];
//	float Indentity[6][6];
//	float KalmanGiansub[6][6];
//	float CovarianceResult[6][6];
//	float CovarianceResultb1[6][6];
//
//	Indentity[0][0]=1;
//	Indentity[1][1]=1;
//	Indentity[2][2]=1;
//	Indentity[3][3]=1;
//	Indentity[4][4]=1;
//	Indentity[5][5]=1;
//
//		 Jacobian[0][0]=0;
//		 Jacobian[0][1]=0;
//		 Jacobian[0][2]=0;
//		 Jacobian[0][3]=(Input->Time*tan(LKF->FriStee)*fabs(LKF->FriStee))/(LKF->FriStee*sqrt(LKF->FriStee*LKF->FriStee+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee)));
//		 Jacobian[0][4]=1;
//		 Jacobian[0][5]=(LKF->FriVely*LENGTH_CAR*LENGTH_CAR*LENGTH_CAR*Input->Time*(1/cos(LKF->FriStee)*1/cos(LKF->FriStee)))/(pow(LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee),1.5)*fabs(LKF->FriStee));
//
//	Inovation = Heading->Yaw - LKF->NexHea;
//
//	matrixMultiplication(Jacobian, LKF->Prediction_CovarianceNex, resultb1);
//	transposeMatrixNxN(Jacobian, JacobianT);
//	matrixMultiplication(resultb1,JacobianT,InovationResul);
//	addScalarToMatrix(6, 6, InovationResul,Heading->Covariane);
//
//
//	matrixMultiplication(LKF->Prediction_CovarianceNex,JacobianT,KalmanGainb1);
//
//	inverseMatrix(Jacobian, JacobianIn);
//
//	matrixMultiplication(JacobianIn,KalmanGainb1,KalmanGain);
//
//	assignMatrix(KalmanGain, KalmanGiansub, 6, 6);
//
//	multiplyMatrixByScalar(KalmanGain, 6, 6, Inovation);
//
//	LKF->FriHea= LKF->NexHea + KalmanGain[0][5];
//
//	matrixMultiplication(KalmanGiansub, Jacobian, CovarianceResult);
//
//	subtractMatrices(Indentity, CovarianceResult, CovarianceResultb1, 6, 6);
//
//	matrixMultiplication(CovarianceResultb1, LKF->Prediction_CovarianceNex,  LKF->Prediction_CovarianceFri);
//
//}
