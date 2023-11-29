/*
 * math.h
 *
 *  Created on: Nov 22, 2023
 *      Author: win 10
 */

#ifndef EKFMATH_H_
#define EKFMATH_H_
#include <EKF.h>
#include "main.h"
void transposeMatrix(float matrix[SIZE][SIZE], float result[SIZE][SIZE], int row, int col);
void assignMatrix(float sourceMatrix[SIZE][SIZE], float destinationMatrix[SIZE][SIZE]);
void subtractMatrices(float firstMatrix[SIZE][SIZE], float secondMatrix[SIZE][SIZE], float result[SIZE][SIZE], int row, int column);
void addScalarToMatrix(int rows,int cols,float matrix[rows][cols], float scalar);
#endif /* EKFMATH_H_ */
