/*
 * math.c
 *
 *  Created on: Nov 22, 2023
 *      Author: win 10
 */
#include <EKF.h>
#include "EKFmath.h"
void assignMatrix(float sourceMatrix[SIZE][SIZE], float destinationMatrix[SIZE][SIZE]) {
    int i, j;
    for (i = 0; i < SIZE; ++i) {
        for (j = 0; j < SIZE; ++j) {
            destinationMatrix[i][j] = sourceMatrix[i][j];
        }
    }
}

void subtractMatrices(float firstMatrix[SIZE][SIZE], float secondMatrix[SIZE][SIZE], float result[SIZE][SIZE], int row, int column) {
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < column; ++j) {
            result[i][j] = firstMatrix[i][j] - secondMatrix[i][j];
        }
    }
}

void multiplyMatrices(float firstMatrix[SIZE][SIZE], float secondMatrix[SIZE][SIZE], float result[SIZE][SIZE], int row, int col) {
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
