#ifndef _VECTOR_MATRIX_H_
#define _VECTOR_MATRIX_H_

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


// Routines for multiplication of matrices and vectors.
void matMulDiagmat(double *A, double *D, double *B, int row, int col);
void matMulMat(double *A, double *B, double *C, int row, int col_tmp, int col);
void transpose(double *A, double *B, int row, int col);
void matMulVec(double *A, double *B, double *C, int row, int col);
void vecSubtract(double *A, double *B, double *C, int row);
void vstack(double *A, double *B, double *C, int m1, int m2, int n);
void innerProd(double *x, double *y, double* z, int m);
void matMulScalar(double *A, double s, double *C, int row, int col);
void vecAdd(double *A, double *B, double *C, int row);

int nnls_(double *a, int *mda, int *m, int *n, double *b, double *x, double *rnorm, 
		  double *w, double *zz, int *index, int *mode);
void chol(double*P, double*L, int n);
void inverseU(double*L, double*invL, int n);

#define DISPLAY 0

#endif 
