#include "vector_matrix.h"


/**********************************************************
 * matMulDiagmat : 
 * Multiplication of a matrix and a diagonal matrix
 * B = A*D 
 * 여기서 D는 diagonal matrix로써 실제로는 vector 형태로 
 * data를 저장한다. D={d1,d2,d3}라면 Matlab의 daug(d1,d2,d3)
 * 형태의 대각행렬을 생각하면 된다.
 * A : input matrix
 * D : input vector for diagonal matrix
 * B : = A*D
 * row, col : A의 dimension 정보 
 * D의 dimension 정보는 자동적으로 A의 dimension에 종속됨
 **********************************************************/
void matMulDiagmat(double *A, double *D, double *B, int row, int col)
{
	int i, j;
	for (i=0;i<row;i++)
		for(j=0;j<col;j++)
			*(B+i*col+j) = *(A+i*col+j)*D[j];

}

/**********************************************************
 * matMulMat
 * Multiplication of two matrices
 * C = A*B 
 * A : row-by-col_tmp (input matrix)
 * B : col_tmp-by-col (input matrix)
 * C : row-by-col     (output matrix)
 **********************************************************/
void matMulMat(double *A, double *B, double *C, int row, int col_tmp, int col)
{
	int i,j,k;
	double sum;
	for(i=0;i<row;i++){
		for(j=0;j<col;j++){
			sum=0.0;
			for(k=0;k<col_tmp;k++){
				sum+= (*(A+i+k*row))*(*(B+k+j*col_tmp));
			}
			*(C+i+j*row)=sum;
		}
	}

//	double *A = AAA;

//	*(*(C+0)+0) = *(*(A+0)+0)
//	for(i=0;i<row;i++){
//		for(j=0;j<col;j++){
//			sum = 0.0;
//			for(k=0;k<col_tmp;k++){
////				sum += (*A)[i][k] * (*B)[k][j];
//				sum += (*(*(A+i)+k)) * (*(*(B+k)+j));
//			}
//			*(*(C+i)+j) = sum;
////			*(C+i+j) = sum;
//		}
//	}

}

/******************************************************
 * matMulVec
 * Multiplication of a matrix and a vector
 * C = A*B
 * A : row-by-col matrix
 * B : vector whose length is col
 * C : output vector whose length is also row
 ******************************************************/
void matMulVec(double *A, double *B, double *C, int row, int col)
{
	double sum;
	int i,j;
	for(i=0;i<row;i++){
		sum = 0.0;
		for(j=0;j<col;j++){
			sum += *(A+i+j*row)*B[j];
		}
		C[i] = sum;
	}
}

/******************************************************

 ******************************************************/
void matMulScalar(double *A, double s, double *C, int row, int col)
{
	int i,j;
	for(i=0;i<row;i++){
		for(j=0;j<col;j++){
			*(C+i+j*row) = *(A+i+j*row)*s;
		}
	}
}


/******************************************************
 * vecSubtract
 * Vector subtraction
 * C = A-B
 * A : length row vector
 * B : length row vector
 * C : length row outut vector
 ******************************************************/
void vecSubtract(double *A, double *B, double *C, int row)
{
	int i;
	for(i=0;i<row;i++)
		C[i] = A[i]-B[i];
}
void vecAdd(double *A, double *B, double *C, int row)
{
	int i;
	for(i=0;i<row;i++)
		C[i] = A[i]+B[i];
}
/********************************************
 * 행렬의 전치행렬을 구한다.
 * A : row-by-col 입력행렬
 * B : col-by-row 출력행렬
 ********************************************/
void transpose(double *A, double *B, int row, int col)
{
	int i,j;
	for(i=0;i<row;i++){
		for(j=0;j<col;j++){
			*(B+j+i*col) = *(A+i+j*row);
		}
	}

}


/******************************************************
 * innderProd
 * computes the inner product of two vectors
 * z = x'y
 * x : vector (length m)
 * y : vector (length m)
 * z : inner product value
 * m : length of vectors
 ******************************************************/
void innerProd(double *x, double *y, double* z, int m)
{
	int i;
	double sum = 0;
	for(i=0;i<m;i++) sum += x[i]*y[i];
	*z = sum;
}

/*********************************************************************** 
 * Function vstack : Computes the vertical stack of two matrices
 *                   
 * -- Input Parameters --
 *
 * m1 : int, the number of rows a matrix A
 * m2 : int, the number of rows a matrix B
 * n : int, the number of columns of matrices A, B. *
 * A : double *, array(m1-by-n)
 * B : double *, array(m2-by-n)
 *
 * -- Output Parameters --
 * C : double *, array((m1+m2)-by-n) : contains the vetically stacked 
 *     matrix of A and B, that is, C=[A;B];
 * 
 ***********************************************************************/
void vstack(double *A, double *B, double *C, int m1, int m2, int n)
{
	int i, j, m;
	m = m1 + m2;

	for(i=0;i<m1;i++)
		for(j=0;j<n;j++)
			*(C+i*n+j) = *(A+i*n+j);

	for(i=0;i<m2;i++)
		for(j=0;j<n;j++)
			*(C+m1*n+i*n+j) = *(B+i*n+j);
}

// Inversion of N by N upper triangular matrix
void inverseU(double *L, double *invL, int n)
{
	int i, j, k;
	double sum = 0;
	for (i = n - 1; i >= 0; i--) {
		for (j = 0; j < n; j++) {
			if (i > j)
				invL[i + j * n] = 0;
			else if (i == j)
				invL[i + j * n] = 1 / L[i + j * n];
			else {
				sum = 0;
				for (k = 0; k < n; k++) {
					if (k == i)
						continue;
					else
						sum = sum + L[i + k * n] * invL[k + j * n];
				}
				invL[i + j * n] = -sum / L[i + i * n];
			}
		}
	}
}

// Cholesky Factorization. 
void chol(double *P, double *L, int n)
{
	int k, i, j;
	double sum = 0;

	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			L[i + j * n] = 0;
		}
	}
	for (k = 0; k < n; k++) {
		sum = 0;
		for (i = 0; i <= k - 1; i++)
			sum = sum + L[i + k * n] * L[i + k * n];
		L[k + k * n] = sqrt(P[k + k * n] - sum);
		for (i = k + 1; i < n; i++) {
			sum = 0;
			for (j = 0; j <= k - 1; j++)
				sum = sum + L[j + k * n] * L[j + i * n];
			L[k + i * n] = (P[k + i * n] - sum) / L[k + k * n];
		}
	}
}
