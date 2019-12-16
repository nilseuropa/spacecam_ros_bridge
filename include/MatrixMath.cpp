#include "MatrixMath.h"

#define NR_END 1

MatrixMath Matrix;			// Pre-instantiate

void MatrixMath::copy(double* A, int n, int m, double* B)
{
	int i, j, k;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
		{
			B[n * i + j] = A[n * i + j];
		}
}

//Matrix Multiplication Routine
// C = A*B
void MatrixMath::multiply(double* A, double* B, int m, int p, int n, double* C)
{
	// A = input matrix (m x p)
	// B = input matrix (p x n)
	// m = number of rows in A
	// p = number of columns in A = number of rows in B
	// n = number of columns in B
	// C = output matrix = A*B (m x n)
	int i, j, k;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
		{
			C[n * i + j] = 0;
			for (k = 0; k < p; k++)
				C[n * i + j] = C[n * i + j] + A[p * i + k] * B[n * k + j];
		}
}


//Matrix Addition Routine
void MatrixMath::add(double* A, double* B, int m, int n, double* C)
{
	// A = input matrix (m x n)
	// B = input matrix (m x n)
	// m = number of rows in A = number of rows in B
	// n = number of columns in A = number of columns in B
	// C = output matrix = A+B (m x n)
	int i, j;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
			C[n * i + j] = A[n * i + j] + B[n * i + j];
}


//Matrix Subtraction Routine
void MatrixMath::subtract(double* A, double* B, int m, int n, double* C)
{
	// A = input matrix (m x n)
	// B = input matrix (m x n)
	// m = number of rows in A = number of rows in B
	// n = number of columns in A = number of columns in B
	// C = output matrix = A-B (m x n)
	int i, j;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
			C[n * i + j] = A[n * i + j] - B[n * i + j];
}


//Matrix Transpose Routine
void MatrixMath::transpose(double* A, int m, int n, double* C)
{
	// A = input matrix (m x n)
	// m = number of rows in A
	// n = number of columns in A
	// C = output matrix = the transpose of A (n x m)
	int i, j;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
			C[m * j + i] = A[n * i + j];
}

void MatrixMath::scale(double* A, int m, int n, double k)
{
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			A[n * i + j] = A[n * i + j] * k;
}


//Matrix Inversion Routine
// * This function inverts a matrix based on the Gauss Jordan method.
// * Specifically, it uses partial pivoting to improve numeric stability.
// * The algorithm is drawn from those presented in
//	 NUMERICAL RECIPES: The Art of Scientific Computing.
// * The function returns 1 on success, 0 on failure.
// * NOTE: The argument is ALSO the result matrix, meaning the input matrix is REPLACED
int MatrixMath::invert(double* A, int n)
{
	// A = input matrix AND result matrix
	// n = number of rows = number of columns in A (n x n)
	int pivrow;		// keeps track of current pivot row
	int k, i, j;		// k: overall index along diagonal; i: row index; j: col index
	int pivrows[n]; // keeps track of rows swaps to undo at end
	double tmp;		// used for finding max value and making column swaps

	for (k = 0; k < n; k++)
	{
		// find pivot row, the row with biggest entry in current column
		tmp = 0;
		for (i = k; i < n; i++)
		{
			if (abs(A[i * n + k]) >= tmp)	// 'Avoid using other functions inside abs()?'
			{
				tmp = abs(A[i * n + k]);
				pivrow = i;
			}
		}

		// check for singular matrix
		if (A[pivrow * n + k] == 0.0f)
		{
			//Serial.println("Inversion failed due to singular matrix");
			return 0;
		}

		// Execute pivot (row swap) if needed
		if (pivrow != k)
		{
			// swap row k with pivrow
			for (j = 0; j < n; j++)
			{
				tmp = A[k * n + j];
				A[k * n + j] = A[pivrow * n + j];
				A[pivrow * n + j] = tmp;
			}
		}
		pivrows[k] = pivrow;	// record row swap (even if no swap happened)

		tmp = 1.0f / A[k * n + k];	// invert pivot element
		A[k * n + k] = 1.0f;		// This element of input matrix becomes result matrix

		// Perform row reduction (divide every element by pivot)
		for (j = 0; j < n; j++)
		{
			A[k * n + j] = A[k * n + j] * tmp;
		}

		// Now eliminate all other entries in this column
		for (i = 0; i < n; i++)
		{
			if (i != k)
			{
				tmp = A[i * n + k];
				A[i * n + k] = 0.0f; // The other place where in matrix becomes result mat
				for (j = 0; j < n; j++)
				{
					A[i * n + j] = A[i * n + j] - A[k * n + j] * tmp;
				}
			}
		}
	}

	// Done, now need to undo pivot row swaps by doing column swaps in reverse order
	for (k = n - 1; k >= 0; k--)
	{
		if (pivrows[k] != k)
		{
			for (i = 0; i < n; i++)
			{
				tmp = A[i * n + k];
				A[i * n + k] = A[i * n + pivrows[k]];
				A[i * n + pivrows[k]] = tmp;
			}
		}
	}
	return 1;
}

cMatrix::cMatrix(const int m, const int n) : m(m), n(n) {
	A = new double[m * n];
	this->eye();
}

cMatrix::cMatrix(const int m, const int n, const double A[]) : m(m), n(n) {
	this->A = new double[m * n];
	for (int i = 0; i < m * n; i++) this->A[i] = A[i];
}

cMatrix::~cMatrix() {
	delete [] A;
}

cMatrix& cMatrix::set(const double A[]) {
	for (int i = 0; i < m * n; i++) this->A[i] = A[i];
	return *this;
}

cMatrix& cMatrix::eye() {
	for (int j = 0; j < m; j++)
		for (int i = 0; i < n; i++)
			A[j * n + i] = i == j ? 1.0 : 0.0;
	return *this;
}

cMatrix& cMatrix::zero() {
	for (int j = 0; j < m; j++)
		for (int i = 0; i < n; i++)
			A[j * n + i] = 0.0;
	return *this;
}

cMatrix cMatrix::transpose() {
	cMatrix R(n, m);
	for (int j = 0; j < m; j++)
		for (int i = 0; i < n; i++)
			R.A[i * m + j] = A[j * n + i];
	return R;
}

cMatrix cMatrix::operator*(const cMatrix& B) {
	cMatrix R(m, B.n);
	for (int j = 0; j < m; j++)
		for (int i = 0; i < B.n; i++) {
			R.A[j * B.n + i] = 0.0;
			for (int k = 0; k < n; k++)
				R.A[j * B.n + i] += A[j * n + k] * B.A[k * B.n + i];
		}
	return R;
}

cMatrix cMatrix::operator*(const double s) {
	cMatrix R(m, n);
	for (int j = 0; j < m; j++)
		for (int i = 0; i < n; i++)
			R.A[j * n + i] = A[j * n + i] * s;
	return R;
}

cMatrix cMatrix::operator+(const cMatrix& B) {
	cMatrix R(m, n);
	for (int j = 0; j < m; j++)
		for (int i = 0; i < n; i++)
			R.A[j * n + i] = A[j * n + i] + B.A[j * n + i];
	return R;
}

cMatrix cMatrix::operator-(const cMatrix& B) {
	cMatrix R(m, n);
	for (int j = 0; j < m; j++)
		for (int i = 0; i < n; i++)
			R.A[j * n + i] = A[j * n + i] - B.A[j * n + i];
	return R;
}

cMatrix& cMatrix::operator=(const cMatrix& B) {
	if (m * n != B.m * B.n) {
		delete [] A;
		A = new double[B.m * B.n];
	}
	m = B.m;
	n = B.n;

	for (int i = 0; i < m * n; i++) A[i] = B.A[i];
	return *this;
}

void cMatrix::householderDecomposition(cMatrix& Q, cMatrix& R) {
	double mag, alpha;
	cMatrix u(m, 1), v(m, 1);
	cMatrix P(m, m), I(m, m);

	Q = cMatrix(m, m);
	R = *this;

	for (int i = 0; i < n; i++) {
		u.zero(); v.zero();

		mag = 0.0;
		for (int j = i; j < m; j++) {
			u.A[j] = R.A[j * n + i];
			mag += u.A[j] * u.A[j];
		}
		mag = sqrt(mag);

		alpha = u.A[i] < 0 ? mag : -mag;

		mag = 0.0;
		for (int j = i; j < m; j++) {
			v.A[j] = j == i ? u.A[j] + alpha : u.A[j];
			mag += v.A[j] * v.A[j];
		}
		mag = sqrt(mag);

		if (mag < 0.0000000001) continue;

		for (int j = i; j < m; j++) v.A[j] /= mag;

		P = I - (v * v.transpose()) * 2.0;

		R = P * R;
		Q = Q * P;
	}
}
