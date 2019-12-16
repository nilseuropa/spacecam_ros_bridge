#ifndef MatrixMath_h
#define MatrixMath_h
#include <math.h>

// #define MIN(a,b) ((a)<(b)?(a):(b))
// #define MAX(a,b) ((a)<(b)?(b):(a))

class MatrixMath
{
public:
	void copy(double* A, int n, int m, double* B);
	void multiply(double* A, double* B, int m, int p, int n, double* C);
	void add(double* A, double* B, int m, int n, double* C);
	void subtract(double* A, double* B, int m, int n, double* C);
	void transpose(double* A, int m, int n, double* C);
	void scale(double* A, int m, int n, double k);
	int  invert(double* A, int n);
};
extern MatrixMath Matrix;

class cMatrix {
  private:
  protected:
  public:
	double *A;
	int m, n;

	cMatrix(const int m, const int n);
	cMatrix(const int m, const int n, const double A[]);

	~cMatrix();

	cMatrix& set(const double A[]);
	cMatrix& eye();
	cMatrix& zero();
	cMatrix transpose();
	cMatrix  operator*(const cMatrix& B);
	cMatrix  operator*(const double s);
	cMatrix  operator+(const cMatrix& B);
	cMatrix  operator-(const cMatrix& B);
	cMatrix& operator=(const cMatrix& B);
	void householderDecomposition(cMatrix& Q, cMatrix& R);
	void output();
};

#endif
