#ifndef _NNLS_
#define _NNLS_

int nnls_c(double* a, const int* mda, const int* m, const int* n, double* b, 
	 double* x, double* rnorm, double* w, double* zz, int* index, 
	 int *mode);

#endif