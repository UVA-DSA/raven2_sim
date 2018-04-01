#pragma once
#ifndef INVERSE_H
#define INVERSE_H

// Use this only for square matrices
// Written by Pramod Chembrammel, VR Lab, University at Buffalo, NY, USA
// 12/5/2013

#include "tnt.h"
#include "jama_lu.h"
#include "math.h"

#include <cstdlib>
#include <cassert>

using namespace std;
using namespace TNT;
using namespace JAMA;

// calculates the generalised inverse of square matrix A(m,m)
// calculation follows Cholesky's decompolsition method


template <class T>
Array2D<T>	inverse(const Array2D<T> &A)
{
	int m = A.dim1();
	// Transpose of A
	
	Array2D<double> eye(m,m);
		for (int i = 0; i<m; i++)
			for (int j = 0; j<m; j++)
			{
				if (j == i)
					eye[i][j] = 1.;
				else
					eye[i][j] = 0.;
			}

	LU<double> ILU(A);
	Array2D<double> Ainv = ILU.solve(eye);
	return Ainv;
}

#endif
