// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __CU_MATRIXDECOMPOSITION_CUH__
#define __CU_MATRIXDECOMPOSITION_CUH__

#include "foundation/PxMat33.h"

namespace physx {

// Eigen decomposition code thanks to Matthias Mueller-Fischer!
template<int p, int q, int k> __device__
inline void jacobiRotateT(float* A, float* R)
{
	const int pq_index = 3 * q + p;
	const int qp_index = 3 * p + q;
	const int pp_index = 3 * p + p;
	const int qq_index = 3 * q + q;

	// rotates A through phi in pq-plane to set A(p,q) = 0
	// rotation stored in R whose columns are eigenvectors of A
	if (A[pq_index] == 0.0f)
		return;

	float d = __fdividef(A[pp_index] - A[qq_index], 2.0f*A[pq_index]);
	float dSqPlus1 = d * d + 1.0f;
	float t = __fdividef(1.0f, fabs(d) + sqrtf(dSqPlus1));
	t = copysign(t, d);
	float c = 1.0f * rsqrtf(t*t + 1.0f);
	float s = t * c;

	A[pp_index] += t * A[pq_index];
	A[qq_index] -= t * A[pq_index];
	A[pq_index] = A[qp_index] = 0.0f;

	// transform A
	const int kp = p * 3 + k;
	const int kq = q * 3 + k;
	const int pk = k * 3 + p;
	const int qk = k * 3 + q;

	float Akp = c * A[kp] + s * A[kq];
	float Akq = -s * A[kp] + c * A[kq];
	A[kp] = A[pk] = Akp;
	A[kq] = A[qk] = Akq;

	// store rotation in R (loop unrolled for k = 0,1,2)
	// k = 0
	const int kp0 = p * 3 + 0;
	const int kq0 = q * 3 + 0;

	float Rkp0 = c * R[kp0] + s * R[kq0];
	float Rkq0 = -s * R[kp0] + c * R[kq0];
	R[kp0] = Rkp0;
	R[kq0] = Rkq0;

	// k = 1
	const int kp1 = p * 3 + 1;
	const int kq1 = q * 3 + 1;

	float Rkp1 = c * R[kp1] + s * R[kq1];
	float Rkq1 = -s * R[kp1] + c * R[kq1];
	R[kp1] = Rkp1;
	R[kq1] = Rkq1;

	// k = 2
	const int kp2 = p * 3 + 2;
	const int kq2 = q * 3 + 2;

	float Rkp2 = c * R[kp2] + s * R[kq2];
	float Rkq2 = -s * R[kp2] + c * R[kq2];
	R[kp2] = Rkp2;
	R[kq2] = Rkq2;
}

__device__
inline void jacobiRotate(PxMat33 &A, PxMat33 &R, int p, int q)
{
	// rotates A through phi in pq-plane to set A(p,q) = 0
	// rotation stored in R whose columns are eigenvectors of A
	if (A(p, q) == 0.0f)
		return;

	float d = (A(p, p) - A(q, q)) / (2.0f*A(p, q));
	float t = 1.0f / (fabs(d) + sqrtf(d*d + 1.0f));
	if (d < 0.0f) t = -t;
	float c = 1.0f / sqrtf(t*t + 1.0f);
	float s = t * c;
	A(p, p) += t * A(p, q);
	A(q, q) -= t * A(p, q);
	A(p, q) = A(q, p) = 0.0f;

	// transform A
	int k;
	for (k = 0; k < 3; k++) {
		if (k != p && k != q) {
			float Akp = c * A(k, p) + s * A(k, q);
			float Akq = -s * A(k, p) + c * A(k, q);
			A(k, p) = A(p, k) = Akp;
			A(k, q) = A(q, k) = Akq;
		}
	}

	// store rotation in R
	for (k = 0; k < 3; k++) {
		float Rkp = c * R(k, p) + s * R(k, q);
		float Rkq = -s * R(k, p) + c * R(k, q);
		R(k, p) = Rkp;
		R(k, q) = Rkq;
	}
}

__device__
inline void eigenDecomposition(PxMat33 &A, PxMat33 &R, int numJacobiIterations = 4)
{
	const float epsilon = 1e-15f;

	// only for symmetric matrices!
	R = PxMat33(PxVec3(1.f, 0.f, 0.f), PxVec3(0.f, 1.f, 0.f), PxVec3(0.f, 0.f, 1.f));

	float* fA = static_cast<float*>(&A(0, 0));
	float* fR = static_cast<float*>(&R(0, 0));

#define USE_FAST_JACOBI 1

	for (int i = 0; i < numJacobiIterations; i++)
	{// 3 off diagonal elements
		// find off diagonal element with maximum modulus
		int j = 0;
		float max = fabs(A(0, 1));
		float a = fabs(A(0, 2));
		if (a > max) { j = 1; max = a; }
		a = fabs(A(1, 2));
		if (a > max) { j = 2; max = a; }

		// all small enough -> done
		if (max < epsilon) break;

#if USE_FAST_JACOBI
		// rotate matrix with respect to that element
		if (j == 0) jacobiRotateT<0, 1, 2>(fA, fR);
		else if (j == 1) jacobiRotateT<0, 2, 1>(fA, fR);
		else jacobiRotateT<1, 2, 0>(fA, fR);
#else
		jacobiRotate(A, R, p, q);
#endif
	}
}

}

#endif  // __CU_MATRIXDECOMPOSITION_CUH__