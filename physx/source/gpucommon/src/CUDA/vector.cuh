// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef __CU_COMMON_VECTOR_CUH__
#define __CU_COMMON_VECTOR_CUH__

#include "foundation/PxSimpleTypes.h"
#include "PxgCommonDefines.h"
#include "CmSpatialVector.h"
#include "reduction.cuh"
#include "MemoryAllocator.cuh"

using namespace physx;

static __device__ PxReal rotateR(
	const float vec,
	const PxQuat& quat,
	const PxU32 threadIndexInWarp)
{

	const float* q = reinterpret_cast<const float*>(&quat.x);

	const PxF32 w2 = q[3] * q[3] - 0.5f;

	float v = 0.f;
	float temp = 0.f;
	if (threadIndexInWarp < 3)
	{
		v = 2.f * vec;
		temp = q[threadIndexInWarp] * v;
	}

	const PxReal dot2 = __shfl_sync(FULL_MASK, temp, 0)
		+ __shfl_sync(FULL_MASK, temp, 1)
		+ __shfl_sync(FULL_MASK, temp, 2);

	/*
	temp[0] = (q.y * vz - q.z * vy)*q.w,
	temp[1] = (q.z * vx - q.x * vz)*q.w,
	temp[2] = (q.x * vy - q.y * vx)*q.w
	*/

	if (threadIndexInWarp < 6)
	{
		PxU32 ind0 = (threadIndexInWarp + 1) % 3;
		PxU32 ind1 = (threadIndexInWarp + 2) % 3;

		//thread 0, 1, 2 use ind0 for qInd and ind1 for vInd, thread 3, 4 , 5
		//use ind1 for qInd and ind0 for vInd
		const PxU32 qInd = (threadIndexInWarp < 3) ? ind0 : ind1;
		const PxU32 vInd = (threadIndexInWarp < 3) ? ind1 : ind0;

		//temp[threadIndexInWarp] = q[qInd] * v[vInd];
		const float tV = __shfl_sync(0x3F, v, vInd);
		temp = q[qInd] * tV;
	}

	//__threadfence_block();

	/*
	temp[0] += vx0*w2 + q.x*dot2[0],
	temp[1] += vy0*w2 + q.y*dot2[0],
	temp[2] += vz0*w2 + q.z*dot2[0],
	*/

	const PxReal tTemp = __shfl_sync(FULL_MASK, temp, threadIndexInWarp + 3);
	float out = 0.f;
	if (threadIndexInWarp < 3)
	{
		temp = (temp - tTemp) * q[3];
		//temp[threadIndexInWarp] = (temp[threadIndexInWarp] - temp[threadIndexInWarp + 3]) * q[3];
		out = temp + v * w2 + (q[threadIndexInWarp] * dot2);
	}

	return out;
}


static __device__ PxReal rotateR(
	const PxVec3& vec,
	const PxQuat& quat,
	const PxU32 threadIndexInWarp)
{
	float v = 0.f;
	if (threadIndexInWarp < 3)
		v = vec[threadIndexInWarp];

	return rotateR(v, quat, threadIndexInWarp);
}

static __device__ PxReal rotateR(
	const float4& vec,
	const PxQuat& quat,
	const PxU32 threadIndexInWarp)
{

	const PxVec3& in = reinterpret_cast<const PxVec3&>(vec);
	return rotateR(in, quat, threadIndexInWarp);
}

static __device__ void rotate(
	const PxVec3& in,
	const PxQuat& quat,
	PxVec3& out,
	const PxU32 threadIndexInWarp)
{

	const float outV = rotateR(in, quat, threadIndexInWarp);

	if (threadIndexInWarp < 3)
	{
		out[threadIndexInWarp] = outV;
	}
}


static __device__ void rotateAdd(
	const PxVec3& in,
	const PxQuat& quat,
	PxVec3& out,
	const PxU32 threadIndexInWarp)
{

	const float* q = reinterpret_cast<const float*>(&quat.x);

	const PxF32 w2 = q[3] * q[3] - 0.5f;

	float v = 0.f;
	float temp = 0.f;
	if (threadIndexInWarp < 3)
	{
		v = 2.f * in[threadIndexInWarp];
		temp = q[threadIndexInWarp] * v;
	}

	const PxReal dot2 = __shfl_sync(FULL_MASK, temp, 0)
		+ __shfl_sync(FULL_MASK, temp, 1)
		+ __shfl_sync(FULL_MASK, temp, 2);

	/*
	temp[0] = (q.y * vz - q.z * vy)*q.w,
	temp[1] = (q.z * vx - q.x * vz)*q.w,
	temp[2] = (q.x * vy - q.y * vx)*q.w
	*/

	if (threadIndexInWarp < 6)
	{
		PxU32 ind0 = (threadIndexInWarp + 1) % 3;
		PxU32 ind1 = (threadIndexInWarp + 2) % 3;

		//thread 0, 1, 2 use ind0 for qInd and ind1 for vInd, thread 3, 4 , 5
		//use ind1 for qInd and ind0 for vInd
		const PxU32 qInd = (threadIndexInWarp < 3) ? ind0 : ind1;
		const PxU32 vInd = (threadIndexInWarp < 3) ? ind1 : ind0;

		//temp[threadIndexInWarp] = q[qInd] * v[vInd];
		const float tV = __shfl_sync(0x3F, v, vInd);
		temp = q[qInd] * tV;
	}


	/*
	temp[0] += vx0*w2 + q.x*dot2[0],
	temp[1] += vy0*w2 + q.y*dot2[0],
	temp[2] += vz0*w2 + q.z*dot2[0],
	*/

	const PxReal tTemp = __shfl_sync(FULL_MASK, temp, threadIndexInWarp + 3);
	if (threadIndexInWarp < 3)
	{
		temp = (temp - tTemp) * q[3];
		//temp[threadIndexInWarp] = (temp[threadIndexInWarp] - temp[threadIndexInWarp + 3]) * q[3];
		out[threadIndexInWarp] += temp + v * w2 + (q[threadIndexInWarp] * dot2);
	}
}

static __device__ void rotateInv(
	const PxVec3& in,
	const PxQuat& quat,
	PxVec3& out,
	const PxU32 threadIndexInWarp)
{
	const float* q = reinterpret_cast<const float*>(&quat.x);

	const PxF32 w2 = q[3] * q[3] - 0.5f;

	float v = 0.f;
	float temp = 0.f;
	if (threadIndexInWarp < 3)
	{
		v = 2.f * in[threadIndexInWarp];
		temp = q[threadIndexInWarp] * v;
	}


	const PxReal dot2 = __shfl_sync(FULL_MASK, temp, 0)
		+ __shfl_sync(FULL_MASK, temp, 1)
		+ __shfl_sync(FULL_MASK, temp, 2);

	/*
	temp[0] = -(q.y * vz - q.z * vy)*q.w,
	temp[1] = -(q.z * vx - q.x * vz)*q.w,
	temp[2] = -(q.x * vy - q.y * vx)*q.w
	*/
	if (threadIndexInWarp < 6)
	{
		PxU32 ind0 = (threadIndexInWarp + 1) % 3;
		PxU32 ind1 = (threadIndexInWarp + 2) % 3;

		//thread 0, 1, 2 use ind0 for qInd and ind1 for vInd, thread 3, 4 , 5
		//use ind1 for qInd and ind0 for vInd
		const PxU32 qInd = (threadIndexInWarp < 3) ? ind0 : ind1;
		const PxU32 vInd = (threadIndexInWarp < 3) ? ind1 : ind0;

		const float tV = __shfl_sync(0x3F, v, vInd);

		temp = q[qInd] * tV;
	}


	/*
	temp[0] += vx0*w2 + q.x*dot2[0],
	temp[1] += vy0*w2 + q.y*dot2[0],
	temp[2] += vz0*w2 + q.z*dot2[0],
	*/
	const PxReal tTemp = __shfl_sync(FULL_MASK, temp, threadIndexInWarp + 3);

	if (threadIndexInWarp < 3)
	{
		temp = -(temp - tTemp) * q[3];
		out[threadIndexInWarp] = temp + v * w2 + (q[threadIndexInWarp] * dot2);
	}
}


static __device__ float rotateInvR(
	const float in,
	const PxQuat& quat,
	const PxU32 threadIndexInWarp)
{
	const float* q = reinterpret_cast<const float*>(&quat.x);

	const PxF32 w2 = q[3] * q[3] - 0.5f;

	float v = 0.f;
	float temp = 0.f;
	if (threadIndexInWarp < 3)
	{
		v = 2.f * in;
		temp = q[threadIndexInWarp] * v;
	}


	const PxReal dot2 = __shfl_sync(FULL_MASK, temp, 0)
		+ __shfl_sync(FULL_MASK, temp, 1)
		+ __shfl_sync(FULL_MASK, temp, 2);

	/*
	temp[0] = -(q.y * vz - q.z * vy)*q.w,
	temp[1] = -(q.z * vx - q.x * vz)*q.w,
	temp[2] = -(q.x * vy - q.y * vx)*q.w
	*/
	if (threadIndexInWarp < 6)
	{
		PxU32 ind0 = (threadIndexInWarp + 1) % 3;
		PxU32 ind1 = (threadIndexInWarp + 2) % 3;

		//thread 0, 1, 2 use ind0 for qInd and ind1 for vInd, thread 3, 4 , 5
		//use ind1 for qInd and ind0 for vInd
		const PxU32 qInd = (threadIndexInWarp < 3) ? ind0 : ind1;
		const PxU32 vInd = (threadIndexInWarp < 3) ? ind1 : ind0;

		const float tV = __shfl_sync(0x3F, v, vInd);

		temp = q[qInd] * tV;
	}


	/*
	temp[0] += vx0*w2 + q.x*dot2[0],
	temp[1] += vy0*w2 + q.y*dot2[0],
	temp[2] += vz0*w2 + q.z*dot2[0],
	*/
	const PxReal tTemp = __shfl_sync(FULL_MASK, temp, threadIndexInWarp + 3);

	float out = 0.f;
	if (threadIndexInWarp < 3)
	{
		temp = -(temp - tTemp) * q[3];
		out = temp + v * w2 + (q[threadIndexInWarp] * dot2);
	}

	return out;
}


static __device__ void rotateInvNegate(
	const PxVec3& in,
	const PxQuat& quat,
	PxVec3& out,
	const PxU32 threadIndexInWarp)
{
	const float* q = reinterpret_cast<const float*>(&quat.x);

	const PxF32 w2 = q[3] * q[3] - 0.5f;

	float v = 0.f;
	float temp = 0.f;
	if (threadIndexInWarp < 3)
	{
		v = 2.f * in[threadIndexInWarp];
		temp = q[threadIndexInWarp] * v;
	}


	const PxReal dot2 = __shfl_sync(FULL_MASK, temp, 0)
		+ __shfl_sync(FULL_MASK, temp, 1)
		+ __shfl_sync(FULL_MASK, temp, 2);

	/*
	temp[0] = -(q.y * vz - q.z * vy)*q.w,
	temp[1] = -(q.z * vx - q.x * vz)*q.w,
	temp[2] = -(q.x * vy - q.y * vx)*q.w
	*/
	if (threadIndexInWarp < 6)
	{
		PxU32 ind0 = (threadIndexInWarp + 1) % 3;
		PxU32 ind1 = (threadIndexInWarp + 2) % 3;

		//thread 0, 1, 2 use ind0 for qInd and ind1 for vInd, thread 3, 4 , 5
		//use ind1 for qInd and ind0 for vInd
		const PxU32 qInd = (threadIndexInWarp < 3) ? ind0 : ind1;
		const PxU32 vInd = (threadIndexInWarp < 3) ? ind1 : ind0;

		const float tV = __shfl_sync(0x3F, v, vInd);

		temp = q[qInd] * tV;
	}


	/*
	temp[0] += vx0*w2 + q.x*dot2[0],
	temp[1] += vy0*w2 + q.y*dot2[0],
	temp[2] += vz0*w2 + q.z*dot2[0],
	*/
	const PxReal tTemp = __shfl_sync(FULL_MASK, temp, threadIndexInWarp + 3);

	if (threadIndexInWarp < 3)
	{
		temp = -(temp - tTemp) * q[3];
		out[threadIndexInWarp] = -(temp + v * w2 + (q[threadIndexInWarp] * dot2));
	}
}


static __device__ void cross(const PxVec3& inA,
	const PxVec3& inB,
	PxVec3& out,
	const PxU32 threadIndexInWarp)
{
	//(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
	if (threadIndexInWarp < 3)
	{
		const PxU32 elemInd0 = (threadIndexInWarp + 1) % 3;
		const PxU32 elemInd1 = (threadIndexInWarp + 2) % 3;

		out[threadIndexInWarp] = inA[elemInd0] * inB[elemInd1] - inA[elemInd1] * inB[elemInd0];
	}
}

static __device__ PxReal crossR(
	const PxVec3& inA,
	const PxVec3& inB,
	const PxU32 threadIndexInWarp)
{
	//(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
	PxReal res = 0.f;
	if (threadIndexInWarp < 3)
	{
		const PxU32 elemInd0 = (threadIndexInWarp + 1) % 3;
		const PxU32 elemInd1 = (threadIndexInWarp + 2) % 3;

		res = inA[elemInd0] * inB[elemInd1] - inA[elemInd1] * inB[elemInd0];
	}
	return res;
}

static __device__ PxReal crossR(
	const float inA,
	const float inB,
	const PxU32 threadIndexInWarp)
{
	//(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
	float out = 0.f;
	const PxU32 elemInd0 = (threadIndexInWarp + 1) % 3;
	const PxU32 elemInd1 = (threadIndexInWarp + 2) % 3;

	const float inA0 = __shfl_sync(FULL_MASK, inA, elemInd0);
	const float inA1 = __shfl_sync(FULL_MASK, inA, elemInd1);
	const float inB0 = __shfl_sync(FULL_MASK, inB, elemInd1);
	const float inB1 = __shfl_sync(FULL_MASK, inB, elemInd0);

	if (threadIndexInWarp < 3)
	{
		out = inA0 * inB0 - inA1 * inB1;
		//out = inA[elemInd0] * inB[elemInd1] - inA[elemInd1] * inB[elemInd0];
	}
	return out;
}

static __device__ void crossAdd(const PxVec3& inA,
	const PxVec3& inB,
	PxVec3& out,
	const PxU32 threadIndexInWarp)
{
	//(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
	if (threadIndexInWarp < 3)
	{
		const PxU32 elemInd0 = (threadIndexInWarp + 1) % 3;
		const PxU32 elemInd1 = (threadIndexInWarp + 2) % 3;

		out[threadIndexInWarp] += inA[elemInd0] * inB[elemInd1] - inA[elemInd1] * inB[elemInd0];
	}
}



//
//static __device__ void rotateInv(
//	const float in,
//	const PxQuat& quat,
//	PxVec3& out,
//	const PxU32 threadIndexInWarp)
//{
//	const float* q = reinterpret_cast<const float*>(&quat.x);
//
//	const PxF32 w2 = q[3] * q[3] - 0.5f;
//
//	float v = 0.f;
//	float temp = 0.f;
//	if (threadIndexInWarp < 3)
//	{
//		v = 2.f * in;
//		temp = q[threadIndexInWarp] * v;
//	}
//
//
//	const PxReal dot2 = __shfl_sync(FULL_MASK, temp, 0)
//		+ __shfl_sync(FULL_MASK, temp, 1)
//		+ __shfl_sync(FULL_MASK, temp, 2);
//
//	/*
//	temp[0] = -(q.y * vz - q.z * vy)*q.w,
//	temp[1] = -(q.z * vx - q.x * vz)*q.w,
//	temp[2] = -(q.x * vy - q.y * vx)*q.w
//	*/
//	if (threadIndexInWarp < 6)
//	{
//		PxU32 ind0 = (threadIndexInWarp + 1) % 3;
//		PxU32 ind1 = (threadIndexInWarp + 2) % 3;
//
//		//thread 0, 1, 2 use ind0 for qInd and ind1 for vInd, thread 3, 4 , 5
//		//use ind1 for qInd and ind0 for vInd
//		const PxU32 qInd = (threadIndexInWarp < 3) ? ind0 : ind1;
//		const PxU32 vInd = (threadIndexInWarp < 3) ? ind1 : ind0;
//
//		const float tV = __shfl_sync(FULL_MASK, v, vInd);
//
//		temp = q[qInd] * tV;
//	}
//
//
//	/*
//	temp[0] += vx0*w2 + q.x*dot2[0],
//	temp[1] += vy0*w2 + q.y*dot2[0],
//	temp[2] += vz0*w2 + q.z*dot2[0],
//	*/
//	const PxReal tTemp = __shfl_sync(FULL_MASK, temp, threadIndexInWarp + 3);
//
//	if (threadIndexInWarp < 3)
//	{
//		temp = -(temp - tTemp) * q[3];
//		out[threadIndexInWarp] = temp + v * w2 + (q[threadIndexInWarp] * dot2);
//	}
//}
//
//
//ind is the index into mat
__constant__ int ind[16] = { 3,	12,	9,	6,
7,	13,	2,	8,
11,	14,	4,	1,
15,	0,	5,	10 };


//sign is the sign into mat
__constant__ float sign[16] = { -1.f, 1.f, -1.f, 1.f,
-1.f, -1.f, 1.f, 1.f,
1.f, -1.f, -1.f, 1.f,
1.f, 1.f, 1.f, 1.f };


//out = inA * inB
static __device__ void rotateQuat(
	const PxQuat& inA,
	const PxQuat& inB,
	PxQuat& out,
	ScratchMemoryAllocator& sAlloc,
	const PxU32 threadIndexInWarp)
{
	//PxQuat(	x * q.w + y * q.z - z * q.y + w * q.x, 
	//			y * q.w + z * q.x - x * q.z + w * q.y ,
	//			z * q.w + x * q.y - y * q.x + w * q.z, 
	//			-x * q.x - y * q.y - z * q.z + w * q.w );

	ScratchMemoryMarker marker(sAlloc);

	float* mat = sAlloc.allocAligned<float>(sizeof(float) * 16);

	const float* inAV = reinterpret_cast<const float*>(&inA);
	const float* inBV = reinterpret_cast<const float*>(&inB);
	float* outV = reinterpret_cast<float*>(&out);

	if (threadIndexInWarp < 16)
	{
		const PxU32 colIndex = threadIndexInWarp / 4;
		const PxU32 elemIndex = threadIndexInWarp % 4;
		float* col = &mat[colIndex * 4];
		col[elemIndex] = inAV[colIndex] * inBV[elemIndex] * sign[threadIndexInWarp];
	}

	__syncwarp();

	/*if (threadIndexInWarp == 0)
	{
	PxReal* outv = &out.x;
	outv[0] = mat[0*4 + 3] + mat[1*4 + 2] + mat[2*4 + 1] + mat[3*4 + 0];
	outv[1] = mat[1*4 + 3] + mat[2*4 + 0] + mat[0*4 + 2] + mat[3*4 + 1];
	outv[2] = mat[2*4 + 3] + mat[0*4 + 1] + mat[1*4 + 0] + mat[3*4 + 2];
	outv[3] = mat[3*4 + 3] + mat[0*4 + 0] + mat[1*4 + 1] + mat[2*4 + 2];
	}*/

	PxReal sum = 0.f;
	if (threadIndexInWarp < 8)
	{
		const PxU32 index = 2 * threadIndexInWarp;
		sum = mat[ind[index]] + mat[ind[index + 1]];
	}

	PxReal shuff = __shfl_sync(FULL_MASK, sum, threadIndexInWarp + 1);

	if (threadIndexInWarp < 8 && (threadIndexInWarp & 1) == 0)
	{
		outV[threadIndexInWarp / 2] = sum + shuff;
	}

}

//out = inA * inB
static __device__ float rotateQuatR(
	const float quatA,
	const float quatB,
	const PxU32 threadIndexInWarp)
{
	//PxQuat(	x * q.w + y * q.z - z * q.y + w * q.x, 
	//			y * q.w + z * q.x - x * q.z + w * q.y ,
	//			z * q.w + x * q.y - y * q.x + w * q.z, 
	//			-x * q.x - y * q.y - z * q.z + w * q.w );

	float res = 0.f;
	const PxU32 colIndex = threadIndexInWarp / 4;
	const PxU32 elemIndex = threadIndexInWarp % 4;

	const float valA = __shfl_sync(FULL_MASK, quatA, colIndex);
	const float valB = __shfl_sync(FULL_MASK, quatB, elemIndex);

	if (threadIndexInWarp < 16)
	{
		res = valA * valB * sign[threadIndexInWarp];
	}
	/*if (threadIndexInWarp == 0)
	{
	PxReal* outv = &out.x;
	outv[0] = mat[0*4 + 3] + mat[1*4 + 2] + mat[2*4 + 1] + mat[3*4 + 0];
	outv[1] = mat[1*4 + 3] + mat[2*4 + 0] + mat[0*4 + 2] + mat[3*4 + 1];
	outv[2] = mat[2*4 + 3] + mat[0*4 + 1] + mat[1*4 + 0] + mat[3*4 + 2];
	outv[3] = mat[3*4 + 3] + mat[0*4 + 0] + mat[1*4 + 1] + mat[2*4 + 2];
	}*/

	const PxU32 i = 2 * threadIndexInWarp;
	const PxU32 index = threadIndexInWarp > 7 ? 0 : i;
	const PxReal v0 = __shfl_sync(FULL_MASK, res, ind[index]);
	const PxReal v1 = __shfl_sync(FULL_MASK, res, ind[index + 1]);

	PxReal sum = v0 + v1;

	PxReal shuff = __shfl_sync(FULL_MASK, sum, threadIndexInWarp + 1);

	float out = 0.f;
	if (threadIndexInWarp < 8 && (threadIndexInWarp & 1) == 0)
	{
		out = sum + shuff;
	}

	out = __shfl_sync(FULL_MASK, out, threadIndexInWarp * 2);

	return out;

}


//out = transf.tranform(in)
static __device__ void transform(
	const PxVec3& in,
	const PxTransform& transf,
	PxVec3& out,
	const PxU32 threadIndexInWarp)
{
	float outV = rotateR(in, transf.q, threadIndexInWarp);

	if (threadIndexInWarp < 3)
	{
		out[threadIndexInWarp] = outV + transf.p[threadIndexInWarp];
	}
}

static __device__ PxReal transformR(
	const float in,
	const PxTransform& transf,
	const PxU32 threadIndexInWarp)
{

	float outV = rotateR(in, transf.q, threadIndexInWarp);

	if (threadIndexInWarp < 3)
	{
		outV = outV + transf.p[threadIndexInWarp];
	}

	return outV;
}

static __device__ void transform(
	const float in,
	const PxTransform& transf,
	PxVec3& out,
	const PxU32 threadIndexInWarp)
{
	float outV = rotateR(in, transf.q, threadIndexInWarp);

	if (threadIndexInWarp < 3)
	{
		out[threadIndexInWarp] = outV + transf.p[threadIndexInWarp];
	}
}

// out = transfA.tranform(transfB)
static __device__ void transform(
	const PxTransform& transfA,
	const PxTransform& transfB,
	PxTransform& out,
	ScratchMemoryAllocator& sAlloc,
	const PxU32 threadIndexInWarp)
{
	ScratchMemoryMarker marker(sAlloc);


	rotate(transfB.p, transfA.q, out.p, threadIndexInWarp);
	rotateQuat(transfA.q, transfB.q, out.q, sAlloc, threadIndexInWarp);

	__syncwarp();

	if (threadIndexInWarp < 3)
	{
		out.p[threadIndexInWarp] += transfA.p[threadIndexInWarp];
	}
}

//out = transf.tranformInv(in)
static __device__ void transformInv(
	const PxVec3& in,
	const PxTransform& transf,
	PxVec3& out,
	const PxU32 threadIndexInWarp)
{

	float tIn = 0.f;
	if (threadIndexInWarp < 3)
	{
		tIn = in[threadIndexInWarp] - transf.p[threadIndexInWarp];
	}

	float tOut = rotateInvR(tIn, transf.q, threadIndexInWarp);
	if (threadIndexInWarp < 3)
	{
		out[threadIndexInWarp] = tOut;
	}
}

//out = inA + inB
static __device__ void add(
	const PxVec3& inA,
	const PxVec3& inB,
	PxVec3& out,
	const PxU32 threadIndexInWarp)
{
	if (threadIndexInWarp < 3)
	{
		out[threadIndexInWarp] = inA[threadIndexInWarp] + inB[threadIndexInWarp];
	}
}

//out = inA + inB
static __device__ PxReal addR(
	const PxReal inA,
	const PxReal inB,
	const PxU32 threadIndexInWarp)
{
	return inA + inB;
}

static __device__ void subtract(
	const PxVec3& inA,
	const PxVec3& inB,
	PxVec3& out,
	const PxU32 threadIndexInWarp)
{
	if (threadIndexInWarp < 3)
	{
		out[threadIndexInWarp] = inA[threadIndexInWarp] - inB[threadIndexInWarp];
	}
}

static __device__ PxReal dot(
	const PxVec3& inA,
	const PxVec3& inB,
	const PxU32 threadIndexInWarp)
{
	PxReal outV = 0.f;
	if (threadIndexInWarp < 3)
	{
		outV = inA[threadIndexInWarp] * inB[threadIndexInWarp];
	}

	return __shfl_sync(FULL_MASK, outV, 0) + __shfl_sync(FULL_MASK, outV, 1) + __shfl_sync(FULL_MASK, outV, 2);
}

static __device__ PxReal dot(
	const float4& inA,
	const float4& inB,
	const PxU32 threadIndexInWarp)
{
	PxReal outV = 0.f;
	if (threadIndexInWarp < 3)
	{
		const float* invA = reinterpret_cast<const float*>(&inA);
		const float* invB = reinterpret_cast<const float*>(&inB);
		outV = invA[threadIndexInWarp] * invB[threadIndexInWarp];
	}

	return __shfl_sync(FULL_MASK, outV, 0) + __shfl_sync(FULL_MASK, outV, 1)
		+ __shfl_sync(FULL_MASK, outV, 2) + __shfl_sync(FULL_MASK, outV, 3);
}


static __device__ PxReal dotR(
	const float inA,
	const float inB,
	const PxU32 threadIndexInWarp)
{
	PxReal outV = 0.f;
	if (threadIndexInWarp < 3)
	{
		outV = inA * inB;
	}

	return __shfl_sync(FULL_MASK, outV, 0) + __shfl_sync(FULL_MASK, outV, 1)
		+ __shfl_sync(FULL_MASK, outV, 2);
}

//(inA - inB).magnitudeSquared
static __device__ PxReal negateMagnitudeSquared(
	const PxVec3& inA, const PxVec3& inB,
	const PxU32 threadIndexInWarp)
{
	PxReal tOut = 0.f;
	if (threadIndexInWarp < 3)
	{
		tOut = inA[threadIndexInWarp] - inB[threadIndexInWarp];
		tOut = tOut * tOut;
	}

	return __shfl_sync(FULL_MASK, tOut, 0) + __shfl_sync(FULL_MASK, tOut, 1) + __shfl_sync(FULL_MASK, tOut, 2);
}

static __device__ void negate(
	PxVec3& inOut,
	const PxU32 threadIndexInWarp)
{
	if (threadIndexInWarp < 3)
	{
		inOut[threadIndexInWarp] = -inOut[threadIndexInWarp];
	}
}

static __device__ void negate(
	const PxVec3& in,
	PxVec3& out,
	const PxU32 threadIndexInWarp)
{
	if (threadIndexInWarp < 3)
	{
		out[threadIndexInWarp] = -in[threadIndexInWarp];
	}
}

static __device__ PxReal negateR(
	const PxVec3& in,
	const PxU32 threadIndexInWarp)
{
	float out = 0.f;
	if (threadIndexInWarp < 3)
	{
		out = -in[threadIndexInWarp];
	}
	return out;
}


static __device__ void conjugate(
	const PxQuat& in,
	PxQuat& out,
	const PxU32 threadIndexInWarp)
{
	//return PxQuat(-x, -y, -z, w);

	if (threadIndexInWarp < 4)
	{
		const float* inV = reinterpret_cast<const float*>(&in.x);
		float* outV = reinterpret_cast<float*>(&out.x);
		PxReal val = (threadIndexInWarp == 3) ? inV[threadIndexInWarp] : -inV[threadIndexInWarp];
		outV[threadIndexInWarp] = val;
	}
}


//return a quaterion in 4 thread(0-x, 1-y, 2-z, 3-w)
static __device__ PxReal conjugateR(
	const float in,
	const PxU32 threadIndexInWarp)
{
	//return PxQuat(-x, -y, -z, w);

	return (threadIndexInWarp < 3) ? -in : in;
}

//return a quaterion in 4 thread(0-x, 1-y, 2-z, 3-w)
static __device__ PxReal conjugateR(
	const PxQuat& in,
	const PxU32 threadIndexInWarp)
{
	//return PxQuat(-x, -y, -z, w);

	PxReal out = 0.f;
	if (threadIndexInWarp < 4)
	{
		const float* inV = reinterpret_cast<const float*>(&in.x);
		out = (threadIndexInWarp < 3) ? -inV[threadIndexInWarp] : inV[threadIndexInWarp];
	}
	return out;
}


static __device__ void negateQuat(
	PxQuat& inOut,
	const PxU32 threadIndexInWarp)
{
	if (threadIndexInWarp < 4)
	{
		float* inOutV = reinterpret_cast<float*>(&inOut.x);
		inOutV[threadIndexInWarp] = -inOutV[threadIndexInWarp];
	}
}


static __device__ PxReal normalizedQuatR(
	const float in,
	const PxU32 threadIndexInWarp)
{
	//const float s = 1.0f / magnitude();
	//return PxQuat(x * s, y * s, z * s, w * s);

	PxReal val = in * in;

	val = val + __shfl_sync(FULL_MASK, val, threadIndexInWarp + 1);
	val = val + __shfl_sync(FULL_MASK, val, threadIndexInWarp + 2);

	PxReal s = 1.f / PxSqrt(val);

	s = __shfl_sync(FULL_MASK, s, 0);

	return in * s;
}

static __device__ PxReal normalizedQuatR(
	const PxQuat& in,
	const PxU32 threadIndexInWarp)
{
	float v = 0.f;
	if (threadIndexInWarp < 4)
	{
		const PxReal* inv = &in.x;
		v = inv[threadIndexInWarp];
	}

	return normalizedQuatR(v, threadIndexInWarp);
}


static __device__ void normalizedQuat(
	const PxQuat& in,
	PxQuat& out,
	const PxU32 threadIndexInWarp)
{
	float v = 0.f;
	if (threadIndexInWarp < 4)
	{
		const PxReal* inv = &in.x;
		v = inv[threadIndexInWarp];
	}

	v = normalizedQuatR(v, threadIndexInWarp);

	if (threadIndexInWarp < 4)
	{
		float* outv = &out.x;
		outv[threadIndexInWarp] = v;
	}
}


static __device__ PxReal dotQuatR(
	const float inA,
	const float inB,
	const PxU32 threadIndexInWarp)
{
	//return x * v.x + y * v.y + z * v.z + w * v.w;
	PxReal val = inA * inB;

	val = val + __shfl_sync(FULL_MASK, val, threadIndexInWarp + 1);
	val = val + __shfl_sync(FULL_MASK, val, threadIndexInWarp + 2);

	return __shfl_sync(FULL_MASK, val, 0);
}

//output is quaterion 
static __device__ float expQuatR(
	const float v,
	const PxU32 threadIndexInWarp)
{
	//const PxReal m = v.magnitudeSquared();
	//return m < 1e-24f ? PxQuat(PxIdentity) : PxQuat(PxSqrt(m), v * PxRecipSqrt(m));
	PxReal m = v * v;

	m = m + __shfl_sync(FULL_MASK, m, 1) + __shfl_sync(FULL_MASK, m, 2);
	m = __shfl_sync(FULL_MASK, m, 0);

	float out = 0.f;
	if (m < 1e-24f)
	{
		out = threadIndexInWarp < 3 ? 0.f : 1.f;
	}
	else
	{
		//PxQuat(PxSqrt(m), v * PxRecipSqrt(m))
		const PxReal angleRadians = PxSqrt(m);
		const PxReal a = angleRadians * 0.5f;
		PxReal s = PxSin(a);
		const PxReal w = PxCos(a);
		out = threadIndexInWarp < 3 ? (v * (1.f / angleRadians)) * s : w;
	}

	return out;
}

#endif