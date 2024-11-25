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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef GU_COLLISION_SDF_H
#define GU_COLLISION_SDF_H

#include "GuSDF.h"
#include "foundation/PxPreprocessor.h"

namespace physx
{
namespace Gu
{

// SDF wrapper for collision computations
// may be shared by CPU/GPU code in the future
//
// \detailed CollisionSDF wraps an `SDF` object, providing useful methods for SDF collision.
//
// Conventions
// * The coarse or background SDF is always referred to as _coarse_. Its associated coordinates are called `cPos` and
//   are in units of `mSpacing*mSubgridSize`. The origin is normally at the first grid point.
// * The fine SDF is always referred to as _fine_. Its associated coordinates are called `fPos` and are in units of
//   `mSpacing`. If the sdf is dense, `cPos` is equivalent to `fPos`. The origin is normally at the first grid point.
// * Coordinates in the native space of the SDF are denoted `sPos` and are in native units.

template <int BytesPerSparsePixelT>
PX_FORCE_INLINE PxReal decodeSample(PxReal subgridScalingFactor, PxReal subgridMinSdfValue, const PxU8* data, PxU32 index);

template <>
PX_FORCE_INLINE PxReal decodeSample<1>(PxReal subgridScalingFactor, PxReal subgridMinSdfValue, const PxU8* data, PxU32 index)
{
	return static_cast<PxReal>(data[index]) * subgridScalingFactor + subgridMinSdfValue;
}

template <>
PX_FORCE_INLINE PxReal decodeSample<2>(PxReal subgridScalingFactor, PxReal subgridMinSdfValue, const PxU8* data, PxU32 index)
{
	const PxU16* ptr = reinterpret_cast<const PxU16*>(data);
	return static_cast<PxReal>(ptr[index]) * subgridScalingFactor + subgridMinSdfValue;
}

template <>
PX_FORCE_INLINE PxReal decodeSample<4>(PxReal /*unused*/, PxReal /*unused*/, const PxU8* data, PxU32 index)
{
	//If 4 bytes per subgrid pixel are available, then normal floats are used. No need to
	//de-normalize integer values since the floats already contain real distance values
	const PxReal* ptr = reinterpret_cast<const PxReal*>(data);
	return ptr[index];
}

struct CollisionSDF
{
	CollisionSDF(const SDF& sdf): mSdf(sdf), mSdfBoxLower(sdf.mMeshLower), mFDims(sdf.mDims), mInvGridDx(1.0f / sdf.mSpacing),
								  mInvSubgridSize(sdf.mSubgridSize ? 1.0f / sdf.mSubgridSize : 0), mIsDense(sdf.mSubgridSize == 0)
	{
		// assume that `mMeshLower` is also the location of the lowest grid point
		if (mIsDense)
		{
			mCDims = mFDims;
			mCSamples = mCDims;
			mSdfBoxUpper = sdf.mMeshLower + sdf.mSpacing * PxVec3(static_cast<PxReal>(mFDims.x-1), static_cast<PxReal>(mFDims.y-1), static_cast<PxReal>(mFDims.z-1));
		}
		else
		{
			mCDims = Dim3(mFDims.x / sdf.mSubgridSize, mFDims.y / sdf.mSubgridSize, mFDims.z / sdf.mSubgridSize);
			mCSamples = Dim3(mCDims.x + 1, mCDims.y + 1, mCDims.z + 1);
			mSdfBoxUpper = sdf.mMeshLower + sdf.mSpacing * PxVec3(mFDims);
		}
		if (mSdf.mBytesPerSparsePixel == 1)
			mSubgridScalingFactor = (1.0f / 255.0f) * (mSdf.mSubgridsMaxSdfValue - mSdf.mSubgridsMinSdfValue);
		else if (mSdf.mBytesPerSparsePixel == 2)
			mSubgridScalingFactor = (1.0f / 65535.0f) * (mSdf.mSubgridsMaxSdfValue - mSdf.mSubgridsMinSdfValue);

		const PxU32 fW = mSdf.mSdfSubgrids3DTexBlockDim.x * (mSdf.mSubgridSize + 1),
					fH = mSdf.mSdfSubgrids3DTexBlockDim.y * (mSdf.mSubgridSize + 1);

		mFStrideY = fW;
		mFStrideZ = fW*fH;


	}
	// clamp `fPos` to the grid on which `sdf` is defined
	PX_INLINE PxVec3 clampToFine(const PxVec3& fPos) const
	{
		if (!mIsDense)
			return fPos.maximum(PxVec3(0.0f)).minimum(PxVec3(mFDims));
		return fPos.maximum(PxVec3(0.5f)).minimum(PxVec3(mFDims) + PxVec3(0.5f));
	}

	// clamp `sPos` to the grid on which `sdf` is defined
	PX_INLINE PxVec3 clampToBox(const PxVec3& sPos) const
	{
		if (mIsDense)
			return sPos.maximum(mSdfBoxLower+PxVec3(0.5f*mSdf.mSpacing)).minimum(mSdfBoxUpper+PxVec3(0.5f*mSdf.mSpacing));
		return sPos.maximum(mSdfBoxLower).minimum(mSdfBoxUpper);
	}

	// Utility to convert from x/y/z indices to a linear index given the grid size (only width and height required)
	PX_FORCE_INLINE PX_CUDA_CALLABLE static PxU32 idx3D(PxU32 x, PxU32 y, PxU32 z, PxU32 width, PxU32 height)
	{
		return z * width * height + y * width + x;
	}


	static PX_INLINE PxReal TriLerpWithGradient(
		const PxReal f000,
		const PxReal f100,
		const PxReal f010,
		const PxReal f110,
		const PxReal f001,
		const PxReal f101,
		const PxReal f011,
		const PxReal f111,
		const PxReal tx,
		const PxReal ty,
		const PxReal tz,
		PxVec3* grad = NULL)
	{
		if (grad)
		{
			const PxReal a = f100 - f000;
			const PxReal b = f110 - f010;
			const PxReal c = f101 - f001;
			const PxReal d = f111 - f011;
			grad->x = a + (b - (a)) * ty + (c + (d - (c)) * ty - (a + (b - (a)) * ty)) * tz;
			grad->y = f010 + tx * (b) - (f000 + tx * (a)) + (f011 + tx * (d) - (f001 + tx * (c)) - (f010 + tx * (b) - (f000 + tx * (a)))) * tz;
			grad->z = f001 + tx * (c) + ty * (f011 + tx * (d) - (f001 + tx * (c))) - (f000 + tx * (a) + ty * (f010 + tx * (b) - (f000 + tx * (a))));
		}

		return PxTriLerp(
			f000,
			f100,
			f010,
			f110,
			f001,
			f101,
			f011,
			f111,
			tx,
			ty,
			tz);
	}

	template <int BytesPerSparsePixelT>
	PX_INLINE PxReal interpolateSubgrid (const PxU8* subgridBase, PxU32 baseIdx, PxReal x, PxReal y, PxReal z, PxVec3* gradient = NULL) const
	{
		PX_COMPILE_TIME_ASSERT(
				BytesPerSparsePixelT == 1 || BytesPerSparsePixelT == 2 || BytesPerSparsePixelT == 4);

		return TriLerpWithGradient(
			decodeSample<BytesPerSparsePixelT>(mSubgridScalingFactor, mSdf.mSubgridsMinSdfValue, subgridBase, baseIdx                    ),
			decodeSample<BytesPerSparsePixelT>(mSubgridScalingFactor, mSdf.mSubgridsMinSdfValue, subgridBase, baseIdx+1                  ),
			decodeSample<BytesPerSparsePixelT>(mSubgridScalingFactor, mSdf.mSubgridsMinSdfValue, subgridBase, baseIdx+mFStrideY           ),
			decodeSample<BytesPerSparsePixelT>(mSubgridScalingFactor, mSdf.mSubgridsMinSdfValue, subgridBase, baseIdx+mFStrideY+1         ),
			decodeSample<BytesPerSparsePixelT>(mSubgridScalingFactor, mSdf.mSubgridsMinSdfValue, subgridBase, baseIdx+mFStrideZ           ),
			decodeSample<BytesPerSparsePixelT>(mSubgridScalingFactor, mSdf.mSubgridsMinSdfValue, subgridBase, baseIdx+mFStrideZ+1         ),
			decodeSample<BytesPerSparsePixelT>(mSubgridScalingFactor, mSdf.mSubgridsMinSdfValue, subgridBase, baseIdx+mFStrideZ+mFStrideY  ),
			decodeSample<BytesPerSparsePixelT>(mSubgridScalingFactor, mSdf.mSubgridsMinSdfValue, subgridBase, baseIdx+mFStrideZ+mFStrideY+1),
			x, y, z, gradient);
	}

	// interpolate the values of `array` at position `fPos`
	// input vector `fPos` is in units of subgrid cells, with 0 corresponding to the subgrid origin
	PX_INLINE PxReal evaluateSubgrid(const PxU32 subgridInfo, const PxVec3& fPos, PxVec3* gradient = NULL) const
	{
		const PxU32 sgSamples = mSdf.mSubgridSize + 1;
		PX_ASSERT(fPos.x >= 0 && fPos.y >= 0 && fPos.z >= 0);
		PX_ASSERT(fPos.x < sgSamples && fPos.y < sgSamples && fPos.z < sgSamples);

		// find the subgrid offset in memory
		PxU32 xSubgrid, ySubgrid, zSubgrid;
		SDF::decodeTriple(subgridInfo, xSubgrid, ySubgrid, zSubgrid);
		xSubgrid *= (mSdf.mSubgridSize + 1);
		ySubgrid *= (mSdf.mSubgridSize + 1);
		zSubgrid *= (mSdf.mSubgridSize + 1);

		// reference subgrid point
		const PxU32 x = PxMin(static_cast<PxU32>(fPos.x), sgSamples - 2),
					y = PxMin(static_cast<PxU32>(fPos.y), sgSamples - 2),
					z = PxMin(static_cast<PxU32>(fPos.z), sgSamples - 2);


		// offset values by the subgrid memory offset
		const PxU32 xM = xSubgrid + x, yM = ySubgrid + y, zM = zSubgrid + z;

		const PxU32 base = mFStrideZ * zM + mFStrideY * yM + xM;
		switch (mSdf.mBytesPerSparsePixel)
		{
			case 1:
				return interpolateSubgrid<1>(mSdf.mSubgridSdf, base, fPos.x - x, fPos.y - y, fPos.z - z, gradient);
			case 2:
				return interpolateSubgrid<2>(mSdf.mSubgridSdf, base, fPos.x - x, fPos.y - y, fPos.z - z, gradient);
			case 4:
				return interpolateSubgrid<4>(mSdf.mSubgridSdf, base, fPos.x - x, fPos.y - y, fPos.z - z, gradient);
			default: // never reached
				PX_ASSERT(0);
				return 0;
		}
	}

	// interpolate the values of `array` at position `cPos`.
	// `cPos` must be >= 0 and < `cDims`
	PX_INLINE PxReal evaluateCoarse(const PxVec3& cPos, PxVec3* gradient = NULL) const
	{
		PX_ASSERT(cPos.x >= 0 && cPos.y >= 0 && cPos.z >= 0);
		PX_ASSERT(cPos.x < mCSamples.x && cPos.y < mCSamples.y && cPos.z < mCSamples.z);

		// reference grid point
		const PxU32 x = PxMin(static_cast<PxU32>(cPos.x), mCSamples.x - 2),
					y = PxMin(static_cast<PxU32>(cPos.y), mCSamples.y - 2),
					z = PxMin(static_cast<PxU32>(cPos.z), mCSamples.z - 2);

		const PxU32 w = mCSamples.x, h = mCSamples.y;
		const PxU32 cStrideY = w, cStrideZ = w*h;  // Note that this is sample, not cell, stride
		const PxU32 base = cStrideZ * z + cStrideY * y + x;

		return TriLerpWithGradient(
			mSdf.mSdf[base],
			mSdf.mSdf[base+1],
			mSdf.mSdf[base+cStrideY],
			mSdf.mSdf[base+cStrideY+1],
			mSdf.mSdf[base+cStrideZ],
			mSdf.mSdf[base+cStrideZ+1],
			mSdf.mSdf[base+cStrideZ+cStrideY],
			mSdf.mSdf[base+cStrideZ+cStrideY+1],
			cPos.x - x, cPos.y - y, cPos.z - z, gradient);
	}

	// sample the SDF at `fPos`
	// input vector `fPos` is in units of (sub-) grid cells, with integer values representing nodes
	PX_INLINE PxReal sample(PxVec3 fPos, PxVec3* gradient = NULL) const
	{
		if (mIsDense)
			fPos -= PxVec3(0.5);
		PX_ASSERT(fPos.x >= 0 && fPos.y >= 0 && fPos.z >= 0);
		PX_ASSERT(fPos.x <= mFDims.x && fPos.y <= mFDims.y && fPos.z <= mFDims.z);
		if (mIsDense) // fPos = cPos
			return evaluateCoarse(fPos, gradient);

		// coarse reference gridpoint index
		const Dim3 cBase(
			PxMin(static_cast<PxU32>(fPos.x * mInvSubgridSize), mCSamples.x - 2),
			PxMin(static_cast<PxU32>(fPos.y * mInvSubgridSize), mCSamples.y - 2),
			PxMin(static_cast<PxU32>(fPos.z * mInvSubgridSize), mCSamples.z - 2)
		);

		const PxU32 i = idx3D(cBase.x, cBase.y, cBase.z, mCDims.x, mCDims.y);
		const PxU32 subgridInfo = mSdf.mSubgridStartSlots[i];

		if (subgridInfo == 0xFFFFFFFF) // Evaluate (coarse) background of sparse SDF
			return evaluateCoarse((fPos * mInvSubgridSize).minimum(PxVec3(PxReal(mCSamples.x), PxReal(mCSamples.y), PxReal(mCSamples.z))), gradient);

		// offset to subgrid origin
		PxVec3 fPosInSubgrid;
		fPosInSubgrid.x = PxMax(0.f, fPos.x - cBase.x * mSdf.mSubgridSize);
		fPosInSubgrid.y = PxMax(0.f, fPos.y - cBase.y * mSdf.mSubgridSize);
		fPosInSubgrid.z = PxMax(0.f, fPos.z - cBase.z * mSdf.mSubgridSize);

		return evaluateSubgrid(subgridInfo, fPosInSubgrid, gradient);
	}

	// evaluate & interpolate `sdf` (in `sdf`'s "vertex" space) at `sPos`
	// when outside the sdf grid, this should be considered an upper bound
	// TODO(CA): add more clamps or prove they're unnecessary
	inline PxReal dist(const PxVec3& sPos, PxVec3* gradient = NULL) const
	{
		// clamped to SDF support
		const PxVec3 boxPos = clampToBox(sPos);
		const PxVec3 diff = sPos - boxPos;
		const PxReal diffMag = diff.magnitude();
		const PxVec3 fPos = (boxPos - mSdfBoxLower) * mInvGridDx;
		const PxReal distance = sample(clampToFine(fPos), gradient) + diffMag; // division inaccuracy necessitates clamp
		if (gradient && diffMag > 0.0f)
			*gradient = diff; //A quite coarse approximation but it's only used if the sample point is outside of the sdf's bounding box
		return distance;
	}

	// evaluate & interpolate `sdf` at `sPos` (in `sdf`'s "vertex" space), and compute its gradient
	inline PxVec3 grad(const PxVec3& sPos) const
	{
		// clamped to SDF support
		const PxVec3 boxPos = clampToBox(sPos);
		const PxVec3 fPos = (boxPos - mSdfBoxLower) * mInvGridDx;

		PxVec3 gradient;

		if (	fPos.x >= 1.0f && fPos.x <= mFDims.x - 2.0f &&
				fPos.y >= 1.0f && fPos.y <= mFDims.y - 2.0f &&
				fPos.z >= 1.0f && fPos.z <= mFDims.z - 2.0f)
		{
			gradient.x = sample(PxVec3(fPos.x+1, fPos.y, fPos.z)) - sample(PxVec3(fPos.x-1, fPos.y, fPos.z));
			gradient.y = sample(PxVec3(fPos.x, fPos.y+1, fPos.z)) - sample(PxVec3(fPos.x, fPos.y -1, fPos.z));
			gradient.z = sample(PxVec3(fPos.x, fPos.y, fPos.z+1)) - sample(PxVec3(fPos.x, fPos.y, fPos.z -1));

		}
		else
		{
			const PxReal h = mSdf.mSpacing;
			gradient.x = dist(PxVec3(sPos.x+h, sPos.y, sPos.z)) - dist(PxVec3(sPos.x-h, sPos.y, sPos.z));
			gradient.y = dist(PxVec3(sPos.x, sPos.y+h, sPos.z)) - dist(PxVec3(sPos.x, sPos.y-h, sPos.z));
			gradient.z = dist(PxVec3(sPos.x, sPos.y, sPos.z+h)) - dist(PxVec3(sPos.x, sPos.y, sPos.z-h));
		}
		gradient *= 0.5f / mSdf.mSpacing;

		return gradient;
	}

	// Estimate the value and gradient of `sdf` at `sPos`, using gradient information when `sPos` is
	// outside the SDF grid. Return `PX_MAX_F32` when the distance exceeds `cutoffDistance`
	PX_INLINE PxReal distUsingGradient(const PxVec3& sPos, PxVec3& gradient, const PxReal& cutoffDistance) const
	{
		// clamped to SDF support
		const PxVec3 boxPos = clampToBox(sPos);
		const PxVec3 diff = sPos - boxPos;

		const PxVec3 fPos = (boxPos - mSdfBoxLower) * mInvGridDx;
		const PxReal dist = sample(clampToFine(fPos));

		if (dist > cutoffDistance)
			return PX_MAX_F32;

		gradient = grad(sPos);
		gradient = (gradient.getNormalized() * PxAbs(dist) + diff).getNormalized();
		return dist + gradient.dot(diff);
	}

	// data members
	const SDF& mSdf;
	PxVec3 mSdfBoxLower, mSdfBoxUpper;  // Positions of the first and last grid points
	Dim3 mCDims, mFDims; // background and high-resolution SDF dimensions in cells. Equal if dense.
					   // fDims is equally divisible by subgridSize, which is also in cells, for sparse SDFs
					   // the coarse grid has cDims+1 (cDims) samples per dimension for sparse (dense) SDFs
					   // subgrids have subgridSize+1 samples per dimension
	Dim3 mCSamples;	// Number of samples in each dimension. Equal to cDims for dense, and cDims + 1 for spares SDFs

	PxReal mInvGridDx;		// invSdfDx
	PxReal mInvSubgridSize;  // fineToCoarse
	bool mIsDense;
	PxReal mSubgridScalingFactor;  // purely for optimization
	PxU32 mFStrideY, mFStrideZ;
};

}
}
#endif
