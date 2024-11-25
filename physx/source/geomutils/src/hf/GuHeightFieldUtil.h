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

#ifndef GU_HEIGHTFIELD_UTIL_H
#define GU_HEIGHTFIELD_UTIL_H

#include "geometry/PxHeightFieldGeometry.h"
#include "geometry/PxTriangle.h"
#include "foundation/PxBasicTemplates.h"
#include "foundation/PxSIMDHelpers.h"

#include "GuHeightField.h"
#include "../intersection/GuIntersectionRayTriangle.h"
#include "../intersection/GuIntersectionRayBox.h"

namespace physx
{
#define HF_SWEEP_REPORT_BUFFER_SIZE 64
#define HF_OVERLAP_REPORT_BUFFER_SIZE 64

namespace Gu
{
	class OverlapReport;

	// PT: this is used in the context of sphere-vs-heightfield overlaps
	PX_FORCE_INLINE	PxVec3	getLocalSphereData(PxBounds3& localBounds, const PxTransform& pose0, const PxTransform& pose1, float radius)
	{
		const PxVec3 localSphereCenter = pose1.transformInv(pose0.p);

		const PxVec3 extents(radius);
		localBounds.minimum = localSphereCenter - extents;
		localBounds.maximum = localSphereCenter + extents;

		return localSphereCenter;
	}

	PX_FORCE_INLINE	PxBounds3	getLocalCapsuleBounds(float radius, float halfHeight)
	{
		const PxVec3 extents(halfHeight + radius, radius, radius);
		return PxBounds3(-extents, extents);
	}

	class PX_PHYSX_COMMON_API HeightFieldUtil
	{
	public:
		PxReal							mOneOverRowScale;
		PxReal							mOneOverHeightScale;
		PxReal							mOneOverColumnScale;
		const Gu::HeightField*			mHeightField;
		const PxHeightFieldGeometry*	mHfGeom;

		PX_FORCE_INLINE HeightFieldUtil(const PxHeightFieldGeometry& hfGeom) : mHeightField(static_cast<const Gu::HeightField*>(hfGeom.heightField)), mHfGeom(&hfGeom)
		{
			const PxReal absRowScale = PxAbs(mHfGeom->rowScale);
			const PxReal absColScale = PxAbs(mHfGeom->columnScale);
			//warning #1931-D on WIIU: sizeof is not a type, variable, or dereferenced pointer expression
			PX_COMPILE_TIME_ASSERT(sizeof(reinterpret_cast<PxHeightFieldSample*>(0)->height) == 2);
			//PxReal minHeightPerSample = PX_MIN_HEIGHTFIELD_Y_SCALE;
			PX_ASSERT(mHfGeom->heightScale >= PX_MIN_HEIGHTFIELD_Y_SCALE);
			PX_ASSERT(absRowScale >= PX_MIN_HEIGHTFIELD_XZ_SCALE);
			PX_ASSERT(absColScale >= PX_MIN_HEIGHTFIELD_XZ_SCALE);
			PX_UNUSED(absRowScale);
			PX_UNUSED(absColScale);
			//using physx::intrinsics::fsel;
			//mOneOverHeightScale	= fsel(mHfGeom->heightScale - minHeightPerSample, 1.0f / mHfGeom->heightScale, 1.0f / minHeightPerSample);
			mOneOverHeightScale	= 1.0f / mHfGeom->heightScale;
			mOneOverRowScale	= 1.0f / mHfGeom->rowScale;
			mOneOverColumnScale	= 1.0f / mHfGeom->columnScale;	
		}

		PX_CUDA_CALLABLE	PX_FORCE_INLINE	const Gu::HeightField&			getHeightField()			const	{ return *mHeightField;			}
		PX_CUDA_CALLABLE	PX_FORCE_INLINE	const PxHeightFieldGeometry&	getHeightFieldGeometry()	const	{ return *mHfGeom;				}

							PX_FORCE_INLINE	PxReal							getOneOverRowScale()		const	{ return mOneOverRowScale;		}
							PX_FORCE_INLINE	PxReal							getOneOverHeightScale()		const	{ return mOneOverHeightScale;	}
							PX_FORCE_INLINE	PxReal							getOneOverColumnScale()		const	{ return mOneOverColumnScale;	}
	
		void	computeLocalBounds(PxBounds3& bounds) const;

		PX_FORCE_INLINE	PxReal	getHeightAtShapePoint(PxReal x, PxReal z) const
		{
			return mHfGeom->heightScale * mHeightField->getHeightInternal(x * mOneOverRowScale, z * mOneOverColumnScale);
		}

		PX_FORCE_INLINE	PxVec3	getNormalAtShapePoint(PxReal x, PxReal z) const
		{
			return mHeightField->getNormal_(x * mOneOverRowScale, z * mOneOverColumnScale, mOneOverRowScale, mOneOverHeightScale, mOneOverColumnScale);
		}

		PxU32	getTriangle(const PxTransform&, PxTriangle& worldTri, PxU32* vertexIndices, PxU32* adjacencyIndices, PxTriangleID triangleIndex, bool worldSpaceTranslation=true, bool worldSpaceRotation=true) const;

		void	overlapAABBTriangles(const PxBounds3& localBounds, OverlapReport& callback, PxU32 batchSize=HF_OVERLAP_REPORT_BUFFER_SIZE) const;

		PX_FORCE_INLINE	void	overlapAABBTriangles0to1(const PxTransform& pose0to1, const PxBounds3& bounds0, OverlapReport& callback, PxU32 batchSize=HF_OVERLAP_REPORT_BUFFER_SIZE) const
		{
			// PT: TODO: optimize PxBounds3::transformFast
			//overlapAABBTriangles(PxBounds3::transformFast(pose0to1, bounds0), callback, batchSize);
			{
				// PT: below is the equivalent, slightly faster code. Still not optimal but better.
				// PT: TODO: refactor with GuBounds.cpp

				const PxMat33Padded basis(pose0to1.q);

				// PT: TODO: pass c/e directly
				const PxBounds3 b = PxBounds3::basisExtent(pose0to1.transform(bounds0.getCenter()), basis, bounds0.getExtents());

				overlapAABBTriangles(b, callback, batchSize);
			}
		}

		PX_FORCE_INLINE	void	overlapAABBTriangles(const PxTransform& pose1, const PxBounds3& bounds0, OverlapReport& callback, PxU32 batchSize=HF_OVERLAP_REPORT_BUFFER_SIZE) const
		{
			overlapAABBTriangles0to1(pose1.getInverse(), bounds0, callback, batchSize);
		}

		PX_FORCE_INLINE	void	overlapAABBTriangles(const PxTransform& pose0, const PxTransform& pose1, const PxBounds3& bounds0, OverlapReport& callback, PxU32 batchSize=HF_OVERLAP_REPORT_BUFFER_SIZE) const
		{
			overlapAABBTriangles0to1(pose1.transformInv(pose0), bounds0, callback, batchSize);
		}

		PX_FORCE_INLINE	PxVec3 hf2shapen(const PxVec3& v) const
		{
			return PxVec3(v.x * mOneOverRowScale, v.y * mOneOverHeightScale, v.z * mOneOverColumnScale);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 shape2hfp(const PxVec3& v) const
		{
			return PxVec3(v.x * mOneOverRowScale, v.y * mOneOverHeightScale, v.z * mOneOverColumnScale);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 hf2shapep(const PxVec3& v) const
		{
			return PxVec3(v.x * mHfGeom->rowScale, v.y * mHfGeom->heightScale, v.z * mHfGeom->columnScale);
		}

		PX_INLINE PxVec3 hf2worldp(const PxTransform& pose, const PxVec3& v) const
		{
			const PxVec3 s = hf2shapep(v);
			return pose.transform(s);
		}

		PX_INLINE PxVec3 hf2worldn(const PxTransform& pose, const PxVec3& v) const
		{
			const PxVec3 s = hf2shapen(v);
			return pose.q.rotate(s);
		}
	};

	class PX_PHYSX_COMMON_API HeightFieldTraceUtil : public HeightFieldUtil
	{
		public:
		PX_FORCE_INLINE HeightFieldTraceUtil(const PxHeightFieldGeometry& hfGeom) : HeightFieldUtil(hfGeom)	{}

		// floor and ceil don't clamp down exact integers but we want that
		static PX_FORCE_INLINE PxF32 floorDown(PxF32 x) { PxF32 f = PxFloor(x); return (f == x) ? f-1 : f; }
		static PX_FORCE_INLINE PxF32 ceilUp   (PxF32 x) { PxF32 f = PxCeil (x); return (f == x) ? f+1 : f; }

		// helper class for testing triangle height and reporting the overlapped triangles
		template<class T>
		class OverlapTraceSegment
		{
		public:
			// helper rectangle struct
			struct OverlapRectangle
			{
				PxI32				mMinu;
				PxI32				mMaxu;
				PxI32				mMinv;
				PxI32				mMaxv;

				void invalidate()
				{
					mMinu = 1;
					mMaxu = -1;
					mMinv = 1;
					mMaxv = -1;
				}
			};

			// helper line struct
			struct OverlapLine
			{
				bool				mColumn;
				PxI32				mLine;
				PxI32				mMin;
				PxI32				mMax;

				void invalidate()
				{
					mMin = 1;
					mMax = -1;
				}
			};

		public:
			void operator = (OverlapTraceSegment&) {}

			OverlapTraceSegment(const HeightFieldUtil& hfUtil,const Gu::HeightField& hf)
			  : mInitialized(false), mHfUtil(hfUtil), mHf(hf), mNbIndices(0) {}

			PX_FORCE_INLINE	bool initialized() const { return mInitialized; }

			// prepare for iterations, set the expand u|v
			PX_INLINE void prepare(const PxVec3& aP0, const PxVec3& aP1, const PxVec3& overlapObjectExtent, PxF32& expandu, PxF32& expandv)
			{								
				// height test bounds
				mMinY = (PxMin(aP1.y,aP0.y) - overlapObjectExtent.y) * mHfUtil.getOneOverHeightScale();
				mMaxY = (PxMax(aP1.y,aP0.y) + overlapObjectExtent.y) * mHfUtil.getOneOverHeightScale();

				// sets the clipping variables
				mMinRow = PxI32(mHf.getMinRow((PxMin(aP1.x,aP0.x) - overlapObjectExtent.x)* mHfUtil.getOneOverRowScale()));
				mMaxRow = PxI32(mHf.getMaxRow((PxMax(aP1.x,aP0.x) + overlapObjectExtent.x)* mHfUtil.getOneOverRowScale()));
				mMinColumn = PxI32(mHf.getMinColumn((PxMin(aP1.z,aP0.z) - overlapObjectExtent.z)* mHfUtil.getOneOverColumnScale()));
				mMaxColumn = PxI32(mHf.getMaxColumn((PxMax(aP1.z,aP0.z) + overlapObjectExtent.z)* mHfUtil.getOneOverColumnScale()));

				// sets the expanded u|v coordinates
				expandu = PxCeil(overlapObjectExtent.x*mHfUtil.getOneOverRowScale());
				expandv = PxCeil(overlapObjectExtent.z*mHfUtil.getOneOverColumnScale());

				// sets the offset that will be overlapped in each axis
				mOffsetU = PxI32(expandu) + 1;
				mOffsetV = PxI32(expandv) + 1;				
			}

			// sets all necessary variables and makes initial rectangle setup and overlap
			PX_INLINE bool init(const PxI32 ui, const PxI32 vi, const PxI32 nbVi, const PxI32 step_ui, const PxI32 step_vi, T* aCallback)
			{
				mInitialized = true;
				mCallback = aCallback;
				mNumColumns = nbVi;
				mStep_ui = step_ui > 0 ? 0 : -1;
				mStep_vi = step_vi > 0 ? 0 : -1;

				// sets the rectangles
				mCurrentRectangle.invalidate();
				mPreviousRectangle.mMinu = ui - mOffsetU;
				mPreviousRectangle.mMaxu = ui + mOffsetU;
				mPreviousRectangle.mMinv = vi - mOffsetV;
				mPreviousRectangle.mMaxv = vi + mOffsetV;

				// visits all cells in given initial rectangle
				if(!visitCells(mPreviousRectangle))
					return false;

				// reports all overlaps
				if(!reportOverlaps())
					return false;

				return true;
			}

			// u|v changed, check for new rectangle - compare with previous one and parse 
			// the added line, which is a result from the rectangle compare
			PX_INLINE bool step(const PxI32 ui, const PxI32 vi)
			{
				mCurrentRectangle.mMinu = ui - mOffsetU;
				mCurrentRectangle.mMaxu = ui + mOffsetU;
				mCurrentRectangle.mMinv = vi - mOffsetV;
				mCurrentRectangle.mMaxv = vi + mOffsetV;
				OverlapLine line = OverlapLine();
				line.invalidate();
				computeRectangleDifference(mCurrentRectangle,mPreviousRectangle,line);
				
				if(!visitCells(line))
					return false;
				if(!reportOverlaps())
					return false;

				mPreviousRectangle = mCurrentRectangle;
				return true;
			}

			PX_INLINE void computeRectangleDifference(const OverlapRectangle& currentRectangle, const OverlapRectangle& previousRectangle, OverlapLine& line)
			{
				// check if u changes - add the row for visit
				if(currentRectangle.mMinu != previousRectangle.mMinu)
				{										
					line.mColumn = false;
					line.mLine = currentRectangle.mMinu < previousRectangle.mMinu ? currentRectangle.mMinu : currentRectangle.mMaxu;
					line.mMin = currentRectangle.mMinv;
					line.mMax = currentRectangle.mMaxv;
					return;
				}

				// check if v changes - add the column for visit
				if(currentRectangle.mMinv != previousRectangle.mMinv)
				{										
					line.mColumn = true;
					line.mLine = currentRectangle.mMinv < previousRectangle.mMinv ? currentRectangle.mMinv : currentRectangle.mMaxv;
					line.mMin = currentRectangle.mMinu;
					line.mMax = currentRectangle.mMaxu;
				}
			}

			// visits all cells in given rectangle
			PX_INLINE bool visitCells(const OverlapRectangle& rectangle)
			{
				for(PxI32 ui = rectangle.mMinu + mStep_ui; ui <= rectangle.mMaxu + mStep_ui; ui++)
				{
					if(ui < mMinRow)
						continue;
					if(ui >= mMaxRow)
						break;
					for(PxI32 vi = rectangle.mMinv + mStep_vi; vi <= rectangle.mMaxv + mStep_vi; vi++)
					{
						if(vi < mMinColumn)
							continue;
						if(vi >= mMaxColumn)
							break;
						const PxI32 vertexIndex = ui*mNumColumns + vi;
						if(!testVertexIndex(PxU32(vertexIndex)))
							return false;
					}
				}
				return true;
			}

			// visits all cells in given line - can be row or column
			PX_INLINE bool visitCells(const OverlapLine& line)
			{
				if(line.mMin > line.mMax)
					return true;

				if(line.mColumn)
				{
					const PxI32 vi = line.mLine + mStep_vi;
					// early exit if column is out of hf clip area
					if(vi < mMinColumn)
						return true;
					if(vi >= mMaxColumn)
						return true;

					for(PxI32 ui = line.mMin + mStep_ui; ui <= line.mMax + mStep_ui; ui++)
					{
						// early exit or continue if row is out of hf clip area
						if(ui >= mMaxRow)
							break;
						// continue if we did not reach the valid area, we can still get there
						if(ui < mMinRow)
							continue;
						// if the cell has not been tested test and report 
						if(!testVertexIndex(PxU32(mNumColumns * ui + vi)))
							return false;
					}
				}
				else
				{
					const PxI32 ui = line.mLine + mStep_ui;
					// early exit if row is out of hf clip area
					if(ui < mMinRow)
						return true;
					if(ui >= mMaxRow)
						return true;

					for(PxI32 vi = line.mMin + mStep_vi; vi <= line.mMax + mStep_vi; vi++)
					{
						// early exit or continue if column is out of hf clip area
						if(vi >= mMaxColumn)
							break;
						// continue if we did not reach the valid area, we can still get there
						if(vi < mMinColumn)
							continue;
						// if the cell has not been tested test and report 
						if(!testVertexIndex(PxU32(mNumColumns * ui + vi)))
							return false;
					}
				}
				return true;
			}

			// does height check and if succeeded adds to report
			PX_INLINE bool testVertexIndex(const PxU32 vertexIndex)
			{
				const PxReal h0 = mHf.getHeight(vertexIndex);
				const PxReal h1 = mHf.getHeight(vertexIndex + 1);
				const PxReal h2 = mHf.getHeight(vertexIndex + mNumColumns);
				const PxReal h3 = mHf.getHeight(vertexIndex + mNumColumns + 1);
				// actual height test, if some height pass we accept the cell
				if(!((mMaxY < h0 && mMaxY < h1 && mMaxY < h2 && mMaxY < h3) || (mMinY > h0 && mMinY > h1 && mMinY > h2 && mMinY > h3)))
				{
					// check if the triangle is not a hole
					if(mHf.getMaterialIndex0(vertexIndex) != PxHeightFieldMaterial::eHOLE)
					{
						if(!addIndex(vertexIndex*2))
							return false;
					}
					if(mHf.getMaterialIndex1(vertexIndex) != PxHeightFieldMaterial::eHOLE)
					{
						if(!addIndex(vertexIndex*2 + 1))
							return false;
					}
				}
				return true;
			}

			// add triangle index, if we get out of buffer size, report them
			bool addIndex(PxU32 triangleIndex)
			{
				if(mNbIndices == HF_SWEEP_REPORT_BUFFER_SIZE)
				{
					if(!reportOverlaps())
						return false;
				}

				mIndexBuffer[mNbIndices++] = triangleIndex;
				return true;
			}

			PX_FORCE_INLINE bool reportOverlaps()
			{
				if(mNbIndices)
				{
					if(!mCallback->onEvent(mNbIndices, mIndexBuffer))
						return false;
					mNbIndices = 0;
				}
				return true;
			}

		private:
			bool					mInitialized;
			const HeightFieldUtil&	mHfUtil;
			const Gu::HeightField&	mHf;
			T*						mCallback;
			PxI32					mOffsetU;
			PxI32					mOffsetV;
			float					mMinY;
			float					mMaxY;
			PxI32					mMinRow;
			PxI32					mMaxRow;
			PxI32					mMinColumn;
			PxI32					mMaxColumn;
			PxI32					mNumColumns;
			PxI32					mStep_ui;
			PxI32					mStep_vi;
			OverlapRectangle		mPreviousRectangle;
			OverlapRectangle		mCurrentRectangle;
			PxU32					mIndexBuffer[HF_SWEEP_REPORT_BUFFER_SIZE];
			PxU32					mNbIndices;
		};

		// If useUnderFaceCalblack is false, traceSegment will report segment/triangle hits via
		//   faceHit(const Gu::HeightFieldUtil& hf, const PxVec3& point, PxU32 triangleIndex)
		// Otherwise traceSegment will report all triangles the segment passes under via
		//   underFaceHit(const Gu::HeightFieldUtil& hf, const PxVec3& triNormal, const PxVec3& crossedEdge,
		//     PxF32 x, PxF32 z, PxF32 rayHeight, PxU32 triangleIndex)
		//   where x,z is the point of previous intercept in hf coords, rayHeight is at that same point
		//   crossedEdge is the edge vector crossed from last call to underFaceHit, undefined for first call
		//   Note that underFaceHit can be called when a line is above a triangle if it's within AABB for that hf cell
		// Note that backfaceCull is ignored if useUnderFaceCallback is true
		//  overlapObjectExtent (localSpace) and overlap are used for triangle collecting using an inflated tracesegment
		// Note that hfLocalBounds are passed as a parameter instead of being computed inside the traceSegment. 
		//	The localBounds can be obtained: PxBounds3 hfLocalBounds; hfUtil.computeLocalBounds(hfLocalBounds); and passed as 
		//  a parameter.		
		template<class T, bool useUnderFaceCallback, bool overlap>
		PX_INLINE void traceSegment(const PxVec3& aP0, const PxVec3& rayDir, const float rayLength , T* aCallback, const PxBounds3& hfLocalBounds, bool backfaceCull,
			const PxVec3* overlapObjectExtent = NULL) const
		{			
			PxF32 tnear, tfar;
			if(!Gu::intersectRayAABB2(hfLocalBounds.minimum, hfLocalBounds.maximum, aP0, rayDir, rayLength, tnear, tfar)) 
				return;

			const PxVec3 p0 = aP0 + rayDir * tnear;
			const PxVec3 p1 = aP0 + rayDir * tfar;

			// helper class used for overlap tests
			OverlapTraceSegment<T> overlapTraceSegment(*this, *mHeightField);

			// values which expand the HF area
			PxF32 expandu = 0.0f, expandv = 0.0f;

			if (overlap)
			{
				// setup overlap variables
				overlapTraceSegment.prepare(aP0,aP0 + rayDir*rayLength,*overlapObjectExtent,expandu,expandv);
			}

			// row = x|u, column = z|v
			const PxF32 rowScale = mHfGeom->rowScale, columnScale = mHfGeom->columnScale, heightScale = mHfGeom->heightScale;
			const PxI32 nbVi = PxI32(mHeightField->getNbColumnsFast()), nbUi = PxI32(mHeightField->getNbRowsFast());
			PX_ASSERT(nbVi > 0 && nbUi > 0);

			// clampEps is chosen so that we get a reasonable clamp value for 65536*0.9999999f = 65535.992187500000
			const PxF32 clampEps = 1e-7f; // shrink u,v to within 1e-7 away from the world bounds

			// we now clamp uvs to [1e-7, rowLimit-1e-7] to avoid out of range uvs and eliminate related checks in the loop
			const PxF32 nbUcells = PxF32(nbUi-1)*(1.0f-clampEps), nbVcells = PxF32(nbVi-1)*(1.0f-clampEps);

			// if u0,v0 is near an integer, shift up or down in direction opposite to du,dv by PxMax(|u,v|*1e-7, 1e-7)
			// (same direction as du,dv for u1,v1)
			// we do this to ensure that we get at least one intersection with u or v when near the cell edge to eliminate special cases in the loop
			// we need to extend the field for the inflated radius, we will now operate even with negative u|v

			// map p0 from (x, z, y) to (u0, v0, h0)
			// we need to use the unclamped values, otherwise we change the direction of the traversal
			const PxF32 uu0 = p0.x * mOneOverRowScale;  
			PxF32 u0 = PxMin(PxMax(uu0, 1e-7f - expandu), nbUcells + expandu); // multiplication rescales the u,v grid steps to 1
			const PxF32 uv0 = p0.z * mOneOverColumnScale; 
			PxF32 v0 = PxMin(PxMax(uv0, 1e-7f - expandv), nbVcells + expandv);
			const PxReal h0 = p0.y; // we don't scale y

			// map p1 from (x, z, y) to (u1, v1, h1)
			// we need to use the unclamped values, otherwise we change the direction of the traversal
			const PxF32 uu1 = p1.x * mOneOverRowScale; 
			const PxF32 uv1 = p1.z * mOneOverColumnScale;
			const PxReal h1 = p1.y; // we don't scale y

			PxF32 du = uu1 - uu0, dv = uv1 - uv0; // recompute du, dv from adjusted uvs
			const PxReal dh = h1 - h0;

			// grid u&v step is always either 1 or -1, we precompute as both integers and floats to avoid conversions
			// so step_uif is +/-1.0f, step_ui is +/-1
			const PxF32 step_uif = PxSign(du), step_vif = PxSign(dv);
			const PxI32 step_ui = PxI32(step_uif), step_vi = PxI32(step_vif);

			// clamp magnitude of du, dv to at least clampEpsilon to avoid special cases when dividing
			const PxF32 divEpsilon = 1e-10f;
			if(PxAbs(du) < divEpsilon)
				du = step_uif * divEpsilon;
			if(PxAbs(dv) < divEpsilon) 
				dv = step_vif * divEpsilon;

			const PxVec3 auhP0(aP0.x*mOneOverRowScale, aP0.y, aP0.z*mOneOverColumnScale);			
			const PxVec3 duhv(rayDir.x*rayLength*mOneOverRowScale, rayDir.y*rayLength, rayDir.z*rayLength*mOneOverColumnScale);
			const PxReal duhvLength = duhv.magnitude();
			PxVec3 duhvNormalized = duhv;
			if(duhvLength > PX_NORMALIZATION_EPSILON)
				duhvNormalized *= 1.0f/duhvLength;

			// Math derivation:
			// points on 2d segment are parametrized as: [u0,v0] + t [du, dv]. We solve for t_u[n], t for nth u-intercept
			// u0 + t_un du = un
			// t_un = (un-u0) / du
			// t_un1 = (un+1-u0) / du        ;  we use +1 since we rescaled the grid step to 1
			// therefore step_tu = t_un - t_un1 = 1/du

			// seed the initial integer cell coordinates with u0, v0 rounded up or down with standard PxFloor/Ceil behavior
			// to ensure we have the correct first cell between (ui,vi) and (ui+step_ui,vi+step_vi)
			PxI32 ui = (du > 0.0f) ? PxI32(PxFloor(u0)) : PxI32(PxCeil(u0));
			PxI32 vi = (dv > 0.0f) ? PxI32(PxFloor(v0)) : PxI32(PxCeil(v0));

			// find the nearest integer u, v in ray traversal direction and corresponding tu and tv
			const PxReal uhit0 = du > 0.0f ? ceilUp(u0) : floorDown(u0);
			const PxReal vhit0 = dv > 0.0f ? ceilUp(v0) : floorDown(v0);

			// tu, tv can be > 1 but since the loop is structured as do {} while(tMin < tEnd) we still visit the first cell
			PxF32 last_tu = 0.0f, last_tv = 0.0f;			
			PxReal tu = (uhit0 - uu0) / du;
			PxReal tv = (vhit0 - uv0) / dv;
			if(tu < 0.0f)	// negative value may happen, as we may have started out of the AABB (since we did enlarge it)
				tu = PxAbs(clampEps / du);
			if(tv < 0.0f)  // negative value may happen, as we may have started out of the AABB (since we did enlarge it)
				tv = PxAbs(clampEps / dv);			

			// compute step_tu and step_tv; t steps per grid cell in u and v direction
			const PxReal step_tu = 1.0f / PxAbs(du), step_tv = 1.0f / PxAbs(dv);

			// t advances at the same rate for u, v and h therefore we can compute h at u,v grid intercepts
			#define COMPUTE_H_FROM_T(t) (h0 + (t) * dh)

			const PxF32 hEpsilon = 1e-4f;
			PxF32 uif = PxF32(ui), vif = PxF32(vi);

			// these are used to remap h values to correspond to u,v increasing order
			PxI32 uflip = 1-step_ui; /*0 or 2*/
			PxI32 vflip = (1-step_vi)/2; /*0 or 1*/

			// this epsilon is needed to ensure that we include the last [t, t+1] range in the do {} while(t<tEnd) loop
			// A.B. in case of overlap we do miss actually a line with this epsilon, should it not be +?
			PxF32 tEnd = 1.0f - 1e-4f;
			if(overlap)
				tEnd = 1.0f + 1e-4f;
			PxF32 tMinUV;

			const Gu::HeightField& hf = *mHeightField;

			// seed hLinePrev as h(0)
			PxReal hLinePrev = COMPUTE_H_FROM_T(0);

			do
			{
				tMinUV = PxMin(tu, tv); // determine where next closest u or v-intercept point is
				PxF32 hLineNext = COMPUTE_H_FROM_T(tMinUV); // compute the corresponding h

				// the operating u|v space has been extended by expandu|expandv if inflation is used
				PX_ASSERT(ui >= 0 - expandu && ui < nbUi + expandu && vi >= 0 - expandv && vi < nbVi + expandv);
				PX_ASSERT(ui+step_ui >= 0 - expandu && ui+step_ui < nbUi + expandu && vi+step_vi >= 0 - expandv && vi+step_vi < nbVi + expandv);

				// handle overlap in overlapCallback
				if(overlap)
				{
					if(!overlapTraceSegment.initialized())
					{
						// initial overlap and setup
						if(!overlapTraceSegment.init(ui,vi,nbVi,step_ui,step_vi,aCallback))
							return;
					}
					else
					{
						// overlap step
						if(!overlapTraceSegment.step(ui,vi))
							return;
					}
				}
				else
				{
				const PxU32 colIndex0 = PxU32(nbVi * ui + vi);
				const PxU32 colIndex1 = PxU32(nbVi * (ui + step_ui) + vi);
				const PxReal h[4] = { // h[0]=h00, h[1]=h01, h[2]=h10, h[3]=h11 - oriented relative to step_uv
					hf.getHeight(colIndex0) * heightScale, hf.getHeight(colIndex0 + step_vi) * heightScale,
					hf.getHeight(colIndex1) * heightScale, hf.getHeight(colIndex1 + step_vi) * heightScale };

				PxF32 minH = PxMin(PxMin(h[0], h[1]), PxMin(h[2], h[3]));
				PxF32 maxH = PxMax(PxMax(h[0], h[1]), PxMax(h[2], h[3]));

				// how much space in h have we covered from previous to current u or v intercept
				PxF32 hLineCellRangeMin = PxMin(hLinePrev, hLineNext);
				PxF32 hLineCellRangeMax = PxMax(hLinePrev, hLineNext);

				// do a quick overlap test in h, this should be rejecting the vast majority of tests
				if(!(hLineCellRangeMin-hEpsilon > maxH || hLineCellRangeMax+hEpsilon < minH) ||
					(useUnderFaceCallback && hLineCellRangeMax < maxH))
				{
					// arrange h so that h00 corresponds to min(uif, uif+step_uif) h10 to max et c.
					// this is only needed for backface culling to work so we know the proper winding order without branches
					// uflip is 0 or 2, vflip is 0 or 1 (corresponding to positive and negative ui_step and vi_step)
					const PxF32 h00 = h[0+uflip+vflip];
					const PxF32 h01 = h[1+uflip-vflip];
					const PxF32 h10 = h[2-uflip+vflip];
					const PxF32 h11 = h[3-uflip-vflip];

					const PxF32 minuif = PxMin(uif, uif+step_uif);
					const PxF32 maxuif = PxMax(uif, uif+step_uif);
					const PxF32 minvif = PxMin(vif, vif+step_vif);
					const PxF32 maxvif = PxMax(vif, vif+step_vif);
					const PxVec3 p00(minuif, h00, minvif);
					const PxVec3 p01(minuif, h01, maxvif);
					const PxVec3 p10(maxuif, h10, minvif);
					const PxVec3 p11(maxuif, h11, maxvif);

					const PxF32 enlargeEpsilon = 0.0001f;
					const PxVec3* p00a = &p00, *p01a = &p01, *p10a = &p10, *p11a = &p11;
					PxU32 minui = PxU32(PxMin(ui+step_ui, ui)), minvi = PxU32(PxMin(vi+step_vi, vi));

					// row = x|u, column = z|v
					const PxU32 vertIndex = nbVi * minui + minvi;
					const PxU32 cellIndex = vertIndex; // this adds a dummy unused cell in the end of each row; was -minui
					bool isZVS = hf.isZerothVertexShared(vertIndex);
					if(!isZVS)
					{
						// rotate the pointers for flipped edge cells
						p10a = &p00;
						p00a = &p01;
						p01a = &p11;
						p11a = &p10;
					}

					// For triangle index computation, see illustration in Gu::HeightField::getTriangleNormal()
					// Since row = u, column = v
					// for zeroth vert shared the 10 index is the corner of the 0-index triangle, and 01 is 1-index
					// if zeroth vertex is not shared, the 00 index is the corner of 0-index triangle
					if(!useUnderFaceCallback)
					{
						PxReal triT0 = PX_MAX_REAL, triT1 = PX_MAX_REAL;
						bool hit0 = false, hit1 = false;
						PxF32 triU0, triV0, triU1, triV1;

						// PT: TODO: consider testing hole first and skipping ray-tri test. Might be faster.
						if(Gu::intersectRayTriangle(auhP0, duhvNormalized, *p10a, *p00a, *p11a, triT0, triU0, triV0, backfaceCull, enlargeEpsilon) && triT0 >= 0.0f && triT0 <= duhvLength && (hf.getMaterialIndex0(vertIndex) != PxHeightFieldMaterial::eHOLE))
						{
							hit0 = true;
						}
						else
							triT0 = PX_MAX_REAL;

						if(Gu::intersectRayTriangle(auhP0, duhvNormalized, *p01a, *p11a, *p00a, triT1, triU1, triV1, backfaceCull, enlargeEpsilon) && triT1 >= 0.0f && triT1 <= duhvLength && (hf.getMaterialIndex1(vertIndex) != PxHeightFieldMaterial::eHOLE))
						{
							hit1 = true;
						}
						else
							triT1 = PX_MAX_REAL;

						if(hit0 && triT0 <= triT1)
						{
							const PxVec3 hitPoint((auhP0.x + duhvNormalized.x*triT0) * rowScale, auhP0.y + duhvNormalized.y * triT0, (auhP0.z + duhvNormalized.z*triT0) * columnScale);
							if(!aCallback->faceHit(*this, hitPoint, cellIndex*2, triU0, triV0))
								return;
							if(hit1) // possible to hit both triangles in a cell with eMESH_MULTIPLE
							{
								PxVec3 hitPoint1((auhP0.x + duhvNormalized.x*triT1) * rowScale, auhP0.y + duhvNormalized.y * triT1, (auhP0.z + duhvNormalized.z*triT1) * columnScale);
								if(!aCallback->faceHit(*this, hitPoint1, cellIndex*2 + 1, triU1, triV1))
									return;
							}
						}
						else if(hit1 && triT1 <= triT0)
						{
							PxVec3 hitPoint((auhP0.x + duhvNormalized.x*triT1) * rowScale, auhP0.y + duhvNormalized.y * triT1, (auhP0.z + duhvNormalized.z*triT1) * columnScale);
							if(!aCallback->faceHit(*this, hitPoint, cellIndex*2 + 1, triU1, triV1))
								return;
							if(hit0) // possible to hit both triangles in a cell with eMESH_MULTIPLE
							{
								PxVec3 hitPoint1((auhP0.x + duhvNormalized.x*triT0) * rowScale, auhP0.y + duhvNormalized.y * triT0, (auhP0.z + duhvNormalized.z*triT0) * columnScale);
								if(!aCallback->faceHit(*this, hitPoint1, cellIndex*2, triU0, triV0))
									return;
							}
						}
					}
					else
					{
						// TODO: quite a few optimizations are possible here. edges can be shared, intersectRayTriangle inlined etc
						// Go to shape space. Height is already in shape space so we only scale x and z
						const PxVec3 p00s(p00a->x * rowScale, p00a->y, p00a->z * columnScale);
						const PxVec3 p01s(p01a->x * rowScale, p01a->y, p01a->z * columnScale);
						const PxVec3 p10s(p10a->x * rowScale, p10a->y, p10a->z * columnScale);
						const PxVec3 p11s(p11a->x * rowScale, p11a->y, p11a->z * columnScale);

						PxVec3 triNormals[2] = { (p00s - p10s).cross(p11s - p10s), (p11s - p01s).cross(p00s-p01s) };
						triNormals[0] *= PxRecipSqrt(triNormals[0].magnitudeSquared());
						triNormals[1] *= PxRecipSqrt(triNormals[1].magnitudeSquared());
						// since the heightfield can be mirrored with negative rowScale or columnScale, this assert doesn't hold
						//PX_ASSERT(triNormals[0].y >= 0.0f && triNormals[1].y >= 0.0f);

						// at this point we need to compute the edge direction that we crossed
						// also since we don't DDA the w we need to find u,v for w-intercept (w refers to diagonal adjusted with isZVS)
						const PxF32 wnu = isZVS ? -1.0f : 1.0f, wnv = 1.0f; // uv-normal to triangle edge that splits the cell
						const PxF32 wpu = uif + 0.5f * step_uif, wpv = vif + 0.5f * step_vif; // a point on triangle edge that splits the cell
						// note that (wpu, wpv) is on both edges (for isZVS and non-ZVS cases) which is nice

						// we clamp tNext to 1 because we still want to issue callbacks even if we stay in one cell
						// note that tNext can potentially be arbitrarily large for a segment contained within a cell
						const PxF32 tNext = PxMin(PxMin(tu, tv), 1.0f), tPrev = PxMax(last_tu, last_tv);

						// compute uvs corresponding to tPrev, tNext
						const PxF32 unext = u0 + tNext*du, vnext = v0 + tNext*dv;
						const PxF32 uprev = u0 + tPrev*du, vprev = v0 + tPrev*dv;

						const PxReal& h00_ = h[0], &h01_ = h[1], &h10_ = h[2]/*, h11_ = h[3]*/; // aliases for step-oriented h

						// (wpu, wpv) is a point on the diagonal
						// we compute a dot of ((unext, vnext) - (wpu, wpv), wn) to see on which side of triangle edge we are
						// if the dot is positive we need to add 1 to triangle index
						const PxU32 dotPrevGtz = PxU32(((uprev - wpu) * wnu + (vprev - wpv) * wnv) > 0);
						const PxU32 dotNextGtz = PxU32(((unext - wpu) * wnu + (vnext - wpv) * wnv) > 0);
						const PxU32 triIndex0 = cellIndex*2 + dotPrevGtz;
						const PxU32 triIndex1 = cellIndex*2 + dotNextGtz;
						PxU32 isHole0 = PxU32(hf.getMaterialIndex0(vertIndex) == PxHeightFieldMaterial::eHOLE);
						PxU32 isHole1 = PxU32(hf.getMaterialIndex1(vertIndex) == PxHeightFieldMaterial::eHOLE);
						if(triIndex0 > triIndex1)
							PxSwap<PxU32>(isHole0, isHole1);

						// TODO: compute height at u,v inside here, change callback param to PxVec3
						PxVec3 crossedEdge;
						if(last_tu > last_tv) // previous intercept was at u, so we use u=const edge
							crossedEdge = PxVec3(0.0f, h01_-h00_, step_vif * columnScale);
						else // previous intercept at v, use v=const edge
							crossedEdge = PxVec3(step_uif * rowScale, h10_-h00_, 0.0f);

						if(!isHole0 && !aCallback->underFaceHit(*this, triNormals[dotPrevGtz], crossedEdge,
								uprev * rowScale, vprev * columnScale, COMPUTE_H_FROM_T(tPrev), triIndex0))
							return;

						if(triIndex1 != triIndex0 && !isHole1) // if triIndex0 != triIndex1 that means we cross the triangle edge
						{
							// Need to compute tw, the t for ray intersecting the diagonal within the current cell
							// dot((wnu, wnv), (u0+tw*du, v0+tw*dv)-(wpu, wpv)) = 0
							// wnu*(u0+tw*du-wpu) + wnv*(v0+tw*dv-wpv) = 0
							// wnu*u0+wnv*v0-wnu*wpu-wnv*wpv + tw*(du*wnu + dv*wnv) = 0
							const PxF32 denom = du*wnu + dv*wnv;
							if(PxAbs(denom) > 1e-6f)
							{
								const PxF32 tw = (wnu*(wpu-u0)+wnv*(wpv-v0)) / denom;
								if(!aCallback->underFaceHit(*this, triNormals[dotNextGtz], p10s-p01s,
										(u0+tw*du) * rowScale, (v0+tw*dv) * columnScale, COMPUTE_H_FROM_T(tw), triIndex1))
									return;
							}
						}
					}
				}
				}

				if(tu < tv)
				{
					last_tu = tu;
					ui += step_ui;
					// AP: very rare condition, wasn't able to repro but we need this if anyway (DE6565)
					if(ui+step_ui< (0 - expandu) || ui+step_ui>=(nbUi + expandu)) // should hold true for ui without step from previous iteration
						break;
					uif += step_uif;
					tu += step_tu;
				}
				else
				{
					last_tv = tv;
					vi += step_vi;
					// AP: very rare condition, wasn't able to repro but we need this if anyway (DE6565)
					if(vi+step_vi< (0 - expandv) || vi+step_vi>=(nbVi + expandv)) // should hold true for vi without step from previous iteration
						break;
					vif += step_vif;
					tv += step_tv;
				}
				hLinePrev = hLineNext;
			}
			// since min(tu,tv) is the END of the active interval we need to check if PREVIOUS min(tu,tv) was past interval end
			// since we update tMinUV in the beginning of the loop, at this point it stores the min(last tu,last tv)
			while (tMinUV < tEnd);
			#undef COMPUTE_H_FROM_T
		}
	};

} // namespace Gu

}

#endif
