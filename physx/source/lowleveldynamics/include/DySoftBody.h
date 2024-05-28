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

#ifndef DY_SOFTBODY_H
#define DY_SOFTBODY_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxPinnedArray.h"
#include "DySoftBodyCore.h"
#include "PxvGeometry.h"

namespace physx
{
	namespace Sc
	{
		class SoftBodySim;
	}

	namespace Dy
	{

		typedef size_t SoftBodyHandle;

		struct SoftBodyCore;

		typedef PxPinnedArray<PxU64> SoftBodyFilterArray;

		class SoftBody
		{
			PX_NOCOPY(SoftBody)
		public:
			SoftBody(Sc::SoftBodySim* sim, Dy::SoftBodyCore& core) : 
				mSoftBodySoftBodyFilterPairs(NULL), mSim(sim), mCore(core), mElementId(0xffffffff), mGpuRemapId(0xffffffff)
			{
				mFilterDirty = false;
				mFilterInDirtyList = false;
				mDirtySoftBodyForFilterPairs = NULL;
				mSoftBodySoftBodyFilterPairs = NULL;
			}

			~SoftBody()
			{
				if (mDirtySoftBodyForFilterPairs)
				{
					Dy::SoftBody** dirtySoftBodies = mDirtySoftBodyForFilterPairs->begin();

					const PxU32 size = mDirtySoftBodyForFilterPairs->size();

					for (PxU32 i = 0; i < size; ++i)
					{
						if (dirtySoftBodies[i] == this)
						{
							dirtySoftBodies[i] = NULL;
						}
					}

					if (mSoftBodySoftBodyFilterPairs)
						PX_FREE(mSoftBodySoftBodyFilterPairs);
				}
			}

			PX_FORCE_INLINE PxReal getMaxPenetrationBias() const { return mCore.maxPenBias; }

			PX_FORCE_INLINE Sc::SoftBodySim* getSoftBodySim() const { return mSim; }

			PX_FORCE_INLINE void setGpuRemapId(const PxU32 remapId) 
			{ 
				mGpuRemapId = remapId;
				PxTetrahedronMeshGeometryLL& geom = mShapeCore->mGeometry.get<PxTetrahedronMeshGeometryLL>();
				geom.materialsLL.gpuRemapId = remapId;
			}

			PX_FORCE_INLINE PxU32 getGpuRemapId() { return mGpuRemapId; }

			PX_FORCE_INLINE void setElementId(const PxU32 elementId) { mElementId = elementId; }
			PX_FORCE_INLINE PxU32 getElementId() { return mElementId; }

			PX_FORCE_INLINE PxsShapeCore& getShapeCore() { return *mShapeCore; }
			PX_FORCE_INLINE void setShapeCore(PxsShapeCore* shapeCore) { mShapeCore = shapeCore; }

			PX_FORCE_INLINE void setSimShapeCore(PxTetrahedronMesh* simulationMesh, PxSoftBodyAuxData* simulationState) 
			{ 
				mSimulationMesh = simulationMesh;
				mSoftBodyAuxData = simulationState;
			}

			PX_FORCE_INLINE const PxTetrahedronMesh* getCollisionMesh() const { return mShapeCore->mGeometry.get<PxTetrahedronMeshGeometryLL>().tetrahedronMesh; }
			PX_FORCE_INLINE PxTetrahedronMesh* getCollisionMesh() { return mShapeCore->mGeometry.get<PxTetrahedronMeshGeometryLL>().tetrahedronMesh; }

			PX_FORCE_INLINE const PxTetrahedronMesh* getSimulationMesh() const { return mSimulationMesh; }
			PX_FORCE_INLINE PxTetrahedronMesh* getSimulationMesh() { return mSimulationMesh; }

			PX_FORCE_INLINE const PxSoftBodyAuxData* getSoftBodyAuxData() const { return mSoftBodyAuxData; }
			PX_FORCE_INLINE PxSoftBodyAuxData* getSoftBodyAuxData() { return mSoftBodyAuxData; }


			PX_FORCE_INLINE const SoftBodyCore& getCore() const { return mCore; }
			PX_FORCE_INLINE SoftBodyCore& getCore() { return mCore; }

			PX_FORCE_INLINE PxU16 getIterationCounts() const { return mCore.solverIterationCounts; }

			PX_FORCE_INLINE PxU32 getGpuSoftBodyIndex() const { return mGpuRemapId;  }

			//These variables are used in the constraint partition
			PxU16 maxSolverFrictionProgress;
			PxU16 maxSolverNormalProgress;
			PxU32 solverProgress;
			PxU8  numTotalConstraints;

			PxArray<PxU32>			mParticleSoftBodyAttachments;
			PxArray<PxU32>			mRigidSoftBodyAttachments;
			PxArray<PxU32>			mClothSoftBodyAttachments;
			PxArray<PxU32>			mSoftSoftBodyAttachments;

			SoftBodyFilterArray*    mSoftBodySoftBodyFilterPairs;
			PxArray <Dy::SoftBody*>* mDirtySoftBodyForFilterPairs; //pointer to the array of mDirtySoftBodyForFilterPairs in PxgSimulationController.cpp

			PxArray<PxU32>			mSoftBodySoftBodyAttachmentIdReferences;
			bool					mFilterDirty;
			bool					mFilterInDirtyList;

			
		private:
			Sc::SoftBodySim*	mSim;
			SoftBodyCore&		mCore;
			PxsShapeCore*		mShapeCore; 

			PxTetrahedronMesh*   mSimulationMesh;
			PxSoftBodyAuxData*	mSoftBodyAuxData;

			PxU32				mElementId; //this is used for the bound array, contactDist
			PxU32				mGpuRemapId;
		};

		PX_FORCE_INLINE SoftBody* getSoftBody(SoftBodyHandle handle)
		{
			return reinterpret_cast<SoftBody*>(handle);
		}

	}
}

#endif
