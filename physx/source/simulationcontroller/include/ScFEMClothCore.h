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

#ifndef PX_PHYSICS_SCP_FEMCLOTH_CORE
#define PX_PHYSICS_SCP_FEMCLOTH_CORE

#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_GPU_PHYSX
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
#include "PxFEMCloth.h"
#endif
#include "PxFEMParameter.h"
#include "../../lowleveldynamics/include/DyFEMClothCore.h"
#include "foundation/PxAssert.h"
#include "ScActorCore.h"
#include "ScShapeCore.h"
#include "PxFiltering.h"
#include "ScRigidCore.h" //KS - required for ShapeChangeNotifyFlags. Ideally, we should move that to a separate shared file
#include "PxConeLimitedConstraint.h"

namespace physx
{
	namespace Sc
	{
		class FEMClothSim;

		class FEMClothCore : public ActorCore
		{
			// PX_SERIALIZATION
		public:
			FEMClothCore(const PxEMPTY) : ActorCore(PxEmpty) {}
			//static void getBinaryMetaData(PxOutputStream& stream);

			//~PX_SERIALIZATION
			FEMClothCore();
			~FEMClothCore();

			//---------------------------------------------------------------------------------
			// External API
			//---------------------------------------------------------------------------------
			PxFEMParameters				getParameter() const;
			void						setParameter(const PxFEMParameters& paramters);

			void						addRigidFilter(Sc::BodyCore* core, PxU32 vertId);
			void						removeRigidFilter(Sc::BodyCore* core, PxU32 vertId);

			PxU32						addRigidAttachment(Sc::BodyCore* core, PxU32 vertId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint);
			void						removeRigidAttachment(Sc::BodyCore* core, PxU32 handle);

			void						addTriRigidFilter(Sc::BodyCore* core, PxU32 triIdx);
			void						removeTriRigidFilter(Sc::BodyCore* core, PxU32 triIdx);

			PxU32						addTriRigidAttachment(Sc::BodyCore* core, PxU32 triIdx, const PxVec4& barycentric, 
										const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint);
			void						removeTriRigidAttachment(Sc::BodyCore* core, PxU32 handle);

			void						addClothFilter(Sc::FEMClothCore* otherCore, PxU32 otherTriIdx, PxU32 triIdx);
			void						removeClothFilter(Sc::FEMClothCore* otherCore, PxU32 otherTriIdx0, PxU32 triIdx);

			PxU32						addClothAttachment(Sc::FEMClothCore* otherCore, PxU32 otherTriIdx, const PxVec4& otherTriBarycentric, PxU32 triIdx, 
										const PxVec4& triBarycentric);
			void						removeClothAttachment(Sc::FEMClothCore* otherCore, PxU32 handle);

#if 0 // disabled until future use.
			void						setDrag(const PxReal v) { mCore.drag = v; }
			PxReal						getDrag() const { return mCore.drag; }

			void						setLift(const PxReal v) { mCore.lift = v; }
			PxReal						getLift() const { return mCore.lift; }

			void						setWind(const PxVec3& wind) { mCore.wind = wind; }
			PxVec3						getWind() const { return mCore.wind; }

			void						setAirDensity(const float airDensity) { mCore.airDensity = airDensity; }
			PxReal						getAirDensity() const { return mCore.airDensity; }

			void						setBendingActivationAngle(const PxReal angle) { mCore.mBendingActivationAngle = angle; }
			PxReal						getBendingActivationAngle() const { return mCore.mBendingActivationAngle; }
#endif

			void						setBendingScales(const PxReal* const bendingScales, PxU32 nbElements);
	        const PxReal*				getBendingScales() const;
	        PxU32						getNbBendingScales() const { return mCore.mBendingScales.size(); }

			void						setMaxVelocity(const float maxVelocity) { mCore.maxVelocity = maxVelocity; }
			PxReal						getMaxVelocity() const { return mCore.maxVelocity; }

			void						setMaxDepenetrationVelocity(const float maxDepenetrationVelocity) { mCore.maxDepenetrationVelocity = maxDepenetrationVelocity; }
			PxReal						getMaxDepenetrationVelocity() const { return mCore.maxDepenetrationVelocity; }

			void						setNbCollisionPairUpdatesPerTimestep(const PxU32 frequency) { mCore.nbCollisionPairUpdatesPerTimestep = frequency; }
			PxU32						getNbCollisionPairUpdatesPerTimestep() const { return mCore.nbCollisionPairUpdatesPerTimestep; }

			void						setNbCollisionSubsteps(const PxU32 frequency) { mCore.nbCollisionSubsteps = frequency; }
			PxU32						getNbCollisionSubsteps() const { return mCore.nbCollisionSubsteps; }

			PxU16						getSolverIterationCounts() const { return mCore.solverIterationCounts; }
			void						setSolverIterationCounts(PxU16 c);
			PxActor*					getPxActor() const;

			void						attachShapeCore(ShapeCore* shapeCore);

			PxReal						getWakeCounter() const;
			void						setWakeCounter(const PxReal v);
			void						setWakeCounterInternal(const PxReal v);

			//---------------------------------------------------------------------------------
			// Internal API
			//---------------------------------------------------------------------------------
		public:

			FEMClothSim*		getSim() const;

			PX_FORCE_INLINE	const Dy::FEMClothCore&	getCore() const { return mCore; }

			PX_FORCE_INLINE	Dy::FEMClothCore& getCore() { return mCore; }

			static PX_FORCE_INLINE FEMClothCore&	getFEMClothCore(FEMClothCore& core)
			{
				size_t offset = PX_OFFSET_OF_RT(FEMClothCore, mCore);
				return *reinterpret_cast<FEMClothCore*>(reinterpret_cast<PxU8*>(&core) - offset);
			}

			void								setSimulationFilterData(const PxFilterData& data);

			PxFilterData						getSimulationFilterData() const;

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
			PxFEMClothFlags						getFlags() const { return mCore.mFlags; }

			void								setFlags(PxFEMClothFlags flags);
#endif

			PX_FORCE_INLINE	PxU64&				getGpuMemStat() { return mGpuMemStat; }

			void onShapeChange(ShapeCore& shape, ShapeChangeNotifyFlags notifyFlags);

		private:
			Dy::FEMClothCore				mCore;
			PxFilterData					mFilterData;
			PxU64							mGpuMemStat;
		};

	} // namespace Sc
}
#endif

#endif
