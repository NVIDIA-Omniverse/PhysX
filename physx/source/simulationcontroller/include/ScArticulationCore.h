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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef SC_ARTICULATION_CORE_H
#define SC_ARTICULATION_CORE_H

#include "ScActorCore.h"
#include "DyFeatherstoneArticulation.h"

namespace physx
{
class PxNodeIndex;

namespace Sc
{
	class ArticulationSim;

	class ArticulationCore
	{
		//---------------------------------------------------------------------------------
		// Construction, destruction & initialization
		//---------------------------------------------------------------------------------

// PX_SERIALIZATION
		public:
													ArticulationCore(const PxEMPTY) : mSim(NULL), mCore(PxEmpty) {}
		static		void							getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
													ArticulationCore();
													~ArticulationCore();

		//---------------------------------------------------------------------------------
		// External API
		//---------------------------------------------------------------------------------
		PX_FORCE_INLINE	PxReal						getSleepThreshold()					const	{ return mCore.sleepThreshold;			}
		PX_FORCE_INLINE	void						setSleepThreshold(const PxReal v)			{ mCore.sleepThreshold = v;				}

		PX_FORCE_INLINE	PxReal						getFreezeThreshold()				const	{ return mCore.freezeThreshold;			}
		PX_FORCE_INLINE	void						setFreezeThreshold(const PxReal v)			{ mCore.freezeThreshold = v;			}

		PX_FORCE_INLINE	PxU16						getSolverIterationCounts()			const	{ return mCore.solverIterationCounts;	}
		PX_FORCE_INLINE	void						setSolverIterationCounts(PxU16 c)			{ mCore.solverIterationCounts = c;		}

		PX_FORCE_INLINE	PxReal						getWakeCounter()					const	{ return mCore.wakeCounter;				}
		PX_FORCE_INLINE	void						setWakeCounterInternal(const PxReal v)		{ mCore.wakeCounter = v;				}
						void						setWakeCounter(const PxReal v);

		PX_FORCE_INLINE	PxReal						getMaxLinearVelocity()				const	{ return mCore.maxLinearVelocity;		}
						void						setMaxLinearVelocity(const PxReal max);

		PX_FORCE_INLINE	PxReal						getMaxAngularVelocity()				const	{ return mCore.maxAngularVelocity;		}
						void						setMaxAngularVelocity(const PxReal max);

						bool						isSleeping() const;
						void						wakeUp(PxReal wakeCounter);
						void						putToSleep();

		//---------------------------------------------------------------------------------
		// external reduced coordinate API
		//---------------------------------------------------------------------------------
						void						setArticulationFlags(PxArticulationFlags flags);
						PxArticulationFlags			getArticulationFlags() const { return mCore.flags; }

						PxU32						getDofs() const;

						PxArticulationCache*		createCache() const;

						PxU32						getCacheDataSize() const;

						void						zeroCache(PxArticulationCache& cache) const;

						bool						applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag)const;
		
						void						copyInternalStateToCache
														(PxArticulationCache& cache, const PxArticulationCacheFlags flag, const bool isGpuSimEnabled) const;

						void						packJointData(const PxReal* maximum, PxReal* reduced) const;

						void						unpackJointData(const PxReal* reduced, PxReal* maximum) const;

						void						commonInit() const;

						void						computeGeneralizedGravityForce(PxArticulationCache& cache) const;

						void						computeCoriolisAndCentrifugalForce(PxArticulationCache& cache) const;

						void						computeGeneralizedExternalForce(PxArticulationCache& cache) const;

						void						computeJointAcceleration(PxArticulationCache& cache) const;

						void						computeJointForce(PxArticulationCache& cache) const;

						void						computeDenseJacobian(PxArticulationCache& cache, PxU32& nRows, PxU32& nCols) const;

						void						computeCoefficientMatrix(PxArticulationCache& cache) const;

						bool						computeLambda(PxArticulationCache& cache, PxArticulationCache& rollBackCache, const PxReal* const jointTorque, const PxVec3 gravity, const PxU32 maxIter) const;

						void						computeGeneralizedMassMatrix(PxArticulationCache& cache) const;

						PxU32						getCoefficientMatrixSize() const;

						PxSpatialVelocity			getLinkAcceleration(const PxU32 linkId, const bool isGpuSimEnabled) const;

						PxU32						getGpuArticulationIndex() const;				

						void						updateKinematic(PxArticulationKinematicFlags flags);
		//---------------------------------------------------------------------------------
		// Internal API
		//---------------------------------------------------------------------------------
	public:
		PX_FORCE_INLINE	void						setSim(ArticulationSim* sim)
													{
														PX_ASSERT((sim==0) ^ (mSim == 0));
														mSim = sim;
													}
		PX_FORCE_INLINE	ArticulationSim*			getSim()			const	{ return mSim;			}

		PX_FORCE_INLINE	Dy::ArticulationCore&		getCore()			{ return mCore;			}

		static PX_FORCE_INLINE ArticulationCore&	getArticulationCore(ArticulationCore& core)
													{
														const size_t offset = PX_OFFSET_OF(ArticulationCore, mCore);
														return *reinterpret_cast<ArticulationCore*>(reinterpret_cast<PxU8*>(&core) - offset);
													}

						PxNodeIndex					getIslandNodeIndex() const;

						void						setGlobalPose();

	private:
						ArticulationSim*			mSim;
						Dy::ArticulationCore		mCore;
	};

} // namespace Sc

}

#endif
