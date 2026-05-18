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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef SC_ARTICULATION_SIM_H
#define SC_ARTICULATION_SIM_H

#include "foundation/PxUserAllocated.h"
#include "foundation/PxPinnedArray.h"
#include "foundation/PxPinnedBitMap.h"
#include "ScArticulationCore.h" 
#include "PxsSimpleIslandManager.h"
#include "DyArticulationTendon.h"

namespace physx
{

namespace Bp
{
	class BoundsArray;
}

namespace Sc
{
	class BodySim;
	class BodyCore;
	class ArticulationJointSim;
	class ArticulationSpatialTendonSim;
	class ArticulationFixedTendonSim;
	class ArticulationMimicJointSim;
	class ArticulationCore;
	class Scene;
	class ConstraintSim;

	struct ArticulationSimDirtyFlag
	{
		enum Enum
		{
			eNONE = 0,
			eUPDATE = 1 << 0
		};
	};

	typedef PxFlags<ArticulationSimDirtyFlag::Enum, PxU32> ArticulationSimDirtyFlags;

	class ArticulationSim : public Dy::FeatherstoneArticulation
	{
		PX_NOCOPY(ArticulationSim)
	public:
											ArticulationSim(ArticulationCore& core, 
												Scene& scene,
												BodyCore& root);

											~ArticulationSim();

		PX_FORCE_INLINE	Dy::FeatherstoneArticulation*	getLowLevelArticulation()	{ return this; }
		PX_FORCE_INLINE	ArticulationCore&				getCore() const { return mCore; }
								
								//we don't need removeBody method anymore because when the articulation is removed from the scene, the articulation sim will
								//get completely distroy and when we re-add the articulation to the scene, all the data will get recomputed
								void		addBody(BodySim& body, BodySim* parent, ArticulationJointSim* joint);

								void		removeBody(BodySim& body);

								//we don't need complementary removeTendon/removeMimicJoint functions because 
								//the articulation sim will be completely destroyed when the articulation is removed from the scene. 
								//When we re-add the articulation to the scene all the data will be recomputed.
	
								void		addTendon(ArticulationSpatialTendonSim* const);
								
								void		addTendon(ArticulationFixedTendonSim* const);
							
								void		addMimicJoint(ArticulationMimicJointSim* const mimicJoint, const PxU32 linkA, const PxU32 linkB);

								void		createLLStructure();						// resize LL memory if necessary
								void		initializeConfiguration();
								void		debugCheckWakeCounterOfLinks(PxReal wakeCounter) const;
								void		debugCheckSleepStateOfLinks(bool isSleeping) const;

								bool		isSleeping() const;
								void		internalWakeUp(PxReal wakeCounter);	// called when sim sets sleep timer
								void		sleepCheck(PxReal dt);
								void		putToSleep();
								void		updateCCDLinks(PxArray<BodySim*>& sims);
								void		updateCached_NotThreadSafe(PxBitMapPinned* shapeChangedMap);
								void		markShapesUpdated(PxBitMapPinned* shapeChangedMap);
								void		updateContactDistance(PxReal* contactDistance, PxReal dt, const Bp::BoundsArray& boundsArray);

								void		setActive(bool b, bool asPartOfCreation=false);

								void		updateForces(PxReal dt);
								void		saveLastCCDTransform();

								void		clearAcceleration(PxReal dt);

					void					setFixedBaseLink(bool value);

					PxArticulationCache*	createCache();

					PxU32					getCacheDataSize() const;

					void					zeroCache(PxArticulationCache&) const;

					bool					applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag)/* const*/;

					void					computeGeneralizedGravityForce(PxArticulationCache& cache);

					void					computeJointAcceleration(PxArticulationCache& cache);

					bool					computeLambda(PxArticulationCache& cache, PxArticulationCache& rollBackCache, const PxReal* jointTorque, const PxVec3 gravity, const PxU32 maxIter);

					PxU32					getCoefficientMatrixSize() const;

	PX_FORCE_INLINE	PxSpatialVelocity		getLinkAcceleration(PxU32 linkId, bool isGpuSimEnabled) const
											{
												Cm::SpatialVector accel = getMotionAcceleration(linkId, isGpuSimEnabled);
												return reinterpret_cast<PxSpatialVelocity&>(accel);
											}

	PX_FORCE_INLINE PxNodeIndex				getIslandNodeIndex() const { return mIslandNodeIndex; }

					PxU32					findBodyIndex(BodySim& body) const;

					void					setJointDirty(Dy::ArticulationJointCore& jointCore);

					void					addLoopConstraint(ConstraintSim* constraint);
					void					removeLoopConstraint(ConstraintSim* constraint);

					void					setArticulationDirty(PxU32 flag);

	PX_FORCE_INLINE	void						setDirtyFlag(ArticulationSimDirtyFlag::Enum flag) { mDirtyFlags = flag; }
	PX_FORCE_INLINE	ArticulationSimDirtyFlags	getDirtyFlag() const { return mDirtyFlags; }

	PX_FORCE_INLINE	const Dy::ArticulationLink&	getLink(const PxU32 linkId) const { return mLinks[linkId]; }

					PxU32					getRootActorIndex() const;
					
					void					updateKinematic(PxArticulationKinematicFlags flags);

					void					copyJointStatus(const PxU32 linkIndex);

	PX_FORCE_INLINE	bool					isLLArticulationInitialized()	const	{ return mIsLLArticulationInitialized; }

	// PT: the following pass-through functions could simply be removed by unifying the ArticulationSim & FeatherstoneArticulation names.
	PX_FORCE_INLINE	void					commonInit()																	{ initializeCommonData();					}
	PX_FORCE_INLINE	void					computeCoriolisAndCentrifugalForce(PxArticulationCache& cache)					{ getCoriolisAndCentrifugalForce(cache);	}
	PX_FORCE_INLINE	void					computeGeneralizedExternalForce(PxArticulationCache& cache)						{ getGeneralizedExternalForce(cache);		}
	PX_FORCE_INLINE	void					computeJointForce(PxArticulationCache& cache)									{ getJointForce(cache);						}
	PX_FORCE_INLINE	void					computeDenseJacobian(PxArticulationCache& cache, PxU32& nRows, PxU32& nCols)	{ getDenseJacobian(cache, nRows, nCols);	}
	PX_FORCE_INLINE	void					computeCoefficientMatrix(PxArticulationCache& cache)							{ getCoefficientMatrixWithLoopJoints(mLoopConstraints.begin(), mLoopConstraints.size(), cache);	}
	PX_FORCE_INLINE	void					computeGeneralizedMassMatrix(PxArticulationCache& cache)						{ getGeneralizedMassMatrixCRB(cache);		}
	PX_FORCE_INLINE	PxVec3					computeArticulationCOM(const bool rootFrame)									{ return getArticulationCOM(rootFrame);		}
	PX_FORCE_INLINE	void					computeCentroidalMomentumMatrix(PxArticulationCache& cache)						{ getCentroidalMomentumMatrix(cache);		}
	// This method allows user teleport the root links and the articulation system update all other links pose
	PX_FORCE_INLINE	void					setGlobalPose()																	{ teleportRootLink();	}

	private:
					Scene&											mScene;
					ArticulationCore&								mCore;
					PxArray<Dy::ArticulationLink>					mLinks;
					PxArray<BodySim*>								mBodies;
					PxArray<ArticulationJointSim*>					mJoints;
					PxArray<Dy::ArticulationSpatialTendon*>			mSpatialTendons;
					PxArray<Dy::ArticulationFixedTendon*>			mFixedTendons;
					PxArray<Dy::ArticulationMimicJointCore*>		mMimicJoints;
					
					PxNodeIndex										mIslandNodeIndex;
					PxArray <Dy::ArticulationLoopConstraint>		mLoopConstraints;
					bool											mIsLLArticulationInitialized;
					ArticulationSimDirtyFlags						mDirtyFlags;
	};

	ArticulationSim* getArticulationSim(const IG::IslandSim& islandSim, PxNodeIndex nodeIndex);

} // namespace Sc

}

#endif
