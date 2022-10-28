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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef SC_ARTICULATION_SIM_H
#define SC_ARTICULATION_SIM_H

#include "foundation/PxUserAllocated.h"
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
	class ArticulationJointSim;
	class ArticulationSpatialTendonSim;
	class ArticulationFixedTendonSim;
	class ArticulationSensorSim;
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


	class ArticulationSim : public PxUserAllocated 
	{
	public:
											ArticulationSim(ArticulationCore& core, 
												Scene& scene,
												BodyCore& root);

											~ArticulationSim();

		PX_FORCE_INLINE	Dy::FeatherstoneArticulation*	getLowLevelArticulation() const { return mLLArticulation; }
		PX_FORCE_INLINE	ArticulationCore&				getCore() const { return mCore; }
								
								//we don't need removeBody method anymore because when the articulation is removed from the scene, the articulation sim will
								//get completely distroy and when we re-add the articulation to the scene, all the data will get recomputed
								void		addBody(BodySim& body, 
													BodySim* parent, 
													ArticulationJointSim* joint);

								void		removeBody(BodySim& body);
					

								//we don't need removeTendon method anymore because when the articulation is removed from the scene, the articulation sim will
								//get completely distroy and when we re-add the articulation to the scene, all the data will get recomputed
								void		addTendon(ArticulationSpatialTendonSim*);
								
								//we don't need removeTendon method anymore because when the articulation is removed from the scene, the articulation sim will
								//get completely distroy and when we re-add the articulation to the scene, all the data will get recomputed
								void		addTendon(ArticulationFixedTendonSim*);
							
								//we don't need removeSensor method anymore because when the articulation is removed from the scene, the articulation sim will
								//get completely distroy and when we re-add the articulation to the scene, all the data will get recomputed
								void		addSensor(ArticulationSensorSim* sensor, const PxU32 linkID);
								
								void		createLLStructure();						// resize LL memory if necessary
								void		initializeConfiguration();
								void		debugCheckWakeCounterOfLinks(PxReal wakeCounter) const;
								void		debugCheckSleepStateOfLinks(bool isSleeping) const;

								bool		isSleeping() const;
								void		internalWakeUp(PxReal wakeCounter);	// called when sim sets sleep timer
								void		sleepCheck(PxReal dt);
								void		putToSleep();
								void		updateCCDLinks(PxArray<BodySim*>& sims);
								void		updateCached(PxBitMapPinned* shapehapeChangedMap);
								void		markShapesUpdated(PxBitMapPinned* shapeChangedMap);
								void		updateContactDistance(PxReal* contactDistance, const PxReal dt, const Bp::BoundsArray& boundsArray);

								void		setActive(const bool b, const PxU32 infoFlag=0);

								void		updateForces(PxReal dt, bool notify = true);
								void		saveLastCCDTransform();

								void		clearAcceleration(PxReal dt);


					void					setKinematicLink(const bool value);
					//external reduced coordinate implementation
					PxU32					getDofs() const;

					//This function return the dof of the inbound joint, which belong to a link with corresponding linkID
					PxU32					getDof(const PxU32 linkID) const;

					PxArticulationCache*	createCache();

					PxU32					getCacheDataSize() const;

					PxU32					getScratchMemorySize() const;

					void					zeroCache(PxArticulationCache&) const;

					bool					applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag) const;

					void					copyInternalStateToCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag) const;

					void					packJointData(const PxReal* maximum, PxReal* reduced) const;

					void					unpackJointData(const PxReal* reduced, PxReal* maximum) const;

					void					commonInit();

					void					computeGeneralizedGravityForce(PxArticulationCache& cache);

					void					computeCoriolisAndCentrifugalForce(PxArticulationCache& cache);

					void					computeGeneralizedExternalForce(PxArticulationCache& cache);

					void					computeJointAcceleration(PxArticulationCache& cache);

					void					computeJointForce(PxArticulationCache& cache);

					void					computeKinematicJacobian(const PxU32 linkID, PxArticulationCache& cache);

					void					computeDenseJacobian(PxArticulationCache& cache, PxU32& nRows, PxU32& nCols);

					void					computeCoefficientMatrix(PxArticulationCache& cache);

					bool					computeLambda(PxArticulationCache& cache, PxArticulationCache& rollBackCache, const PxReal* jointTorque, const PxVec3 gravity, const PxU32 maxIter);

					void					computeGeneralizedMassMatrix(PxArticulationCache& cache);

					PxU32					getCoefficientMatrixSize() const;

					void					setRootLinearVelocity(const PxVec3& velocity);
					void					setRootAngularVelocity(const PxVec3& velocity);
					PxSpatialVelocity		getLinkVelocity(const PxU32 linkId) const;

					PxSpatialVelocity		getLinkAcceleration(const PxU32 linkId) const;

	
					//internal method implementation
	PX_FORCE_INLINE PxNodeIndex		getIslandNodeIndex() const { return mIslandNodeIndex; }

					void					setGlobalPose();

					PxU32					findBodyIndex(BodySim &body) const;

					void					setJointDirty(Dy::ArticulationJointCore& jointCore);

					void					addLoopConstraint(ConstraintSim* constraint);
					void					removeLoopConstraint(ConstraintSim* constraint);

	PX_FORCE_INLINE	PxU32					getMaxDepth() { return mMaxDepth; }

					void					setArticulationDirty(PxU32 flag);

	PX_FORCE_INLINE	void						setDirtyFlag(ArticulationSimDirtyFlag::Enum flag) { mDirtyFlags = flag; }
	PX_FORCE_INLINE	ArticulationSimDirtyFlags	getDirtyFlag() const { return mDirtyFlags; }

	PX_FORCE_INLINE	const Dy::ArticulationLink&	getLink(const PxU32 linkId) const { return mLinks[linkId]; }

					PxU32					getRootActorIndex() const;
					const PxSpatialForce& getSensorForce(const PxU32 lowLevelIndex) const;
					


					void					updateKinematic(PxArticulationKinematicFlags flags);

					void					copyJointStatus(const PxU32 linkIndex);

	PX_FORCE_INLINE void					getLLArticulationInitialized(bool val) { mIsLLArticultionInitialized = val; }
	PX_FORCE_INLINE	bool					getLLArticulationInitialized() { return mIsLLArticultionInitialized; }
	private:
					ArticulationSim&		operator=(const ArticulationSim&);

					Dy::FeatherstoneArticulation*					mLLArticulation;
					Scene&											mScene;
					ArticulationCore&								mCore;
					PxArray<Dy::ArticulationLink>					mLinks;
					PxArray<BodySim*>								mBodies;
					PxArray<ArticulationJointSim*>					mJoints;
					PxArray<Dy::ArticulationSpatialTendon*>			mSpatialTendons;
					PxArray<Dy::ArticulationFixedTendon*>			mFixedTendons;
					PxArray<Dy::ArticulationSensor*>				mSensors;
					PxArray<PxSpatialForce>							mSensorForces;
					

					PxNodeIndex										mIslandNodeIndex;
					PxArray <Dy::ArticulationLoopConstraint>		mLoopConstraints;
					PxU32											mMaxDepth;
					bool											mIsLLArticultionInitialized;
					ArticulationSimDirtyFlags						mDirtyFlags;
					
	};

} // namespace Sc

}

#endif
