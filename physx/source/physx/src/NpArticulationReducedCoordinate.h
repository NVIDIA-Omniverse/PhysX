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

#ifndef NP_ARTICULATION_RC_H
#define NP_ARTICULATION_RC_H

#include "PxArticulationReducedCoordinate.h"
#include "foundation/PxSimpleTypes.h"

#if PX_ENABLE_DEBUG_VISUALIZATION
	#include "common/PxRenderOutput.h"
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

#include "NpArticulationLink.h"
#include "NpArticulationJointReducedCoordinate.h"
#include "NpArticulationTendon.h"
#include "NpArticulationMimicJoint.h"
#include "ScArticulationCore.h"

namespace physx
{
	class NpArticulationLink;
	class NpScene;
	class PxAggregate;
	class PxConstraint;
	class NpArticulationSpatialTendon;
	class NpArticulationFixedTendon;

	class NpArticulationReducedCoordinate : public PxArticulationReducedCoordinate, public NpBase
	{
	public:
		virtual											~NpArticulationReducedCoordinate();

		// PX_SERIALIZATION
														NpArticulationReducedCoordinate(PxBaseFlags baseFlags)
														: PxArticulationReducedCoordinate(baseFlags),
															NpBase(PxEmpty), mCore(PxEmpty),
															mArticulationLinks(PxEmpty), mLoopJoints(PxEmpty), mSpatialTendons(PxEmpty), mFixedTendons(PxEmpty), mMimicJoints(PxEmpty)
														{
														}
		
					void								preExportDataReset();
					virtual			void				exportExtraData(PxSerializationContext& stream);
					void								importExtraData(PxDeserializationContext& context);
					void								resolveReferences(PxDeserializationContext& context);
					virtual			void				requiresObjects(PxProcessPxBaseCallback& c);

		static		NpArticulationReducedCoordinate*	createObject(PxU8*& address, PxDeserializationContext& context);
		static		void								getBinaryMetaData(PxOutputStream& stream);
		//~PX_SERIALIZATION

		virtual			void							release()	PX_OVERRIDE	PX_FINAL;
		//---------------------------------------------------------------------------------
		// PxArticulationReducedCoordinate implementation
		//---------------------------------------------------------------------------------
		virtual			PxScene*					getScene() const	PX_OVERRIDE	PX_FINAL	{ return NpBase::getNpScene(); }

		virtual			void						setSleepThreshold(PxReal threshold)	PX_OVERRIDE	PX_FINAL;
		virtual			PxReal						getSleepThreshold() const	PX_OVERRIDE	PX_FINAL;

		virtual			void						setStabilizationThreshold(PxReal threshold)	PX_OVERRIDE	PX_FINAL;
		virtual			PxReal						getStabilizationThreshold() const	PX_OVERRIDE	PX_FINAL;

		virtual			void						setWakeCounter(PxReal wakeCounterValue)	PX_OVERRIDE	PX_FINAL;
		virtual			PxReal						getWakeCounter() const	PX_OVERRIDE	PX_FINAL;

		virtual			void						setSolverIterationCounts(PxU32 positionIters, PxU32 velocityIters)	PX_OVERRIDE	PX_FINAL;// { mImpl.setSolverIterationCounts(positionIters, velocityIters); }
		virtual			void						getSolverIterationCounts(PxU32 & positionIters, PxU32 & velocityIters) const	PX_OVERRIDE	PX_FINAL;// { mImpl.getSolverIterationCounts(positionIters, velocityIters); }

		virtual			bool						isSleeping() const	PX_OVERRIDE	PX_FINAL;
		virtual			void						wakeUp()	PX_OVERRIDE	PX_FINAL;
		virtual			void						putToSleep()	PX_OVERRIDE	PX_FINAL;

		virtual			void						setMaxCOMLinearVelocity(const PxReal maxLinearVelocity)	PX_OVERRIDE	PX_FINAL;
		virtual			PxReal						getMaxCOMLinearVelocity() const	PX_OVERRIDE	PX_FINAL;
		virtual			void						setMaxCOMAngularVelocity(const PxReal maxAngularVelocity)	PX_OVERRIDE	PX_FINAL;
		virtual			PxReal						getMaxCOMAngularVelocity() const	PX_OVERRIDE	PX_FINAL;

		virtual			PxU32						getNbLinks() const	PX_OVERRIDE	PX_FINAL;
		virtual			PxU32						getLinks(PxArticulationLink** userBuffer, PxU32 bufferSize, PxU32 startIndex) const	PX_OVERRIDE	PX_FINAL;
		/*{
			return mImpl.getLinks(userBuffer, bufferSize, startIndex);
		}
*/
		virtual			PxU32						getNbShapes() const	PX_OVERRIDE	PX_FINAL;

		virtual			PxBounds3					getWorldBounds(float inflation = 1.01f) const	PX_OVERRIDE	PX_FINAL;

		virtual			PxAggregate*				getAggregate() const	PX_OVERRIDE	PX_FINAL;

		// Debug name
		virtual			void						setName(const char* name)	PX_OVERRIDE	PX_FINAL;
		virtual			const char*					getName() const	PX_OVERRIDE	PX_FINAL;

		virtual PxArticulationLink*					createLink(PxArticulationLink* parent, const PxTransform& pose)	PX_OVERRIDE	PX_FINAL;
		
		virtual		void							setArticulationFlags(PxArticulationFlags flags)	PX_OVERRIDE	PX_FINAL;

		virtual		void							setArticulationFlag(PxArticulationFlag::Enum flag, bool value)	PX_OVERRIDE	PX_FINAL;

		virtual		PxArticulationFlags				getArticulationFlags() const	PX_OVERRIDE	PX_FINAL;

		virtual		PxU32							getDofs() const	PX_OVERRIDE	PX_FINAL;

		virtual		PxArticulationCache*			createCache() const	PX_OVERRIDE	PX_FINAL;

		virtual		PxU32							getCacheDataSize() const	PX_OVERRIDE	PX_FINAL;

		virtual		void							zeroCache(PxArticulationCache& cache) const	PX_OVERRIDE	PX_FINAL;

		virtual		void							applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flags, bool autowake)	PX_OVERRIDE	PX_FINAL;

		virtual		void							copyInternalStateToCache(PxArticulationCache& cache, const PxArticulationCacheFlags flags) const	PX_OVERRIDE	PX_FINAL;

		virtual		void							packJointData(const PxReal* maximum, PxReal* reduced) const	PX_OVERRIDE	PX_FINAL;

		virtual		void							unpackJointData(const PxReal* reduced, PxReal* maximum) const	PX_OVERRIDE	PX_FINAL;

		virtual		void							commonInit() const	PX_OVERRIDE	PX_FINAL;

		virtual		void							computeGeneralizedGravityForce(PxArticulationCache& cache) const	PX_OVERRIDE	PX_FINAL;

		virtual		void							computeCoriolisAndCentrifugalForce(PxArticulationCache& cache) const	PX_OVERRIDE	PX_FINAL;

		virtual		void							computeGeneralizedExternalForce(PxArticulationCache& cache) const	PX_OVERRIDE	PX_FINAL;

		virtual		void							computeJointAcceleration(PxArticulationCache& cache) const	PX_OVERRIDE	PX_FINAL;
		
		virtual		void							computeJointForce(PxArticulationCache& cache) const	PX_OVERRIDE	PX_FINAL;

		virtual		void							computeDenseJacobian(PxArticulationCache& cache, PxU32& nRows, PxU32& nCols) const	PX_OVERRIDE	PX_FINAL;

		virtual		void							computeCoefficientMatrix(PxArticulationCache& cache) const	PX_OVERRIDE	PX_FINAL;

		virtual		bool							computeLambda(PxArticulationCache& cache, PxArticulationCache& rollBackCache, const PxReal* const jointTorque, const PxU32 maxIter) const	PX_OVERRIDE	PX_FINAL;

		virtual		void							computeGeneralizedMassMatrix(PxArticulationCache& cache) const	PX_OVERRIDE	PX_FINAL;

		virtual		void							addLoopJoint(PxConstraint* joint)	PX_OVERRIDE	PX_FINAL;

		virtual		void							removeLoopJoint(PxConstraint* constraint)	PX_OVERRIDE	PX_FINAL;

		virtual		PxU32							getNbLoopJoints() const	PX_OVERRIDE	PX_FINAL;

		virtual		PxU32							getLoopJoints(PxConstraint** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const	PX_OVERRIDE	PX_FINAL;

		virtual		PxU32							getCoefficientMatrixSize() const	PX_OVERRIDE	PX_FINAL;

		virtual		void							setRootGlobalPose(const PxTransform& pose, bool autowake = true)	PX_OVERRIDE	PX_FINAL;

		virtual		PxTransform						getRootGlobalPose() const	PX_OVERRIDE	PX_FINAL;

		virtual		void							setRootLinearVelocity(const PxVec3& velocity, bool autowake = true)	PX_OVERRIDE	PX_FINAL;
		virtual		void							setRootAngularVelocity(const PxVec3& velocity, bool autowake = true)	PX_OVERRIDE	PX_FINAL;
		virtual		PxVec3							getRootLinearVelocity() const	PX_OVERRIDE	PX_FINAL;
		virtual		PxVec3							getRootAngularVelocity() const	PX_OVERRIDE	PX_FINAL;

		virtual		PxSpatialVelocity				getLinkAcceleration(const PxU32 linkId)	PX_OVERRIDE	PX_FINAL;

		virtual		const char*						getConcreteTypeName() const	PX_OVERRIDE	PX_FINAL { return "PxArticulationReducedCoordinate"; }
		
		virtual		PxU32							getGpuArticulationIndex(); // DEPRECATED
		virtual		PxArticulationGPUIndex			getGPUIndex() const PX_OVERRIDE PX_FINAL;

		virtual		PxArticulationSpatialTendon*		createSpatialTendon()	PX_OVERRIDE	PX_FINAL;

		virtual		PxU32								getSpatialTendons(PxArticulationSpatialTendon** userBuffer, PxU32 bufferSize, PxU32 startIndex) const	PX_OVERRIDE	PX_FINAL;

		virtual		PxU32								getNbSpatialTendons()	const	PX_OVERRIDE	PX_FINAL;

		NpArticulationSpatialTendon*					getSpatialTendon(const PxU32 index) const;

		virtual		PxArticulationFixedTendon*			createFixedTendon()	PX_OVERRIDE	PX_FINAL;

		virtual		PxU32								getFixedTendons(PxArticulationFixedTendon** userBuffer, PxU32 bufferSize, PxU32 startIndex) const	PX_OVERRIDE	PX_FINAL;

		virtual		PxU32								getNbFixedTendons()	const	PX_OVERRIDE	PX_FINAL;

		NpArticulationFixedTendon*						getFixedTendon(const PxU32 index) const;

		virtual		PxArticulationMimicJoint*			createMimicJoint(const PxArticulationJointReducedCoordinate& jointA, PxArticulationAxis::Enum axisA, const PxArticulationJointReducedCoordinate& jointB, PxArticulationAxis::Enum axisB, PxReal gearRatio, PxReal offset);

		virtual		PxU32								getMimicJoints(PxArticulationMimicJoint** userBuffer, PxU32 bufferSize, PxU32 startIndex) const;

		virtual		PxU32								getNbMimicJoints() const;

		NpArticulationMimicJoint*						getMimicJoint(const PxU32 index) const;

		virtual		void								updateKinematic(PxArticulationKinematicFlags flags) PX_OVERRIDE PX_FINAL;

		PX_FORCE_INLINE	PxArray<NpArticulationSpatialTendon*>&	getSpatialTendons() { return mSpatialTendons; }
		PX_FORCE_INLINE	PxArray<NpArticulationFixedTendon*>&	getFixedTendons() { return mFixedTendons; }
		PX_FORCE_INLINE	PxArray<NpArticulationMimicJoint*>&		getMimicJoints() { return mMimicJoints; }

		//---------------------------------------------------------------------------------
		// Miscellaneous
		//---------------------------------------------------------------------------------
		NpArticulationReducedCoordinate();

		virtual		bool			isKindOf(const char* name) const { PX_IS_KIND_OF(name, "PxArticulationReducedCoordinate", PxBase); }

		PxArticulationJointReducedCoordinate*			createArticulationJoint(PxArticulationLink& parent,
																				const PxTransform& parentFrame,
																				PxArticulationLink& child,
																				const PxTransform& childFrame);

		PX_INLINE	void							incrementShapeCount() { mNumShapes++; }
		PX_INLINE	void							decrementShapeCount() { mNumShapes--; }


		virtual PxArticulationResidual				getSolverResidual() const;

		//---------------------------------------------------------------------------------
		// Miscellaneous
		//---------------------------------------------------------------------------------
		
		PX_INLINE		void						addToLinkList(NpArticulationLink& link) { mArticulationLinks.pushBack(&link); mNumShapes += link.getNbShapes(); }
		PX_INLINE bool								removeLinkFromList(NpArticulationLink& link) 
		{ 
			PX_ASSERT(mArticulationLinks.find(&link) != mArticulationLinks.end()); 
			mTopologyChanged = true;
			return mArticulationLinks.findAndReplaceWithLast(&link); 
		}
		PX_FORCE_INLINE	NpArticulationLink* const*	getLinks() { return mArticulationLinks.begin(); }

		NpArticulationLink*							getRoot();
		void										setAggregate(PxAggregate* a);

		void										wakeUpInternal(bool forceWakeUp, bool autowake);
		void										autoWakeInternal();

		void										setGlobalPose();

		PX_FORCE_INLINE	Sc::ArticulationCore&		getCore()			{ return mCore; }
		PX_FORCE_INLINE	const Sc::ArticulationCore&	getCore()	const	{ return mCore; }
		static PX_FORCE_INLINE size_t				getCoreOffset()		{ return PX_OFFSET_OF_RT(NpArticulationReducedCoordinate, mCore); }

		PX_INLINE		void		scSetSolverIterationCounts(PxU16 v)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setSolverIterationCounts(v);
			UPDATE_PVD_PROPERTY
		}

		PX_INLINE		void		scSetSleepThreshold(const PxReal v)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setSleepThreshold(v);
			UPDATE_PVD_PROPERTY
		}

		PX_INLINE		void		scSetFreezeThreshold(const PxReal v)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setFreezeThreshold(v);
			UPDATE_PVD_PROPERTY
		}

		PX_INLINE		void		scSetWakeCounter(PxReal counter)
		{
			PX_ASSERT(!isAPIWriteForbiddenExceptSplitSim());
			mCore.setWakeCounter(counter);
			UPDATE_PVD_PROPERTY
		}

		PX_FORCE_INLINE	void		scSetArticulationFlags(PxArticulationFlags flags)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setArticulationFlags(flags);
			UPDATE_PVD_PROPERTY
		}

		PX_FORCE_INLINE	void		scWakeUpInternal(PxReal wakeCounter)
		{
			PX_ASSERT(getNpScene());
			PX_ASSERT(!isAPIWriteForbiddenExceptSplitSim());
			mCore.wakeUp(wakeCounter);
		}

		PX_FORCE_INLINE	void		scSetMaxLinearVelocity(PxReal maxLinearVelocity)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setMaxLinearVelocity(maxLinearVelocity);
			UPDATE_PVD_PROPERTY
		}

		PX_FORCE_INLINE	void		scSetMaxAngularVelocity(PxReal maxAngularVelocity)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			mCore.setMaxAngularVelocity(maxAngularVelocity);
			UPDATE_PVD_PROPERTY
		}

		void			recomputeLinkIDs();

#if PX_ENABLE_DEBUG_VISUALIZATION
public:
	void						visualize(PxRenderOutput& out, NpScene& scene, float scale)	const;
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

	Sc::ArticulationCore		mCore;
	NpArticulationLinkArray		mArticulationLinks;
	PxU32						mNumShapes;
	NpAggregate*				mAggregate;
	const char*					mName;
	PxU32						mCacheVersion;
	bool						mTopologyChanged;

	private:

		void									removeSpatialTendonInternal(NpArticulationSpatialTendon* tendon);
		void									removeFixedTendonInternal(NpArticulationFixedTendon* tendon);
		void									removeMimicJointInternal(NpArticulationMimicJoint* mimicJoint);

		PxArray<NpConstraint*>					mLoopJoints;
		PxArray<NpArticulationSpatialTendon*>	mSpatialTendons;
		PxArray<NpArticulationFixedTendon*>		mFixedTendons;
		PxArray<NpArticulationMimicJoint*>		mMimicJoints;
		
		friend class NpScene;
	};


}

#endif
