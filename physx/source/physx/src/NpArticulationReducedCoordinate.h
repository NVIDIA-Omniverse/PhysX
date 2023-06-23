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

#ifndef NP_ARTICULATION_RC_H
#define NP_ARTICULATION_RC_H

#include "PxArticulationReducedCoordinate.h"

#if PX_ENABLE_DEBUG_VISUALIZATION
	#include "common/PxRenderOutput.h"
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

#include "NpArticulationLink.h"
#include "NpArticulationJointReducedCoordinate.h"
#include "NpArticulationTendon.h"
#include "ScArticulationCore.h"

namespace physx
{
	class NpArticulationLink;
	class NpScene;
	class PxAggregate;
	class PxConstraint;
	class NpArticulationSpatialTendon;
	class NpArticulationFixedTendon;
	class NpArticulationSensor;

	class NpArticulationReducedCoordinate : public PxArticulationReducedCoordinate, public NpBase
	{
	public:
		virtual											~NpArticulationReducedCoordinate();

		// PX_SERIALIZATION
														NpArticulationReducedCoordinate(PxBaseFlags baseFlags)
														: PxArticulationReducedCoordinate(baseFlags),
															NpBase(PxEmpty), mCore(PxEmpty),
															mArticulationLinks(PxEmpty), mLoopJoints(PxEmpty), mSpatialTendons(PxEmpty), mFixedTendons(PxEmpty), mSensors(PxEmpty)
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

		virtual			void							release();
		//---------------------------------------------------------------------------------
		// PxArticulationReducedCoordinate implementation
		//---------------------------------------------------------------------------------
		virtual			PxScene*					getScene() const { return NpBase::getNpScene(); }

		virtual			void						setSleepThreshold(PxReal threshold);
		virtual			PxReal						getSleepThreshold() const;

		virtual			void						setStabilizationThreshold(PxReal threshold);
		virtual			PxReal						getStabilizationThreshold() const;

		virtual			void						setWakeCounter(PxReal wakeCounterValue);
		virtual			PxReal						getWakeCounter() const;

		virtual			void						setSolverIterationCounts(PxU32 positionIters, PxU32 velocityIters);// { mImpl.setSolverIterationCounts(positionIters, velocityIters); }
		virtual			void						getSolverIterationCounts(PxU32 & positionIters, PxU32 & velocityIters) const;// { mImpl.getSolverIterationCounts(positionIters, velocityIters); }

		virtual			bool						isSleeping() const;
		virtual			void						wakeUp();
		virtual			void						putToSleep();

		virtual			void						setMaxCOMLinearVelocity(const PxReal maxLinearVelocity);
		virtual			PxReal						getMaxCOMLinearVelocity() const;
		virtual			void						setMaxCOMAngularVelocity(const PxReal maxAngularVelocity);
		virtual			PxReal						getMaxCOMAngularVelocity() const;

		virtual			PxU32						getNbLinks() const;
		virtual			PxU32						getLinks(PxArticulationLink** userBuffer, PxU32 bufferSize, PxU32 startIndex) const;
		/*{
			return mImpl.getLinks(userBuffer, bufferSize, startIndex);
		}
*/
		virtual			PxU32						getNbShapes() const;

		virtual			PxBounds3					getWorldBounds(float inflation = 1.01f) const;

		virtual			PxAggregate*				getAggregate() const;

		// Debug name
		virtual			void						setName(const char* name);
		virtual			const char*					getName() const;

		virtual PxArticulationLink*					createLink(PxArticulationLink* parent, const PxTransform& pose);

		
		virtual		void							setArticulationFlags(PxArticulationFlags flags);

		virtual		void							setArticulationFlag(PxArticulationFlag::Enum flag, bool value);

		virtual		PxArticulationFlags				getArticulationFlags() const;

		virtual		PxU32							getDofs() const;

		virtual		PxArticulationCache*			createCache() const;

		virtual		PxU32							getCacheDataSize() const;

		virtual		void							zeroCache(PxArticulationCache& cache) const;

		virtual		void							applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flags, bool autowake);

		virtual		void							copyInternalStateToCache(PxArticulationCache& cache, const PxArticulationCacheFlags flags) const;

		virtual		void							packJointData(const PxReal* maximum, PxReal* reduced) const;

		virtual		void							unpackJointData(const PxReal* reduced, PxReal* maximum) const;

		virtual		void							commonInit() const;

		virtual		void							computeGeneralizedGravityForce(PxArticulationCache& cache) const;

		virtual		void							computeCoriolisAndCentrifugalForce(PxArticulationCache& cache) const;

		virtual		void							computeGeneralizedExternalForce(PxArticulationCache& cache) const;

		virtual		void							computeJointAcceleration(PxArticulationCache& cache) const;
		
		virtual		void							computeJointForce(PxArticulationCache& cache) const;


		virtual		void							computeDenseJacobian(PxArticulationCache& cache, PxU32& nRows, PxU32& nCols) const;

		virtual		void							computeCoefficientMatrix(PxArticulationCache& cache) const;

		virtual		bool							computeLambda(PxArticulationCache& cache, PxArticulationCache& rollBackCache, const PxReal* const jointTorque, const PxU32 maxIter) const;

		virtual		void							computeGeneralizedMassMatrix(PxArticulationCache& cache) const;

		virtual		void							addLoopJoint(PxConstraint* joint);

		virtual		void							removeLoopJoint(PxConstraint* constraint);

		virtual		PxU32							getNbLoopJoints() const;

		virtual		PxU32							getLoopJoints(PxConstraint** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const;

		virtual		PxU32							getCoefficientMatrixSize() const;

		virtual		void							setRootGlobalPose(const PxTransform& pose, bool autowake = true);

		virtual		PxTransform						getRootGlobalPose() const;

		virtual		void							setRootLinearVelocity(const PxVec3& velocity, bool autowake = true);
		virtual		void							setRootAngularVelocity(const PxVec3& velocity, bool autowake = true);
		virtual		PxVec3							getRootLinearVelocity() const;
		virtual		PxVec3							getRootAngularVelocity() const;

		virtual		PxSpatialVelocity				getLinkAcceleration(const PxU32 linkId);

		virtual		PxU32							getGpuArticulationIndex();

		virtual		const char*							getConcreteTypeName() const { return "PxArticulationReducedCoordinate"; }

		virtual		PxArticulationSpatialTendon*		createSpatialTendon();

		virtual		PxU32								getSpatialTendons(PxArticulationSpatialTendon** userBuffer, PxU32 bufferSize, PxU32 startIndex) const;

		virtual		PxU32								getNbSpatialTendons();

		NpArticulationSpatialTendon*					getSpatialTendon(const PxU32 index) const;

		virtual		PxArticulationFixedTendon*			createFixedTendon();

		virtual		PxU32								getFixedTendons(PxArticulationFixedTendon** userBuffer, PxU32 bufferSize, PxU32 startIndex) const;

		virtual		PxU32								getNbFixedTendons();

		NpArticulationFixedTendon*						getFixedTendon(const PxU32 index) const;

		virtual		PxArticulationSensor*				createSensor(PxArticulationLink* link, const PxTransform& relativePose);

		virtual		void								releaseSensor(PxArticulationSensor& sensor);

		virtual		PxU32								getSensors(PxArticulationSensor** userBuffer, PxU32 bufferSize, PxU32 startIndex) const;

		virtual		PxU32								getNbSensors();

		NpArticulationSensor*							getSensor(const PxU32 index) const;

		virtual		void								updateKinematic(PxArticulationKinematicFlags flags);

		PX_FORCE_INLINE	PxArray<NpArticulationSpatialTendon*>&	getSpatialTendons() { return mSpatialTendons; }
		PX_FORCE_INLINE	PxArray<NpArticulationFixedTendon*>&	getFixedTendons() { return mFixedTendons; }
		PX_FORCE_INLINE	PxArray<NpArticulationSensor*>&			getSensors() { return mSensors; }

		//---------------------------------------------------------------------------------
		// Miscellaneous
		//---------------------------------------------------------------------------------
		NpArticulationReducedCoordinate();

		virtual		bool			isKindOf(const char* name) const { return !::strcmp("PxArticulationReducedCoordinate", name) || PxBase::isKindOf(name); }

		PxArticulationJointReducedCoordinate*			createArticulationJoint(PxArticulationLink& parent,
																				const PxTransform& parentFrame,
																				PxArticulationLink& child,
																				const PxTransform& childFrame);

		PX_INLINE	void							incrementShapeCount() { mNumShapes++; }
		PX_INLINE	void							decrementShapeCount() { mNumShapes--; }

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
		void									removeSensorInternal(NpArticulationSensor* sensor);

		PxArray<NpConstraint*>					mLoopJoints;
		PxArray<NpArticulationSpatialTendon*>	mSpatialTendons;
		PxArray<NpArticulationFixedTendon*>		mFixedTendons;
		PxArray<NpArticulationSensor*>			mSensors;
		
		friend class NpScene;
	};


}

#endif
