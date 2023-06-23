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

// ****************************************************************************
// This snippet demonstrates the use of immediate articulations.
// ****************************************************************************

#include "PxImmediateMode.h"
#include "PxMaterial.h"
#include "geometry/PxGeometryQuery.h"
#include "geometry/PxConvexMesh.h"
#include "foundation/PxPhysicsVersion.h"
#include "foundation/PxArray.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxMathUtils.h"
#include "foundation/PxFPU.h"
#include "cooking/PxCooking.h"
#include "cooking/PxConvexMeshDesc.h"
#include "ExtConstraintHelper.h"
#include "extensions/PxMassProperties.h"
#include "extensions/PxDefaultAllocator.h"
#include "extensions/PxDefaultErrorCallback.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetutils/SnippetImmUtils.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#ifdef RENDER_SNIPPET
	#include "../snippetrender/SnippetRender.h"
#endif

//Enables TGS or PGS solver
#define USE_TGS 0

//#define PRINT_TIMINGS
#define TEST_IMMEDIATE_JOINTS

//Enables whether we want persistent state caching (contact cache, friction caching) or not. Avoiding persistency results in one-shot collision detection and zero friction 
//correlation but simplifies code by not longer needing to cache persistent pairs.
#define WITH_PERSISTENCY 1

//Toggles whether we batch constraints or not. Constraint batching is an optional process which can improve performance by grouping together independent constraints. These independent constraints
//can be solved in parallel by using multiple lanes of SIMD registers.
#define BATCH_CONTACTS 1

using namespace physx;
using namespace immediate;
using namespace SnippetImmUtils;

static const PxVec3	gGravity(0.0f, -9.81f, 0.0f);
static const float	gContactDistance			= 0.1f;
static const float	gMeshContactMargin			= 0.01f;
static const float	gToleranceLength			= 1.0f;
static const float	gBounceThreshold			= -2.0f;
static const float	gFrictionOffsetThreshold	= 0.04f;
static const float	gCorrelationDistance		= 0.025f;
static const float	gBoundsInflation			= 0.02f;
static const float	gStaticFriction				= 0.5f;
static const float	gDynamicFriction			= 0.5f;
static const float	gRestitution				= 0.0f;
static const float	gMaxDepenetrationVelocity	= 10.0f;
static const float	gMaxContactImpulse			= FLT_MAX;
static const float	gLinearDamping				= 0.1f;
static const float	gAngularDamping				= 0.05f;
static const float	gMaxLinearVelocity			= 100.0f;
static const float	gMaxAngularVelocity			= 100.0f;
static const float	gJointFrictionCoefficient	= 0.05f;
static const PxU32	gNbIterPos					= 4;
static const PxU32	gNbIterVel					= 1;

static bool			gPause						= false;
static bool			gOneFrame					= false;
static bool			gDrawBounds					= false;
static PxU32		gSceneIndex					= 2;
static float		gTime						= 0.0f;

#if WITH_PERSISTENCY
struct PersistentContactPair
{
	PersistentContactPair()
	{
		reset();
	}

	PxCache	cache;
	PxU8*	frictions;
	PxU32	nbFrictions;

	PX_FORCE_INLINE	void	reset()
	{
		cache = PxCache();
		frictions = NULL;
		nbFrictions = 0;
	}
};
#endif

	struct IDS
	{
		PX_FORCE_INLINE	IDS(PxU32 id0, PxU32 id1) : mID0(id0), mID1(id1)	{}
		PxU32	mID0;
		PxU32	mID1;

		PX_FORCE_INLINE	bool operator == (const IDS& other) const
		{
			return mID0 == other.mID0 && mID1 == other.mID1;
		}
	};

	PX_FORCE_INLINE uint32_t PxComputeHash(const IDS& p)
	{
		return PxComputeHash(uint64_t(p.mID0)|(uint64_t(p.mID1)<<32));
	}

struct MassProps
{
	PxVec3	mInvInertia;
	float	mInvMass;
};

static void computeMassProps(MassProps& props, const PxGeometry& geometry, float mass)
{
	if(mass!=0.0f)
	{
		PxMassProperties inertia(geometry);
		inertia = inertia * (mass/inertia.mass);

		PxQuat orient;
		const PxVec3 diagInertia = PxMassProperties::getMassSpaceInertia(inertia.inertiaTensor, orient);
		props.mInvMass = 1.0f/inertia.mass;
		props.mInvInertia.x = diagInertia.x == 0.0f ? 0.0f : 1.0f/diagInertia.x;
		props.mInvInertia.y = diagInertia.y == 0.0f ? 0.0f : 1.0f/diagInertia.y;
		props.mInvInertia.z = diagInertia.z == 0.0f ? 0.0f : 1.0f/diagInertia.z;
	}
	else
	{
		props.mInvMass = 0.0f;
		props.mInvInertia = PxVec3(0.0f);
	}
}

#ifdef TEST_IMMEDIATE_JOINTS
	struct MyJointData : Ext::JointData
	{
		PxU32		mActors[2];
		PxTransform	mLocalFrames[2];

		void		initInvMassScale()
		{
			invMassScale.linear0	= 1.0f;
			invMassScale.angular0	= 1.0f;
			invMassScale.linear1	= 1.0f;
			invMassScale.angular1	= 1.0f;
		}
	};
#endif

	class ImmediateScene
	{
												PX_NOCOPY(ImmediateScene)
		public:
												ImmediateScene();
												~ImmediateScene();

			void								reset();

			PxU32								createActor(const PxGeometry& geometry, const PxTransform& pose, const MassProps* massProps=NULL, PxArticulationLinkCookie* linkCookie=0);

			void								createGroundPlane()
												{
													createActor(PxPlaneGeometry(), PxTransformFromPlaneEquation(PxPlane(0.0f, 1.0f, 0.0f, 0.0f)));
												}

			void								createScene();
#ifdef TEST_IMMEDIATE_JOINTS
			void								createSphericalJoint(PxU32 id0, PxU32 id1, const PxTransform& localFrame0, const PxTransform& localFrame1, const PxTransform* pose0=NULL, const PxTransform* pose1=NULL);
#endif
			void								updateArticulations(float dt);
			void								updateBounds();
			void								broadPhase();
			void								narrowPhase();
			void								buildSolverBodyData(float dt);
			void								buildSolverConstraintDesc();
			void								createContactConstraints(float dt, float invDt, float lengthScale, PxU32 nbPositionIterations);
			void								solveAndIntegrate(float dt);

			TestCacheAllocator*					mCacheAllocator;
			TestConstraintAllocator*			mConstraintAllocator;

			// PT: TODO: revisit this basic design once everything works
			PxArray<PxGeometryHolder>			mGeoms;
			PxArray<PxTransform>				mPoses;
			PxArray<PxBounds3>					mBounds;

			class ImmediateActor
			{
				public:
					ImmediateActor()	{}
					~ImmediateActor()	{}

					enum Type
					{
						eSTATIC,
						eDYNAMIC,
						eLINK,
					};
					Type						mType;
					PxU32						mCollisionGroup;
					MassProps					mMassProps;
					PxVec3						mLinearVelocity;
					PxVec3						mAngularVelocity;
					// PT: ### TODO: revisit, these two could be a union, the cookie is only needed for a brief time during scene creation
					// Or move them to a completely different / hidden array
					PxArticulationLinkCookie	mLinkCookie;
					PxArticulationLinkHandle	mLink;
			};
			PxArray<ImmediateActor>				mActors;
#if USE_TGS
			PxArray<PxTGSSolverBodyData>		mSolverBodyData;
			PxArray<PxTGSSolverBodyVel>			mSolverBodies;
			PxArray<PxTGSSolverBodyTxInertia>	mSolverBodyTxInertias;
#else
			PxArray<PxSolverBodyData>			mSolverBodyData;
			PxArray<PxSolverBody>				mSolverBodies;
#endif
			PxArray<PxArticulationHandle>		mArticulations;
			PxArray<PxSpatialVector>			mTempZ;
			PxArray<PxSpatialVector>			mTempDeltaV;
#ifdef TEST_IMMEDIATE_JOINTS
			PxArray<MyJointData>				mJointData;
#endif
			PxArray<IDS>						mBroadphasePairs;
			PxHashSet<IDS>						mFilteredPairs;

			struct ContactPair
			{
				PxU32	mID0;
				PxU32	mID1;

				PxU32	mNbContacts;
				PxU32	mStartContactIndex;
			};
			// PT: we use separate arrays here because the immediate mode API expects an array of PxContactPoint
			PxArray<ContactPair>					mContactPairs;
			PxArray<PxContactPoint>					mContactPoints;
#if WITH_PERSISTENCY
			PxHashMap<IDS, PersistentContactPair>	mPersistentPairs;
#endif
			PxArray<PxSolverConstraintDesc>			mSolverConstraintDesc;
#if BATCH_CONTACTS
			PxArray<PxSolverConstraintDesc>			mOrderedSolverConstraintDesc;
#endif
			PxArray<PxConstraintBatchHeader>		mHeaders;
			PxArray<PxReal>							mContactForces;

			PxArray<PxVec3>							mMotionLinearVelocity;	// Persistent to avoid runtime allocations but could be managed on the stack
			PxArray<PxVec3>							mMotionAngularVelocity;	// Persistent to avoid runtime allocations but could be managed on the stack

			PxU32									mNbStaticActors;
			PxU32									mNbArticulationLinks;
			PxU32									mMaxNumArticulationsLinks;

			PX_FORCE_INLINE	void	disableCollision(PxU32 i, PxU32 j)
			{
				if(i>j)
					PxSwap(i, j);
				mFilteredPairs.insert(IDS(i, j));
			}

			PX_FORCE_INLINE	bool	isCollisionDisabled(PxU32 i, PxU32 j)	const
			{
				if(i>j)
					PxSwap(i, j);
				return mFilteredPairs.contains(IDS(i, j));
			}

			PxArticulationLinkCookie				mMotorLinkCookie;
			PxArticulationLinkHandle				mMotorLink;

			PxArticulationHandle					endCreateImmediateArticulation(PxArticulationCookie immArt);

			void									allocateTempBuffer(const PxU32 maxLinks);
	};

ImmediateScene::ImmediateScene() :
	mNbStaticActors				(0),
	mNbArticulationLinks		(0),
	mMaxNumArticulationsLinks	(0),
	mMotorLinkCookie			(PxCreateArticulationLinkCookie()),
	mMotorLink					(PxArticulationLinkHandle())
{
	mCacheAllocator = new TestCacheAllocator;
	mConstraintAllocator = new TestConstraintAllocator;
}

ImmediateScene::~ImmediateScene()
{
	reset();

	PX_DELETE(mConstraintAllocator);
	PX_DELETE(mCacheAllocator);
}

void ImmediateScene::reset()
{
	mGeoms.clear();
	mPoses.clear();
	mBounds.clear();
	mActors.clear();
	mSolverBodyData.clear();
	mSolverBodies.clear();
#if USE_TGS
	mSolverBodyTxInertias.clear();
#endif
	mBroadphasePairs.clear();
	mFilteredPairs.clear();
	mContactPairs.clear();
	mContactPoints.clear();
	mSolverConstraintDesc.clear();
#if BATCH_CONTACTS
	mOrderedSolverConstraintDesc.clear();
#endif
	mHeaders.clear();
	mContactForces.clear();
	mMotionLinearVelocity.clear();
	mMotionAngularVelocity.clear();

	const PxU32 size = mArticulations.size();
	for(PxU32 i=0;i<size;i++)	
		PxReleaseArticulation(mArticulations[i]);

	mArticulations.clear();
#ifdef TEST_IMMEDIATE_JOINTS
	mJointData.clear();
#endif
#if WITH_PERSISTENCY
	mPersistentPairs.clear();
#endif
	mNbStaticActors = mNbArticulationLinks = 0;
	mMotorLinkCookie = PxCreateArticulationLinkCookie();
	mMotorLink = PxArticulationLinkHandle();
	gTime = 0.0f;
}

PxU32 ImmediateScene::createActor(const PxGeometry& geometry, const PxTransform& pose, const MassProps* massProps, PxArticulationLinkCookie* linkCookie)
{
	const PxU32 id = mActors.size();
	// PT: we don't support compounds in this simple snippet. 1 actor = 1 shape/geom. 
	PX_ASSERT(mGeoms.size()==id);
	PX_ASSERT(mPoses.size()==id);
	PX_ASSERT(mBounds.size()==id);

	const bool isStaticActor = !massProps;
	if(isStaticActor)
	{
		PX_ASSERT(!linkCookie);
		mNbStaticActors++;
	}
	else
	{
		// PT: make sure we don't create dynamic actors after static ones. We could reorganize the array but
		// in this simple snippet we just enforce the order in which actors are created.
		PX_ASSERT(!mNbStaticActors);
		if(linkCookie)
			mNbArticulationLinks++;
	}

	ImmediateActor actor;
	if(isStaticActor)
		actor.mType			= ImmediateActor::eSTATIC;
	else if(linkCookie)
		actor.mType			= ImmediateActor::eLINK;
	else
		actor.mType			= ImmediateActor::eDYNAMIC;
	actor.mCollisionGroup	= 0;
	actor.mLinearVelocity	= PxVec3(0.0f);
	actor.mAngularVelocity	= PxVec3(0.0f);
	actor.mLinkCookie		= linkCookie ? *linkCookie : PxCreateArticulationLinkCookie();
	actor.mLink				= PxArticulationLinkHandle();	// Not available yet

	if(massProps)
		actor.mMassProps	= *massProps;
	else	
	{
		actor.mMassProps.mInvMass = 0.0f;
		actor.mMassProps.mInvInertia = PxVec3(0.0f);
	}

	mActors.pushBack(actor);

#if USE_TGS
	mSolverBodyData.pushBack(PxTGSSolverBodyData());
	mSolverBodies.pushBack(PxTGSSolverBodyVel());
	mSolverBodyTxInertias.pushBack(PxTGSSolverBodyTxInertia());
#else
	mSolverBodyData.pushBack(PxSolverBodyData());
	mSolverBodies.pushBack(PxSolverBody());
#endif

	mGeoms.pushBack(geometry);
	mPoses.pushBack(pose);
	mBounds.pushBack(PxBounds3());

	return id;
}

static PxArticulationCookie beginCreateImmediateArticulation(bool fixBase)
{
	PxArticulationDataRC data;
	data.flags = fixBase ? PxArticulationFlag::eFIX_BASE : PxArticulationFlag::Enum(0);
	return PxBeginCreateArticulationRC(data);
}

void ImmediateScene::allocateTempBuffer(const PxU32 maxLinks)
{
	mTempZ.resize(maxLinks);
	mTempDeltaV.resize(maxLinks);
}

PxArticulationHandle ImmediateScene::endCreateImmediateArticulation(PxArticulationCookie immArt)
{
	PxU32 expectedNbLinks = 0;
	const PxU32 nbActors = mActors.size();
	for(PxU32 i=0;i<nbActors;i++)
	{
		if(mActors[i].mLinkCookie.articulation)
			expectedNbLinks++;
	}

	PxArticulationLinkHandle* realLinkHandles = PX_ALLOCATE(PxArticulationLinkHandle, sizeof(PxArticulationLinkHandle) * expectedNbLinks, "PxArticulationLinkHandle");

	PxArticulationHandle immArt2 = PxEndCreateArticulationRC(immArt, realLinkHandles, expectedNbLinks);
	mArticulations.pushBack(immArt2);

	mMaxNumArticulationsLinks = PxMax(mMaxNumArticulationsLinks, expectedNbLinks);

	PxU32 nbLinks = 0;
	for(PxU32 i=0;i<nbActors;i++)
	{
		if(mActors[i].mLinkCookie.articulation)
			mActors[i].mLink = realLinkHandles[nbLinks++];
	}
	PX_ASSERT(expectedNbLinks==nbLinks);

	PX_FREE(realLinkHandles);

	return immArt2;
}

static void setupCommonLinkData(PxArticulationLinkDataRC& data, const PxTransform& pose, const MassProps& massProps)
{
	data.pose								= pose;
	data.inverseMass						= massProps.mInvMass;
	data.inverseInertia						= massProps.mInvInertia;
	data.linearDamping						= gLinearDamping;
	data.angularDamping						= gAngularDamping;
	data.maxLinearVelocitySq				= gMaxLinearVelocity * gMaxLinearVelocity;
	data.maxAngularVelocitySq				= gMaxAngularVelocity * gMaxAngularVelocity;
	data.inboundJoint.frictionCoefficient	= gJointFrictionCoefficient;
}

#ifdef TEST_IMMEDIATE_JOINTS
void ImmediateScene::createSphericalJoint(PxU32 id0, PxU32 id1, const PxTransform& localFrame0, const PxTransform& localFrame1, const PxTransform* pose0, const PxTransform* pose1)
{
	const bool isStatic0 = mActors[id0].mType == ImmediateActor::eSTATIC;
	const bool isStatic1 = mActors[id1].mType == ImmediateActor::eSTATIC;

	MyJointData jointData;
	jointData.mActors[0]		= id0;
	jointData.mActors[1]		= id1;
	jointData.mLocalFrames[0]	= localFrame0;
	jointData.mLocalFrames[1]	= localFrame1;
	if(isStatic0)
		jointData.c2b[0]		= pose0->getInverse().transformInv(localFrame0);
	else
		jointData.c2b[0]		= localFrame0;
	if(isStatic1)
		jointData.c2b[1]		= pose1->getInverse().transformInv(localFrame1);
	else
		jointData.c2b[1]		= localFrame1;

	jointData.initInvMassScale();

	mJointData.pushBack(jointData);
	disableCollision(id0, id1);
}
#endif

void ImmediateScene::createScene()
{
	mMotorLink = PxArticulationLinkHandle();

	const PxU32 index = gSceneIndex;
	if(index==0)
	{
		// Box stack
		{
			const PxVec3 extents(0.5f, 0.5f, 0.5f);
			const PxBoxGeometry boxGeom(extents);

			MassProps massProps;
			computeMassProps(massProps, boxGeom, 1.0f);

	//		for(PxU32 i=0;i<8;i++)
	//			createBox(extents, PxTransform(PxVec3(0.0f, extents.y + float(i)*extents.y*2.0f, 0.0f)), 1.0f);

			PxU32 size = 8;
//			PxU32 size = 2;
//			PxU32 size = 1;
			float y = extents.y;
			float x = 0.0f;
			while(size)
			{
				for(PxU32 i=0;i<size;i++)
					createActor(boxGeom, PxTransform(PxVec3(x+float(i)*extents.x*2.0f, y, 0.0f)), &massProps);

				x += extents.x;
				y += extents.y*2.0f;
				size--;
			}
		}

		createGroundPlane();
	}
	else if(index==1)
	{
		// Simple scene with regular spherical joint

#ifdef TEST_IMMEDIATE_JOINTS
		const float boxSize = 1.0f;
		const PxVec3 extents(boxSize, boxSize, boxSize);
		const PxBoxGeometry boxGeom(extents);

		MassProps massProps;
		computeMassProps(massProps, boxGeom, 1.0f);

		const PxVec3 staticPos(0.0f, 6.0f, 0.0f);
		const PxVec3 dynamicPos = staticPos - extents*2.0f;

		const PxTransform dynPose(dynamicPos);
		const PxTransform staticPose(staticPos);

		const PxU32 dynamicObject = createActor(boxGeom, dynPose, &massProps);
		const PxU32 staticObject = createActor(boxGeom, staticPose);

		createSphericalJoint(staticObject, dynamicObject, PxTransform(-extents), PxTransform(extents), &staticPose, &dynPose);
#endif
	}
	else if(index==2)
	{
		// RC articulation with contacts

		if(1)
		{
			const PxBoxGeometry boxGeom(PxVec3(1.0f));

			MassProps massProps;
			computeMassProps(massProps, boxGeom, 0.5f);

			createActor(boxGeom, PxTransform(PxVec3(0.0f, 1.0f, 0.0f)), &massProps);
		}

		const PxU32 nbLinks = 6;
		const float Altitude = 6.0f;
		const PxTransform basePose(PxVec3(0.f, Altitude, 0.f));
		const PxVec3 boxExtents(0.5f, 0.1f, 0.5f);
		const PxBoxGeometry boxGeom(boxExtents);
		const float s = boxExtents.x*1.1f;

		MassProps massProps;
		computeMassProps(massProps, boxGeom, 1.0f);

		PxArticulationCookie immArt = beginCreateImmediateArticulation(true);

		PxArticulationLinkCookie base;
		PxU32 baseID;
		{
			PxArticulationLinkDataRC linkData;
			setupCommonLinkData(linkData, basePose, massProps);
			base = PxAddArticulationLink(immArt, 0, linkData);
			baseID = createActor(boxGeom, basePose, &massProps, &base);
		}

		PxArticulationLinkCookie parent = base;
		PxU32 parentID = baseID;
		PxTransform linkPose = basePose;
		for(PxU32 i=0;i<nbLinks;i++)
		{
			linkPose.p.z += s*2.0f;

			PxArticulationLinkCookie link;
			PxU32 linkID;
			{
				PxArticulationLinkDataRC linkData;
				setupCommonLinkData(linkData, linkPose, massProps);
				//
				linkData.inboundJoint.type			= PxArticulationJointType::eREVOLUTE;
				linkData.inboundJoint.parentPose	= PxTransform(PxVec3(0.0f, 0.0f, s));
				linkData.inboundJoint.childPose		= PxTransform(PxVec3(0.0f, 0.0f, -s));
				linkData.inboundJoint.motion[PxArticulationAxis::eTWIST] = PxArticulationMotion::eFREE;

				link = PxAddArticulationLink(immArt, &parent, linkData);
				linkID = createActor(boxGeom, linkPose, &massProps, &link);

				disableCollision(parentID, linkID);
			}

			parent = link;
			parentID = linkID;
		}

		endCreateImmediateArticulation(immArt);

		allocateTempBuffer(mMaxNumArticulationsLinks);

		createGroundPlane();
	}
	else if(index==3)
	{
		// RC articulation with limits

		const PxU32 nbLinks = 4;
		const float Altitude = 6.0f;
		const PxTransform basePose(PxVec3(0.f, Altitude, 0.f));
		const PxVec3 boxExtents(0.5f, 0.1f, 0.5f);
		const PxBoxGeometry boxGeom(boxExtents);
		const float s = boxExtents.x*1.1f;

		MassProps massProps;
		computeMassProps(massProps, boxGeom, 1.0f);

		PxArticulationCookie immArt = beginCreateImmediateArticulation(true);

		PxArticulationLinkCookie base;
		PxU32 baseID;
		{
			PxArticulationLinkDataRC linkData;
			setupCommonLinkData(linkData, basePose, massProps);
			base = PxAddArticulationLink(immArt, 0, linkData);
			baseID = createActor(boxGeom, basePose, &massProps, &base);
		}

		PxArticulationLinkCookie parent = base;
		PxU32 parentID = baseID;
		PxTransform linkPose = basePose;
		for(PxU32 i=0;i<nbLinks;i++)
		{
			linkPose.p.z += s*2.0f;

			PxArticulationLinkCookie link;
			PxU32 linkID;
			{
				PxArticulationLinkDataRC linkData;
				setupCommonLinkData(linkData, linkPose, massProps);
				//
				linkData.inboundJoint.type			= PxArticulationJointType::eREVOLUTE;
				linkData.inboundJoint.parentPose	= PxTransform(PxVec3(0.0f, 0.0f, s));
				linkData.inboundJoint.childPose		= PxTransform(PxVec3(0.0f, 0.0f, -s));
				linkData.inboundJoint.motion[PxArticulationAxis::eTWIST] = PxArticulationMotion::eLIMITED;
				linkData.inboundJoint.limits[PxArticulationAxis::eTWIST].low = -PxPi/8.0f;
				linkData.inboundJoint.limits[PxArticulationAxis::eTWIST].high = PxPi/8.0f;

				link = PxAddArticulationLink(immArt, &parent, linkData);
				linkID = createActor(boxGeom, linkPose, &massProps, &link);

				disableCollision(parentID, linkID);
			}

			parent = link;
			parentID = linkID;
		}

		endCreateImmediateArticulation(immArt);

		allocateTempBuffer(mMaxNumArticulationsLinks);
	}
	else if(index==4)
	{
		if(0)
		{
			const float Altitude = 6.0f;
			const PxTransform basePose(PxVec3(0.f, Altitude, 0.f));
			const PxVec3 boxExtents(0.5f, 0.1f, 0.5f);
			const PxBoxGeometry boxGeom(boxExtents);

			MassProps massProps;
			computeMassProps(massProps, boxGeom, 1.0f);

			PxArticulationCookie immArt = beginCreateImmediateArticulation(false);

			PxArticulationLinkCookie base;
			PxU32 baseID;
			{
				PxArticulationLinkDataRC linkData;
				setupCommonLinkData(linkData, basePose, massProps);

				base = PxAddArticulationLink(immArt, 0, linkData);
				baseID = createActor(boxGeom, basePose, &massProps, &base);
				PX_UNUSED(baseID);
			}

			endCreateImmediateArticulation(immArt);
			allocateTempBuffer(mMaxNumArticulationsLinks);
			return;
		}


		// RC articulation with drive

		const PxU32 nbLinks = 1;
		const float Altitude = 6.0f;
		const PxTransform basePose(PxVec3(0.f, Altitude, 0.f));
		const PxVec3 boxExtents(0.5f, 0.1f, 0.5f);
		const PxBoxGeometry boxGeom(boxExtents);
		const float s = boxExtents.x*1.1f;

		MassProps massProps;
		computeMassProps(massProps, boxGeom, 1.0f);

		PxArticulationCookie immArt = beginCreateImmediateArticulation(true);

		PxArticulationLinkCookie base;
		PxU32 baseID;
		{
			PxArticulationLinkDataRC linkData;
			setupCommonLinkData(linkData, basePose, massProps);
			base = PxAddArticulationLink(immArt, 0, linkData);
			baseID = createActor(boxGeom, basePose, &massProps, &base);
		}

		PxArticulationLinkCookie parent = base;
		PxU32 parentID = baseID;
		PxTransform linkPose = basePose;
		for(PxU32 i=0;i<nbLinks;i++)
		{
			linkPose.p.z += s*2.0f;

			PxArticulationLinkCookie link;
			PxU32 linkID;
			{
				PxArticulationLinkDataRC linkData;
				setupCommonLinkData(linkData, linkPose, massProps);
				//
				linkData.inboundJoint.type			= PxArticulationJointType::eREVOLUTE;
				linkData.inboundJoint.parentPose	= PxTransform(PxVec3(0.0f, 0.0f, s));
				linkData.inboundJoint.childPose		= PxTransform(PxVec3(0.0f, 0.0f, -s));
				linkData.inboundJoint.motion[PxArticulationAxis::eTWIST] = PxArticulationMotion::eFREE;
				linkData.inboundJoint.drives[PxArticulationAxis::eTWIST].stiffness = 0.0f;
				linkData.inboundJoint.drives[PxArticulationAxis::eTWIST].damping = 1000.0f;
				linkData.inboundJoint.drives[PxArticulationAxis::eTWIST].maxForce = FLT_MAX;
				linkData.inboundJoint.drives[PxArticulationAxis::eTWIST].driveType = PxArticulationDriveType::eFORCE;
				linkData.inboundJoint.targetVel[PxArticulationAxis::eTWIST] = 4.0f;

				link = PxAddArticulationLink(immArt, &parent, linkData);
				linkID = createActor(boxGeom, linkPose, &massProps, &link);

				disableCollision(parentID, linkID);

				mMotorLinkCookie = link;
				mMotorLink = PxArticulationLinkHandle();
			}

			parent = link;
			parentID = linkID;
		}

		endCreateImmediateArticulation(immArt);
		allocateTempBuffer(mMaxNumArticulationsLinks);
		//### not nice, revisit
		mMotorLink = mActors[1].mLink;
	}
	else if(index==5)
	{
		// Scissor lift
		const PxReal runnerLength = 2.f;
		const PxReal placementDistance = 1.8f;

		const PxReal cosAng = (placementDistance) / (runnerLength);

		const PxReal angle = PxAcos(cosAng);
		const PxReal sinAng = PxSin(angle);

		const PxQuat leftRot(-angle, PxVec3(1.f, 0.f, 0.f));
		const PxQuat rightRot(angle, PxVec3(1.f, 0.f, 0.f));

		PxArticulationCookie immArt = beginCreateImmediateArticulation(false);

		//

		PxArticulationLinkCookie base;
		PxU32 baseID;
		{
			const PxBoxGeometry boxGeom(0.5f, 0.25f, 1.5f);

			MassProps massProps;
			computeMassProps(massProps, boxGeom, 3.0f);

			const PxTransform pose(PxVec3(0.f, 0.25f, 0.f));
			PxArticulationLinkDataRC linkData;
			setupCommonLinkData(linkData, pose, massProps);
			base = PxAddArticulationLink(immArt, 0, linkData);
			baseID = createActor(boxGeom, pose, &massProps, &base);
		}

		//

		PxArticulationLinkCookie leftRoot;
		PxU32 leftRootID;
		{
			const PxBoxGeometry boxGeom(0.5f, 0.05f, 0.05f);

			MassProps massProps;
			computeMassProps(massProps, boxGeom, 1.0f);

			const PxTransform pose(PxVec3(0.f, 0.55f, -0.9f));
			PxArticulationLinkDataRC linkData;
			setupCommonLinkData(linkData, pose, massProps);

				linkData.inboundJoint.type			= PxArticulationJointType::eFIX;
				linkData.inboundJoint.parentPose	= PxTransform(PxVec3(0.f, 0.25f, -0.9f));
				linkData.inboundJoint.childPose		= PxTransform(PxVec3(0.f, -0.05f, 0.f));

			leftRoot = PxAddArticulationLink(immArt, &base, linkData);
			leftRootID = createActor(boxGeom, pose, &massProps, &leftRoot);
			disableCollision(baseID, leftRootID);
		}

		//

		PxArticulationLinkCookie rightRoot;
		PxU32 rightRootID;
		{
			const PxBoxGeometry boxGeom(0.5f, 0.05f, 0.05f);

			MassProps massProps;
			computeMassProps(massProps, boxGeom, 1.0f);

			const PxTransform pose(PxVec3(0.f, 0.55f, 0.9f));
			PxArticulationLinkDataRC linkData;
			setupCommonLinkData(linkData, pose, massProps);

				linkData.inboundJoint.type			= PxArticulationJointType::ePRISMATIC;
				linkData.inboundJoint.parentPose	= PxTransform(PxVec3(0.f, 0.25f, 0.9f));
				linkData.inboundJoint.childPose		= PxTransform(PxVec3(0.f, -0.05f, 0.f));
				linkData.inboundJoint.motion[PxArticulationAxis::eZ] = PxArticulationMotion::eLIMITED;
				linkData.inboundJoint.limits[PxArticulationAxis::eZ].low = -1.4f;
				linkData.inboundJoint.limits[PxArticulationAxis::eZ].high = 0.2f;
				if(0)
				{
					linkData.inboundJoint.drives[PxArticulationAxis::eZ].stiffness = 100000.f;
					linkData.inboundJoint.drives[PxArticulationAxis::eZ].damping = 0.f;
					linkData.inboundJoint.drives[PxArticulationAxis::eZ].maxForce = PX_MAX_F32;
					linkData.inboundJoint.drives[PxArticulationAxis::eZ].driveType = PxArticulationDriveType::eFORCE;
				}

			rightRoot = PxAddArticulationLink(immArt, &base, linkData);
			rightRootID = createActor(boxGeom, pose, &massProps, &rightRoot);
			disableCollision(baseID, rightRootID);
		}

		//

		const PxU32 linkHeight = 3;
		PxU32 currLeftID = leftRootID;
		PxU32 currRightID = rightRootID;
		PxArticulationLinkCookie currLeft = leftRoot;
		PxArticulationLinkCookie currRight = rightRoot;
		PxQuat rightParentRot(PxIdentity);
		PxQuat leftParentRot(PxIdentity);
		for(PxU32 i=0; i<linkHeight; ++i)
		{
			const PxVec3 pos(0.5f, 0.55f + 0.1f*(1 + i), 0.f);

			PxArticulationLinkCookie leftLink;
			PxU32 leftLinkID;
			{
				const PxBoxGeometry boxGeom(0.05f, 0.05f, 1.f);

				MassProps massProps;
				computeMassProps(massProps, boxGeom, 1.0f);

				const PxTransform pose(pos + PxVec3(0.f, sinAng*(2 * i + 1), 0.f), leftRot);
				PxArticulationLinkDataRC linkData;
				setupCommonLinkData(linkData, pose, massProps);

					const PxVec3 leftAnchorLocation = pos + PxVec3(0.f, sinAng*(2 * i), -0.9f);
					linkData.inboundJoint.type			= PxArticulationJointType::eREVOLUTE;
					linkData.inboundJoint.parentPose	= PxTransform(mPoses[currLeftID].transformInv(leftAnchorLocation), leftParentRot);
					linkData.inboundJoint.childPose		= PxTransform(PxVec3(0.f, 0.f, -1.f), rightRot);
					linkData.inboundJoint.motion[PxArticulationAxis::eTWIST] = PxArticulationMotion::eLIMITED;
					linkData.inboundJoint.limits[PxArticulationAxis::eTWIST].low = -PxPi;
					linkData.inboundJoint.limits[PxArticulationAxis::eTWIST].high = angle;

				leftLink = PxAddArticulationLink(immArt, &currLeft, linkData);
				leftLinkID = createActor(boxGeom, pose, &massProps, &leftLink);
				disableCollision(currLeftID, leftLinkID);
				mActors[leftLinkID].mCollisionGroup = 1;
			}
			leftParentRot = leftRot;

			//

			PxArticulationLinkCookie rightLink;
			PxU32 rightLinkID;
			{
				const PxBoxGeometry boxGeom(0.05f, 0.05f, 1.f);

				MassProps massProps;
				computeMassProps(massProps, boxGeom, 1.0f);

				const PxTransform pose(pos + PxVec3(0.f, sinAng*(2 * i + 1), 0.f), rightRot);
				PxArticulationLinkDataRC linkData;
				setupCommonLinkData(linkData, pose, massProps);

					const PxVec3 rightAnchorLocation = pos + PxVec3(0.f, sinAng*(2 * i), 0.9f);
					linkData.inboundJoint.type			= PxArticulationJointType::eREVOLUTE;
					linkData.inboundJoint.parentPose	= PxTransform(mPoses[currRightID].transformInv(rightAnchorLocation), rightParentRot);
					linkData.inboundJoint.childPose		= PxTransform(PxVec3(0.f, 0.f, 1.f), leftRot);
					linkData.inboundJoint.motion[PxArticulationAxis::eTWIST] = PxArticulationMotion::eLIMITED;
					linkData.inboundJoint.limits[PxArticulationAxis::eTWIST].low = -angle;
					linkData.inboundJoint.limits[PxArticulationAxis::eTWIST].high = PxPi;

				rightLink = PxAddArticulationLink(immArt, &currRight, linkData);
				rightLinkID = createActor(boxGeom, pose, &massProps, &rightLink);
				disableCollision(currRightID, rightLinkID);
				mActors[rightLinkID].mCollisionGroup = 1;
			}
			rightParentRot = rightRot;

#ifdef TEST_IMMEDIATE_JOINTS
			createSphericalJoint(leftLinkID, rightLinkID, PxTransform(PxIdentity), PxTransform(PxIdentity));
#else
			disableCollision(leftLinkID, rightLinkID);
#endif
			currLeftID = rightLinkID;
			currRightID = leftLinkID;
			currLeft = rightLink;
			currRight = leftLink;
		}

		//

		PxArticulationLinkCookie leftTop;
		PxU32 leftTopID;
		{
			const PxBoxGeometry boxGeom(0.5f, 0.05f, 0.05f);

			MassProps massProps;
			computeMassProps(massProps, boxGeom, 1.0f);

			const PxTransform pose(mPoses[currLeftID].transform(PxTransform(PxVec3(-0.5f, 0.f, -1.0f), leftParentRot)));
			PxArticulationLinkDataRC linkData;
			setupCommonLinkData(linkData, pose, massProps);

				linkData.inboundJoint.type			= PxArticulationJointType::eREVOLUTE;
				linkData.inboundJoint.parentPose	= PxTransform(PxVec3(0.f, 0.f, -1.f), mPoses[currLeftID].q.getConjugate());
				linkData.inboundJoint.childPose		= PxTransform(PxVec3(0.5f, 0.f, 0.f), pose.q.getConjugate());
				linkData.inboundJoint.motion[PxArticulationAxis::eTWIST] = PxArticulationMotion::eFREE;

			leftTop = PxAddArticulationLink(immArt, &currLeft, linkData);
			leftTopID = createActor(boxGeom, pose, &massProps, &leftTop);
			disableCollision(currLeftID, leftTopID);
			mActors[leftTopID].mCollisionGroup = 1;
		}

		//

		PxArticulationLinkCookie rightTop;
		PxU32 rightTopID;
		{
			// TODO: use a capsule here
//	PxRigidActorExt::createExclusiveShape(*rightTop, PxCapsuleGeometry(0.05f, 0.8f), *gMaterial);
			const PxBoxGeometry boxGeom(0.5f, 0.05f, 0.05f);

			MassProps massProps;
			computeMassProps(massProps, boxGeom, 1.0f);

			const PxTransform pose(mPoses[currRightID].transform(PxTransform(PxVec3(-0.5f, 0.f, 1.0f), rightParentRot)));
			PxArticulationLinkDataRC linkData;
			setupCommonLinkData(linkData, pose, massProps);

				linkData.inboundJoint.type			= PxArticulationJointType::eREVOLUTE;
				linkData.inboundJoint.parentPose	= PxTransform(PxVec3(0.f, 0.f, 1.f), mPoses[currRightID].q.getConjugate());
				linkData.inboundJoint.childPose		= PxTransform(PxVec3(0.5f, 0.f, 0.f), pose.q.getConjugate());
				linkData.inboundJoint.motion[PxArticulationAxis::eTWIST] = PxArticulationMotion::eFREE;

			rightTop = PxAddArticulationLink(immArt, &currRight, linkData);
			rightTopID = createActor(boxGeom, pose, &massProps, &rightTop);
			disableCollision(currRightID, rightTopID);
			mActors[rightTopID].mCollisionGroup = 1;
		}

		//

		currLeftID = leftRootID;
		currRightID = rightRootID;
		currLeft = leftRoot;
		currRight = rightRoot;
		rightParentRot = PxQuat(PxIdentity);
		leftParentRot = PxQuat(PxIdentity);

		for(PxU32 i=0; i<linkHeight; ++i)
		{
			const PxVec3 pos(-0.5f, 0.55f + 0.1f*(1 + i), 0.f);

			PxArticulationLinkCookie leftLink;
			PxU32 leftLinkID;
			{
				const PxBoxGeometry boxGeom(0.05f, 0.05f, 1.f);

				MassProps massProps;
				computeMassProps(massProps, boxGeom, 1.0f);

				const PxTransform pose(pos + PxVec3(0.f, sinAng*(2 * i + 1), 0.f), leftRot);
				PxArticulationLinkDataRC linkData;
				setupCommonLinkData(linkData, pose, massProps);

					const PxVec3 leftAnchorLocation = pos + PxVec3(0.f, sinAng*(2 * i), -0.9f);
					linkData.inboundJoint.type			= PxArticulationJointType::eREVOLUTE;
					linkData.inboundJoint.parentPose	= PxTransform(mPoses[currLeftID].transformInv(leftAnchorLocation), leftParentRot);
					linkData.inboundJoint.childPose		= PxTransform(PxVec3(0.f, 0.f, -1.f), rightRot);
					linkData.inboundJoint.motion[PxArticulationAxis::eTWIST] = PxArticulationMotion::eLIMITED;
					linkData.inboundJoint.limits[PxArticulationAxis::eTWIST].low = -PxPi;
					linkData.inboundJoint.limits[PxArticulationAxis::eTWIST].high = angle;

				leftLink = PxAddArticulationLink(immArt, &currLeft, linkData);
				leftLinkID = createActor(boxGeom, pose, &massProps, &leftLink);
				disableCollision(currLeftID, leftLinkID);
				mActors[leftLinkID].mCollisionGroup = 1;
			}
			leftParentRot = leftRot;

			//

			PxArticulationLinkCookie rightLink;
			PxU32 rightLinkID;
			{
				const PxBoxGeometry boxGeom(0.05f, 0.05f, 1.f);

				MassProps massProps;
				computeMassProps(massProps, boxGeom, 1.0f);

				const PxTransform pose(pos + PxVec3(0.f, sinAng*(2 * i + 1), 0.f), rightRot);
				PxArticulationLinkDataRC linkData;
				setupCommonLinkData(linkData, pose, massProps);

					const PxVec3 rightAnchorLocation = pos + PxVec3(0.f, sinAng*(2 * i), 0.9f);
					linkData.inboundJoint.type			= PxArticulationJointType::eREVOLUTE;
					linkData.inboundJoint.parentPose	= PxTransform(mPoses[currRightID].transformInv(rightAnchorLocation), rightParentRot);
					linkData.inboundJoint.childPose		= PxTransform(PxVec3(0.f, 0.f, 1.f), leftRot);
					linkData.inboundJoint.motion[PxArticulationAxis::eTWIST] = PxArticulationMotion::eLIMITED;
					linkData.inboundJoint.limits[PxArticulationAxis::eTWIST].low = -angle;
					linkData.inboundJoint.limits[PxArticulationAxis::eTWIST].high = PxPi;

				rightLink = PxAddArticulationLink(immArt, &currRight, linkData);
				rightLinkID = createActor(boxGeom, pose, &massProps, &rightLink);
				disableCollision(currRightID, rightLinkID);
				mActors[rightLinkID].mCollisionGroup = 1;
			}
			rightParentRot = rightRot;

#ifdef TEST_IMMEDIATE_JOINTS
			createSphericalJoint(leftLinkID, rightLinkID, PxTransform(PxIdentity), PxTransform(PxIdentity));
#else
			disableCollision(leftLinkID, rightLinkID);
#endif
			currLeftID = rightLinkID;
			currRightID = leftLinkID;
			currLeft = rightLink;
			currRight = leftLink;
		}

		//

#ifdef TEST_IMMEDIATE_JOINTS
		createSphericalJoint(currLeftID, leftTopID, PxTransform(PxVec3(0.f, 0.f, -1.f)), PxTransform(PxVec3(-0.5f, 0.f, 0.f)));
		createSphericalJoint(currRightID, rightTopID, PxTransform(PxVec3(0.f, 0.f, 1.f)), PxTransform(PxVec3(-0.5f, 0.f, 0.f)));
#else
		disableCollision(currLeftID, leftTopID);
		disableCollision(currRightID, rightTopID);
#endif
		//

		// Create top
		{
			PxArticulationLinkCookie top;
			PxU32 topID;
			{
				const PxBoxGeometry boxGeom(0.5f, 0.1f, 1.5f);

				MassProps massProps;
				computeMassProps(massProps, boxGeom, 1.0f);

				const PxTransform pose(PxVec3(0.f, mPoses[leftTopID].p.y + 0.15f, 0.f));
				PxArticulationLinkDataRC linkData;
				setupCommonLinkData(linkData, pose, massProps);

					linkData.inboundJoint.type			= PxArticulationJointType::eFIX;
					linkData.inboundJoint.parentPose	= PxTransform(PxVec3(0.f, 0.0f, 0.f));
					linkData.inboundJoint.childPose		= PxTransform(PxVec3(0.f, -0.15f, -0.9f));

				top = PxAddArticulationLink(immArt, &leftTop, linkData);
				topID = createActor(boxGeom, pose, &massProps, &top);
				disableCollision(leftTopID, topID);
			}
		}

		endCreateImmediateArticulation(immArt);

		allocateTempBuffer(mMaxNumArticulationsLinks);

		createGroundPlane();
	}
	else if(index==6)
	{
		//const float scaleFactor = 0.25f;
		const float scaleFactor = 1.0f;
		const float halfHeight = 1.0f*scaleFactor;
		const float radius = 1.0f*scaleFactor;
		const PxU32 nbCirclePts = 20;
		const PxU32 totalNbVerts = nbCirclePts*2;
		PxVec3 verts[totalNbVerts];
		const float step = 3.14159f*2.0f/float(nbCirclePts);
		for(PxU32 i=0;i<nbCirclePts;i++)
		{
			verts[i].x = sinf(float(i) * step) * radius;
			verts[i].y = cosf(float(i) * step) * radius;
			verts[i].z = 0.0f;
		}

		const PxVec3 offset(0.0f, 0.0f, halfHeight);
		PxVec3* verts2 = verts + nbCirclePts;
		for(PxU32 i=0;i<nbCirclePts;i++)
		{
			const PxVec3 P = verts[i];
			verts[i] = P - offset;
			verts2[i] = P + offset;
		}

		const PxTolerancesScale scale;
		PxCookingParams params(scale);

		PxConvexMeshDesc convexDesc;
		convexDesc.points.count		= totalNbVerts;
		convexDesc.points.stride	= sizeof(PxVec3);
		convexDesc.points.data		= verts;
		convexDesc.flags			= PxConvexFlag::eCOMPUTE_CONVEX;
		PxConvexMesh* convexMesh = PxCreateConvexMesh(params, convexDesc);

		const PxConvexMeshGeometry convexGeom(convexMesh);

		MassProps massProps;
		computeMassProps(massProps, convexGeom, 1.0f);

		const PxQuat rot = PxShortestRotation(PxVec3(0.0f, 0.0f, 1.0f), PxVec3(0.0f, 1.0f, 0.0f));

		PxU32 Nb = 14;
		float altitude = radius;
		float offsetX = 0.0f;
		while(Nb)
		{
			for(PxU32 i=0;i<Nb;i++)
			{
				createActor(convexGeom, PxTransform(PxVec3(offsetX + float(i)*radius*2.2f, altitude, 0.0f), rot), &massProps);
			}
			Nb--;
			altitude += halfHeight*2.0f+0.01f;
			offsetX += radius*1.1f;
		}
		createGroundPlane();
	}
}

void ImmediateScene::updateArticulations(float dt)
{
#if USE_TGS
	const float stepDt = dt/gNbIterPos;
	const float invTotalDt = 1.0f/dt;
	const float stepInvDt = 1.0f/stepDt;
#endif
	
	const PxU32 nbArticulations = mArticulations.size();
	for(PxU32 i=0;i<nbArticulations;i++)
	{
		PxArticulationHandle articulation = mArticulations[i];
#if USE_TGS
		PxComputeUnconstrainedVelocitiesTGS(articulation, gGravity, stepDt, dt, stepInvDt, invTotalDt, 1.0f);
#else
		PxComputeUnconstrainedVelocities(articulation, gGravity, dt, 1.0f);
#endif
	}
}

void ImmediateScene::updateBounds()
{
	PX_SIMD_GUARD

	// PT: in this snippet we simply recompute all bounds each frame (i.e. even static ones)
	const PxU32 nbActors = mActors.size();
	for(PxU32 i=0;i<nbActors;i++)
		PxGeometryQuery::computeGeomBounds(mBounds[i], mGeoms[i].any(), mPoses[i], gBoundsInflation, 1.0f, PxGeometryQueryFlag::Enum(0));
}

void ImmediateScene::broadPhase()
{
	// PT: in this snippet we simply do a brute-force O(n^2) broadphase between all actors
	mBroadphasePairs.clear();
	const PxU32 nbActors = mActors.size();
	for(PxU32 i=0; i<nbActors; i++)
	{
		const ImmediateActor::Type type0 = mActors[i].mType;

		for(PxU32 j=i+1; j<nbActors; j++)
		{
			const ImmediateActor::Type type1 = mActors[j].mType;

			// Filtering
			{
				if(type0==ImmediateActor::eSTATIC && type1==ImmediateActor::eSTATIC)
					continue;

				if(mActors[i].mCollisionGroup==1 && mActors[j].mCollisionGroup==1)
					continue;

				if(isCollisionDisabled(i, j))
					continue;
			}

			if(mBounds[i].intersects(mBounds[j]))
			{
				mBroadphasePairs.pushBack(IDS(i, j));
			}
#if WITH_PERSISTENCY
			else
			{
				//No collision detection performed at all so clear contact cache and friction data
				mPersistentPairs.erase(IDS(i, j));
			}
#endif
		}
	}
}

void ImmediateScene::narrowPhase()
{
	class ContactRecorder : public PxContactRecorder
	{
		public:
						ContactRecorder(ImmediateScene* scene, PxU32 id0, PxU32 id1) : mScene(scene), mID0(id0), mID1(id1), mHasContacts(false)	{}

		virtual	bool	recordContacts(const PxContactPoint* contactPoints, PxU32 nbContacts, PxU32 /*index*/)
		{
			{
				ImmediateScene::ContactPair pair;
				pair.mID0				= mID0;
				pair.mID1				= mID1;
				pair.mNbContacts		= nbContacts;
				pair.mStartContactIndex	= mScene->mContactPoints.size();
				mScene->mContactPairs.pushBack(pair);
				mHasContacts = true;
			}

			for(PxU32 i=0; i<nbContacts; i++)
			{
				// Fill in solver-specific data that our contact gen does not produce...
				PxContactPoint point = contactPoints[i];
				point.maxImpulse		= PX_MAX_F32;
				point.targetVel			= PxVec3(0.0f);
				point.staticFriction	= gStaticFriction;
				point.dynamicFriction	= gDynamicFriction;
				point.restitution		= gRestitution;
				point.materialFlags		= PxMaterialFlag::eIMPROVED_PATCH_FRICTION;
				mScene->mContactPoints.pushBack(point);
			}
			return true;
		}

		ImmediateScene*	mScene;
		PxU32			mID0;
		PxU32			mID1;
		bool			mHasContacts;
	};

	mCacheAllocator->reset();
	mConstraintAllocator->release();
	mContactPairs.resize(0);
	mContactPoints.resize(0);

	const PxU32 nbPairs = mBroadphasePairs.size();
	for(PxU32 i=0;i<nbPairs;i++)
	{
		const IDS& pair = mBroadphasePairs[i];

		const PxTransform& tr0 = mPoses[pair.mID0];
		const PxTransform& tr1 = mPoses[pair.mID1];

		const PxGeometry* pxGeom0 = &mGeoms[pair.mID0].any();
		const PxGeometry* pxGeom1 = &mGeoms[pair.mID1].any();

		ContactRecorder contactRecorder(this, pair.mID0, pair.mID1);

#if WITH_PERSISTENCY
		PersistentContactPair& persistentData = mPersistentPairs[IDS(pair.mID0, pair.mID1)];

		PxGenerateContacts(&pxGeom0, &pxGeom1, &tr0, &tr1, &persistentData.cache, 1, contactRecorder, gContactDistance, gMeshContactMargin, gToleranceLength, *mCacheAllocator);
		if(!contactRecorder.mHasContacts)
		{
			//Contact generation run but no touches found so clear cached friction data
			persistentData.frictions = NULL;
			persistentData.nbFrictions = 0;
		}
#else
		PxCache cache;
		PxGenerateContacts(&pxGeom0, &pxGeom1, &tr0, &tr1, &cache, 1, contactRecorder, gContactDistance, gMeshContactMargin, gToleranceLength, *mCacheAllocator);
#endif
	}

	if(1)
	{
		printf("Narrow-phase: %d contacts         \r", mContactPoints.size());
	}
}

void ImmediateScene::buildSolverBodyData(float dt)
{
	const PxU32 nbActors = mActors.size();
	for(PxU32 i=0;i<nbActors;i++)
	{
		if(mActors[i].mType==ImmediateActor::eSTATIC)
		{
#if USE_TGS
			PxConstructStaticSolverBodyTGS(mPoses[i], mSolverBodies[i], mSolverBodyTxInertias[i], mSolverBodyData[i]);
#else
			PxConstructStaticSolverBody(mPoses[i], mSolverBodyData[i]);
#endif
		}
		else
		{
			PxRigidBodyData data;
			data.linearVelocity				= mActors[i].mLinearVelocity;
			data.angularVelocity			= mActors[i].mAngularVelocity;
			data.invMass					= mActors[i].mMassProps.mInvMass;
			data.invInertia					= mActors[i].mMassProps.mInvInertia;
			data.body2World					= mPoses[i];
			data.maxDepenetrationVelocity	= gMaxDepenetrationVelocity;
			data.maxContactImpulse			= gMaxContactImpulse;
			data.linearDamping				= gLinearDamping;
			data.angularDamping				= gAngularDamping;
			data.maxLinearVelocitySq		= gMaxLinearVelocity*gMaxLinearVelocity;
			data.maxAngularVelocitySq		= gMaxAngularVelocity*gMaxAngularVelocity;
#if USE_TGS
			PxConstructSolverBodiesTGS(&data, &mSolverBodies[i], &mSolverBodyTxInertias[i], &mSolverBodyData[i], 1, gGravity, dt);
#else
			PxConstructSolverBodies(&data, &mSolverBodyData[i], 1, gGravity, dt);
#endif
		}
	}
}

#if USE_TGS
static void setupDesc(PxSolverConstraintDesc& desc, const ImmediateScene::ImmediateActor* actors, PxTGSSolverBodyVel* solverBodies, PxU32 id, bool aorb)
#else
static void setupDesc(PxSolverConstraintDesc& desc, const ImmediateScene::ImmediateActor* actors, PxSolverBody* solverBodies, PxU32 id, bool aorb)
#endif
{
	if(!aorb)
		desc.bodyADataIndex	= id;
	else
		desc.bodyBDataIndex	= id;

	const PxArticulationLinkHandle& link = actors[id].mLink;
	if(link.articulation)
	{
		if(!aorb)
		{
			desc.articulationA	= link.articulation;
			desc.linkIndexA		= link.linkId;
		}
		else
		{
			desc.articulationB	= link.articulation;
			desc.linkIndexB		= link.linkId;
		}
	}
	else
	{
		if(!aorb)
		{
#if USE_TGS
			desc.tgsBodyA		= &solverBodies[id];
#else
			desc.bodyA			= &solverBodies[id];
#endif
			desc.linkIndexA		= PxSolverConstraintDesc::RIGID_BODY;
		}
		else
		{
#if USE_TGS
			desc.tgsBodyB		= &solverBodies[id];
#else
			desc.bodyB			= &solverBodies[id];
#endif
			desc.linkIndexB		= PxSolverConstraintDesc::RIGID_BODY;
		}
	}
}

void ImmediateScene::buildSolverConstraintDesc()
{
	const PxU32 nbContactPairs = mContactPairs.size();
#ifdef TEST_IMMEDIATE_JOINTS
	const PxU32 nbJoints = mJointData.size();
	mSolverConstraintDesc.resize(nbContactPairs+nbJoints);
#else
	mSolverConstraintDesc.resize(nbContactPairs);
#endif

	for(PxU32 i=0; i<nbContactPairs; i++)
	{
		const ContactPair& pair = mContactPairs[i];
		PxSolverConstraintDesc& desc = mSolverConstraintDesc[i];

		setupDesc(desc, mActors.begin(), mSolverBodies.begin(), pair.mID0, false);
		setupDesc(desc, mActors.begin(), mSolverBodies.begin(), pair.mID1, true);

		//Cache pointer to our contact data structure and identify which type of constraint this is. We'll need this later after batching.
		//If we choose not to perform batching and instead just create a single header per-pair, then this would not be necessary because
		//the constraintDescs would not have been reordered
		desc.constraint				= reinterpret_cast<PxU8*>(const_cast<ContactPair*>(&pair));
		desc.constraintLengthOver16	= PxSolverConstraintDesc::eCONTACT_CONSTRAINT;
	}

#ifdef TEST_IMMEDIATE_JOINTS
	for(PxU32 i=0; i<nbJoints; i++)
	{
		const MyJointData& jointData = mJointData[i];
		PxSolverConstraintDesc& desc = mSolverConstraintDesc[nbContactPairs+i];

		const PxU32 id0 = jointData.mActors[0];
		const PxU32 id1 = jointData.mActors[1];

		setupDesc(desc, mActors.begin(), mSolverBodies.begin(), id0, false);
		setupDesc(desc, mActors.begin(), mSolverBodies.begin(), id1, true);

		desc.constraint	= reinterpret_cast<PxU8*>(const_cast<MyJointData*>(&jointData));
		desc.constraintLengthOver16 = PxSolverConstraintDesc::eJOINT_CONSTRAINT;
	}
#endif
}

#ifdef TEST_IMMEDIATE_JOINTS

// PT: this is copied from PxExtensions, it's the solver prep function for spherical joints
//TAG:solverprepshader
static PxU32 SphericalJointSolverPrep(Px1DConstraint* constraints,
	PxVec3p& body0WorldOffset,
	PxU32 /*maxConstraints*/,
	PxConstraintInvMassScale& invMassScale,
	const void* constantBlock,							  
	const PxTransform& bA2w,
	const PxTransform& bB2w,
	bool /*useExtendedLimits*/,
	PxVec3p& cA2wOut, PxVec3p& cB2wOut)
{
	const MyJointData& data = *reinterpret_cast<const MyJointData*>(constantBlock);

	PxTransform32 cA2w, cB2w;
	Ext::joint::ConstraintHelper ch(constraints, invMassScale, cA2w, cB2w, body0WorldOffset, data, bA2w, bB2w);

	Ext::joint::applyNeighborhoodOperator(cA2w, cB2w);

/*	if(data.jointFlags & PxSphericalJointFlag::eLIMIT_ENABLED)
	{
		PxQuat swing, twist;
		PxSeparateSwingTwist(cA2w.q.getConjugate() * cB2w.q, swing, twist);
		PX_ASSERT(PxAbs(swing.x)<1e-6f);

		// PT: TODO: refactor with D6 joint code
		PxVec3 axis;
		PxReal error;
		const PxReal pad = data.limit.isSoft() ? 0.0f : data.limit.contactDistance;
		const Cm::ConeLimitHelperTanLess coneHelper(data.limit.yAngle, data.limit.zAngle, pad);
		const bool active = coneHelper.getLimit(swing, axis, error);				
		if(active)
			ch.angularLimit(cA2w.rotate(axis), error, data.limit);
	}*/

	PxVec3 ra, rb;
	ch.prepareLockedAxes(cA2w.q, cB2w.q, cA2w.transformInv(cB2w.p), 7, 0, ra, rb);
	cA2wOut = ra + bA2w.p;
	cB2wOut = rb + bB2w.p;

	return ch.getCount();
}
#endif

#if USE_TGS
static void setupDesc(PxTGSSolverContactDesc& contactDesc, const ImmediateScene::ImmediateActor* actors, PxTGSSolverBodyTxInertia* txInertias, PxTGSSolverBodyData* solverBodyData, PxTransform* poses, const PxU32 id, const bool aorb)
{
	PxTransform& bodyFrame = aorb ? contactDesc.bodyFrame1 : contactDesc.bodyFrame0;
	PxSolverConstraintPrepDescBase::BodyState& bodyState = aorb ? contactDesc.bodyState1 : contactDesc.bodyState0;
	const PxTGSSolverBodyData*& data = aorb ? contactDesc.bodyData1 : contactDesc.bodyData0;
	const PxTGSSolverBodyTxInertia*& txI = aorb ? contactDesc.body1TxI : contactDesc.body0TxI;

	const PxArticulationLinkHandle& link = actors[id].mLink;
	if(link.articulation)
	{
		PxArticulationLinkDerivedDataRC linkData;
		bool status = PxGetLinkData(link, linkData);
		PX_ASSERT(status);
		PX_UNUSED(status);

		data = NULL;
		txI = NULL;
		bodyFrame = linkData.pose;
		bodyState = PxSolverConstraintPrepDescBase::eARTICULATION;
	}
	else
	{
		data = &solverBodyData[id];
		txI = &txInertias[id];
		bodyFrame = poses[id];
		bodyState = actors[id].mType == ImmediateScene::ImmediateActor::eDYNAMIC ? PxSolverConstraintPrepDescBase::eDYNAMIC_BODY : PxSolverConstraintPrepDescBase::eSTATIC_BODY;
	}
}
#else
static void setupDesc(PxSolverContactDesc& contactDesc, const ImmediateScene::ImmediateActor* actors, PxSolverBodyData* solverBodyData, const PxU32 id, const bool aorb)
{
	PxTransform& bodyFrame = aorb ? contactDesc.bodyFrame1 : contactDesc.bodyFrame0;
	PxSolverConstraintPrepDescBase::BodyState& bodyState = aorb ? contactDesc.bodyState1 : contactDesc.bodyState0;
	const PxSolverBodyData*& data = aorb ? contactDesc.data1 : contactDesc.data0;

	const PxArticulationLinkHandle& link = actors[id].mLink;
	if(link.articulation)
	{
		PxArticulationLinkDerivedDataRC linkData;
		bool status = PxGetLinkData(link, linkData);
		PX_ASSERT(status);
		PX_UNUSED(status);

		data		= NULL;
		bodyFrame	= linkData.pose;
		bodyState	= PxSolverConstraintPrepDescBase::eARTICULATION;
	}
	else
	{
		data		= &solverBodyData[id];
		bodyFrame	= solverBodyData[id].body2World;
		bodyState	= actors[id].mType == ImmediateScene::ImmediateActor::eDYNAMIC ? PxSolverConstraintPrepDescBase::eDYNAMIC_BODY : PxSolverConstraintPrepDescBase::eSTATIC_BODY;
	}
}
#endif

#if USE_TGS
static void setupJointDesc(PxTGSSolverConstraintPrepDesc& jointDesc, const ImmediateScene::ImmediateActor* actors, PxTGSSolverBodyTxInertia* txInertias, PxTGSSolverBodyData* solverBodyData, PxTransform* poses, const PxU32 bodyDataIndex, const bool aorb)
{
	if(!aorb)
	{
		jointDesc.bodyData0 = &solverBodyData[bodyDataIndex];
		jointDesc.body0TxI = &txInertias[bodyDataIndex];
	}
	else
	{
		jointDesc.bodyData1 = &solverBodyData[bodyDataIndex];
		jointDesc.body1TxI = &txInertias[bodyDataIndex];
	}

	PxTransform& bodyFrame = aorb ? jointDesc.bodyFrame1 : jointDesc.bodyFrame0;
	PxSolverConstraintPrepDescBase::BodyState& bodyState = aorb ? jointDesc.bodyState1 : jointDesc.bodyState0;

	if(actors[bodyDataIndex].mLink.articulation)
	{
		PxArticulationLinkDerivedDataRC linkData;
		bool status = PxGetLinkData(actors[bodyDataIndex].mLink, linkData);
		PX_ASSERT(status);
		PX_UNUSED(status);

		bodyFrame = linkData.pose;
		bodyState = PxSolverConstraintPrepDescBase::eARTICULATION;
	}
	else
	{
		//This may seem redundant but the bodyFrame is not defined by the bodyData object when using articulations.
		// PT: TODO: this is a bug in the immediate mode snippet
		if(actors[bodyDataIndex].mType == ImmediateScene::ImmediateActor::eSTATIC)
		{
			bodyFrame = PxTransform(PxIdentity);
			bodyState = PxSolverConstraintPrepDescBase::eSTATIC_BODY;
		}
		else
		{
			bodyFrame = poses[bodyDataIndex];
			bodyState = PxSolverConstraintPrepDescBase::eDYNAMIC_BODY;
		}
	}
}
#else
static void setupJointDesc(PxSolverConstraintPrepDesc& jointDesc, const ImmediateScene::ImmediateActor* actors, PxSolverBodyData* solverBodyData, const PxU32 bodyDataIndex, const bool aorb)
{
	if(!aorb)
		jointDesc.data0	= &solverBodyData[bodyDataIndex];
	else
		jointDesc.data1	= &solverBodyData[bodyDataIndex];

	PxTransform& bodyFrame = aorb ? jointDesc.bodyFrame1 : jointDesc.bodyFrame0;
	PxSolverConstraintPrepDescBase::BodyState& bodyState = aorb ? jointDesc.bodyState1 : jointDesc.bodyState0;

	if(actors[bodyDataIndex].mLink.articulation)
	{
		PxArticulationLinkDerivedDataRC linkData;
		bool status = PxGetLinkData(actors[bodyDataIndex].mLink, linkData);
		PX_ASSERT(status);
		PX_UNUSED(status);

		bodyFrame	= linkData.pose;
		bodyState	= PxSolverConstraintPrepDescBase::eARTICULATION;
	}
	else
	{
		//This may seem redundant but the bodyFrame is not defined by the bodyData object when using articulations.
		// PT: TODO: this is a bug in the immediate mode snippet
		if(actors[bodyDataIndex].mType == ImmediateScene::ImmediateActor::eSTATIC)
		{
			bodyFrame	= PxTransform(PxIdentity);
			bodyState	= PxSolverConstraintPrepDescBase::eSTATIC_BODY;
		}
		else
		{
			bodyFrame	= solverBodyData[bodyDataIndex].body2World;
			bodyState	= PxSolverConstraintPrepDescBase::eDYNAMIC_BODY;
		}
	}
}
#endif

void ImmediateScene::createContactConstraints(float dt, float invDt, float lengthScale, const PxU32 nbPosIterations)
{
	//Only referenced if using TGS solver
	PX_UNUSED(lengthScale);
	PX_UNUSED(nbPosIterations);
#if USE_TGS
	const float stepDt = dt/float(nbPosIterations);
	const float stepInvDt = invDt*float(nbPosIterations);
#endif
#if BATCH_CONTACTS
	mHeaders.resize(mSolverConstraintDesc.size());

	const PxU32 nbBodies = mActors.size() - mNbStaticActors;

	mOrderedSolverConstraintDesc.resize(mSolverConstraintDesc.size());
	PxArray<PxSolverConstraintDesc>& orderedDescs = mOrderedSolverConstraintDesc;

#if USE_TGS
	const PxU32 nbContactHeaders = physx::immediate::PxBatchConstraintsTGS(	mSolverConstraintDesc.begin(), mContactPairs.size(), mSolverBodies.begin(), nbBodies,
																			mHeaders.begin(), orderedDescs.begin(),
																			mArticulations.begin(), mArticulations.size());

	//2 batch the joints...
	const PxU32 nbJointHeaders = physx::immediate::PxBatchConstraintsTGS(	mSolverConstraintDesc.begin() + mContactPairs.size(), mJointData.size(), mSolverBodies.begin(), nbBodies,
																			mHeaders.begin() + nbContactHeaders, orderedDescs.begin() + mContactPairs.size(),
																			mArticulations.begin(), mArticulations.size());
#else
	//1 batch the contacts
	const PxU32 nbContactHeaders = physx::immediate::PxBatchConstraints(mSolverConstraintDesc.begin(), mContactPairs.size(), mSolverBodies.begin(), nbBodies,
																		mHeaders.begin(), orderedDescs.begin(),
																		mArticulations.begin(), mArticulations.size());

	//2 batch the joints...
	const PxU32 nbJointHeaders = physx::immediate::PxBatchConstraints(	mSolverConstraintDesc.begin() + mContactPairs.size(), mJointData.size(), mSolverBodies.begin(), nbBodies,
																		mHeaders.begin() + nbContactHeaders, orderedDescs.begin() + mContactPairs.size(),
																		mArticulations.begin(), mArticulations.size());
#endif

	const PxU32 totalHeaders = nbContactHeaders + nbJointHeaders;

	mHeaders.forceSize_Unsafe(totalHeaders);
#else
	PxArray<PxSolverConstraintDesc>& orderedDescs = mSolverConstraintDesc;

	const PxU32 nbContactHeaders = mContactPairs.size();
	#ifdef TEST_IMMEDIATE_JOINTS
	const PxU32 nbJointHeaders = mJointData.size();
	PX_ASSERT(nbContactHeaders+nbJointHeaders==mSolverConstraintDesc.size());
	mHeaders.resize(nbContactHeaders+nbJointHeaders);
	#else
	PX_ASSERT(nbContactHeaders==mSolverConstraintDesc.size());
	PX_UNUSED(dt);
	mHeaders.resize(nbContactHeaders);
	#endif
	
	// We are bypassing the constraint batching so we create dummy PxConstraintBatchHeaders
	for(PxU32 i=0; i<nbContactHeaders; i++)
	{
		PxConstraintBatchHeader& hdr = mHeaders[i];
		hdr.startIndex = i;
		hdr.stride = 1;
		hdr.constraintType = PxSolverConstraintDesc::eCONTACT_CONSTRAINT;
	}
	#ifdef TEST_IMMEDIATE_JOINTS
	for(PxU32 i=0; i<nbJointHeaders; i++)
	{
		PxConstraintBatchHeader& hdr = mHeaders[nbContactHeaders+i];
		hdr.startIndex = i;
		hdr.stride = 1;
		hdr.constraintType = PxSolverConstraintDesc::eJOINT_CONSTRAINT;
	}
	#endif
#endif

	mContactForces.resize(mContactPoints.size());

	for(PxU32 i=0; i<nbContactHeaders; i++)
	{
		PxConstraintBatchHeader& header = mHeaders[i];
		PX_ASSERT(header.constraintType == PxSolverConstraintDesc::eCONTACT_CONSTRAINT);

#if USE_TGS
		PxTGSSolverContactDesc contactDescs[4];
#else
		PxSolverContactDesc contactDescs[4];
#endif

#if WITH_PERSISTENCY
		PersistentContactPair* persistentPairs[4];
#endif

		for(PxU32 a=0; a<header.stride; a++)
		{
			PxSolverConstraintDesc& constraintDesc = orderedDescs[header.startIndex + a];
			
			//Extract the contact pair that we saved in this structure earlier.
			const ContactPair& pair = *reinterpret_cast<const ContactPair*>(constraintDesc.constraint);
#if USE_TGS
			PxTGSSolverContactDesc& contactDesc = contactDescs[a];
			PxMemZero(&contactDesc, sizeof(contactDesc));

			setupDesc(contactDesc, mActors.begin(), mSolverBodyTxInertias.begin(), mSolverBodyData.begin(), mPoses.begin(), pair.mID0, false);
			setupDesc(contactDesc, mActors.begin(), mSolverBodyTxInertias.begin(), mSolverBodyData.begin(), mPoses.begin(), pair.mID1, true);

			contactDesc.body0					= constraintDesc.tgsBodyA;
			contactDesc.body1					= constraintDesc.tgsBodyB;

			contactDesc.torsionalPatchRadius	= 0.0f;
			contactDesc.minTorsionalPatchRadius	= 0.0f;
#else
			PxSolverContactDesc& contactDesc = contactDescs[a];
			PxMemZero(&contactDesc, sizeof(contactDesc));

			setupDesc(contactDesc, mActors.begin(), mSolverBodyData.begin(), pair.mID0, false);
			setupDesc(contactDesc, mActors.begin(), mSolverBodyData.begin(), pair.mID1, true);

			contactDesc.body0					= constraintDesc.bodyA;
			contactDesc.body1					= constraintDesc.bodyB;
#endif
			contactDesc.contactForces			= &mContactForces[pair.mStartContactIndex];
			contactDesc.contacts				= &mContactPoints[pair.mStartContactIndex];
			contactDesc.numContacts				= pair.mNbContacts;

#if WITH_PERSISTENCY
			const PxHashMap<IDS, PersistentContactPair>::Entry* e = mPersistentPairs.find(IDS(pair.mID0, pair.mID1));
			PX_ASSERT(e);
			{
				PersistentContactPair& pcp = const_cast<PersistentContactPair&>(e->second);
				contactDesc.frictionPtr			= pcp.frictions;
				contactDesc.frictionCount		= PxU8(pcp.nbFrictions);
				persistentPairs[a]				= &pcp;
			}
#else
			contactDesc.frictionPtr				= NULL;
			contactDesc.frictionCount			= 0;
#endif
			contactDesc.maxCCDSeparation		= PX_MAX_F32;

			contactDesc.desc					= &constraintDesc;
			contactDesc.invMassScales.angular0 = contactDesc.invMassScales.angular1 = contactDesc.invMassScales.linear0 = contactDesc.invMassScales.linear1 = 1.0f;
		}

#if USE_TGS
		PxCreateContactConstraintsTGS(&header, 1, contactDescs, *mConstraintAllocator, stepInvDt, invDt, gBounceThreshold, gFrictionOffsetThreshold, gCorrelationDistance);
#else
		PxCreateContactConstraints(&header, 1, contactDescs, *mConstraintAllocator, invDt, gBounceThreshold, gFrictionOffsetThreshold, gCorrelationDistance, mTempZ.begin());
#endif

#if WITH_PERSISTENCY
		//Cache friction information...
		for (PxU32 a = 0; a < header.stride; ++a)
		{
#if USE_TGS
			const PxTGSSolverContactDesc& contactDesc = contactDescs[a];
#else
			const PxSolverContactDesc& contactDesc = contactDescs[a];
#endif
			PersistentContactPair& pcp = *persistentPairs[a];
			pcp.frictions = contactDesc.frictionPtr;
			pcp.nbFrictions = contactDesc.frictionCount;
		}
#endif
	}

#ifdef TEST_IMMEDIATE_JOINTS
	for(PxU32 i=0; i<nbJointHeaders; i++)
	{
		PxConstraintBatchHeader& header = mHeaders[nbContactHeaders+i];
		PX_ASSERT(header.constraintType == PxSolverConstraintDesc::eJOINT_CONSTRAINT);

		{
#if USE_TGS
			PxTGSSolverConstraintPrepDesc jointDescs[4];
#else
			PxSolverConstraintPrepDesc jointDescs[4];
#endif
			PxImmediateConstraint constraints[4];

			header.startIndex += mContactPairs.size();

			for(PxU32 a=0; a<header.stride; a++)
			{
				PxSolverConstraintDesc& constraintDesc = orderedDescs[header.startIndex + a];
				//Extract the contact pair that we saved in this structure earlier.
				const MyJointData& jd = *reinterpret_cast<const MyJointData*>(constraintDesc.constraint);

				constraints[a].prep = SphericalJointSolverPrep;
				constraints[a].constantBlock = &jd;
#if USE_TGS
				PxTGSSolverConstraintPrepDesc& jointDesc = jointDescs[a];
				jointDesc.body0 = constraintDesc.tgsBodyA;
				jointDesc.body1 = constraintDesc.tgsBodyB;
				setupJointDesc(jointDesc, mActors.begin(), mSolverBodyTxInertias.begin(), mSolverBodyData.begin(), mPoses.begin(), constraintDesc.bodyADataIndex, false);
				setupJointDesc(jointDesc, mActors.begin(), mSolverBodyTxInertias.begin(), mSolverBodyData.begin(), mPoses.begin(), constraintDesc.bodyBDataIndex, true);
#else
				PxSolverConstraintPrepDesc& jointDesc = jointDescs[a];
				jointDesc.body0 = constraintDesc.bodyA;
				jointDesc.body1 = constraintDesc.bodyB;
				setupJointDesc(jointDesc, mActors.begin(), mSolverBodyData.begin(), constraintDesc.bodyADataIndex, false);
				setupJointDesc(jointDesc, mActors.begin(), mSolverBodyData.begin(), constraintDesc.bodyBDataIndex, true);
#endif				
				jointDesc.desc					= &constraintDesc;
				jointDesc.writeback				= NULL;
				jointDesc.linBreakForce			= PX_MAX_F32;
				jointDesc.angBreakForce			= PX_MAX_F32;
				jointDesc.minResponseThreshold	= 0;
				jointDesc.disablePreprocessing	= false;
				jointDesc.improvedSlerp			= false;
				jointDesc.driveLimitsAreForces	= false;
				jointDesc.invMassScales.angular0 = jointDesc.invMassScales.angular1 = jointDesc.invMassScales.linear0 = jointDesc.invMassScales.linear1 = 1.0f;
			}

#if USE_TGS
			immediate::PxCreateJointConstraintsWithImmediateShadersTGS(&header, 1, constraints, jointDescs, *mConstraintAllocator, stepDt, dt, stepInvDt, invDt, lengthScale);
#else
			immediate::PxCreateJointConstraintsWithImmediateShaders(&header, 1, constraints, jointDescs, *mConstraintAllocator, dt, invDt, mTempZ.begin());
#endif
		}
	}
#endif
}

void ImmediateScene::solveAndIntegrate(float dt)
{
#ifdef PRINT_TIMINGS
	unsigned long long time0 = __rdtsc();
#endif
	const PxU32 totalNbActors = mActors.size();
	const PxU32 nbDynamicActors = totalNbActors - mNbStaticActors - mNbArticulationLinks;
	const PxU32 nbDynamic = nbDynamicActors + mNbArticulationLinks;

	mMotionLinearVelocity.resize(nbDynamic);
	mMotionAngularVelocity.resize(nbDynamic);

	const PxU32 nbArticulations = mArticulations.size();
	PxArticulationHandle* articulations = mArticulations.begin();

#if USE_TGS
	const float stepDt = dt/float(gNbIterPos);

	immediate::PxSolveConstraintsTGS(mHeaders.begin(), mHeaders.size(),
#if BATCH_CONTACTS
		mOrderedSolverConstraintDesc.begin(),
#else
		mSolverConstraintDesc.begin(),
#endif
		mSolverBodies.begin(), mSolverBodyTxInertias.begin(),
		nbDynamic, gNbIterPos, gNbIterVel, stepDt, 1.0f / stepDt, nbArticulations, articulations,
		mTempZ.begin(), mTempDeltaV.begin());
#else

	PxMemZero(mSolverBodies.begin(), mSolverBodies.size() * sizeof(PxSolverBody));

	PxSolveConstraints(	mHeaders.begin(), mHeaders.size(),
#if BATCH_CONTACTS
						mOrderedSolverConstraintDesc.begin(),
#else
						mSolverConstraintDesc.begin(),
#endif
						mSolverBodies.begin(),
						mMotionLinearVelocity.begin(), mMotionAngularVelocity.begin(), nbDynamic, gNbIterPos, gNbIterVel,
						dt, 1.0f/dt, nbArticulations, articulations,
						mTempZ.begin(), mTempDeltaV.begin());
#endif

#ifdef PRINT_TIMINGS
	unsigned long long time1 = __rdtsc();
#endif

#if USE_TGS
	PxIntegrateSolverBodiesTGS(mSolverBodies.begin(), mSolverBodyTxInertias.begin(), mPoses.begin(), nbDynamicActors, dt);
	for (PxU32 i = 0; i<nbArticulations; i++)
		PxUpdateArticulationBodiesTGS(articulations[i], dt);

	for (PxU32 i = 0; i<nbDynamicActors; i++)
	{
		PX_ASSERT(mActors[i].mType == ImmediateActor::eDYNAMIC);
		const PxTGSSolverBodyVel& data = mSolverBodies[i];
		mActors[i].mLinearVelocity = data.linearVelocity;
		mActors[i].mAngularVelocity = data.angularVelocity;
	}
#else
	PxIntegrateSolverBodies(mSolverBodyData.begin(), mSolverBodies.begin(), mMotionLinearVelocity.begin(), mMotionAngularVelocity.begin(), nbDynamicActors, dt);
	for (PxU32 i = 0; i<nbArticulations; i++)
		PxUpdateArticulationBodies(articulations[i], dt);

	for (PxU32 i = 0; i<nbDynamicActors; i++)
	{
		PX_ASSERT(mActors[i].mType == ImmediateActor::eDYNAMIC);
		const PxSolverBodyData& data = mSolverBodyData[i];
		mActors[i].mLinearVelocity = data.linearVelocity;
		mActors[i].mAngularVelocity = data.angularVelocity;
		mPoses[i] = data.body2World;
	}
#endif

	for(PxU32 i=0;i<mNbArticulationLinks;i++)
	{
		const PxU32 j = nbDynamicActors + i;
		PX_ASSERT(mActors[j].mType==ImmediateActor::eLINK);

		PxArticulationLinkDerivedDataRC data;
		bool status = PxGetLinkData(mActors[j].mLink, data);
		PX_ASSERT(status);
		PX_UNUSED(status);

		mActors[j].mLinearVelocity = data.linearVelocity;
		mActors[j].mAngularVelocity = data.angularVelocity;
		mPoses[j] = data.pose;
	}

#ifdef PRINT_TIMINGS
	unsigned long long time2 = __rdtsc();
	printf("solve: %d           \n", (time1-time0)/1024);
	printf("integrate: %d           \n", (time2-time1)/1024);
#endif
}

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation	= NULL;
static ImmediateScene*			gScene		= NULL;

///////////////////////////////////////////////////////////////////////////////

PxU32 getNbGeoms()
{
	return gScene ? gScene->mGeoms.size() : 0;
}

const PxGeometryHolder* getGeoms()
{
	if(!gScene || !gScene->mGeoms.size())
		return NULL;
	return &gScene->mGeoms[0];
}

const PxTransform* getGeomPoses()
{
	if(!gScene || !gScene->mPoses.size())
		return NULL;
	return &gScene->mPoses[0];
}

PxU32 getNbArticulations()
{
	return gScene ? gScene->mArticulations.size() : 0;
}

PxArticulationHandle* getArticulations()
{
	if(!gScene || !gScene->mArticulations.size())
		return NULL;
	return &gScene->mArticulations[0];
}

PxU32 getNbBounds()
{
	if(!gDrawBounds)
		return 0;
	return gScene ? gScene->mBounds.size() : 0;
}

const PxBounds3* getBounds()
{
	if(!gDrawBounds)
		return NULL;
	if(!gScene || !gScene->mBounds.size())
		return NULL;
	return &gScene->mBounds[0];
}

PxU32 getNbContacts()
{
	return gScene ? gScene->mContactPoints.size() : 0;
}

const PxContactPoint* getContacts()
{
	if(!gScene || !gScene->mContactPoints.size())
		return NULL;
	return &gScene->mContactPoints[0];
}

///////////////////////////////////////////////////////////////////////////////

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gScene = new ImmediateScene;
	gScene->createScene();
}

void stepPhysics(bool /*interactive*/)
{
	if(!gScene)
		return;

	if(gPause && !gOneFrame)
		return;
	gOneFrame = false;

	const float dt = 1.0f/60.0f;
	const float invDt = 60.0f;

	{
#ifdef PRINT_TIMINGS
		unsigned long long time = __rdtsc();
#endif
		gScene->updateArticulations(dt);
#ifdef PRINT_TIMINGS
		time = __rdtsc() - time;
		printf("updateArticulations: %d           \n", time/1024);
#endif
	}
	{
#ifdef PRINT_TIMINGS
		unsigned long long time = __rdtsc();
#endif
		gScene->updateBounds();
#ifdef PRINT_TIMINGS
		time = __rdtsc() - time;
		printf("updateBounds: %d           \n", time/1024);
#endif
	}
	{
#ifdef PRINT_TIMINGS
		unsigned long long time = __rdtsc();
#endif
		gScene->broadPhase();
#ifdef PRINT_TIMINGS
		time = __rdtsc() - time;
		printf("broadPhase: %d           \n", time/1024);
#endif
	}
	{
#ifdef PRINT_TIMINGS
		unsigned long long time = __rdtsc();
#endif
		gScene->narrowPhase();
#ifdef PRINT_TIMINGS
		time = __rdtsc() - time;
		printf("narrowPhase: %d           \n", time/1024);
#endif
	}
	{
#ifdef PRINT_TIMINGS
		unsigned long long time = __rdtsc();
#endif
		gScene->buildSolverBodyData(dt);
#ifdef PRINT_TIMINGS
		time = __rdtsc() - time;
		printf("buildSolverBodyData: %d           \n", time/1024);
#endif
	}
	{
#ifdef PRINT_TIMINGS
		unsigned long long time = __rdtsc();
#endif
		gScene->buildSolverConstraintDesc();
#ifdef PRINT_TIMINGS
		time = __rdtsc() - time;
		printf("buildSolverConstraintDesc: %d           \n", time/1024);
#endif
	}
	{
#ifdef PRINT_TIMINGS
		unsigned long long time = __rdtsc();
#endif
		gScene->createContactConstraints(dt, invDt, 1.f, gNbIterPos);
#ifdef PRINT_TIMINGS
		time = __rdtsc() - time;
		printf("createContactConstraints: %d           \n", time/1024);
#endif
	}
	{
#ifdef PRINT_TIMINGS
//		unsigned long long time = __rdtsc();
#endif
		gScene->solveAndIntegrate(dt);
#ifdef PRINT_TIMINGS
//		time = __rdtsc() - time;
//		printf("solveAndIntegrate: %d           \n", time/1024);
#endif
	}

	if(gScene->mMotorLink.articulation)
	{
		gTime += 0.1f;
		const float target = sinf(gTime) * 4.0f;
//		printf("target: %f\n", target);

		PxArticulationJointDataRC data;
		bool status = PxGetJointData(gScene->mMotorLink, data);
		PX_ASSERT(status);

		data.targetVel[PxArticulationAxis::eTWIST] = target;

		const PxVec3 boxExtents(0.5f, 0.1f, 0.5f);
		const float s = boxExtents.x*1.1f + fabsf(sinf(gTime))*0.5f;

		data.parentPose	= PxTransform(PxVec3(0.0f, 0.0f, s));
		data.childPose	= PxTransform(PxVec3(0.0f, 0.0f, -s));

		status = PxSetJointData(gScene->mMotorLink, data);
		PX_ASSERT(status);
		PX_UNUSED(status);
	}

}

void cleanupPhysics(bool /*interactive*/)
{
	PX_DELETE(gScene);
	PX_RELEASE(gFoundation);

	printf("SnippetImmediateArticulation done.\n");
}

void keyPress(unsigned char key, const PxTransform& /*camera*/)
{
	if(key=='b' || key=='B')
		gDrawBounds = !gDrawBounds;

	if(key=='p' || key=='P')
		gPause = !gPause;

	if(key=='o' || key=='O')
	{
		gPause = true;
		gOneFrame = true;
	}

	if(gScene)
	{
		if(key>=1 && key<=7)
		{
			gSceneIndex = key-1;
			gScene->reset();
			gScene->createScene();
		}

		if(key=='r' || key=='R')
		{
			gScene->reset();
			gScene->createScene();
		}
	}
}

void renderText()
{
#ifdef RENDER_SNIPPET
	Snippets::print("Press F1 to F7 to select a scene.");
#endif
}

int snippetMain(int, const char*const*)
{
	printf("Immediate articulation snippet. Use these keys:\n");
	printf(" P        - enable/disable pause\n");
	printf(" O        - step simulation for one frame\n");
	printf(" R        - reset scene\n");
	printf(" F1 to F6 - select scene\n");
	printf("\n");
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}

