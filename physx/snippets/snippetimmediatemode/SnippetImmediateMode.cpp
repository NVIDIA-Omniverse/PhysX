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

// ****************************************************************************
// This snippet illustrates simple use of physx
//
// It creates a number of box stacks on a plane, and if rendering, allows the
// user to create new stacks and fire a ball from the camera position
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#include "foundation/PxArray.h"
#include "PxImmediateMode.h"
#include "extensions/PxMassProperties.h"
#include "../snippetcommon/SnippetPrint.h"
#include "extensions/PxRigidActorExt.h"

#define USE_TGS 1

//Enables whether we want persistent state caching (contact cache, friction caching) or not. Avoiding persistency results in one-shot collision detection and zero friction 
//correlation but simplifies code by not longer needing to cache persistent pairs.
#define WITH_PERSISTENCY 1

//Toggles between using low-level inertia computation code or using the RigidBodyExt inertia computation code. The former can operate without the need for PxRigidDynamics being constructed.
#define USE_LOWLEVEL_INERTIA_COMPUTATION 1

//Toggles whether we batch constraints or not. Constraint batching is an optional process which can improve performance by grouping together independent constraints. These independent constraints
//can be solved in parallel by using multiple lanes of SIMD registers.
#define BATCH_CONTACTS 1

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxPhysics*				gPhysics	= NULL;
static PxDefaultCpuDispatcher*	gDispatcher = NULL;
static PxScene*					gScene		= NULL;
static PxMaterial*				gMaterial	= NULL;
static PxPvd*					gPvd = NULL;

physx::PxArray<PxConstraint*>* gConstraints = NULL;

static PxReal stackZ = 10.0f;

//Enable to 1 to use centimeter units instead of meter units.
//Instructive to demonstrate which values used in immediate mode are unit-dependent
#define USE_CM_UNITS 0

#if !USE_CM_UNITS
const PxReal gUnitScale = 1.f;
float cameraSpeed = 1.f;
#else
const PxReal gUnitScale = 100.f;
float cameraSpeed = 100.f;
#endif

#if WITH_PERSISTENCY
struct PersistentContactPair
{
	PxCache cache;
	PxU8* frictions;
	PxU32 nbFrictions;
};

PxArray<PersistentContactPair>* allContactCache;
#endif

class BlockBasedAllocator
{
	struct AllocationPage : PxUserAllocated
	{
		static const PxU32 PageSize = 32 * 1024;
		PxU8 mPage[PageSize];

		PxU32 currentIndex;

		AllocationPage() : currentIndex(0){}

		PxU8* allocate(const PxU32 size)
		{
			PxU32 alignedSize = (size + 15)&(~15);
			if ((currentIndex + alignedSize) < PageSize)
			{
				PxU8* ret = &mPage[currentIndex];
				currentIndex += alignedSize;
				return ret;
			}
			return NULL;
		}
	};

	AllocationPage* currentPage;

	physx::PxArray<AllocationPage*> mAllocatedBlocks;
	PxU32 mCurrentIndex;

public:
	BlockBasedAllocator() : currentPage(NULL), mCurrentIndex(0)
	{
	}

	virtual PxU8* allocate(const PxU32 byteSize)
	{
		if (currentPage)
		{
			PxU8* data = currentPage->allocate(byteSize);
			if (data)
				return data;
		}

		if (mCurrentIndex < mAllocatedBlocks.size())
		{
			currentPage = mAllocatedBlocks[mCurrentIndex++];
			currentPage->currentIndex = 0;
			return currentPage->allocate(byteSize);
		}
		currentPage = PX_NEW(AllocationPage);
		mAllocatedBlocks.pushBack(currentPage);
		mCurrentIndex = mAllocatedBlocks.size();

		return currentPage->allocate(byteSize);
	}

	void release()
	{
		for (PxU32 a = 0; a < mAllocatedBlocks.size(); ++a)
			PX_DELETE(mAllocatedBlocks[a]);
		mAllocatedBlocks.clear();
		currentPage = NULL;
		mCurrentIndex = 0;
	}

	void reset()
	{
		currentPage = NULL;
		mCurrentIndex = 0;
	}

	virtual ~BlockBasedAllocator()
	{
		release();
	}
};

class TestCacheAllocator : public physx::PxCacheAllocator
{
	BlockBasedAllocator mAllocator[2];
	PxU32 currIdx;

public:

	TestCacheAllocator() : currIdx(0)
	{
	}

	virtual PxU8* allocateCacheData(const PxU32 byteSize)
	{
		return mAllocator[currIdx].allocate(byteSize);
	}

	void release() { currIdx = 1 - currIdx; mAllocator[currIdx].release(); }

	void reset() { currIdx = 1 - currIdx; mAllocator[currIdx].reset(); }

	virtual ~TestCacheAllocator(){}
};

class TestConstraintAllocator : public PxConstraintAllocator
{
	BlockBasedAllocator mConstraintAllocator;
	BlockBasedAllocator mFrictionAllocator[2];

	PxU32 currIdx;
public:

	TestConstraintAllocator() : currIdx(0)
	{
	}
	virtual PxU8* reserveConstraintData(const PxU32 byteSize){ return mConstraintAllocator.allocate(byteSize); }
	virtual PxU8* reserveFrictionData(const PxU32 byteSize){ return mFrictionAllocator[currIdx].allocate(byteSize); }

	void release() { currIdx = 1 - currIdx; mConstraintAllocator.release(); mFrictionAllocator[currIdx].release(); }

	virtual ~TestConstraintAllocator() {}
};

static TestCacheAllocator* gCacheAllocator = NULL;
static TestConstraintAllocator* gConstraintAllocator = NULL;

struct ContactPair
{
	PxRigidDynamic* actor0;
	PxRigidActor* actor1;

	PxU32 idx0, idx1;

	PxU32 startContactIndex;
	PxU32 nbContacts;
};

class TestContactRecorder : public immediate::PxContactRecorder
{
	PxArray<ContactPair>& mContactPairs;
	PxArray<Gu::ContactPoint>& mContactPoints;
	PxRigidDynamic& mActor0;
	PxRigidActor& mActor1;
	PxU32 mIdx0, mIdx1;
	bool mHasContacts;
public:
	
	TestContactRecorder(PxArray<ContactPair>& contactPairs, PxArray<Gu::ContactPoint>& contactPoints, PxRigidDynamic& actor0, 
	PxRigidActor& actor1, PxU32 idx0, PxU32 idx1) : mContactPairs(contactPairs), mContactPoints(contactPoints),
		mActor0(actor0), mActor1(actor1), mIdx0(idx0), mIdx1(idx1), mHasContacts(false)
	{
	}

	virtual bool recordContacts(const Gu::ContactPoint* contactPoints, const PxU32 nbContacts, const PxU32 index)
	{
		PX_UNUSED(index);
		{
			ContactPair pair;
			pair.actor0				= &mActor0;
			pair.actor1				= &mActor1;
			pair.nbContacts			= nbContacts;
			pair.startContactIndex	= mContactPoints.size();
			pair.idx0				= mIdx0;
			pair.idx1				= mIdx1;
			mContactPairs.pushBack(pair);
			mHasContacts = true;
		}

		for (PxU32 c = 0; c < nbContacts; ++c)
		{
			//Fill in solver-specific data that our contact gen does not produce...

			Gu::ContactPoint point = contactPoints[c];
			point.maxImpulse		= PX_MAX_F32;
			point.targetVel			= PxVec3(0.f);
			point.staticFriction	= 0.5f;
			point.dynamicFriction	= 0.5f;
			point.restitution		= 0.2f;
			point.materialFlags		= 0;
			mContactPoints.pushBack(point);
		}
		return true;
	}

	PX_FORCE_INLINE bool hasContacts() const	{ return mHasContacts; }
private:
	PX_NOCOPY(TestContactRecorder)
};

static bool generateContacts(	const PxGeometryHolder& geom0, const PxGeometryHolder& geom1, PxRigidDynamic& actor0, PxRigidActor& actor1, PxCacheAllocator& cacheAllocator,
								PxArray<Gu::ContactPoint>& contactPoints, PxArray<ContactPair>& contactPairs, PxU32 idx0, PxU32 idx1, PxCache& cache)
{
	const PxTransform tr0 = actor0.getGlobalPose();
	const PxTransform tr1 = actor1.getGlobalPose();

	TestContactRecorder recorder(contactPairs, contactPoints, actor0, actor1, idx0, idx1);

	const PxGeometry* pxGeom0 = &geom0.any();
	const PxGeometry* pxGeom1 = &geom1.any();

	physx::immediate::PxGenerateContacts(&pxGeom0, &pxGeom1, &tr0, &tr1, &cache, 1, recorder, gUnitScale*0.04f, gUnitScale*0.01f, gUnitScale, cacheAllocator);

	return recorder.hasContacts();
}

static PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(0))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

static void updateInertia(PxRigidBody* body, PxReal density)
{
#if !USE_LOWLEVEL_INERTIA_COMPUTATION
	PX_UNUSED(density);
	//Compute the inertia of the rigid body using the helper function in PxRigidBodyExt
	PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
#else
	//Compute the inertia/mass of the bodies using the more low-level PxMassProperties interface
	//This was written for readability rather than performance. Complexity can be avoided if you know that you are dealing with a single shape body

	PxU32 nbShapes = body->getNbShapes();

	//Keep track of an array of inertia tensors and local poses.
	physx::PxArray<PxMassProperties> inertias;
	physx::PxArray<PxTransform> localPoses;

	for (PxU32 a = 0; a < nbShapes; ++a)
	{
		PxShape* shape;

		body->getShapes(&shape, 1, a);
		//(1) initialize an inertia tensor based on the shape's geometry
		PxMassProperties inertia(shape->getGeometry());
		//(2) Scale the inertia tensor based on density. If you use a single density instead of a density per-shape, this could be performed just prior to
		//extracting the massSpaceInertiaTensor
		inertia = inertia * density;

		inertias.pushBack(inertia);
		localPoses.pushBack(shape->getLocalPose());
	}

	//(3)Sum all the inertia tensors - can be skipped if the shape count is 1
	PxMassProperties inertia = PxMassProperties::sum(inertias.begin(), localPoses.begin(), inertias.size());
	//(4)Get the diagonalized inertia component and frame of the mass space orientation
	PxQuat orient;
	const PxVec3 diagInertia = PxMassProperties::getMassSpaceInertia(inertia.inertiaTensor, orient);
	//(4) Set properties on the rigid body
	body->setMass(inertia.mass);
	body->setCMassLocalPose(PxTransform(inertia.centerOfMass, orient));
	body->setMassSpaceInertiaTensor(diagInertia);
#endif
}

/*static*/ void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	for (PxU32 i = 0; i<size; i++)
	{
		for (PxU32 j = 0; j<size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			
			updateInertia(body, 10.f);

			gScene->addActor(*body);
		}
	}
	shape->release();
}

struct Triangle
{
	PxU32 ind0, ind1, ind2;
};

static PxTriangleMesh* createMeshGround()
{
	const PxU32 gridSize = 8;
	const PxReal gridStep = 512.f / (gridSize-1);

	PxVec3 verts[gridSize * gridSize];

	const PxU32 nbTriangles = 2 * (gridSize - 1) * (gridSize-1);

	Triangle indices[nbTriangles];

	for (PxU32 a = 0; a < gridSize; ++a)
	{
		for (PxU32 b = 0; b < gridSize; ++b)
		{
			verts[a * gridSize + b] = PxVec3(-400.f + b*gridStep, 0.f, -400.f + a*gridStep);
		}
	}

	for (PxU32 a = 0; a < (gridSize-1); ++a)
	{
		for (PxU32 b = 0; b < (gridSize-1); ++b)
		{
			Triangle& tri0 = indices[(a * (gridSize-1) + b) * 2];
			Triangle& tri1 = indices[((a * (gridSize-1) + b) * 2) + 1];

			tri0.ind0 = a * gridSize + b + 1;
			tri0.ind1 = a * gridSize + b;
			tri0.ind2 = (a + 1) * gridSize + b + 1;

			tri1.ind0 = (a + 1) * gridSize + b + 1;
			tri1.ind1 = a * gridSize + b;
			tri1.ind2 = (a + 1) * gridSize + b;
		}
	}

	PxTriangleMeshDesc meshDesc;
	meshDesc.points.data = verts;
	meshDesc.points.count = gridSize * gridSize;
	meshDesc.points.stride = sizeof(PxVec3);
	meshDesc.triangles.count = nbTriangles;
	meshDesc.triangles.data = indices;
	meshDesc.triangles.stride = sizeof(Triangle);

	PxCookingParams cookingParams(gPhysics->getTolerancesScale());
	PxTriangleMesh* triMesh = PxCreateTriangleMesh(cookingParams, meshDesc, gPhysics->getPhysicsInsertionCallback());

	return triMesh;
}

static void updateContactPairs()
{
#if WITH_PERSISTENCY
	allContactCache->clear();

	const PxU32 nbDynamic = gScene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
	const PxU32 nbStatic = gScene->getNbActors(PxActorTypeFlag::eRIGID_STATIC);

	const PxU32 totalPairs = (nbDynamic * (nbDynamic - 1)) / 2 + nbDynamic * nbStatic;

	allContactCache->resize(totalPairs);

	for (PxU32 a = 0; a < totalPairs; ++a)
	{
		(*allContactCache)[a].frictions = NULL;
		(*allContactCache)[a].nbFrictions = 0;
		(*allContactCache)[a].cache.mCachedData = 0;
		(*allContactCache)[a].cache.mCachedSize = 0;
		(*allContactCache)[a].cache.mManifoldFlags = 0;
		(*allContactCache)[a].cache.mPairData = 0;
	}
#endif
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::ePROFILE);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f)*gUnitScale;
	gDispatcher = PxDefaultCpuDispatcherCreate(0);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;

	gScene = gPhysics->createScene(sceneDesc);

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.2f);

	gConstraints = new physx::PxArray<PxConstraint*>();

	const bool useGroundMesh = true;	// Use a triangle mesh or a plane for the ground
	if(useGroundMesh)
	{
		PxTriangleMesh* mesh = createMeshGround();

		PxTriangleMeshGeometry geom(mesh);

		PxRigidStatic* groundMesh = gPhysics->createRigidStatic(PxTransform(PxVec3(0, 2, 0)));
		PxRigidActorExt::createExclusiveShape(*groundMesh, geom, *gMaterial);
		gScene->addActor(*groundMesh);
	}
	else
	{
		PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
		gScene->addActor(*groundPlane);
	}

	for (PxU32 i = 0; i<4; i++)
		createStack(PxTransform(PxVec3(0, 2, stackZ -= 10.0f*gUnitScale)), 20, gUnitScale);

	PxRigidDynamic* ball = createDynamic(PxTransform(PxVec3(0, 20, 100)*gUnitScale), PxSphereGeometry(2 * gUnitScale), PxVec3(0, -25, -100)*gUnitScale);
	PxRigidDynamic* ball2 = createDynamic(PxTransform(PxVec3(0, 24, 100)*gUnitScale), PxSphereGeometry(2 * gUnitScale), PxVec3(0, -25, -100)*gUnitScale);
	PxRigidDynamic* ball3 = createDynamic(PxTransform(PxVec3(0, 27, 100)*gUnitScale), PxSphereGeometry(2 * gUnitScale), PxVec3(0, -25, -100)*gUnitScale);
	updateInertia(ball, 1000.f);
	updateInertia(ball2, 1000.f);

	PxD6Joint* joint = PxD6JointCreate(*gPhysics, ball, PxTransform(PxVec3(0, 4, 0)*gUnitScale), ball2, PxTransform(PxVec3(0, -2, 0)*gUnitScale));
	PxD6Joint* joint2 = PxD6JointCreate(*gPhysics, ball2, PxTransform(PxVec3(0, 4, 0)*gUnitScale), ball3, PxTransform(PxVec3(0, -2, 0)*gUnitScale));
	
	gConstraints->pushBack(joint->getConstraint());
	gConstraints->pushBack(joint2->getConstraint());

	gCacheAllocator = new TestCacheAllocator;
	gConstraintAllocator = new TestConstraintAllocator;
#if WITH_PERSISTENCY
	allContactCache = new PxArray<PersistentContactPair>();
#endif

	updateContactPairs();
}

void stepPhysics(bool /*interactive*/)
{
	gCacheAllocator->reset();
	gConstraintAllocator->release();

	const PxU32 nbStatics = gScene->getNbActors(PxActorTypeFlag::eRIGID_STATIC);
	const PxU32 nbDynamics = gScene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);

	const PxU32 totalActors = nbDynamics + nbStatics;

	PxArray<ContactPair> activeContactPairs;
	PxArray<Gu::ContactPoint> contactPoints;

	activeContactPairs.reserve(4 * totalActors);
	
	PxArray<PxActor*> actors(totalActors);
	PxArray<PxBounds3> shapeBounds(totalActors);
	PxArray<PxGeometryHolder> mGeometries(totalActors);

#if USE_TGS
	PxArray<PxTGSSolverBodyVel> bodies(totalActors);
	PxArray<PxTGSSolverBodyData> bodyData(totalActors);
	PxArray<PxTGSSolverBodyTxInertia> txInertia(totalActors);
	PxArray<PxTransform> globalPoses(totalActors);
#else
	PxArray<PxSolverBody> bodies(totalActors);
	PxArray<PxSolverBodyData> bodyData(totalActors);
#endif

	gScene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, actors.begin(), nbDynamics);

	gScene->getActors(PxActorTypeFlag::eRIGID_STATIC, actors.begin() + nbDynamics, nbStatics);

	//Now do collision detection...Brute force every dynamic against every dynamic/static
	for (PxU32 a = 0; a < totalActors; ++a)
	{
		PxRigidActor* actor = actors[a]->is<PxRigidActor>();

		PxShape* shape;
		actor->getShapes(&shape, 1);

		//Compute the AABBs. We inflate these by 2cm margins
		shapeBounds[a] = PxShapeExt::getWorldBounds(*shape, *actor, 1.f);
		shapeBounds[a].minimum -= PxVec3(0.02f)*gUnitScale;
		shapeBounds[a].maximum += PxVec3(0.02f)*gUnitScale;

		mGeometries[a].storeAny(shape->getGeometry());
	}

	//Broad phase for active pairs...
	for (PxU32 a = 0; a < nbDynamics; ++a)
	{
		PxRigidDynamic* dyn0 = actors[a]->is<PxRigidDynamic>();
		for (PxU32 b = a + 1; b < totalActors; ++b)
		{
			PxRigidActor* actor1 = actors[b]->is<PxRigidActor>();

			if (shapeBounds[a].intersects(shapeBounds[b]))
			{
				ContactPair pair;
				pair.actor0 = dyn0;
				pair.actor1 = actor1;
				pair.idx0 = a;
				pair.idx1 = b;

				activeContactPairs.pushBack(pair);
			}
#if WITH_PERSISTENCY
			else
			{
				const PxU32 startIndex = a == 0 ? 0 : (a * totalActors) - (a * (a + 1)) / 2;

				PersistentContactPair& persistentData = (*allContactCache)[startIndex + (b - a - 1)];

				//No collision detection performed at all so clear contact cache and friction data
				persistentData.frictions = NULL;
				persistentData.nbFrictions = 0;
				persistentData.cache = PxCache();
			}
#endif
		}
	}

	const PxU32 nbActivePairs = activeContactPairs.size();
	ContactPair* activePairs = activeContactPairs.begin();

	activeContactPairs.forceSize_Unsafe(0);

	contactPoints.reserve(4 * nbActivePairs);

	for (PxU32 a = 0; a < nbActivePairs; ++a)
	{
		const ContactPair& pair = activePairs[a];

		PxRigidDynamic* dyn0 = pair.actor0;
		const PxGeometryHolder& holder0 = mGeometries[pair.idx0];

		PxRigidActor* actor1 = pair.actor1;
		const PxGeometryHolder& holder1 = mGeometries[pair.idx1];

#if WITH_PERSISTENCY
		const PxU32 startIndex = pair.idx0 == 0 ? 0 : (pair.idx0 * totalActors) - (pair.idx0 * (pair.idx0 + 1)) / 2;

		PersistentContactPair& persistentData = (*allContactCache)[startIndex + (pair.idx1 - pair.idx0 - 1)];

		if (!generateContacts(holder0, holder1, *dyn0, *actor1, *gCacheAllocator, contactPoints, activeContactPairs, pair.idx0, pair.idx1, persistentData.cache))
		{
			//Contact generation run but no touches found so clear cached friction data
			persistentData.frictions = NULL;
			persistentData.nbFrictions = 0;
		}
#else
		PxCache cache;
		generateContacts(holder0, holder1, *dyn0, *actor1, *gCacheAllocator, contactPoints, activeContactPairs, pair.idx0, pair.idx1, cache);
#endif
	}
	
	const PxReal dt = 1.f / 60.f;
	const PxReal invDt = 60.f;

	const PxU32 nbPositionIterations = 4;
	const PxU32 nbVelocityIterations = 1;

#if USE_TGS
	const PxReal stepDt = dt/PxReal(nbPositionIterations);
	const PxReal invStepDt = invDt * PxReal(nbPositionIterations);
#endif

	const PxVec3 gravity(0.f, -9.8f* gUnitScale, 0.f);

	//Construct solver bodies
	for (PxU32 a = 0; a < nbDynamics; ++a)
	{
		PxRigidDynamic* dyn = actors[a]->is<PxRigidDynamic>();

		immediate::PxRigidBodyData data;
		data.linearVelocity				= dyn->getLinearVelocity();
		data.angularVelocity			= dyn->getAngularVelocity();
		data.invMass					= dyn->getInvMass();
		data.invInertia					= dyn->getMassSpaceInvInertiaTensor();		
		data.body2World					= dyn->getGlobalPose();
		data.maxDepenetrationVelocity	= dyn->getMaxDepenetrationVelocity();
		data.maxContactImpulse			= dyn->getMaxContactImpulse();
		data.linearDamping				= dyn->getLinearDamping();
		data.angularDamping				= dyn->getAngularDamping();
		data.maxLinearVelocitySq		= 100.f*100.f*gUnitScale*gUnitScale;
		data.maxAngularVelocitySq		= 7.f*7.f;

#if USE_TGS
		physx::immediate::PxConstructSolverBodiesTGS(&data, &bodies[a], &txInertia[a], &bodyData[a], 1, gravity, dt);
		globalPoses[a] = data.body2World;
#else
		physx::immediate::PxConstructSolverBodies(&data, &bodyData[a], 1, gravity, dt);
#endif

		size_t szA = size_t(a);
		dyn->userData = reinterpret_cast<void*>(szA);
	}

	//Construct static bodies
	for (PxU32 a = nbDynamics; a < totalActors; ++a)
	{
		PxRigidStatic* sta = actors[a]->is<PxRigidStatic>();
#if USE_TGS
		physx::immediate::PxConstructStaticSolverBodyTGS(sta->getGlobalPose(), bodies[a], txInertia[a], bodyData[a]);
		globalPoses[a] = sta->getGlobalPose();
#else
		physx::immediate::PxConstructStaticSolverBody(sta->getGlobalPose(), bodyData[a]);
#endif
		size_t szA = a;
		sta->userData = reinterpret_cast<void*>(szA);
	}

	PxArray<PxSolverConstraintDesc> descs(activeContactPairs.size() + gConstraints->size());

	for (PxU32 a = 0; a < activeContactPairs.size(); ++a)
	{
		PxSolverConstraintDesc& desc = descs[a];
		ContactPair& pair = activeContactPairs[a];

		//Set body pointers and bodyData idxs
#if USE_TGS
		desc.tgsBodyA = &bodies[pair.idx0];
		desc.tgsBodyB = &bodies[pair.idx1];
#else
		desc.bodyA = &bodies[pair.idx0];
		desc.bodyB = &bodies[pair.idx1];
#endif

		desc.bodyADataIndex = PxU16(pair.idx0);
		desc.bodyBDataIndex = PxU16(pair.idx1);
		desc.linkIndexA = PxSolverConstraintDesc::RIGID_BODY;
		desc.linkIndexB = PxSolverConstraintDesc::RIGID_BODY;

		//Cache pointer to our contact data structure and identify which type of constraint this is. We'll need this later after batching.
		//If we choose not to perform batching and instead just create a single header per-pair, then this would not be necessary because
		//the constraintDescs would not have been reordered
		desc.constraint = reinterpret_cast<PxU8*>(&pair);
		desc.constraintLengthOver16 = PxSolverConstraintDesc::eCONTACT_CONSTRAINT;
	}

	for (PxU32 a = 0; a < gConstraints->size(); ++a)
	{
		PxConstraint* constraint = (*gConstraints)[a];

		PxSolverConstraintDesc& desc = descs[activeContactPairs.size() + a];

		PxRigidActor* actor0, *actor1;
		constraint->getActors(actor0, actor1);

		PxU32 id0 = PxU32(size_t(actor0->userData));
		PxU32 id1 = PxU32(size_t(actor1->userData));

#if USE_TGS
		desc.tgsBodyA = &bodies[id0];
		desc.tgsBodyB = &bodies[id1];
#else
		desc.bodyA = &bodies[id0];
		desc.bodyB = &bodies[id1];
#endif

		desc.bodyADataIndex = PxU16(id0);
		desc.bodyBDataIndex = PxU16(id1);
		desc.linkIndexA = PxSolverConstraintDesc::RIGID_BODY;
		desc.linkIndexB = PxSolverConstraintDesc::RIGID_BODY;

		desc.constraint = reinterpret_cast<PxU8*>(constraint);
		desc.constraintLengthOver16 = PxSolverConstraintDesc::eJOINT_CONSTRAINT;
	}

	PxArray<PxConstraintBatchHeader> headers(descs.size());
	PxArray<PxReal> contactForces(contactPoints.size());

	//Technically, you can batch the contacts and joints all at once using a single call but doing so mixes them in the orderedDescs array, which means that it is impossible to 
	//batch all contact or all joint dispatches into a single call. While we don't do this in this snippet (we instead process a single header at a time), our approach could be extended to
	//dispatch all contact headers at once if that was necessary.

#if BATCH_CONTACTS
	PxArray<PxSolverConstraintDesc> tempOrderedDescs(descs.size());
	physx::PxArray<PxSolverConstraintDesc>& orderedDescs = tempOrderedDescs;
#if USE_TGS
	//1 batch the contacts
	const PxU32 nbContactHeaders = physx::immediate::PxBatchConstraintsTGS(descs.begin(), activeContactPairs.size(), bodies.begin(), nbDynamics, headers.begin(), orderedDescs.begin());

	//2 batch the joints...
	const PxU32 nbJointHeaders = physx::immediate::PxBatchConstraintsTGS(descs.begin() + activeContactPairs.size(), gConstraints->size(), bodies.begin(), nbDynamics, headers.begin() + nbContactHeaders, orderedDescs.begin() + activeContactPairs.size());
#else
	//1 batch the contacts
	const PxU32 nbContactHeaders = physx::immediate::PxBatchConstraints(descs.begin(), activeContactPairs.size(), bodies.begin(), nbDynamics, headers.begin(), orderedDescs.begin());

	//2 batch the joints...
	const PxU32 nbJointHeaders = physx::immediate::PxBatchConstraints(descs.begin() + activeContactPairs.size(), gConstraints->size(), bodies.begin(), nbDynamics, headers.begin() + nbContactHeaders, orderedDescs.begin() + activeContactPairs.size());
#endif
#else	
	physx::PxArray<PxSolverConstraintDesc>& orderedDescs = descs;
	
	//We are bypassing the constraint batching so we create dummy PxConstraintBatchHeaders
	const PxU32 nbContactHeaders = activeContactPairs.size();
	const PxU32 nbJointHeaders = gConstraints->size();

	for (PxU32 i = 0; i < nbContactHeaders; ++i)
	{
		PxConstraintBatchHeader& hdr = headers[i];
		hdr.startIndex = i;
		hdr.stride = 1;
		hdr.constraintType = PxSolverConstraintDesc::eCONTACT_CONSTRAINT;
	}

	for (PxU32 i = 0; i < nbJointHeaders; ++i)
	{
		PxConstraintBatchHeader& hdr = headers[nbContactHeaders+i];
		hdr.startIndex = i;
		hdr.stride = 1;
		hdr.constraintType = PxSolverConstraintDesc::eJOINT_CONSTRAINT;
	}
#endif

	const PxU32 totalHeaders = nbContactHeaders + nbJointHeaders;

	headers.forceSize_Unsafe(totalHeaders);

	//1 - Create all the contact constraints. We do this by looping over all the headers and, for each header, constructing the PxSolverContactDesc objects, then creating that contact constraint.
	//We could alternatively create all the PxSolverContactDesc objects in a single pass, then create batch create that constraint
	for (PxU32 i = 0; i < nbContactHeaders; ++i)
	{
		PxConstraintBatchHeader& header = headers[i];

		PX_ASSERT(header.constraintType == PxSolverConstraintDesc::eCONTACT_CONSTRAINT);
#if USE_TGS
		PxTGSSolverContactDesc contactDescs[4];
#else
		PxSolverContactDesc contactDescs[4];
#endif

#if WITH_PERSISTENCY
		const ContactPair* pairs[4];
#endif
		for (PxU32 a = 0; a < header.stride; ++a)
		{
			PxSolverConstraintDesc& constraintDesc = orderedDescs[header.startIndex + a];
#if USE_TGS
			PxTGSSolverContactDesc& contactDesc = contactDescs[a];
#else
			PxSolverContactDesc& contactDesc = contactDescs[a];
#endif
			PxMemZero(&contactDesc, sizeof(contactDesc));
			//Extract the contact pair that we saved in this structure earlier.
			const ContactPair& pair = *reinterpret_cast<const ContactPair*>(constraintDesc.constraint);

#if WITH_PERSISTENCY
			pairs[a] = &pair;
#endif
#if USE_TGS
			contactDesc.body0 = constraintDesc.tgsBodyA;
			contactDesc.body1 = constraintDesc.tgsBodyB;
			contactDesc.bodyData0 = &bodyData[constraintDesc.bodyADataIndex];
			contactDesc.bodyData1 = &bodyData[constraintDesc.bodyBDataIndex];
			contactDesc.body0TxI = &txInertia[constraintDesc.bodyADataIndex];
			contactDesc.body1TxI = &txInertia[constraintDesc.bodyBDataIndex];

			//This may seem redundant but the bodyFrame is not defined by the bodyData object when using articulations. This
			//example does not use articulations.
			contactDesc.bodyFrame0 = globalPoses[constraintDesc.bodyADataIndex];
			contactDesc.bodyFrame1 = globalPoses[constraintDesc.bodyBDataIndex];
#else
			contactDesc.body0 = constraintDesc.bodyA;
			contactDesc.body1 = constraintDesc.bodyB;
			contactDesc.data0 = &bodyData[constraintDesc.bodyADataIndex];
			contactDesc.data1 = &bodyData[constraintDesc.bodyBDataIndex];

			//This may seem redundant but the bodyFrame is not defined by the bodyData object when using articulations. This
			//example does not use articulations.
			contactDesc.bodyFrame0 = contactDesc.data0->body2World;
			contactDesc.bodyFrame1 = contactDesc.data1->body2World;
#endif
			contactDesc.contactForces = &contactForces[pair.startContactIndex];
			contactDesc.contacts = &contactPoints[pair.startContactIndex];
			contactDesc.numContacts = pair.nbContacts;

#if WITH_PERSISTENCY
			const PxU32 startIndex = pair.idx0 == 0 ? 0 : (pair.idx0 * totalActors) - (pair.idx0 * (pair.idx0 + 1)) / 2;
			contactDesc.frictionPtr = (*allContactCache)[startIndex + (pair.idx1 - pair.idx0 - 1)].frictions;
			contactDesc.frictionCount = PxU8((*allContactCache)[startIndex + (pair.idx1 - pair.idx0 - 1)].nbFrictions);
#else
			contactDesc.frictionPtr = NULL;
			contactDesc.frictionCount = 0;
#endif
			contactDesc.shapeInteraction = NULL;
			contactDesc.maxCCDSeparation = PX_MAX_F32;
			
			contactDesc.bodyState0 = PxSolverConstraintPrepDescBase::eDYNAMIC_BODY;
			contactDesc.bodyState1 = pair.actor1->is<PxRigidDynamic>() ? PxSolverConstraintPrepDescBase::eDYNAMIC_BODY : PxSolverConstraintPrepDescBase::eSTATIC_BODY;
			contactDesc.desc = &constraintDesc;
			contactDesc.invMassScales.angular0 = contactDesc.invMassScales.angular1 = contactDesc.invMassScales.linear0 = contactDesc.invMassScales.linear1 = 1.f;
		}

#if USE_TGS
		immediate::PxCreateContactConstraintsTGS(&header, 1, contactDescs, *gConstraintAllocator, invStepDt, invDt, -2.f * gUnitScale, 0.04f * gUnitScale, 0.025f * gUnitScale);
#else
		immediate::PxCreateContactConstraints(&header, 1, contactDescs, *gConstraintAllocator, invDt, -2.f * gUnitScale, 0.04f * gUnitScale, 0.025f * gUnitScale);
#endif

#if WITH_PERSISTENCY
		for (PxU32 a = 0; a < header.stride; ++a)
		{
			//Cache friction information...
#if USE_TGS
			PxTGSSolverContactDesc& contactDesc = contactDescs[a];
#else
			PxSolverContactDesc& contactDesc = contactDescs[a];
#endif
			//PxSolverConstraintDesc& constraintDesc = orderedDescs[header.startIndex + a];
			const ContactPair& pair = *pairs[a];

			const PxU32 startIndex = pair.idx0 == 0 ? 0 : (pair.idx0 * totalActors) - (pair.idx0 * (pair.idx0 + 1)) / 2;

			(*allContactCache)[startIndex + (pair.idx1 - pair.idx0 - 1)].frictions = contactDesc.frictionPtr;
			(*allContactCache)[startIndex + (pair.idx1 - pair.idx0 - 1)].nbFrictions = contactDesc.frictionCount;
		}
#endif
	}

	for (PxU32 i = nbContactHeaders; i < totalHeaders; ++i)
	{
		PxConstraintBatchHeader& header = headers[i];

		PX_ASSERT(header.constraintType == PxSolverConstraintDesc::eJOINT_CONSTRAINT);

		{
#if USE_TGS
			PxTGSSolverConstraintPrepDesc jointDescs[4];
#else
			PxSolverConstraintPrepDesc jointDescs[4];
#endif

			PxConstraint* constraints[4];

			header.startIndex += activeContactPairs.size();

			for (PxU32 a = 0; a < header.stride; ++a)
			{
				PxSolverConstraintDesc& constraintDesc = orderedDescs[header.startIndex + a];
				//Extract the contact pair that we saved in this structure earlier.
				PxConstraint& constraint = *reinterpret_cast<PxConstraint*>(constraintDesc.constraint);

				constraints[a] = &constraint;

#if USE_TGS
				PxTGSSolverConstraintPrepDesc& jointDesc = jointDescs[a];
				jointDesc.body0 = constraintDesc.tgsBodyA;
				jointDesc.body1 = constraintDesc.tgsBodyB;
				jointDesc.bodyData0 = &bodyData[constraintDesc.bodyADataIndex];
				jointDesc.bodyData1 = &bodyData[constraintDesc.bodyBDataIndex];
				jointDesc.body0TxI = &txInertia[constraintDesc.bodyADataIndex];
				jointDesc.body1TxI = &txInertia[constraintDesc.bodyBDataIndex];

				//This may seem redundant but the bodyFrame is not defined by the bodyData object when using articulations. This
				//example does not use articulations.
				jointDesc.bodyFrame0 = globalPoses[constraintDesc.bodyADataIndex];
				jointDesc.bodyFrame1 = globalPoses[constraintDesc.bodyBDataIndex];
#else
				PxSolverConstraintPrepDesc& jointDesc = jointDescs[a];
				jointDesc.body0 = constraintDesc.bodyA;
				jointDesc.body1 = constraintDesc.bodyB;
				jointDesc.data0 = &bodyData[constraintDesc.bodyADataIndex];
				jointDesc.data1 = &bodyData[constraintDesc.bodyBDataIndex];

				//This may seem redundant but the bodyFrame is not defined by the bodyData object when using articulations. This
				//example does not use articulations.
				jointDesc.bodyFrame0 = jointDesc.data0->body2World;
				jointDesc.bodyFrame1 = jointDesc.data1->body2World;
#endif

				PxRigidActor* actor0, *actor1;

				constraint.getActors(actor0, actor1);

				jointDesc.bodyState0 = PxSolverConstraintPrepDescBase::eDYNAMIC_BODY;
				jointDesc.bodyState1 = actor1 == NULL ? PxSolverConstraintPrepDescBase::eSTATIC_BODY : actor1->is<PxRigidDynamic>() ? PxSolverConstraintPrepDescBase::eDYNAMIC_BODY : PxSolverConstraintPrepDescBase::eSTATIC_BODY;
				jointDesc.desc = &constraintDesc;
				jointDesc.invMassScales.angular0 = jointDesc.invMassScales.angular1 = jointDesc.invMassScales.linear0 = jointDesc.invMassScales.linear1 = 1.f;
				jointDesc.writeback = NULL;
				constraint.getBreakForce(jointDesc.linBreakForce, jointDesc.angBreakForce);
				jointDesc.minResponseThreshold = constraint.getMinResponseThreshold();
				jointDesc.disablePreprocessing = !!(constraint.getFlags() & PxConstraintFlag::eDISABLE_PREPROCESSING);				
				jointDesc.improvedSlerp = !!(constraint.getFlags() & PxConstraintFlag::eIMPROVED_SLERP);
				jointDesc.driveLimitsAreForces = !!(constraint.getFlags() & PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES);
			}

#if USE_TGS
			immediate::PxCreateJointConstraintsWithShadersTGS(&header, 1, constraints, jointDescs, *gConstraintAllocator, stepDt, dt, invStepDt, invDt, 1.f);
#else
			immediate::PxCreateJointConstraintsWithShaders(&header, 1, constraints, jointDescs, *gConstraintAllocator, dt, invDt);
#endif
		}
	}

#if USE_TGS
	immediate::PxSolveConstraintsTGS(headers.begin(), headers.size(), orderedDescs.begin(), bodies.begin(), txInertia.begin(), nbDynamics, nbPositionIterations, nbVelocityIterations, stepDt, invStepDt);

	immediate::PxIntegrateSolverBodiesTGS(bodies.begin(), txInertia.begin(), globalPoses.begin(), nbDynamics, dt);

	for (PxU32 a = 0; a < nbDynamics; ++a)
	{
		PxRigidDynamic* dynamic = actors[a]->is<PxRigidDynamic>();

		const PxTGSSolverBodyVel& body = bodies[a];

		dynamic->setLinearVelocity(body.linearVelocity);
		dynamic->setAngularVelocity(body.angularVelocity);
		dynamic->setGlobalPose(globalPoses[a]);
	}
#else
	//Solve all the constraints produced earlier. Intermediate motion linear/angular velocity buffers are filled in. These contain intermediate delta velocity information that is used
	//the PxIntegrateSolverBody
	PxArray<PxVec3> motionLinearVelocity(nbDynamics);
	PxArray<PxVec3> motionAngularVelocity(nbDynamics);

	//Zero the bodies array. This buffer contains the delta velocities and are accumulated during the simulation. For correct behavior, it is vital
	//that this buffer is zeroed.
	PxMemZero(bodies.begin(), bodies.size() * sizeof(PxSolverBody));

	immediate::PxSolveConstraints(headers.begin(), headers.size(), orderedDescs.begin(), bodies.begin(), motionLinearVelocity.begin(), motionAngularVelocity.begin(), nbDynamics, nbPositionIterations, nbVelocityIterations);

	immediate::PxIntegrateSolverBodies(bodyData.begin(), bodies.begin(), motionLinearVelocity.begin(), motionAngularVelocity.begin(), nbDynamics, dt);

	for (PxU32 a = 0; a < nbDynamics; ++a)
	{
		PxRigidDynamic* dynamic = actors[a]->is<PxRigidDynamic>();

		const PxSolverBodyData& data = bodyData[a];

		dynamic->setLinearVelocity(data.linearVelocity);
		dynamic->setAngularVelocity(data.angularVelocity);
		dynamic->setGlobalPose(data.body2World);
	}
#endif
}
	
void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);

	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		PX_RELEASE(transport);
	}

	delete gCacheAllocator;
	delete gConstraintAllocator;

#if WITH_PERSISTENCY
	delete allContactCache;
#endif

	PX_RELEASE(gFoundation);

	printf("SnippetImmediateMode done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	switch(toupper(key))
	{
		case ' ':	createDynamic(camera, PxSphereGeometry(3.0f*gUnitScale), camera.rotate(PxVec3(0, 0, -1)) * 200*gUnitScale);	updateContactPairs();  break;
	}
}

int snippetMain(int, const char*const*)
{
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
