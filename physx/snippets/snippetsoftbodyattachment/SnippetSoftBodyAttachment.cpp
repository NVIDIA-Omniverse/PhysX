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

// ****************************************************************************
// This snippet demonstrates how to tie rigid and softbodies together.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetsoftbody/SnippetSoftBody.h"
#include "../snippetsoftbody/MeshGenerator.h"
#include "extensions/PxTetMakerExt.h"
#include "extensions/PxTetrahedronMeshExt.h"
#include "extensions/PxSoftBodyExt.h"

using namespace physx;
using namespace meshgenerator;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation			= NULL;
static PxPhysics*				gPhysics			= NULL;
static PxCudaContextManager*	gCudaContextManager	= NULL;
static PxDefaultCpuDispatcher*	gDispatcher			= NULL;
static PxScene*					gScene				= NULL;
static PxMaterial*				gMaterial			= NULL;
static PxPvd*					gPvd				= NULL;
std::vector<SoftBody>			gSoftBodies;

static PxFilterFlags softBodyRigidBodyFilter(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	PX_UNUSED(attributes0);
	PX_UNUSED(attributes1);
	PX_UNUSED(constantBlock);
	PX_UNUSED(constantBlockSize);
	if (filterData0.word2 != 0 && filterData0.word2 != filterData1.word2)
		return PxFilterFlag::eKILL;
	pairFlags |= PxPairFlag::eCONTACT_DEFAULT;
	return PxFilterFlag::eDEFAULT;
}

void addSoftBody(PxSoftBody* softBody, const PxFEMParameters& femParams, PxFEMSoftBodyMaterial* femMaterial,
	const PxTransform& transform, const PxReal density, const PxReal scale, const PxU32 iterCount/*, PxMaterial* tetMeshMaterial*/)
{
	PxShape* shape = softBody->getShape();

	PxVec4* simPositionInvMassPinned;
	PxVec4* simVelocityPinned;
	PxVec4* collPositionInvMassPinned;
	PxVec4* restPositionPinned;

	PxSoftBodyExt::allocateAndInitializeHostMirror(*softBody, gCudaContextManager, simPositionInvMassPinned, simVelocityPinned, collPositionInvMassPinned, restPositionPinned);
	
	const PxReal maxInvMassRatio = 50.f;

	softBody->setParameter(femParams);
	shape->setSoftBodyMaterials(&femMaterial, 1);
	softBody->setSolverIterationCounts(iterCount);

	PxSoftBodyExt::transform(*softBody, transform, scale, simPositionInvMassPinned, simVelocityPinned, collPositionInvMassPinned, restPositionPinned);
	PxSoftBodyExt::updateMass(*softBody, density, maxInvMassRatio, simPositionInvMassPinned);
	PxSoftBodyExt::copyToDevice(*softBody, PxSoftBodyDataFlag::eALL, simPositionInvMassPinned, simVelocityPinned, collPositionInvMassPinned, restPositionPinned);

		
	SoftBody sBody(softBody, gCudaContextManager);

	gSoftBodies.push_back(sBody);

	PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, simPositionInvMassPinned);
	PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, simVelocityPinned);
	PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, collPositionInvMassPinned);
	PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, restPositionPinned);
}

static PxSoftBody* createSoftBody(const PxCookingParams& params, const PxArray<PxVec3>& triVerts, const PxArray<PxU32>& triIndices, bool useCollisionMeshForSimulation = false)
{
	PxFEMSoftBodyMaterial* material = PxGetPhysics().createFEMSoftBodyMaterial(1e+6f, 0.45f, 0.5f);
	material->setDamping(0.005f);

	PxSoftBodyMesh* softBodyMesh;

	PxU32 numVoxelsAlongLongestAABBAxis = 8;

	PxSimpleTriangleMesh surfaceMesh;
	surfaceMesh.points.count = triVerts.size();
	surfaceMesh.points.data = triVerts.begin();
	surfaceMesh.triangles.count = triIndices.size() / 3;
	surfaceMesh.triangles.data = triIndices.begin();

	if (useCollisionMeshForSimulation)
	{
		softBodyMesh = PxSoftBodyExt::createSoftBodyMeshNoVoxels(params, surfaceMesh, gPhysics->getPhysicsInsertionCallback());
	}
	else
	{
		softBodyMesh = PxSoftBodyExt::createSoftBodyMesh(params, surfaceMesh, numVoxelsAlongLongestAABBAxis, gPhysics->getPhysicsInsertionCallback());
	}

	//Alternatively one can cook a softbody mesh in a single step
	//tetMesh = cooking.createSoftBodyMesh(simulationMeshDesc, collisionMeshDesc, softbodyDesc, physics.getPhysicsInsertionCallback());
	PX_ASSERT(softBodyMesh);

	if (!gCudaContextManager)
		return NULL;
	PxSoftBody* softBody = gPhysics->createSoftBody(*gCudaContextManager);
	if (softBody)
	{
		PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE;

		PxFEMSoftBodyMaterial* materialPtr = PxGetPhysics().createFEMSoftBodyMaterial(1e+6f, 0.45f, 0.5f);
		PxTetrahedronMeshGeometry geometry(softBodyMesh->getCollisionMesh());
		PxShape* shape = gPhysics->createShape(geometry, &materialPtr, 1, true, shapeFlags);
		if (shape)
		{
			softBody->attachShape(*shape);
			shape->setSimulationFilterData(PxFilterData(0, 0, 2, 0));
		}
		softBody->attachSimulationMesh(*softBodyMesh->getSimulationMesh(), *softBodyMesh->getSoftBodyAuxData());

		gScene->addActor(*softBody);

		PxFEMParameters femParams;
		addSoftBody(softBody, femParams, material, PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(PxIdentity)), 100.f, 1.0f, 30);
		softBody->setSoftBodyFlag(PxSoftBodyFlag::eDISABLE_SELF_COLLISION, true);
	}
	return softBody;
}

static PxRigidDynamic* createRigidCube(PxReal halfExtent, const PxVec3& position)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);

	shape->setSimulationFilterData(PxFilterData(0, 0, 1, 0));

	PxTransform localTm(position);
	PxRigidDynamic* body = gPhysics->createRigidDynamic(localTm);
	body->attachShape(*shape);
	PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
	gScene->addActor(*body);

	shape->release();

	return body;
}

static void connectCubeToSoftBody(PxRigidDynamic* cube, PxReal cubeHalfExtent, const PxVec3& cubePosition, PxSoftBody* softBody, PxU32 pointGridResolution = 10)
{
	float f = 2.0f * cubeHalfExtent / (pointGridResolution - 1);
	for (PxU32 ix = 0; ix < pointGridResolution; ++ix)
	{
		PxReal x = ix * f - cubeHalfExtent;
		for (PxU32 iy = 0; iy < pointGridResolution; ++iy)
		{
			PxReal y = iy * f - cubeHalfExtent;
			for (PxU32 iz = 0; iz < pointGridResolution; ++iz)
			{
				PxReal z = iz * f - cubeHalfExtent;
				PxVec3 p(x, y, z);
				PxVec4 bary;
				PxI32 tet = PxTetrahedronMeshExt::findTetrahedronContainingPoint(softBody->getCollisionMesh(), p + cubePosition, bary);
				if (tet >= 0)
					softBody->addTetRigidAttachment(cube, tet, bary, p);
			}
		}
	}
}

static void createSoftbodies(const PxCookingParams& params)
{
	if (gCudaContextManager == NULL)
	{
		printf("The Softbody feature is currently only supported on GPU\n");
		return;
	}

	PxArray<PxVec3> triVerts;
	PxArray<PxU32> triIndices;
	
	PxReal maxEdgeLength = 1;

	createCube(triVerts, triIndices, PxVec3(0, 9.5, 0), 2.5);
	PxRemeshingExt::limitMaxEdgeLength(triIndices, triVerts, maxEdgeLength);
	PxSoftBody* softBodyCube = createSoftBody(params, triVerts, triIndices, true);

	createSphere(triVerts, triIndices, PxVec3(0,4.5,0), 2.5, maxEdgeLength);
	PxSoftBody* softBodySphere = createSoftBody(params, triVerts, triIndices);

	createConeY(triVerts, triIndices, PxVec3(0, 12.5, 0), 2.0f, 3.5);
	PxRemeshingExt::limitMaxEdgeLength(triIndices, triVerts, maxEdgeLength);
	PxSoftBody* softBodyCone = createSoftBody(params, triVerts, triIndices);

	PxReal halfExtent = 1;
	PxVec3 cubePosA(0, 7.25, 0);
	PxVec3 cubePosB(0, 11.75, 0);
	PxRigidDynamic* rigidCubeA = createRigidCube(halfExtent, cubePosA);
	PxRigidDynamic* rigidCubeB = createRigidCube(halfExtent, cubePosB);
	
	connectCubeToSoftBody(rigidCubeA, 2*halfExtent, cubePosA, softBodySphere);
	connectCubeToSoftBody(rigidCubeA, 2*halfExtent, cubePosA, softBodyCube);

	connectCubeToSoftBody(rigidCubeB, 2*halfExtent, cubePosB, softBodyCube);
	connectCubeToSoftBody(rigidCubeB, 2*halfExtent, cubePosB, softBodyCone);
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);
	
	// initialize cuda
	PxCudaContextManagerDesc cudaContextManagerDesc;
	gCudaContextManager = PxCreateCudaContextManager(*gFoundation, cudaContextManagerDesc, PxGetProfilerCallback());
	if (gCudaContextManager && !gCudaContextManager->contextIsValid())
	{
		PX_RELEASE(gCudaContextManager);
		printf("Failed to initialize cuda context.\n");
	}

	PxTolerancesScale scale;
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, scale, true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);

	PxCookingParams params(scale);
	params.meshWeldTolerance = 0.001f;
	params.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES);
	params.buildTriangleAdjacencies = false;
	params.buildGPUData = true;

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);

	if (!sceneDesc.cudaContextManager)
		sceneDesc.cudaContextManager = gCudaContextManager;
	
	sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
	sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;

	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;

	sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
	sceneDesc.gpuMaxNumPartitions = 8;

	sceneDesc.filterShader = softBodyRigidBodyFilter;
	sceneDesc.solverType = PxSolverType::ePGS;

	gScene = gPhysics->createScene(sceneDesc);
	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);

	createSoftbodies(params);
}

void stepPhysics(bool /*interactive*/)
{
	const PxReal dt = 1.0f / 60.f;

	gScene->simulate(dt);
	gScene->fetchResults(true);

	for (PxU32 i = 0; i < gSoftBodies.size(); i++)
	{
		SoftBody* sb = &gSoftBodies[i]; 
		sb->copyDeformedVerticesFromGPU();		
	}
}
	
void cleanupPhysics(bool /*interactive*/)
{
	for (PxU32 i = 0; i < gSoftBodies.size(); i++)
		gSoftBodies[i].release();
	gSoftBodies.clear();
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if (gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		PX_RELEASE(gPvd);
		PX_RELEASE(transport);
	}
	PxCloseExtensions();
	PX_RELEASE(gCudaContextManager);
	PX_RELEASE(gFoundation);

	printf("SnippetSoftBodyAttachment done.\n");
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
