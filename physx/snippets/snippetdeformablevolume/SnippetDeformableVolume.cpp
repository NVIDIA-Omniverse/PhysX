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
// This snippet demonstrates how to setup deformable volumes.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetdeformablevolume/SnippetDeformableVolume.h"
#include "../snippetdeformablevolume/MeshGenerator.h"
#include "extensions/PxTetMakerExt.h"
#include "extensions/PxDeformableVolumeExt.h"

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
PxArray<DeformableVolume>		gDeformableVolumes;

void addDeformableVolume(PxDeformableVolume* deformableVolume, const PxTransform& transform, const PxReal density, const PxReal scale)
{
	PxVec4* simPositionInvMassPinned;
	PxVec4* simVelocityPinned;
	PxVec4* collPositionInvMassPinned;
	PxVec4* restPositionPinned;

	PxDeformableVolumeExt::allocateAndInitializeHostMirror(*deformableVolume, gCudaContextManager, simPositionInvMassPinned, simVelocityPinned, collPositionInvMassPinned, restPositionPinned);
	
	const PxReal maxInvMassRatio = 50.f;

	PxDeformableVolumeExt::transform(*deformableVolume, transform, scale, simPositionInvMassPinned, simVelocityPinned, collPositionInvMassPinned, restPositionPinned);
	PxDeformableVolumeExt::updateMass(*deformableVolume, density, maxInvMassRatio, simPositionInvMassPinned);
	PxDeformableVolumeExt::copyToDevice(*deformableVolume, PxDeformableVolumeDataFlag::eALL, simPositionInvMassPinned, simVelocityPinned, collPositionInvMassPinned, restPositionPinned);

	DeformableVolume volume(deformableVolume, gCudaContextManager);

	gDeformableVolumes.pushBack(volume);

	PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, simPositionInvMassPinned);
	PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, simVelocityPinned);
	PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, collPositionInvMassPinned);
	PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, restPositionPinned);
}

static PxDeformableVolume* createDeformableVolume(const PxCookingParams& params, const PxArray<PxVec3>& triVerts, const PxArray<PxU32>& triIndices, bool useCollisionMeshForSimulation = false)
{
	PxDeformableVolumeMesh* deformableVolumeMesh;

	PxU32 numVoxelsAlongLongestAABBAxis = 8;

	PxSimpleTriangleMesh surfaceMesh;
	surfaceMesh.points.count = triVerts.size();
	surfaceMesh.points.data = triVerts.begin();
	surfaceMesh.triangles.count = triIndices.size() / 3;
	surfaceMesh.triangles.data = triIndices.begin();

	if (useCollisionMeshForSimulation)
	{
		deformableVolumeMesh = PxDeformableVolumeExt::createDeformableVolumeMeshNoVoxels(params, surfaceMesh, gPhysics->getPhysicsInsertionCallback());
	}
	else 
	{
		deformableVolumeMesh = PxDeformableVolumeExt::createDeformableVolumeMesh(params, surfaceMesh, numVoxelsAlongLongestAABBAxis, gPhysics->getPhysicsInsertionCallback());
	}	

	//Alternatively one can cook a deformable volume mesh in a single step
	//tetMesh = cooking.createDeformableVolumeMesh(simulationMeshDesc, collisionMeshDesc, deformableVolumeDesc, physics.getPhysicsInsertionCallback());
	PX_ASSERT(deformableVolumeMesh);

	if (!gCudaContextManager)
		return NULL;
	PxDeformableVolume* deformableVolume = gPhysics->createDeformableVolume(*gCudaContextManager);
	if (deformableVolume)
	{
		PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE;

		PxDeformableVolumeMaterial* materialPtr = PxGetPhysics().createDeformableVolumeMaterial(2.e+5f, 0.3f, 0.1f);
		PxTetrahedronMeshGeometry geometry(deformableVolumeMesh->getCollisionMesh());
		PxShape* shape = gPhysics->createShape(geometry, &materialPtr, 1, true, shapeFlags);
		if (shape)
		{
			deformableVolume->attachShape(*shape);
			shape->setSimulationFilterData(PxFilterData(0, 0, 2, 0));
		}
		deformableVolume->attachSimulationMesh(*deformableVolumeMesh->getSimulationMesh(), *deformableVolumeMesh->getDeformableVolumeAuxData());

		gScene->addActor(*deformableVolume);
		
		addDeformableVolume(deformableVolume, PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(PxIdentity)), 100.f, 1.0f);
		deformableVolume->setDeformableBodyFlag(PxDeformableBodyFlag::eDISABLE_SELF_COLLISION, true);
		deformableVolume->setSolverIterationCounts(30);
	}
	return deformableVolume;
}

static void createDeformableVolumes(const PxCookingParams& params)
{
	if (gCudaContextManager == NULL)
	{
		printf("The Deformable Volume feature is currently only supported on GPU\n");
		return;
	}

	PxArray<PxVec3> triVerts;
	PxArray<PxU32> triIndices;
	
	PxReal maxEdgeLength = 1;

	createCube(triVerts, triIndices, PxVec3(0.0, 9, 0), 2.5);
	PxRemeshingExt::limitMaxEdgeLength(triIndices, triVerts, maxEdgeLength);
	createDeformableVolume(params, triVerts, triIndices);

	createSphere(triVerts, triIndices, PxVec3(0, 4.5, 0), 2.5, maxEdgeLength);
	createDeformableVolume(params, triVerts, triIndices);

	createConeY(triVerts, triIndices, PxVec3(0.1, 11.5, 0), 2.0f, 3.5);
	PxRemeshingExt::limitMaxEdgeLength(triIndices, triVerts, maxEdgeLength);
	createDeformableVolume(params, triVerts, triIndices);
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

	sceneDesc.solverType = PxSolverType::eTGS;

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

	createDeformableVolumes(params);
}

void stepPhysics(bool /*interactive*/)
{
	const PxReal dt = 1.0f / 60.f;

	gScene->simulate(dt);
	gScene->fetchResults(true);

	for (PxU32 i = 0; i < gDeformableVolumes.size(); i++)
	{
		DeformableVolume* dv = &gDeformableVolumes[i];
		dv->copyDeformedVerticesFromGPU();
	}
}
	
void cleanupPhysics(bool /*interactive*/)
{
	for (PxU32 i = 0; i < gDeformableVolumes.size(); i++)
		gDeformableVolumes[i].release();
	gDeformableVolumes.reset();

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

	printf("SnippetDeformableVolume done.\n");
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
