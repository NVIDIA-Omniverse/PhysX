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
// This snippet demonstrates how to setup triangle meshes with SDFs.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetsdf/MeshGenerator.h"


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



static PxTriangleMesh* createMesh(PxCookingParams& params, const PxArray<PxVec3>& triVerts, const PxArray<PxU32>& triIndices, PxReal sdfSpacing, 
	PxU32 sdfSubgridSize = 6, PxSdfBitsPerSubgridPixel::Enum bitsPerSdfSubgridPixel = PxSdfBitsPerSubgridPixel::e16_BIT_PER_PIXEL)
{
	PxTriangleMeshDesc meshDesc;	
	meshDesc.points.count = triVerts.size();
	meshDesc.triangles.count = triIndices.size() / 3;
	meshDesc.points.stride = sizeof(float) * 3;
	meshDesc.triangles.stride = sizeof(int) * 3;
	meshDesc.points.data = triVerts.begin();
	meshDesc.triangles.data = triIndices.begin();

	params.meshPreprocessParams |= PxMeshPreprocessingFlag::eENABLE_INERTIA;
	params.meshWeldTolerance = 1e-7f;	

	PxSDFDesc sdfDesc;

	if (sdfSpacing > 0.f)
	{
		sdfDesc.spacing = sdfSpacing;
		sdfDesc.subgridSize = sdfSubgridSize;
		sdfDesc.bitsPerSubgridPixel = bitsPerSdfSubgridPixel;
		sdfDesc.numThreadsForSdfConstruction = 16;

		meshDesc.sdfDesc = &sdfDesc;
	}

	bool enableCaching = false;

	if (enableCaching)
	{
		const char* path = "C:\\tmp\\PhysXSDFSnippetData.dat";
		bool ok = false;
		FILE* fp = fopen(path, "rb");
		if (fp)
		{
			fclose(fp);
			ok = true;
		}

		if (!ok)
		{
			PxDefaultFileOutputStream stream(path);
			ok = PxCookTriangleMesh(params, meshDesc, stream);
		}

		if (ok)
		{
			PxDefaultFileInputData stream(path);
			PxTriangleMesh* triangleMesh = gPhysics->createTriangleMesh(stream);
			return triangleMesh;
		}
		return NULL;
	}
	else
		return PxCreateTriangleMesh(params, meshDesc, gPhysics->getPhysicsInsertionCallback());
}

static void addInstance(const PxTransform& transform, PxTriangleMesh* mesh)
{
	PxRigidDynamic* dyn = gPhysics->createRigidDynamic(transform);
	dyn->setLinearDamping(0.2f);
	dyn->setAngularDamping(0.1f);
	PxTriangleMeshGeometry geom;
	geom.triangleMesh = mesh;
	geom.scale = PxVec3(0.1f, 0.1f, 0.1f);

	dyn->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES, true);
	dyn->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, true);

	PxShape* shape = PxRigidActorExt::createExclusiveShape(*dyn, geom, *gMaterial);
	shape->setContactOffset(0.05f);
	shape->setRestOffset(0.0f);

	PxRigidBodyExt::updateMassAndInertia(*dyn, 100.f);

	gScene->addActor(*dyn);

	dyn->setWakeCounter(100000000.f);
	dyn->setSolverIterationCounts(50, 1);
	dyn->setMaxDepenetrationVelocity(5.f);
}

static void createBowls(PxCookingParams& params)
{
	if (gCudaContextManager == NULL)
	{
		printf("SDF meshes are currently only supported on GPU.\n");
		return;
	}
	PxArray<PxVec3> triVerts;
	PxArray<PxU32> triIndices;
	
	PxReal maxEdgeLength = 1;

	createBowl(triVerts, triIndices, PxVec3(0, 4.5, 0), 6.0f, maxEdgeLength);
	PxTriangleMesh* mesh = createMesh(params, triVerts, triIndices, 0.05f);
	
	PxQuat rotate(PxIdentity);
	const PxU32 numInstances = 100;
	for (PxU32 i = 0; i < numInstances; ++i)
	{
		PxTransform transform(PxVec3(0, 5.f + i * 0.5f, 0), rotate);
		addInstance(transform, mesh);
	}
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

	createBowls(params);
}

void stepPhysics(bool /*interactive*/)
{
	const PxReal dt = 1.0f / 60.f;

	gScene->simulate(dt);
	gScene->fetchResults(true);
}
	
void cleanupPhysics(bool /*interactive*/)
{
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

	printf("SnippetSDF done.\n");
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
