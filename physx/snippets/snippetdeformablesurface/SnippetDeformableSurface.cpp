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
#include "extensions/PxDeformableSurfaceExt.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetdeformablesurface/SnippetDeformableSurface.h"

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation			= NULL;
static PxPhysics*				gPhysics			= NULL;
static PxCudaContextManager*	gCudaContextManager	= NULL;
static PxDefaultCpuDispatcher*	gDispatcher			= NULL;
static PxScene*					gScene				= NULL;
static PxMaterial*				gMaterial			= NULL;
static PxPvd*					gPvd				= NULL;
static bool						gIsRunning			= true;
PxArray<TestSurface>			gTestSurfaces;

PxRigidDynamic* sphere;

static void initObstacles()
{
	PxShape* shape = gPhysics->createShape(PxSphereGeometry(3.0f), *gMaterial);
	sphere = gPhysics->createRigidDynamic(PxTransform(PxVec3(0.f, 5.0f, 0.f)));
	sphere->attachShape(*shape);
	sphere->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
	gScene->addActor(*sphere);
	shape->release();
}

static PxDeformableSurface* createDeformableSurface(PxPhysics& physics, PxTriangleMesh* triangleMesh, PxDeformableSurfaceMaterial** materials, const PxU32 nbMaterials, PxCudaContextManager* cudaContextManager)
{
	if (!triangleMesh)
		return NULL;

	PxDeformableSurface* deformableSurface = physics.createDeformableSurface(*cudaContextManager);
	if (deformableSurface)
	{
		PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE;

		PxTriangleMeshGeometry geometry(triangleMesh);
		PxShape* shape = physics.createShape(geometry, materials, PxU16(nbMaterials), true, shapeFlags);
		if (shape)
		{
			deformableSurface->attachShape(*shape);
		}
	}

	return deformableSurface;
}

static PxDeformableSurface* createDeformableSurface(PxPhysics& physics, const PxCookingParams& ckParams, PxArray<PxVec3>& vertices, PxArray<PxU32>& triangles,
	PxDeformableSurfaceMaterial** materials, const PxU32 nbMaterials, PxCudaContextManager* cudaContextManager)
{
	PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = vertices.size();
	meshDesc.triangles.count = triangles.size() / 3;
	meshDesc.points.stride = sizeof(float) * 3;
	meshDesc.triangles.stride = sizeof(int) * 3;
	meshDesc.points.data = vertices.begin();
	meshDesc.triangles.data = triangles.begin();

	PxDeformableMaterialTableIndex* materialIndices = NULL;
	if (nbMaterials > 1)
	{
		const PxU32 totalTriangles = meshDesc.triangles.count;
		materialIndices = new PxDeformableMaterialTableIndex[totalTriangles];
		const PxU32 averageTrianglePerMaterials = totalTriangles / nbMaterials;

		PxU32 accumulatedTriangle = averageTrianglePerMaterials;

		PxU32 index = 0;
		for (PxU32 i = 0; i < totalTriangles; ++i)
		{
			materialIndices[i] = PxDeformableMaterialTableIndex(index);
			if (i == accumulatedTriangle)
			{
				index = index < (nbMaterials - 1) ? index + 1 : index;
				accumulatedTriangle += averageTrianglePerMaterials;
			}
		}
		meshDesc.materialIndices.stride = sizeof(PxDeformableMaterialTableIndex);
		meshDesc.materialIndices.data = materialIndices;
	}

	PxTriangleMesh* triangleMesh = PxCreateTriangleMesh(ckParams, meshDesc, physics.getPhysicsInsertionCallback());

	return createDeformableSurface(physics, triangleMesh, materials, nbMaterials, cudaContextManager);
}

static PX_FORCE_INLINE PxU32 id(PxU32 x, PxU32 y, PxU32 numY)
{
	return x * numY + y;
}

static PX_FORCE_INLINE PxReal computeTriangleMass(const PxU32* triangle, const PxVec3* vertices, PxReal thickness, PxReal density)
{
	PxReal area = 0.5f * (vertices[triangle[1]] - vertices[triangle[0]]).cross(vertices[triangle[2]] - vertices[triangle[0]]).magnitude();
	return area * thickness * density;
}

static PxDeformableSurface* addQuadDeformableSurface(PxPhysics& physics, const PxCookingParams& ckParams, const PxTransform& transform, PxU32 numX, PxU32 numZ, PxReal sizeX, PxReal sizeZ,
	PxDeformableSurfaceMaterial** materials, const PxU32 nbMaterials, PxCudaContextManager* cudaContextManager, PxReal thickness, PxReal density)
{
	if (gCudaContextManager == NULL)
	{
		return NULL;
	}

	PxArray<PxVec3> vertices;
	PxArray<PxU32> triangles;
	PxArray<PxVec3> velocity;
	PxArray<PxReal> triangleMasses;

	vertices.reserve(numX * numZ);
	velocity.reserve(numX * numZ);
	triangles.reserve(3 * 2 * (numX - 1) * (numZ - 1));
	triangleMasses.reserve(2 * (numX - 1) * (numZ - 1));

	PxReal scalingX = sizeX / (numX - 1);
	PxReal scalingZ = sizeZ / (numZ - 1);

	for (PxU32 i = 0; i < numX; ++i)
	{
		for (PxU32 j = 0; j < numZ; ++j)
		{
			PxVec3 pos(i * scalingX, 0.0f, j * scalingZ);
			vertices.pushBack(pos);
			velocity.pushBack(PxVec3(0.0f));
		}
	}

	for (PxU32 i = 1; i < numX; ++i)
	{
		for (PxU32 j = 1; j < numZ; ++j)
		{
			triangles.pushBack(id(i - 1, j - 1, numZ));
			triangles.pushBack(id(i, j - 1, numZ));
			triangles.pushBack(id(i - 1, j, numZ));			
			triangleMasses.pushBack(computeTriangleMass(&triangles[triangles.size() - 3], vertices.begin(), thickness, density));

			triangles.pushBack(id(i - 1, j, numZ));
			triangles.pushBack(id(i, j - 1, numZ));
			triangles.pushBack(id(i, j, numZ));
			triangleMasses.pushBack(computeTriangleMass(&triangles[triangles.size() - 3], vertices.begin(), thickness, density));
		}
	}

	PxDeformableSurface* deformableSurface = createDeformableSurface(physics, ckParams, vertices, triangles, materials, nbMaterials, cudaContextManager);
	gScene->addActor(*deformableSurface);

	PxVec4* posInvMassPinned;
	PxVec4* velocityPinned;
	PxVec4* restPositionPinned;
	PxDeformableSurfaceExt::allocateAndInitializeHostMirror(*deformableSurface, vertices.begin(), velocity.begin(), vertices.begin(), 0.5f,
		transform, cudaContextManager, posInvMassPinned, velocityPinned, restPositionPinned);

	PxDeformableSurfaceExt::distributeTriangleMassToVertices(*deformableSurface, triangleMasses.begin(), posInvMassPinned);

	PxShape* surfaceShape = deformableSurface->getShape();
	surfaceShape->setContactOffset(2.0f * thickness);
	surfaceShape->setRestOffset(thickness);
	surfaceShape->setDeformableSurfaceMaterials(materials, PxU16(nbMaterials));

	PxDeformableSurfaceExt::copyToDevice(*deformableSurface, PxDeformableSurfaceDataFlag::eALL, vertices.size(), posInvMassPinned, velocityPinned, restPositionPinned);
	
	TestSurface testSurface(deformableSurface, gCudaContextManager);

	gTestSurfaces.pushBack(testSurface);

	PX_EXT_PINNED_MEMORY_FREE(*cudaContextManager, posInvMassPinned);
	PX_EXT_PINNED_MEMORY_FREE(*cudaContextManager, velocityPinned);
	PX_EXT_PINNED_MEMORY_FREE(*cudaContextManager, restPositionPinned);

	return deformableSurface;
}

static void createScene(PxCookingParams& cookingParams)
{
	PxReal thickness = 0.01f;
	PxReal bendingStiffness = 0.00001f;

	PxDeformableSurfaceMaterial* deformableSurfaceMaterial = gPhysics->createDeformableSurfaceMaterial(1.e10f, 0.3f, 0.5f, thickness, bendingStiffness);

	PxReal size = 15.0f;
	PxDeformableSurface* deformableSurface = addQuadDeformableSurface(*gPhysics, cookingParams, 
		PxTransform(PxVec3(-0.5f * size, 10.0f, -0.5f * size)), 200, 200, size, size, &deformableSurfaceMaterial, 1, gCudaContextManager, thickness, 500.0f);
	if (deformableSurface)
	{
		deformableSurface->setSelfCollisionFilterDistance(thickness * 2.5f);
		deformableSurface->setLinearDamping(0.f);
		deformableSurface->setDeformableBodyFlag(PxDeformableBodyFlag::eDISABLE_SELF_COLLISION, false);

		deformableSurface->setMaxVelocity(1000.0f);

		PxU32 collisionPairUpdateFrequency = 1;
		PxU32 nbCollisionSubsteps = 1;
		deformableSurface->setNbCollisionPairUpdatesPerTimestep(collisionPairUpdateFrequency);
		deformableSurface->setNbCollisionSubsteps(nbCollisionSubsteps);
	}

	initObstacles();
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
		printf("The deformable surface feature is currently only supported on GPU.\n");
	}

	PxTolerancesScale scale;
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, scale, true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);

	PxCookingParams params(scale);
	params.meshWeldTolerance = 0.001f;
	params.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES);
	params.buildTriangleAdjacencies = false;
	params.buildGPUData = true;
	params.midphaseDesc = PxMeshMidPhase::eBVH34;
	//params.meshPreprocessParams |= PxMeshPreprocessingFlag::eFORCE_32BIT_INDICES;
	params.meshPreprocessParams |= PxMeshPreprocessingFlag::eENABLE_VERT_MAPPING;

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

	createScene(params);
}

PxReal simTime = 0;
void stepPhysics(bool /*interactive*/)
{
	if (gIsRunning)
	{
		const PxReal dt = 1.0f / 60.0f;

		bool rotatingSphere = true;
		if (rotatingSphere)
		{
			const PxReal speed = 2.0f;
			PxTransform pose = sphere->getGlobalPose();
			sphere->setKinematicTarget(PxTransform(pose.p, PxQuat(PxCos(simTime*speed), PxVec3(0, 1, 0))));
		}

		gScene->simulate(dt);
		gScene->fetchResults(true);
		
		for (PxU32 i = 0; i < gTestSurfaces.size(); i++)
		{
			TestSurface* c = &gTestSurfaces[i];
			c->copyDeformedVerticesFromGPU();
		}

		simTime += dt;
	}
}
	
void cleanupPhysics(bool /*interactive*/)
{
	for (PxU32 i = 0; i < gTestSurfaces.size(); i++)
		gTestSurfaces[i].release();
	gTestSurfaces.reset();

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

	printf("SnippetDeformableSurface done.\n");
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
