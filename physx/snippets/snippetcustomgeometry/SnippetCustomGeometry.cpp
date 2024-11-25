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
// This snippet shows how to use custom geometries in PhysX.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"

// temporary disable this snippet, cannot work without rendering we cannot include GL directly
#ifdef RENDER_SNIPPET

#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetrender/SnippetRender.h"

#include "VoxelMap.h"

using namespace physx;

static PxDefaultAllocator gAllocator;
static PxDefaultErrorCallback gErrorCallback;
static PxFoundation* gFoundation = NULL;
static PxPhysics* gPhysics = NULL;
static PxDefaultCpuDispatcher* gDispatcher = NULL;
static PxScene* gScene = NULL;
static PxMaterial* gMaterial = NULL;
static PxPvd* gPvd = NULL;
static PxRigidStatic* gActor = NULL;

static const int gVoxelMapDim = 20;
static const float gVoxelMapSize = 80.0f;
static VoxelMap* gVoxelMap;

static PxArray<PxVec3> gVertices;
static PxArray<PxU32> gIndices;
static PxU32 gVertexCount;
static PxU32 gIndexCount;
static PxGeometryHolder gVoxelGeometryHolder;
static const PxU32 gVertexOrder[12] = {
	0, 2, 1, 2, 3, 1,
	0, 1, 2, 2, 1, 3
};

PX_INLINE void cookVoxelFace(bool reverseWinding) {
	for (int i = 0; i < 6; ++i) {
		gIndices[gIndexCount + i] = gVertexCount + gVertexOrder[i + (reverseWinding ? 6 : 0)];
	}
	gVertexCount += 4;
	gIndexCount += 6;
}

void cookVoxelMesh() {

	int faceCount = 0;
	gVertexCount = 0;
	gIndexCount = 0;

	float vx[2] = {gVoxelMap->voxelSize().x * -0.5f, gVoxelMap->voxelSize().x * 0.5f};
	float vy[2] = {gVoxelMap->voxelSize().y * -0.5f, gVoxelMap->voxelSize().y * 0.5f};
	float vz[2] = {gVoxelMap->voxelSize().z * -0.5f, gVoxelMap->voxelSize().z * 0.5f};

	for (int x = 0; x < gVoxelMap->dimX(); ++x)
		for (int y = 0; y < gVoxelMap->dimY(); ++y)
			for (int z = 0; z < gVoxelMap->dimZ(); ++z)
				if (gVoxelMap->voxel(x, y, z))
				{
					if (!gVoxelMap->voxel(x+1, y, z)) {faceCount++;}
					if (!gVoxelMap->voxel(x-1, y, z)) {faceCount++;}
					if (!gVoxelMap->voxel(x, y+1, z)) {faceCount++;}
					if (!gVoxelMap->voxel(x, y-1, z)) {faceCount++;}
					if (!gVoxelMap->voxel(x, y, z+1)) {faceCount++;}
					if (!gVoxelMap->voxel(x, y, z-1)) {faceCount++;}
				}

	gVertices.resize(faceCount*4);
	gIndices.resize(faceCount*6);

	for (int x = 0; x < gVoxelMap->dimX(); ++x)
	{
		for (int y = 0; y < gVoxelMap->dimY(); ++y)
		{
			for (int z = 0; z < gVoxelMap->dimZ(); ++z)
			{
				PxVec3 voxelPos = gVoxelMap->voxelPos(x, y, z);

				if (gVoxelMap->voxel(x, y, z))
				{
					if (!gVoxelMap->voxel(x+1, y, z)) {
						gVertices[gVertexCount + 0] = voxelPos + PxVec3(vx[1], vy[0], vz[0]);
						gVertices[gVertexCount + 1] = voxelPos + PxVec3(vx[1], vy[0], vz[1]);
						gVertices[gVertexCount + 2] = voxelPos + PxVec3(vx[1], vy[1], vz[0]);
						gVertices[gVertexCount + 3] = voxelPos + PxVec3(vx[1], vy[1], vz[1]);
						cookVoxelFace(false);
					}
					if (!gVoxelMap->voxel(x-1, y, z)) {
						gVertices[gVertexCount + 0] = voxelPos + PxVec3(vx[0], vy[0], vz[0]);
						gVertices[gVertexCount + 1] = voxelPos + PxVec3(vx[0], vy[0], vz[1]);
						gVertices[gVertexCount + 2] = voxelPos + PxVec3(vx[0], vy[1], vz[0]);
						gVertices[gVertexCount + 3] = voxelPos + PxVec3(vx[0], vy[1], vz[1]);
						cookVoxelFace(true);
					}
					if (!gVoxelMap->voxel(x, y+1, z)) {
						gVertices[gVertexCount + 0] = voxelPos + PxVec3(vx[0], vy[1], vz[0]);
						gVertices[gVertexCount + 1] = voxelPos + PxVec3(vx[0], vy[1], vz[1]);
						gVertices[gVertexCount + 2] = voxelPos + PxVec3(vx[1], vy[1], vz[0]);
						gVertices[gVertexCount + 3] = voxelPos + PxVec3(vx[1], vy[1], vz[1]);
						cookVoxelFace(true);
					}
					if (!gVoxelMap->voxel(x, y-1, z)) {
						gVertices[gVertexCount + 0] = voxelPos + PxVec3(vx[0], vy[0], vz[0]);
						gVertices[gVertexCount + 1] = voxelPos + PxVec3(vx[0], vy[0], vz[1]);
						gVertices[gVertexCount + 2] = voxelPos + PxVec3(vx[1], vy[0], vz[0]);
						gVertices[gVertexCount + 3] = voxelPos + PxVec3(vx[1], vy[0], vz[1]);
						cookVoxelFace(false);
					}
					if (!gVoxelMap->voxel(x, y, z+1)) {
						gVertices[gVertexCount + 0] = voxelPos + PxVec3(vx[0], vy[0], vz[1]);
						gVertices[gVertexCount + 1] = voxelPos + PxVec3(vx[0], vy[1], vz[1]);
						gVertices[gVertexCount + 2] = voxelPos + PxVec3(vx[1], vy[0], vz[1]);
						gVertices[gVertexCount + 3] = voxelPos + PxVec3(vx[1], vy[1], vz[1]);
						cookVoxelFace(false);
					}
					if (!gVoxelMap->voxel(x, y, z-1)) {
						gVertices[gVertexCount + 0] = voxelPos + PxVec3(vx[0], vy[0], vz[0]);
						gVertices[gVertexCount + 1] = voxelPos + PxVec3(vx[0], vy[1], vz[0]);
						gVertices[gVertexCount + 2] = voxelPos + PxVec3(vx[1], vy[0], vz[0]);
						gVertices[gVertexCount + 3] = voxelPos + PxVec3(vx[1], vy[1], vz[0]);
						cookVoxelFace(true);
					}
				}
			}
		}
	}

	const PxTolerancesScale scale;
	PxCookingParams params(scale);
	params.midphaseDesc.setToDefault(PxMeshMidPhase::eBVH34);
	params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
	params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;
	PxTriangleMeshDesc triangleMeshDesc;
	triangleMeshDesc.points.count = gVertexCount;
	triangleMeshDesc.points.data = gVertices.begin();
	triangleMeshDesc.points.stride = sizeof(PxVec3);
	triangleMeshDesc.triangles.count = gIndexCount / 3;
	triangleMeshDesc.triangles.data = gIndices.begin();
	triangleMeshDesc.triangles.stride = 3 * sizeof(PxU32);
	PxTriangleMesh* gTriangleMesh = PxCreateTriangleMesh(params, triangleMeshDesc);
	gVoxelGeometryHolder.storeAny( PxTriangleMeshGeometry(gTriangleMesh) );
}

static PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0), PxReal density = 1.0f)
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, density);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

static void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	for (PxU32 i = 0; i < size; i++)
	{
		for (PxU32 j = 0; j < size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
}

void initVoxelMap()
{
	gVoxelMap = PX_NEW(VoxelMap);
	gVoxelMap->setDimensions(gVoxelMapDim, gVoxelMapDim, gVoxelMapDim);
	gVoxelMap->setVoxelSize(gVoxelMapSize / gVoxelMapDim, gVoxelMapSize / gVoxelMapDim, gVoxelMapSize / gVoxelMapDim);
	gVoxelMap->setWaveVoxels();
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;

	gScene = gPhysics->createScene(sceneDesc);
	gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
	gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	// Create voxel map actor
	initVoxelMap();
	PxRigidStatic* voxelMapActor = gPhysics->createRigidStatic(PxTransform(PxVec3(0, gVoxelMapSize * 0.5f, 0)));
	PxShape* shape = PxRigidActorExt::createExclusiveShape(*voxelMapActor, PxCustomGeometry(*gVoxelMap), *gMaterial);
	shape->setFlag(PxShapeFlag::eVISUALIZATION, true);
	gScene->addActor(*voxelMapActor);
	gActor = voxelMapActor;
	gActor->setActorFlag(PxActorFlag::eVISUALIZATION, true);

	// Ground plane
	PxRigidStatic* planeActor = gPhysics->createRigidStatic(PxTransform(PxQuat(PX_PIDIV2, PxVec3(0, 0, 1))));
	PxRigidActorExt::createExclusiveShape(*planeActor, PxPlaneGeometry(), *gMaterial);
	gScene->addActor(*planeActor);

	gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.f);
	gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);

	createStack(PxTransform(PxVec3(0, 22, 0)), 10, 2.0f);

	cookVoxelMesh();
}

void debugRender()
{
	PxTransform pose = gActor->getGlobalPose();
	Snippets::renderGeoms(1, &gVoxelGeometryHolder, &pose, false, PxVec3(0.5f));
}

void stepPhysics(bool /*interactive*/)
{
	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);
}

void cleanupPhysics(bool /*interactive*/)
{
	PX_DELETE(gVoxelMap);
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	gVertices.reset();
	gIndices.reset();
	if (gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		PX_RELEASE(gPvd);
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);

	printf("SnippetCustomGeometry done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	switch (toupper(key))
	{
	case ' ':	createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0, 0, -1)) * 200, 3.0f);	break;
	}
}

int snippetMain(int, const char* const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for (PxU32 i = 0; i < frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}

#else
int snippetMain(int, const char* const*)
{
	return 0;
}

#endif

