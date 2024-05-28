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
// This snippet shows how to use deformable meshes in PhysX.
// ****************************************************************************

#include <ctype.h>
#include <vector>
#include "PxPhysicsAPI.h"

// temporary disable this snippet, cannot work without rendering we cannot include GL directly
#ifdef RENDER_SNIPPET

#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#include "../snippetrender/SnippetRender.h"

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxPhysics*				gPhysics	= NULL;
static PxDefaultCpuDispatcher*	gDispatcher = NULL;
static PxScene*					gScene		= NULL;
static PxMaterial*				gMaterial	= NULL;
static PxPvd*					gPvd        = NULL;
static PxTriangleMesh*			gMesh		= NULL;
static PxRigidStatic*			gActor		= NULL;

static const PxU32 gGridSize = 8;
static const PxReal gGridStep = 512.0f / PxReal(gGridSize-1);
static float gTime = 0.0f;

static PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(0), PxReal density=1.0f)
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, density);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

static void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	for(PxU32 i=0; i<size;i++)
	{
		for(PxU32 j=0;j<size-i;j++)
		{
			PxTransform localTm(PxVec3(PxReal(j*2) - PxReal(size-i), PxReal(i*2+1), 0) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
}

struct Triangle
{
	PxU32 ind0, ind1, ind2;
};

static void updateVertices(PxVec3* verts, float amplitude=0.0f)
{
	const PxU32 gridSize = gGridSize;
	const PxReal gridStep = gGridStep;

	for(PxU32 a=0; a<gridSize; a++)
	{
		const float coeffA = float(a)/float(gridSize);
		for(PxU32 b=0; b<gridSize; b++)
		{
			const float coeffB = float(b)/float(gridSize);

			const float y = 20.0f + sinf(coeffA*PxTwoPi)*cosf(coeffB*PxTwoPi)*amplitude;

			verts[a * gridSize + b] = PxVec3(-400.0f + b*gridStep, y, -400.0f + a*gridStep);
		}
	}
}

static PxTriangleMesh* createMeshGround(const PxCookingParams& params)
{
	const PxU32 gridSize = gGridSize;

	PxVec3 verts[gridSize * gridSize];

	const PxU32 nbTriangles = 2 * (gridSize - 1) * (gridSize-1);

	Triangle indices[nbTriangles];

	updateVertices(verts);

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

	PxTriangleMesh* triMesh = PxCreateTriangleMesh(params, meshDesc, gPhysics->getPhysicsInsertionCallback());

	return triMesh;
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;

	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	gMaterial = gPhysics->createMaterial(1.0f, 1.0f, 0.0f);

	PxCookingParams cookingParams(gPhysics->getTolerancesScale());

	if(0)
	{
		cookingParams.midphaseDesc.setToDefault(PxMeshMidPhase::eBVH33);
	}
	else
	{
		cookingParams.midphaseDesc.setToDefault(PxMeshMidPhase::eBVH34);
		cookingParams.midphaseDesc.mBVH34Desc.quantized = false;
	}
	// We need to disable the mesh cleaning part so that the vertex mapping remains untouched.
	cookingParams.meshPreprocessParams	= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;

	PxTriangleMesh* mesh = createMeshGround(cookingParams);
	gMesh = mesh;

	PxTriangleMeshGeometry geom(mesh);

	PxRigidStatic* groundMesh = gPhysics->createRigidStatic(PxTransform(PxVec3(0, 2, 0)));
	gActor = groundMesh;
	PxShape* shape = gPhysics->createShape(geom, *gMaterial);

	{
		shape->setContactOffset(0.02f);
		// A negative rest offset helps to avoid jittering when the deformed mesh moves away from objects resting on it.
		shape->setRestOffset(-0.5f);
	}

	groundMesh->attachShape(*shape);
	gScene->addActor(*groundMesh);

	createStack(PxTransform(PxVec3(0,22,0)), 10, 2.0f);
}

PxBounds3 gBounds;
void debugRender()
{
	const PxVec3 c = gBounds.getCenter();
	const PxVec3 e = gBounds.getExtents();

	PxVec3 pts[8];
	pts[0] = c + PxVec3(-e.x, -e.y, e.z);
	pts[1] = c + PxVec3(-e.x,  e.y, e.z);
	pts[2] = c + PxVec3( e.x,  e.y, e.z);
	pts[3] = c + PxVec3( e.x, -e.y, e.z);
	pts[4] = c + PxVec3(-e.x, -e.y, -e.z);
	pts[5] = c + PxVec3(-e.x,  e.y, -e.z);
	pts[6] = c + PxVec3( e.x,  e.y, -e.z);
	pts[7] = c + PxVec3( e.x, -e.y, -e.z);

	std::vector<PxVec3> gContactVertices;
	struct AddQuad
	{
		static void func(std::vector<PxVec3>& v, const PxVec3* pts_, PxU32 index0, PxU32 index1, PxU32 index2, PxU32 index3)
		{
			v.push_back(pts_[index0]);
			v.push_back(pts_[index1]);

			v.push_back(pts_[index1]);
			v.push_back(pts_[index2]);

			v.push_back(pts_[index2]);
			v.push_back(pts_[index3]);

			v.push_back(pts_[index3]);
			v.push_back(pts_[index0]);
		}
	};

	AddQuad::func(gContactVertices, pts, 0, 1, 2, 3);
	AddQuad::func(gContactVertices, pts, 4, 5, 6, 7);
	AddQuad::func(gContactVertices, pts, 0, 1, 5, 4);
	AddQuad::func(gContactVertices, pts, 2, 3, 7, 6);

	glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
	glDisable(GL_LIGHTING);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, &gContactVertices[0]);
	glDrawArrays(GL_LINES, 0, GLint(gContactVertices.size()));
	glDisableClientState(GL_VERTEX_ARRAY);
	glEnable(GL_LIGHTING);
}

void stepPhysics(bool /*interactive*/)
{
	{
		PxVec3* verts = gMesh->getVerticesForModification();
		gTime += 0.01f;
		updateVertices(verts, sinf(gTime)*20.0f);

		{
//		unsigned long long time = __rdtsc();
		gBounds = gMesh->refitBVH();
//		time = __rdtsc() - time;
//		printf("Time: %d\n", int(time));
		}

		// Reset filtering to tell the broadphase about the new mesh bounds.
		gScene->resetFiltering(*gActor);
	}
	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);
}

void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if(gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		PX_RELEASE(gPvd);
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);
	
	printf("SnippetDeformableMesh done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	switch(toupper(key))
	{
	case ' ':	createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0,0,-1))*200, 3.0f);	break;
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

#else
int snippetMain(int, const char*const*)
{
	return 0;
}

#endif


