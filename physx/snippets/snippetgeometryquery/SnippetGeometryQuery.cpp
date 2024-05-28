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
// This snippet illustrates how to use a PxGeometryQuery for raycasts.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#ifdef RENDER_SNIPPET
	#include "../snippetrender/SnippetRender.h"
#endif

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;

enum Geom
{
	GEOM_BOX,
	GEOM_SPHERE,
	GEOM_CAPSULE,
	GEOM_CONVEX,
	GEOM_MESH,

	GEOM_COUNT
};

static const PxU32	gScenarioCount	= GEOM_COUNT;
static PxU32		gScenario		= 0;

static PxConvexMesh* createConvexMesh(const PxVec3* verts, const PxU32 numVerts, const PxCookingParams& params)
{
	PxConvexMeshDesc convexDesc;
	convexDesc.points.count		= numVerts;
	convexDesc.points.stride	= sizeof(PxVec3);
	convexDesc.points.data		= verts;
	convexDesc.flags			= PxConvexFlag::eCOMPUTE_CONVEX;
	return PxCreateConvexMesh(params, convexDesc);
}

static PxConvexMesh* createCylinderMesh(const PxF32 width, const PxF32 radius, const PxCookingParams& params)
{
	PxVec3 points[2*16];
	for(PxU32 i = 0; i < 16; i++)
	{
		const PxF32 cosTheta = PxCos(i*PxPi*2.0f/16.0f);
		const PxF32 sinTheta = PxSin(i*PxPi*2.0f/16.0f);
		const PxF32 y = radius*cosTheta;
		const PxF32 z = radius*sinTheta;
		points[2*i+0] = PxVec3(-width/2.0f, y, z);
		points[2*i+1] = PxVec3(+width/2.0f, y, z);
	}
	return createConvexMesh(points, 32, params);
}

static void initScene()
{
}

static void releaseScene()
{
}

static PxConvexMesh* gConvexMesh = NULL;
static PxTriangleMesh* gTriangleMesh = NULL;
static PxBoxGeometry gBoxGeom(PxVec3(1.0f, 2.0f, 0.5f));
static PxSphereGeometry gSphereGeom(1.5f);
static PxCapsuleGeometry gCapsuleGeom(1.0f, 1.0f);
static PxConvexMeshGeometry gConvexGeom;
static PxTriangleMeshGeometry gMeshGeom;
const PxGeometry& getTestGeometry()
{
	switch(gScenario)
	{
		case GEOM_BOX:
			return gBoxGeom;
		case GEOM_SPHERE:
			return gSphereGeom;
		case GEOM_CAPSULE:
			return gCapsuleGeom;
		case GEOM_CONVEX:
			gConvexGeom.convexMesh = gConvexMesh;
			return gConvexGeom;
		case GEOM_MESH:
			gMeshGeom.triangleMesh = gTriangleMesh;
			gMeshGeom.scale.scale = PxVec3(2.0f);
			return gMeshGeom;
	}
	static PxSphereGeometry pt(0.0f);
	return pt;
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	const PxTolerancesScale scale;
	PxCookingParams params(scale);
	params.midphaseDesc.setToDefault(PxMeshMidPhase::eBVH34);
//	params.midphaseDesc.mBVH34Desc.quantized = false;
	params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
	params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;

	gConvexMesh = createCylinderMesh(3.0f, 1.0f, params);

	{
		PxTriangleMeshDesc meshDesc;
		meshDesc.points.count		= SnippetUtils::Bunny_getNbVerts();
		meshDesc.points.stride		= sizeof(PxVec3);
		meshDesc.points.data		= SnippetUtils::Bunny_getVerts();
		meshDesc.triangles.count	= SnippetUtils::Bunny_getNbFaces();
		meshDesc.triangles.stride	= sizeof(int)*3;
		meshDesc.triangles.data		= SnippetUtils::Bunny_getFaces();

		gTriangleMesh = PxCreateTriangleMesh(params, meshDesc);
	}

	initScene();
}

void stepPhysics(bool /*interactive*/)
{
}

void cleanupPhysics(bool /*interactive*/)
{
	releaseScene();

	PX_RELEASE(gConvexMesh);
	PX_RELEASE(gFoundation);
	
	printf("SnippetGeometryQuery done.\n");
}

void keyPress(unsigned char key, const PxTransform& /*camera*/)
{
	if(key >= 1 && key <= gScenarioCount)
	{
		gScenario = key - 1;
		releaseScene();
		initScene();
	}

	if(key == 'r' || key == 'R')
	{
		releaseScene();
		initScene();
	}
}

void renderText()
{
#ifdef RENDER_SNIPPET
	Snippets::print("Press F1 to F5 to select a geometry object.");
#endif
}

int snippetMain(int, const char*const*)
{
	printf("GeometryQuery snippet. Use these keys:\n");
	printf(" F1 to F5 - select different geom\n");
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




