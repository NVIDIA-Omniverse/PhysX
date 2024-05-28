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
// This snippet illustrates how to use a PxBVH to implement view-frustum culling.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "foundation/PxArray.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#ifdef RENDER_SNIPPET
	#include "../snippetrender/SnippetCamera.h"
	#include "../snippetrender/SnippetRender.h"
#endif

//#define BENCHMARK_MODE

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;

namespace
{
	class CustomScene
	{
		public:
			CustomScene();
			~CustomScene();

			void	release();
			void	addGeom(const PxGeometry& geom, const PxTransform& pose);
			void	createBVH();
			void	render()	const;

			struct Object
			{
				PxGeometryHolder	mGeom;
				PxTransform			mPose;
			};

			PxArray<Object>	mObjects;
			PxBVH*			mBVH;
	};

CustomScene::CustomScene() : mBVH(NULL)
{
}

CustomScene::~CustomScene()
{
}

void CustomScene::release()
{
	PX_RELEASE(mBVH);
	mObjects.reset();
	PX_DELETE_THIS;
}

void CustomScene::addGeom(const PxGeometry& geom, const PxTransform& pose)
{
	Object obj;
	obj.mGeom.storeAny(geom);
	obj.mPose = pose;
	mObjects.pushBack(obj);
}

void CustomScene::createBVH()
{
	const PxU32 nbObjects = mObjects.size();
	PxBounds3* bounds = new PxBounds3[nbObjects];
	for(PxU32 i=0;i<nbObjects;i++)
	{
		Object& obj = mObjects[i];
		PxGeometryQuery::computeGeomBounds(bounds[i], obj.mGeom.any(), obj.mPose);
	}

	PxBVHDesc bvhDesc;
	bvhDesc.bounds.count = nbObjects;
	bvhDesc.bounds.data = bounds;
	bvhDesc.bounds.stride = sizeof(PxBounds3);
	bvhDesc.enlargement	= 0.0f;
	mBVH = PxCreateBVH(bvhDesc);
	delete [] bounds;
}

enum FrustumPlaneIndex
{
	FRUSTUM_PLANE_LEFT		= 0,
	FRUSTUM_PLANE_RIGHT		= 1,
	FRUSTUM_PLANE_TOP		= 2,
	FRUSTUM_PLANE_BOTTOM	= 3,
	FRUSTUM_PLANE_NEAR		= 4,
	FRUSTUM_PLANE_FAR		= 5,
};

void CustomScene::render() const
{
#ifdef RENDER_SNIPPET

	if(0)
	{
		// This codepath to draw all objects without culling
		const PxVec3 color(1.0f, 0.5f, 0.25f);
		for(PxU32 i=0;i<mObjects.size();i++)
		{
			const CustomScene::Object& obj = mObjects[i];
			Snippets::renderGeoms(1, &obj.mGeom, &obj.mPose, false, color);
		}
		return;
	}

	// Extract planes from the view/proj matrices. You could also build them from the frustum's vertices.
	PxPlane planes[6];
	{
		float VM[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, VM);
		const PxMat44 ViewMatrix(VM);

		float PM[16];
		glGetFloatv(GL_PROJECTION_MATRIX, PM);
		const PxMat44 ProjMatrix(PM);

		const PxMat44 combo = ProjMatrix * ViewMatrix;

		planes[FRUSTUM_PLANE_LEFT].n.x	= -(combo.column0[3] + combo.column0[0]);
		planes[FRUSTUM_PLANE_LEFT].n.y	= -(combo.column1[3] + combo.column1[0]);
		planes[FRUSTUM_PLANE_LEFT].n.z	= -(combo.column2[3] + combo.column2[0]);
		planes[FRUSTUM_PLANE_LEFT].d	= -(combo.column3[3] + combo.column3[0]);

		planes[FRUSTUM_PLANE_RIGHT].n.x	= -(combo.column0[3] - combo.column0[0]);
		planes[FRUSTUM_PLANE_RIGHT].n.y	= -(combo.column1[3] - combo.column1[0]);
		planes[FRUSTUM_PLANE_RIGHT].n.z	= -(combo.column2[3] - combo.column2[0]);
		planes[FRUSTUM_PLANE_RIGHT].d	= -(combo.column3[3] - combo.column3[0]);

		planes[FRUSTUM_PLANE_TOP].n.x	= -(combo.column0[3] - combo.column0[1]);
		planes[FRUSTUM_PLANE_TOP].n.y	= -(combo.column1[3] - combo.column1[1]);
		planes[FRUSTUM_PLANE_TOP].n.z	= -(combo.column2[3] - combo.column2[1]);
		planes[FRUSTUM_PLANE_TOP].d		= -(combo.column3[3] - combo.column3[1]);

		planes[FRUSTUM_PLANE_BOTTOM].n.x	= -(combo.column0[3] + combo.column0[1]);
		planes[FRUSTUM_PLANE_BOTTOM].n.y	= -(combo.column1[3] + combo.column1[1]);
		planes[FRUSTUM_PLANE_BOTTOM].n.z	= -(combo.column2[3] + combo.column2[1]);
		planes[FRUSTUM_PLANE_BOTTOM].d		= -(combo.column3[3] + combo.column3[1]);

		planes[FRUSTUM_PLANE_NEAR].n.x	= -(combo.column0[3] + combo.column0[2]);
		planes[FRUSTUM_PLANE_NEAR].n.y	= -(combo.column1[3] + combo.column1[2]);
		planes[FRUSTUM_PLANE_NEAR].n.z	= -(combo.column2[3] + combo.column2[2]);
		planes[FRUSTUM_PLANE_NEAR].d	= -(combo.column3[3] + combo.column3[2]);

		planes[FRUSTUM_PLANE_FAR].n.x	= -(combo.column0[3] - combo.column0[2]);
		planes[FRUSTUM_PLANE_FAR].n.y	= -(combo.column1[3] - combo.column1[2]);
		planes[FRUSTUM_PLANE_FAR].n.z	= -(combo.column2[3] - combo.column2[2]);
		planes[FRUSTUM_PLANE_FAR].d		= -(combo.column3[3] - combo.column3[2]);

		for(PxU32 i=0;i<6;i++)
			planes[i].normalize();
	}

	if(mBVH)
	{
		struct LocalCB : PxBVH::OverlapCallback
		{
			LocalCB(const CustomScene& scene) : mScene(scene)
			{
				mVisibles.reserve(10000);
			}

			virtual bool reportHit(PxU32 boundsIndex)
			{
				mVisibles.pushBack(boundsIndex);
				return true;
			}

			const CustomScene&	mScene;
			PxArray<PxU32>		mVisibles;
			LocalCB& operator=(const LocalCB&){return *this;}
		};

		LocalCB cb(*this);

#ifdef BENCHMARK_MODE
		unsigned long long time = __rdtsc();
#endif
		mBVH->cull(6, planes, cb);

		char buffer[256];
#ifdef BENCHMARK_MODE
		time = __rdtsc() - time;
		sprintf(buffer, "%d visible objects (%d)\n", cb.mVisibles.size(), int(time/1024));
#else
		sprintf(buffer, "%d visible objects\n", cb.mVisibles.size());
#endif
		const PxVec3 color(1.0f, 0.5f, 0.25f);
		for(PxU32 i=0;i<cb.mVisibles.size();i++)
		{
			const CustomScene::Object& obj = mObjects[cb.mVisibles[i]];
			Snippets::renderGeoms(1, &obj.mGeom, &obj.mPose, false, color);
		}

		if(1)
		{
			const PxU32 nbObjects = mBVH->getNbBounds();
			for(PxU32 i=0;i<nbObjects;i++)
				Snippets::DrawBounds(mBVH->getBounds()[i]);
		}

		Snippets::print(buffer);
	}
#endif
}


}

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

static CustomScene* gScene = NULL;

void renderScene()
{
	if(gScene)
		gScene->render();
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

	gScene = new CustomScene;

#ifdef BENCHMARK_MODE
	{
		SnippetUtils::BasicRandom rnd(42);

		PxVec3 v;
		const float coeff = 30.0f;
		for(int i=0;i<1000;i++)
		{
			rnd.unitRandomPt(v); v*=coeff;
			gScene->addGeom(PxBoxGeometry(PxVec3(1.0f, 2.0f, 0.5f)), PxTransform(v+PxVec3(0.0f, 0.0f, 0.0f)));
			rnd.unitRandomPt(v); v*=coeff;
			gScene->addGeom(PxSphereGeometry(1.5f), PxTransform(v+PxVec3(4.0f, 0.0f, 0.0f)));
			rnd.unitRandomPt(v); v*=coeff;
			gScene->addGeom(PxCapsuleGeometry(1.0f, 1.0f), PxTransform(v+PxVec3(-4.0f, 0.0f, 0.0f)));
			rnd.unitRandomPt(v); v*=coeff;
			gScene->addGeom(PxConvexMeshGeometry(gConvexMesh), PxTransform(v+PxVec3(0.0f, 0.0f, 4.0f)));
			rnd.unitRandomPt(v); v*=coeff;
			gScene->addGeom(PxTriangleMeshGeometry(gTriangleMesh), PxTransform(v+PxVec3(0.0f, 0.0f, -4.0f)));
		}
	}
#else
	{
		gScene->addGeom(PxBoxGeometry(PxVec3(1.0f, 2.0f, 0.5f)), PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));
		gScene->addGeom(PxSphereGeometry(1.5f), PxTransform(PxVec3(4.0f, 0.0f, 0.0f)));
		gScene->addGeom(PxCapsuleGeometry(1.0f, 1.0f), PxTransform(PxVec3(-4.0f, 0.0f, 0.0f)));
		gScene->addGeom(PxConvexMeshGeometry(gConvexMesh), PxTransform(PxVec3(0.0f, 0.0f, 4.0f)));
		gScene->addGeom(PxTriangleMeshGeometry(gTriangleMesh), PxTransform(PxVec3(0.0f, 0.0f, -4.0f)));
	}
#endif
	gScene->createBVH();
	
	initScene();
}

void stepPhysics(bool /*interactive*/)
{
}

void cleanupPhysics(bool /*interactive*/)
{
	releaseScene();

	PX_RELEASE(gScene);
	PX_RELEASE(gConvexMesh);
	PX_RELEASE(gFoundation);
	
	printf("SnippetFrustumQuery done.\n");
}

void keyPress(unsigned char /*key*/, const PxTransform& /*camera*/)
{
/*	if(key >= 1 && key <= gScenarioCount)
	{
		gScenario = key - 1;
		releaseScene();
		initScene();
	}

	if(key == 'r' || key == 'R')
	{
		releaseScene();
		initScene();
	}*/
}

int snippetMain(int, const char*const*)
{
	printf("Frustum query snippet.\n");

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




