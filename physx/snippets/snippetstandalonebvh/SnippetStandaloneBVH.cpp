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
// This snippet illustrates how to use a standalone PxBVH.
// It creates a small custom scene (no PxScene) and creates a PxBVH for the
// scene objects. The BVH is then used to raytrace the scene. The snippet
// also shows how to update the BVH after the objects have moved.
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

using namespace physx;

// Change this to use either refit the full BVH or just a subset of nodes
static const bool gUsePartialRefit = false;

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
			void	render();
			bool	raycast(const PxVec3& origin, const PxVec3& unitDir, float maxDist, PxGeomRaycastHit& hit)	const;
			void	updateObjects();

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

void CustomScene::updateObjects()
{
	static float time = 0.0f;
	time += 0.01f;

	if(gUsePartialRefit)
	{
		// This version is more efficient if you have to update a small subset of nodes

		const PxU32 nbObjects = mObjects.size();

		for(PxU32 i=0;i<nbObjects;i+=3)	// Don't update all objects
		{
	//		const float coeff = float(i)/float(nbObjects);
			const float coeff = float(i);

			Object& obj = mObjects[i];

			obj.mPose.p.x = sinf(time)*cosf(time+coeff)*10.0f;
			obj.mPose.p.y = sinf(time*1.17f)*cosf(time*1.17f+coeff)*2.0f;
			obj.mPose.p.z = sinf(time*0.33f)*cosf(time*0.33f+coeff)*10.0f;

			PxMat33 rotX;	PxSetRotX(rotX, time+coeff);
			PxMat33 rotY;	PxSetRotY(rotY, time*1.17f+coeff);
			PxMat33 rotZ;	PxSetRotZ(rotZ, time*0.33f+coeff);
			PxMat33 rot = rotX * rotY * rotZ;
			obj.mPose.q = PxQuat(rot);
			obj.mPose.q.normalize();

			PxBounds3 newBounds;
			PxGeometryQuery::computeGeomBounds(newBounds, obj.mGeom.any(), obj.mPose);

			mBVH->updateBounds(i, newBounds);
		}
		mBVH->partialRefit();
	}
	else
	{
		// This version is more efficient if you have to update all nodes

		PxBounds3* bounds = mBVH->getBoundsForModification();

		const PxU32 nbObjects = mObjects.size();
		for(PxU32 i=0;i<nbObjects;i++)
		{
	//		const float coeff = float(i)/float(nbObjects);
			const float coeff = float(i);

			Object& obj = mObjects[i];

			obj.mPose.p.x = sinf(time)*cosf(time+coeff)*10.0f;
			obj.mPose.p.y = sinf(time*1.17f)*cosf(time*1.17f+coeff)*2.0f;
			obj.mPose.p.z = sinf(time*0.33f)*cosf(time*0.33f+coeff)*10.0f;

			PxMat33 rotX;	PxSetRotX(rotX, time+coeff);
			PxMat33 rotY;	PxSetRotY(rotY, time*1.17f+coeff);
			PxMat33 rotZ;	PxSetRotZ(rotZ, time*0.33f+coeff);
			PxMat33 rot = rotX * rotY * rotZ;
			obj.mPose.q = PxQuat(rot);
			obj.mPose.q.normalize();

			PxGeometryQuery::computeGeomBounds(bounds[i], obj.mGeom.any(), obj.mPose);
		}

		mBVH->refit();
	}
}

void CustomScene::createBVH()
{
	const PxU32 nbObjects = mObjects.size();
	PxBounds3* bounds = new PxBounds3[nbObjects];
	for(PxU32 i=0;i<nbObjects;i++)
	{
		const Object& obj = mObjects[i];
		PxGeometryQuery::computeGeomBounds(bounds[i], obj.mGeom.any(), obj.mPose);
	}

	PxBVHDesc bvhDesc;
	bvhDesc.bounds.count = nbObjects;
	bvhDesc.bounds.data = bounds;
	bvhDesc.bounds.stride = sizeof(PxBounds3);
	bvhDesc.numPrimsPerLeaf = 1;
	mBVH = PxCreateBVH(bvhDesc);
	delete [] bounds;
}

	struct LocalCB : PxBVH::RaycastCallback
	{
		LocalCB(const CustomScene& scene, const PxVec3& origin, const PxVec3& dir, PxGeomRaycastHit& hit) :
			mScene	(scene),
			mHit	(hit),
			mOrigin	(origin),
			mDir	(dir),
			mStatus	(false)
		{
		}

		virtual bool reportHit(PxU32 boundsIndex, PxReal& distance)
		{
			const CustomScene::Object& obj = mScene.mObjects[boundsIndex];
			if(PxGeometryQuery::raycast(mOrigin, mDir, obj.mGeom.any(), obj.mPose, distance, PxHitFlag::eDEFAULT, 1, &mLocalHit, sizeof(PxGeomRaycastHit)))
			{
				if(mLocalHit.distance<distance)
				{
					distance = mLocalHit.distance;
					mHit = mLocalHit;
					mStatus = true;
				}
			}
			return true;
		}

		const CustomScene&	mScene;
		PxGeomRaycastHit&	mHit;
		PxGeomRaycastHit	mLocalHit;
		const PxVec3&		mOrigin;
		const PxVec3&		mDir;
		bool				mStatus;
		PX_NOCOPY(LocalCB)
	};

bool CustomScene::raycast(const PxVec3& origin, const PxVec3& unitDir, float maxDist, PxGeomRaycastHit& hit) const
{
	if(!mBVH)
		return false;

	LocalCB CB(*this, origin, unitDir, hit);
	mBVH->raycast(origin, unitDir, maxDist, CB);
	return CB.mStatus;
}

void CustomScene::render()
{
	updateObjects();

#ifdef RENDER_SNIPPET
	const PxVec3 color(1.0f, 0.5f, 0.25f);

	const PxU32 nbObjects = mObjects.size();
	for(PxU32 i=0;i<nbObjects;i++)
	{
		const Object& obj = mObjects[i];
		Snippets::renderGeoms(1, &obj.mGeom, &obj.mPose, false, color);
	}


	struct DrawBounds : PxBVH::TraversalCallback
	{
		virtual bool	visitNode(const PxBounds3& bounds)
		{
			Snippets::DrawBounds(bounds);
			return true;
		}

		virtual bool	reportLeaf(PxU32, const PxU32*)
		{
			return true;
		}
	}drawBounds;
	mBVH->traverse(drawBounds);

	const PxU32 screenWidth = Snippets::getScreenWidth();
	const PxU32 screenHeight = Snippets::getScreenHeight();
	Snippets::Camera* sCamera = Snippets::getCamera();
	const PxVec3 camPos = sCamera->getEye();
	const PxVec3 camDir = sCamera->getDir();
#if PX_DEBUG
	const PxU32	RAYTRACING_RENDER_WIDTH = 64;
	const PxU32	RAYTRACING_RENDER_HEIGHT = 64;
#else
	const PxU32	RAYTRACING_RENDER_WIDTH = 256;
	const PxU32	RAYTRACING_RENDER_HEIGHT = 256;
#endif

	const PxU32 textureWidth = RAYTRACING_RENDER_WIDTH;
	const PxU32 textureHeight = RAYTRACING_RENDER_HEIGHT;

	GLubyte* pixels = new GLubyte[textureWidth*textureHeight*4];

	const float fScreenWidth = float(screenWidth)/float(RAYTRACING_RENDER_WIDTH);
	const float fScreenHeight = float(screenHeight)/float(RAYTRACING_RENDER_HEIGHT);

	GLubyte* buffer = pixels;
	for(PxU32 j=0;j<RAYTRACING_RENDER_HEIGHT;j++)
	{
		const PxU32 yi = PxU32(fScreenHeight*float(j));
		for(PxU32 i=0;i<RAYTRACING_RENDER_WIDTH;i++)
		{
			const PxU32 xi = PxU32(fScreenWidth*float(i));
			const PxVec3 dir = Snippets::computeWorldRay(xi, yi, camDir);

			PxGeomRaycastHit hit;
			if(raycast(camPos, dir, 5000.0f, hit))
			{
				buffer[0] = 128+GLubyte(hit.normal.x*127.0f);
				buffer[1] = 128+GLubyte(hit.normal.y*127.0f);
				buffer[2] = 128+GLubyte(hit.normal.z*127.0f);
				buffer[3] = 255;
			}
			else
			{
				buffer[0] = 0;
				buffer[1] = 0;
				buffer[2] = 0;
				buffer[3] = 255;
			}

			buffer+=4;
		}
	}

	const GLuint texID = Snippets::CreateTexture(textureWidth, textureHeight, pixels, false);

#if PX_DEBUG
	Snippets::DisplayTexture(texID, 256, 10);
#else
	Snippets::DisplayTexture(texID, RAYTRACING_RENDER_WIDTH, 10);
#endif

	delete [] pixels;
	Snippets::ReleaseTexture(texID);
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
	gScene->addGeom(PxBoxGeometry(PxVec3(1.0f, 2.0f, 0.5f)), PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));
	gScene->addGeom(PxSphereGeometry(1.5f), PxTransform(PxVec3(4.0f, 0.0f, 0.0f)));
	gScene->addGeom(PxCapsuleGeometry(1.0f, 1.0f), PxTransform(PxVec3(-4.0f, 0.0f, 0.0f)));
	gScene->addGeom(PxConvexMeshGeometry(gConvexMesh), PxTransform(PxVec3(0.0f, 0.0f, 4.0f)));
	gScene->addGeom(PxTriangleMeshGeometry(gTriangleMesh), PxTransform(PxVec3(0.0f, 0.0f, -4.0f)));
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
	
	printf("SnippetStandaloneBVH done.\n");
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
	printf("Standalone BVH snippet.\n");

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




