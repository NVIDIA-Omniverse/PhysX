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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

// ****************************************************************************
// This snippet illustrates how to use a custom query system and the low-level
// cooking functions.
//
// This is similar to SnippetStandaloneBVH, but this time using a more
// advanced query system instead of a single PxBVH.
//
// This snippet illustrates a basic setup and a single type of query
// (raycast closest hit). For more queries see SnippetQuerySystemAllQueries.
//
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "GuQuerySystem.h"
#include "GuFactory.h"
#include "GuCooking.h"
#include "foundation/PxArray.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#ifdef RENDER_SNIPPET
	#include "../snippetrender/SnippetCamera.h"
	#include "../snippetrender/SnippetRender.h"
#endif

using namespace physx;
using namespace Gu;

// The query system can compute the bounds for you, or you can do it manually. Manual bounds
// computation can be useful for specific effects (like using temporal bounds) or if there is
// an external system that already computed the bounds and recomputing them would be a waste.
// Automatic bounds computation is easier to use and less error-prone.
static const bool gManualBoundsComputation = false;

// The query system can delay internal transform/bounds updates or use the data immediately.
// Delaying updates can be faster due to batching, but it uses more memory to store the data until
// the actual update happens. Delaying the update can also serve as a double-buffering mechanism,
// allowing one to query the old state of the system until a later user-controlled point in time.
static const bool gUseDelayedUpdates = true;

// Bounds in the query system can be inflated a bit to fight numerical inaccuracy errors that can happen
// when a ray or a query-volume just touches the bounds. Users can manually inflate bounds or let the
// system do it. Because the system can compute the bounds automatically, it is necessary to let it know
// about the inflation value.
static const float gBoundsInflation = 0.001f;

#define MAX_NB_OBJECTS	32

namespace
{
	class CustomScene : public Adapter
	{
		public:
			CustomScene();
			~CustomScene()	{}

			// Adapter
			virtual	const PxGeometry&	getGeometry(const PrunerPayload& payload)	const;
			//~Adapter

			void	release();
			void	addGeom(const PxGeometry& geom, const PxTransform& pose);
			void	render();
			bool	raycast(const PxVec3& origin, const PxVec3& unitDir, float maxDist, PxGeomRaycastHit& hit)	const;
			void	updateObjects();

			struct Object
			{
				PxGeometryHolder	mGeom;
				ActorShapeData		mData;
			};

			PxU32			mNbObjects;
			Object			mObjects[MAX_NB_OBJECTS];
			QuerySystem*	mQuerySystem;
			PxU32			mPrunerIndex;
	};

static const PxGeometry& getGeometryFromPayload(const PrunerPayload& payload)
{
	const CustomScene* cs = reinterpret_cast<const CustomScene*>(payload.data[1]);
	return cs->mObjects[PxU32(payload.data[0])].mGeom.any();
}

const PxGeometry& CustomScene::getGeometry(const PrunerPayload& payload) const
{
	// This function is called by the system to compute bounds. It will never be
	// called if 'gManualBoundsComputation' is true.
	PX_ASSERT(!gManualBoundsComputation);

	return getGeometryFromPayload(payload);
}

void CustomScene::release()
{
	PX_DELETE(mQuerySystem);
	PX_DELETE_THIS;
}

CustomScene::CustomScene() : mNbObjects(0)
{
	// The contextID is a parameter sent to the profiler to identify the owner of a profile event.
	// In PhysX this is usually the PxScene pointer, and it is used by PVD to group together all profile events of a given scene.
	// We do not have a PxScene object here so we can put an arbitrary value there. The value will only be used by
	// PVD to display profiling results.
	const PxU64 contextID = PxU64(this);

	// First we create a query system and give it an adapter. The adapter is used to retrieve the geometry
	// of objects in the system. This is needed when the system automatically computes bounds for users.
	// The geometry is not stored directly in the system to avoid duplication, and make it easy to reuse
	// the system with PxShape-based objects. In this snippet the system will call the
	// 'CustomScene::getGeometry' function above to fetch an object's geometry.
	mQuerySystem = PX_NEW(QuerySystem)(contextID, gBoundsInflation, *this);

	// PhysX uses a hardcoded number of pruners (one for static objects, one for dynamic objects,
	// and an optional one for compound). The query system here is more flexible and supports an
	// arbitrary number of pruners, which have to be created by users and added to the system
	// explicitly. In this snippet we just use a single pruner of a chosen type:
	Pruner* pruner = createAABBPruner(contextID, true, COMPANION_PRUNER_INCREMENTAL, BVH_SPLATTER_POINTS, 4);

	// Then we add it to the query system, which takes ownership of the object (it will delete
	// the pruner when the query system is released). Each pruner is given an index by the
	// system, used in 'addPrunerShape' to identify which pruner each object is added to.
	mPrunerIndex = mQuerySystem->addPruner(pruner, 0);
}

void CustomScene::addGeom(const PxGeometry& geom, const PxTransform& pose)
{
	PX_ASSERT(mQuerySystem);

	// The query system operates on anonymous 'payloads', which are basically glorified user-data.
	// We put here what we need to implement 'CustomScene::getGeometry'.
	PrunerPayload payload;
	payload.data[0] = mNbObjects;	// This will be the index of our new object, see below.
	payload.data[1] = size_t(this);

	// We store the geometry first, because in automatic mode the 'CustomScene::getGeometry' function
	// will be called by 'addPrunerShape' below, so we need the geometry to be properly setup first.
	Object& obj = mObjects[mNbObjects];
	obj.mGeom.storeAny(geom);

	// The query system manages a built-in timestamp for static objects, which is used by external
	// sub-systems like character controllers to invalidate their caches. This is not needed in
	// this snippet so any value works here.
	const bool isDynamic = true;

	// In automatic mode the system will compute the bounds for us.
	// In manual mode we compute the bounds first and pass them to the system.
	//
	// Note that contrary to bounds, the transforms are duplicated and stored within the query
	// system. In this snippet we take advantage of this by not storing the poses anywhere
	// else - we will retrieve them from the query system when we need them. In a more complex
	// example this could create a duplication of the transforms, but this 'double-buffering' is
	// actually done on purpose to make sure the query system can run in parallel to the app's
	// code when/if it modifies the objects' poses. The poses are double-buffered but the geometries
	// are not, because poses of dynamic objects change each frame while geometries do not.
	if(gManualBoundsComputation)
	{
		PxBounds3 bounds;
		PxGeometryQuery::computeGeomBounds(bounds, geom, pose, 0.0f, 1.0f + gBoundsInflation);
		obj.mData = mQuerySystem->addPrunerShape(payload, mPrunerIndex, isDynamic, pose, &bounds);
	}
	else
	{
		obj.mData = mQuerySystem->addPrunerShape(payload, mPrunerIndex, isDynamic, pose, NULL);
	}

	mNbObjects++;
}

void CustomScene::updateObjects()
{
	if(!mQuerySystem)
		return;

	static float time = 0.0f;
	time += 0.01f;

	const PxU32 nbObjects = mNbObjects;
	for(PxU32 i=0;i<nbObjects;i++)
	{
//		const float coeff = float(i)/float(nbObjects);
		const float coeff = float(i);

		// Compute an arbitrary new pose for this object
		PxTransform pose;
		{
			pose.p.x = sinf(time)*cosf(time+coeff)*10.0f;
			pose.p.y = sinf(time*1.17f)*cosf(time*1.17f+coeff)*2.0f;
			pose.p.z = sinf(time*0.33f)*cosf(time*0.33f+coeff)*10.0f;

			PxMat33 rotX;	PxSetRotX(rotX, time+coeff);
			PxMat33 rotY;	PxSetRotY(rotY, time*1.17f+coeff);
			PxMat33 rotZ;	PxSetRotZ(rotZ, time*0.33f+coeff);
			PxMat33 rot = rotX * rotY * rotZ;
			pose.q = PxQuat(rot);
			pose.q.normalize();
		}

		// Now we're going to tell the query system about it. It is important to
		// understand that updating the query system is a multiple-steps process:
		// a) we need to store the new transform and/or bounds in the system.
		// b) the internal data-structures have to be updated to take a) into account.
		//
		// For example if the internal data-structure is an AABB-tree (but it doesn't
		// have to be, this is pruner-dependent) then (a) would be writing the new bounds
		// value in a leaf node, while (b) would be refitting the tree accordingly. Or
		// in a different implementation (a) could be storing the bounds in a linear
		// array and (b) could be rebuilding the tree from scratch.
		//
		// The important point is that there is a per-object update (a), and a global
		// per-pruner update (b). It would be very inefficient to do (a) and (b)
		// sequentially for each object (we don't want to rebuild the tree more than
		// once for example) so the update process is separated into two clearly
		// distinct phases. The phase we're dealing with here is (a), via the
		// 'updatePrunerShape' function.

		const Object& obj = mObjects[i];

		if(gManualBoundsComputation)
		{
			PxBounds3 bounds;
			PxGeometryQuery::computeGeomBounds(bounds, obj.mGeom.any(), pose, 0.0f, 1.0f + gBoundsInflation);
			mQuerySystem->updatePrunerShape(obj.mData, !gUseDelayedUpdates, pose, &bounds);
		}
		else
		{
			// Note: in this codepath the system will compute the bounds automatically:
			// - if 'immediately' is true, the system will call back 'CustomScene::getGeometry'
			//   during the 'updatePrunerShape' call.
			// - otherwise it will call 'CustomScene::getGeometry' later during the
			//   'mQuerySystem->update' call (below).
			mQuerySystem->updatePrunerShape(obj.mData, !gUseDelayedUpdates, pose, NULL);
		}
	}

	// This is the per-pruner update (b) we mentioned just above. It commits the
	// updates we just made and reflects them into the internal data-structures.
	//
	// Note that this function must also be called after adding & removing objects.
	//
	// Finally, this function also manages the incremental rebuild of internal structures
	// if the 'buildStep' parameter is true. This is a convenience function that does
	// everything needed in a single call. If all the updates to the system happen in
	//  a single place, like in this snippet, then this is everything you need.
	mQuerySystem->update(true, true);
}

namespace
{
	struct CustomPrunerFilterCallback : public PrunerFilterCallback
	{
		virtual	const PxGeometry*	validatePayload(const PrunerPayload& payload, PxHitFlags& /*hitFlags*/)
		{
			return &getGeometryFromPayload(payload);
		}
	};
}

static CustomPrunerFilterCallback	gFilterCallback;
static CachedFuncs					gCachedFuncs;

bool CustomScene::raycast(const PxVec3& origin, const PxVec3& unitDir, float maxDist, PxGeomRaycastHit& hit) const
{
	if(!mQuerySystem)
		return false;

	// In this snippet our update loop is simple:
	//
	// - update all objects ('mQuerySystem->updatePrunerShape')
	// - call 'mQuerySystem->update'
	// - then perform raycasts
	//
	// Because we call 'mQuerySystem->update' just before performing the raycasts,
	// we guarantee that the internal data-structures are up-to-date and we can
	// immediately call 'mQuerySystem->raycast'.
	//
	// However things can become more complicated:
	// - sometimes 'mQuerySystem->updatePrunerShape' is immediately followed by a
	//   raycast (ex: spawning an object, using a raycast to locate it on the map, repeat)
	// - sometimes raycasts happen from multiple threads in no clear order
	//
	// In these cases the following call to 'commitUpdates' is needed to commit the
	// minimal amount of updates to the system, so that correct raycast results are
	// guaranteed. In particular this call omits the 'build step' performed in the
	// main 'mQuerySystem->update' function (users should have one build step per
	// frame). This function is also thread-safe, i.e. you can call commitUpdates
	// and raycasts from multiple threads (contrary to 'mQuerySystem->update').
	//
	// Note that in PxScene::raycast() this call is always executed. But this custom
	// query system lets users control when & where it happens. The system becomes
	// more flexible, but puts more burden on users.

	if(0)
		mQuerySystem->commitUpdates();

	// After that the raycast code itself is rather simple.

	DefaultPrunerRaycastClosestCallback CB(gFilterCallback, gCachedFuncs.mCachedRaycastFuncs, origin, unitDir, maxDist, PxHitFlag::eDEFAULT);

	mQuerySystem->raycast(origin, unitDir, maxDist, CB, NULL);

	if(CB.mFoundHit)
		hit = CB.mClosestHit;

	return CB.mFoundHit;
}

void CustomScene::render()
{
	updateObjects();

#ifdef RENDER_SNIPPET
	const PxVec3 color(1.0f, 0.5f, 0.25f);

	const PxU32 nbObjects = mNbObjects;
	for(PxU32 i=0;i<nbObjects;i++)
	{
		const Object& obj = mObjects[i];

		PrunerPayloadData ppd;
		mQuerySystem->getPayloadData(obj.mData, &ppd);

		Snippets::DrawBounds(*ppd.mBounds);

		Snippets::renderGeoms(1, &obj.mGeom, ppd.mTransform, false, color);
	}

	//mQuerySystem->visualize(true, true, PxRenderOutput)

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

	{
		// Contrary to PxBVH, the Gu-level query system does not contain any built-in "SIMD guard".
		// It is up to users to make sure the SIMD control word is properly setup before calling
		// these low-level functions. See also OPTIM_SKIP_INTERNAL_SIMD_GUARD in SnippetPathTracing.
		PX_SIMD_GUARD

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

static CustomScene* gScene = NULL;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxConvexMesh*			gConvexMesh = NULL;
static PxTriangleMesh*			gTriangleMesh = NULL;

void initPhysics(bool /*interactive*/)
{
	// We first initialize the PhysX libs we need to create PxGeometry-based objects.
	// That is only Foundation, since we'll use the low-level cooking functions here.
	// (We don't need to initialize the cooking library).
	// Also note how we are not going to use a PxScene in this snippet, and in fact
	// we're not going to need anything from the main PhysX_xx.dll, we only use
	// PhysXCommon_xx.dll.
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	// Cook one convex mesh and one triangle mesh used in this snippet
	{
		// Some cooking parameters will have an impact on the performance of our queries,
		// some will not. Generally speaking midphase-related parameters are still important
		// here, while anything related to contact-generation can be disabled.
		const PxTolerancesScale scale;
		PxCookingParams params(scale);
		params.midphaseDesc.setToDefault(PxMeshMidPhase::eBVH34);
	//	params.midphaseDesc.mBVH34Desc.quantized = false;
		params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
		params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;

		// The convex mesh
		{
			const PxF32 width = 3.0f;
			const PxF32 radius = 1.0f;

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

			PxConvexMeshDesc convexDesc;
			convexDesc.points.count		= 32;
			convexDesc.points.stride	= sizeof(PxVec3);
			convexDesc.points.data		= points;
			convexDesc.flags			= PxConvexFlag::eCOMPUTE_CONVEX;
			gConvexMesh = immediateCooking::createConvexMesh(params, convexDesc);
		}

		// The triangle mesh
		{
			PxTriangleMeshDesc meshDesc;
			meshDesc.points.count		= SnippetUtils::Bunny_getNbVerts();
			meshDesc.points.stride		= sizeof(PxVec3);
			meshDesc.points.data		= SnippetUtils::Bunny_getVerts();
			meshDesc.triangles.count	= SnippetUtils::Bunny_getNbFaces();
			meshDesc.triangles.stride	= sizeof(int)*3;
			meshDesc.triangles.data		= SnippetUtils::Bunny_getFaces();

			gTriangleMesh = immediateCooking::createTriangleMesh(params, meshDesc);
		}
	}

	// Create our custom scene and populate it with some custom PxGeometry-based objects
	{
		gScene = new CustomScene;
		gScene->addGeom(PxBoxGeometry(PxVec3(1.0f, 2.0f, 0.5f)), PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));
		gScene->addGeom(PxSphereGeometry(1.5f), PxTransform(PxVec3(4.0f, 0.0f, 0.0f)));
		gScene->addGeom(PxCapsuleGeometry(1.0f, 1.0f), PxTransform(PxVec3(-4.0f, 0.0f, 0.0f)));
		gScene->addGeom(PxConvexMeshGeometry(gConvexMesh), PxTransform(PxVec3(0.0f, 0.0f, 4.0f)));
		gScene->addGeom(PxTriangleMeshGeometry(gTriangleMesh), PxTransform(PxVec3(0.0f, 0.0f, -4.0f)));
	}
}

void renderScene()
{
	if(gScene)
		gScene->render();
}

void stepPhysics(bool /*interactive*/)
{
}

void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gTriangleMesh);
	PX_RELEASE(gConvexMesh);
	PX_RELEASE(gFoundation);

	printf("SnippetStandaloneQuerySystem done.\n");
}

void keyPress(unsigned char /*key*/, const PxTransform& /*camera*/)
{
}

int snippetMain(int, const char*const*)
{
	printf("Standalone Query System snippet.\n");

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




