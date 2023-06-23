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
// This snippet demonstrates all queries supported by the low-level query system.
// Please get yourself familiar with SnippetStandaloneQuerySystem first.
// ****************************************************************************

#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "GuQuerySystem.h"
#include "GuFactory.h"
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
#ifdef RENDER_SNIPPET
	using namespace Snippets;
#endif

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxConvexMesh*			gConvexMesh = NULL;
static PxTriangleMesh*			gTriangleMesh = NULL;

static const bool				gManualBoundsComputation = false;
static const bool				gUseDelayedUpdates = true;
static const float				gBoundsInflation = 0.001f;

static bool						gPause = false;
static bool						gOneFrame = false;

enum QueryScenario
{
	RAYCAST_CLOSEST,
	RAYCAST_ANY,
	RAYCAST_MULTIPLE,

	SWEEP_CLOSEST,
	SWEEP_ANY,
	SWEEP_MULTIPLE,

	OVERLAP_ANY,
	OVERLAP_MULTIPLE,

	NB_SCENES
};

static QueryScenario gSceneIndex = RAYCAST_CLOSEST;

// Tweak number of created objects per scene
static const PxU32 gFactor[NB_SCENES] = { 2, 1, 2, 2, 1, 2, 1, 2 };
// Tweak amplitude of created objects per scene
static const float gAmplitude[NB_SCENES] = { 4.0f, 4.0f, 4.0f, 4.0f, 8.0f, 4.0f, 4.0f, 4.0f };

static const PxVec3 gCamPos[NB_SCENES] = {
	PxVec3(-2.199769f, 3.448516f, 10.943871f),
	PxVec3(-2.199769f, 3.448516f, 10.943871f),
	PxVec3(-2.199769f, 3.448516f, 10.943871f),

	PxVec3(-2.199769f, 3.448516f, 10.943871f),
	PxVec3(-3.404853f, 4.865191f, 17.692263f),
	PxVec3(-2.199769f, 3.448516f, 10.943871f),

	PxVec3(-2.199769f, 3.448516f, 10.943871f),
	PxVec3(-2.199769f, 3.448516f, 10.943871f),
};

static const PxVec3 gCamDir[NB_SCENES] = {
	PxVec3(0.172155f, -0.202382f, -0.964056f),
	PxVec3(0.172155f, -0.202382f, -0.964056f),
	PxVec3(0.172155f, -0.202382f, -0.964056f),

	PxVec3(0.172155f, -0.202382f, -0.964056f),
	PxVec3(0.172155f, -0.202382f, -0.964056f),
	PxVec3(0.172155f, -0.202382f, -0.964056f),

	PxVec3(0.172155f, -0.202382f, -0.964056f),
	PxVec3(0.172155f, -0.202382f, -0.964056f),
};

#define MAX_NB_OBJECTS	32

///////////////////////////////////////////////////////////////////////////////

// The following functions determine how we use the pruner payloads in this snippet

static PX_FORCE_INLINE void setupPayload(PrunerPayload& payload, PxU32 objectIndex, const PxGeometryHolder* gh)
{
	payload.data[0] = objectIndex;
	payload.data[1] = size_t(gh);
}

static PX_FORCE_INLINE PxU32 getObjectIndexFromPayload(const PrunerPayload& payload)
{
	return PxU32(payload.data[0]);
}

static PX_FORCE_INLINE const PxGeometry& getGeometryFromPayload(const PrunerPayload& payload)
{
	const PxGeometryHolder* gh = reinterpret_cast<const PxGeometryHolder*>(payload.data[1]);
	return gh->any();
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	struct CustomRaycastHit : PxGeomRaycastHit
	{
		PxU32	mObjectIndex;
	};

	struct CustomSweepHit : PxGeomSweepHit
	{
		PxU32	mObjectIndex;
	};

	class CustomScene : public Adapter
	{
		public:
			CustomScene();
			~CustomScene()	{}

			// Adapter
			virtual	const PxGeometry&	getGeometry(const PrunerPayload& payload)	const;
			//~Adapter

			void	release();
			void	addGeom(const PxGeometry& geom, const PxTransform& pose, bool isDynamic);
			void	render();
			void	updateObjects();
			void	runQueries();

			bool	raycastClosest(const PxVec3& origin, const PxVec3& unitDir, float maxDist, CustomRaycastHit& hit)				const;
			bool	raycastAny(const PxVec3& origin, const PxVec3& unitDir, float maxDist)											const;
			bool	raycastMultiple(const PxVec3& origin, const PxVec3& unitDir, float maxDist, PxArray<CustomRaycastHit>& hits)	const;

			bool	sweepClosest(const PxGeometry& geom, const PxTransform& pose, const PxVec3& unitDir, float maxDist, CustomSweepHit& hit)			const;
			bool	sweepAny(const PxGeometry& geom, const PxTransform& pose, const PxVec3& unitDir, float maxDist)										const;
			bool	sweepMultiple(const PxGeometry& geom, const PxTransform& pose, const PxVec3& unitDir, float maxDist, PxArray<CustomSweepHit>& hits)	const;

			bool	overlapAny(const PxGeometry& geom, const PxTransform& pose)								const;
			bool	overlapMultiple(const PxGeometry& geom, const PxTransform& pose, PxArray<PxU32>& hits)	const;

			struct Object
			{
				PxGeometryHolder	mGeom;
				ActorShapeData		mData;
				PxVec3				mTouchedColor;
				bool				mTouched;
			};

			PxU32			mNbObjects;
			Object			mObjects[MAX_NB_OBJECTS];
			QuerySystem*	mQuerySystem;
			PxU32			mPrunerIndex;
	};

const PxGeometry& CustomScene::getGeometry(const PrunerPayload& payload) const
{
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
	const PxU64 contextID = PxU64(this);
	mQuerySystem = PX_NEW(QuerySystem)(contextID, gBoundsInflation, *this);
	Pruner* pruner = createAABBPruner(contextID, true, COMPANION_PRUNER_INCREMENTAL, BVH_SPLATTER_POINTS, 4);
	mPrunerIndex = mQuerySystem->addPruner(pruner, 0);

	const PxU32 nb = gFactor[gSceneIndex];
	for(PxU32 i=0;i<nb;i++)
	{
		addGeom(PxBoxGeometry(PxVec3(1.0f, 2.0f, 0.5f)), PxTransform(PxVec3(0.0f, 0.0f, 0.0f)), true);
		addGeom(PxSphereGeometry(1.5f), PxTransform(PxVec3(4.0f, 0.0f, 0.0f)), true);
		addGeom(PxCapsuleGeometry(1.0f, 1.0f), PxTransform(PxVec3(-4.0f, 0.0f, 0.0f)), true);
		addGeom(PxConvexMeshGeometry(gConvexMesh), PxTransform(PxVec3(0.0f, 0.0f, 4.0f)), true);
		addGeom(PxTriangleMeshGeometry(gTriangleMesh), PxTransform(PxVec3(0.0f, 0.0f, -4.0f)), true);
	}

#ifdef RENDER_SNIPPET
	Camera* camera = getCamera();
	camera->setPose(gCamPos[gSceneIndex], gCamDir[gSceneIndex]);
#endif
}

void CustomScene::addGeom(const PxGeometry& geom, const PxTransform& pose, bool isDynamic)
{
	PX_ASSERT(mQuerySystem);

	Object& obj = mObjects[mNbObjects];
	obj.mGeom.storeAny(geom);

	PrunerPayload payload;
	setupPayload(payload, mNbObjects, &obj.mGeom);

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

	if(gPause && !gOneFrame)
	{
		mQuerySystem->update(true, true);
		return;
	}
	gOneFrame = false;

	static float time = 0.0f;
	time += 0.005f;

	const PxU32 nbObjects = mNbObjects;
	for(PxU32 i=0;i<nbObjects;i++)
	{
		const Object& obj = mObjects[i];

		if(!getDynamic(getPrunerInfo(obj.mData)))
			continue;

//		const float coeff = float(i)/float(nbObjects);
		const float coeff = float(i);

		const float amplitude = gAmplitude[gSceneIndex];

		// Compute an arbitrary new pose for this object
		PxTransform pose;
		{
			const float phase = PxPi * 2.0f * float(i)/float(nbObjects);
			pose.p.z = 0.0f;
			pose.p.y = sinf(phase+time*1.17f)*amplitude;
			pose.p.x = cosf(phase+time*1.17f)*amplitude;

			PxMat33 rotX;	PxSetRotX(rotX, time+coeff);
			PxMat33 rotY;	PxSetRotY(rotY, time*1.17f+coeff);
			PxMat33 rotZ;	PxSetRotZ(rotZ, time*0.33f+coeff);
			PxMat33 rot = rotX * rotY * rotZ;
			pose.q = PxQuat(rot);
			pose.q.normalize();
		}

		if(gManualBoundsComputation)
		{
			PxBounds3 bounds;
			PxGeometryQuery::computeGeomBounds(bounds, obj.mGeom.any(), pose, 0.0f, 1.0f + gBoundsInflation);
			mQuerySystem->updatePrunerShape(obj.mData, !gUseDelayedUpdates, pose, &bounds);
		}
		else
		{
			mQuerySystem->updatePrunerShape(obj.mData, !gUseDelayedUpdates, pose, NULL);
		}
	}

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

///////////////////////////////////////////////////////////////////////////////

// Custom queries. It is not mandatory to use e.g. different raycast queries for different usages - one could just use the same code
// for everything. But it is -possible- to do so, the system is flexible enough to let users decide what to do.

// Common raycast usage, returns the closest touched object (single hit).
bool CustomScene::raycastClosest(const PxVec3& origin, const PxVec3& unitDir, float maxDist, CustomRaycastHit& hit) const
{
	DefaultPrunerRaycastClosestCallback CB(gFilterCallback, gCachedFuncs.mCachedRaycastFuncs, origin, unitDir, maxDist, PxHitFlag::eDEFAULT);

	mQuerySystem->raycast(origin, unitDir, maxDist, CB, NULL);

	if(CB.mFoundHit)
	{
		static_cast<PxGeomRaycastHit&>(hit) = CB.mClosestHit;
		hit.mObjectIndex = getObjectIndexFromPayload(CB.mClosestPayload);
	}

	return CB.mFoundHit;
}

///////////////////////////////////////////////////////////////////////////////

// "Shadow feeler" usage, returns boolean result, early exits as soon as an impact is found.
bool CustomScene::raycastAny(const PxVec3& origin, const PxVec3& unitDir, float maxDist) const
{
	DefaultPrunerRaycastAnyCallback CB(gFilterCallback, gCachedFuncs.mCachedRaycastFuncs, origin, unitDir, maxDist);

	mQuerySystem->raycast(origin, unitDir, maxDist, CB, NULL);

	return CB.mFoundHit;
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	// Beware, there is something a bit subtle here. The query can return multiple hits per query and/or
	// multiple hits per object. For example a raycast against a scene can touch multiple simple objects,
	// like e.g. multiple spheres, and return the set of touched spheres to users. But a single raycast
	// against a complex object like a mesh can also return multiple hits, against multiple triangles of
	// that same mesh. There are use cases for all of these, and it is up to users to decide what they
	// need. In this example we do "multiple hits per query", but a single hit per mesh. That is, we do
	// not use PxHitFlag::eMESH_MULTIPLE. That is why our CustomRaycastCallback only has a single
	// PxGeomRaycastHit member (mLocalHit).
	struct CustomRaycastMultipleCallback : public DefaultPrunerRaycastCallback
	{
		// This mandatory local hit structure is where DefaultPrunerRaycastCallback writes its
		// temporary hit when traversing the pruners.
		PxGeomRaycastHit			mLocalHit;

		// This is the user-provided array where we'll write our results.
		PxArray<CustomRaycastHit>&	mHits;

		CustomRaycastMultipleCallback(PxArray<CustomRaycastHit>& hits, PrunerFilterCallback& filterCB, const GeomRaycastTable& funcs, const PxVec3& origin, const PxVec3& dir, float distance) :
			DefaultPrunerRaycastCallback(filterCB, funcs, origin, dir, distance, 1, &mLocalHit, PxHitFlag::eDEFAULT, false),
			mHits						(hits)	{}

		// So far this looked similar to the DefaultPrunerRaycastClosestCallback code. But we customize the behavior in
		// the following function.
		virtual	bool	reportHits(const PrunerPayload& payload, PxU32 nbHits, PxGeomRaycastHit* hits)
		{
			// Because we didn't use PxHitFlag::eMESH_MULTIPLE we should only be called for a single hit
			PX_ASSERT(nbHits==1);
			PX_UNUSED(nbHits);

			// We process each hit the way we processed the closest hit at the end of CustomScene::raycastClosest
			CustomRaycastHit customHit;
			static_cast<PxGeomRaycastHit&>(customHit) = hits[0];
			customHit.mObjectIndex = getObjectIndexFromPayload(payload);

			// Then we gather the new hit in our user-provided array. This is written for clarity, not performance.
			// Here we could instead call a user-provided callback.
			mHits.pushBack(customHit);

			// We return false to tell the system to ignore that hit now (otherwise the code would shrink the ray etc).
			// This is also the way one does "post filtering".
			return false;
		}

		PX_NOCOPY(CustomRaycastMultipleCallback)
	};
}

// Generic usage, returns all hits touched by the ray, it's up to users to process them / sort them / etc.
// We're using a PxArray here but it's an arbitrary choice, one could also return hits via a callback, etc.
bool CustomScene::raycastMultiple(const PxVec3& origin, const PxVec3& unitDir, float maxDist, PxArray<CustomRaycastHit>& hits) const
{
	CustomRaycastMultipleCallback CB(hits, gFilterCallback, gCachedFuncs.mCachedRaycastFuncs, origin, unitDir, maxDist);

	mQuerySystem->raycast(origin, unitDir, maxDist, CB, NULL);

	return hits.size()!=0;
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	// The sweep code is more complicated because the swept geometry is arbitrary (to some extent), and for performance reason there is no
	// generic sweep callback available operating on an anonymous PxGeometry. So the code below is what you can do instead.
	// It can be simplified in your app though if you know you only need to sweep one kind of geometry (say sphere sweeps).
	template<class CallbackT>
	static bool _sweepClosestT(CustomSweepHit& hit, const QuerySystem& sqs, const PxGeometry& geom, const PxTransform& pose, const PxVec3& unitDir, float maxDist)
	{
		const ShapeData queryVolume(geom, pose, 0.0f);

		CallbackT pcb(gFilterCallback, gCachedFuncs.mCachedSweepFuncs, geom, pose, queryVolume, unitDir, maxDist, PxHitFlag::eDEFAULT | PxHitFlag::ePRECISE_SWEEP, false);

		sqs.sweep(queryVolume, unitDir, pcb.mClosestHit.distance, pcb, NULL);

		if(pcb.mFoundHit)
		{
			static_cast<PxGeomSweepHit&>(hit) = pcb.mClosestHit;
			hit.mObjectIndex = getObjectIndexFromPayload(pcb.mClosestPayload);
		}
		return pcb.mFoundHit;
	}
}

// Common sweep usage, returns the closest touched object (single hit).
bool CustomScene::sweepClosest(const PxGeometry& geom, const PxTransform& pose, const PxVec3& unitDir, float maxDist, CustomSweepHit& hit) const
{
	switch(PxU32(geom.getType()))
	{
		case PxGeometryType::eSPHERE:			{ return _sweepClosestT<DefaultPrunerSphereSweepCallback>(hit, *mQuerySystem, geom, pose, unitDir, maxDist);	}
		case PxGeometryType::eCAPSULE:			{ return _sweepClosestT<DefaultPrunerCapsuleSweepCallback>(hit, *mQuerySystem, geom, pose, unitDir, maxDist);	}
		case PxGeometryType::eBOX:				{ return _sweepClosestT<DefaultPrunerBoxSweepCallback>(hit, *mQuerySystem, geom, pose, unitDir, maxDist);		}
		case PxGeometryType::eCONVEXMESH:		{ return _sweepClosestT<DefaultPrunerConvexSweepCallback>(hit, *mQuerySystem, geom, pose, unitDir, maxDist);	}
		default:								{ PX_ASSERT(0); return false;																					}
	}
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	// This could easily be combined with _sweepClosestT to make the code shorter.
	template<class CallbackT>
	static bool _sweepAnyT(const QuerySystem& sqs, const PxGeometry& geom, const PxTransform& pose, const PxVec3& unitDir, float maxDist)
	{
		const ShapeData queryVolume(geom, pose, 0.0f);

		CallbackT pcb(gFilterCallback, gCachedFuncs.mCachedSweepFuncs, geom, pose, queryVolume, unitDir, maxDist, PxHitFlag::eMESH_ANY | PxHitFlag::ePRECISE_SWEEP, true);

		sqs.sweep(queryVolume, unitDir, pcb.mClosestHit.distance, pcb, NULL);

		return pcb.mFoundHit;
	}
}

// Returns boolean result, early exits as soon as an impact is found.
bool CustomScene::sweepAny(const PxGeometry& geom, const PxTransform& pose, const PxVec3& unitDir, float maxDist) const
{
	switch(PxU32(geom.getType()))
	{
		case PxGeometryType::eSPHERE:			{ return _sweepAnyT<DefaultPrunerSphereSweepCallback>(*mQuerySystem, geom, pose, unitDir, maxDist);		}
		case PxGeometryType::eCAPSULE:			{ return _sweepAnyT<DefaultPrunerCapsuleSweepCallback>(*mQuerySystem, geom, pose, unitDir, maxDist);	}
		case PxGeometryType::eBOX:				{ return _sweepAnyT<DefaultPrunerBoxSweepCallback>(*mQuerySystem, geom, pose, unitDir, maxDist);		}
		case PxGeometryType::eCONVEXMESH:		{ return _sweepAnyT<DefaultPrunerConvexSweepCallback>(*mQuerySystem, geom, pose, unitDir, maxDist);		}
		default:								{ PX_ASSERT(0); return false;																			}
	}
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	// We use a similar strategy as for raycasts. Please refer to CustomRaycastMultipleCallback above.
	template<class BaseCallbackT>
	struct CustomSweepMultipleCallback : public BaseCallbackT
	{
		PxArray<CustomSweepHit>&	mHits;

		CustomSweepMultipleCallback(PxArray<CustomSweepHit>& hits, PrunerFilterCallback& filterCB, const GeomSweepFuncs& funcs,
									const PxGeometry& geom, const PxTransform& pose, const ShapeData& queryVolume, const PxVec3& dir, float distance) :
			BaseCallbackT	(filterCB, funcs, geom, pose, queryVolume, dir, distance, PxHitFlag::eDEFAULT|PxHitFlag::ePRECISE_SWEEP, false),
			mHits			(hits)	{}

		virtual	bool	reportHit(const PrunerPayload& payload, PxGeomSweepHit& hit)
		{
			CustomSweepHit customHit;
			static_cast<PxGeomSweepHit&>(customHit) = hit;
			customHit.mObjectIndex = getObjectIndexFromPayload(payload);
			mHits.pushBack(customHit);
			return false;
		}

		PX_NOCOPY(CustomSweepMultipleCallback)
	};

	// Previous template was customizing the callback for multiple hits, this template customizes the previous template for different geoms.
	template<class CallbackT>
	static bool _sweepMultipleT(PxArray<CustomSweepHit>& hits, const QuerySystem& sqs, const PxGeometry& geom, const PxTransform& pose, const PxVec3& unitDir, float maxDist)
	{
		const ShapeData queryVolume(geom, pose, 0.0f);

		CustomSweepMultipleCallback<CallbackT> pcb(hits, gFilterCallback, gCachedFuncs.mCachedSweepFuncs, geom, pose, queryVolume, unitDir, maxDist);

		sqs.sweep(queryVolume, unitDir, pcb.mClosestHit.distance, pcb, NULL);

		return hits.size()!=0;
	}
}

// Generic usage, returns all hits touched by the swept volume, it's up to users to process them / sort them / etc.
// We're using a PxArray here but it's an arbitrary choice, one could also return hits via a callback, etc.
bool CustomScene::sweepMultiple(const PxGeometry& geom, const PxTransform& pose, const PxVec3& unitDir, float maxDist, PxArray<CustomSweepHit>& hits) const
{
	switch(PxU32(geom.getType()))
	{
		case PxGeometryType::eSPHERE:			{ return _sweepMultipleT<DefaultPrunerSphereSweepCallback>(hits, *mQuerySystem, geom, pose, unitDir, maxDist);	}
		case PxGeometryType::eCAPSULE:			{ return _sweepMultipleT<DefaultPrunerCapsuleSweepCallback>(hits, *mQuerySystem, geom, pose, unitDir, maxDist);	}
		case PxGeometryType::eBOX:				{ return _sweepMultipleT<DefaultPrunerBoxSweepCallback>(hits, *mQuerySystem, geom, pose, unitDir, maxDist);		}
		case PxGeometryType::eCONVEXMESH:		{ return _sweepMultipleT<DefaultPrunerConvexSweepCallback>(hits, *mQuerySystem, geom, pose, unitDir, maxDist);	}
		default:								{ PX_ASSERT(0); return false;																					}
	}
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	struct CustomOverlapAnyCallback : public DefaultPrunerOverlapCallback
	{
		bool	mFoundHit;

		CustomOverlapAnyCallback(PrunerFilterCallback& filterCB, const GeomOverlapTable* funcs, const PxGeometry& geometry, const PxTransform& pose) :
			DefaultPrunerOverlapCallback(filterCB, funcs, geometry, pose), mFoundHit(false)	{}

		virtual	bool	reportHit(const PrunerPayload& /*payload*/)
		{
			mFoundHit = true;
			return false;	// Early exits as soon as we hit something
		}
	};
}

// Simple boolean overlap. The query returns true if the passed shape touches anything, otherwise it returns false if space was free.
bool CustomScene::overlapAny(const PxGeometry& geom, const PxTransform& pose) const
{
	CustomOverlapAnyCallback pcb(gFilterCallback, gCachedFuncs.mCachedOverlapFuncs, geom, pose);

	const ShapeData queryVolume(geom, pose, 0.0f);
	mQuerySystem->overlap(queryVolume, pcb, NULL);

	return pcb.mFoundHit;
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	struct CustomOverlapMultipleCallback : public DefaultPrunerOverlapCallback
	{
		PxArray<PxU32>&	mHits;

		CustomOverlapMultipleCallback(PxArray<PxU32>& hits, PrunerFilterCallback& filterCB, const GeomOverlapTable* funcs, const PxGeometry& geometry, const PxTransform& pose) :
			DefaultPrunerOverlapCallback(filterCB, funcs, geometry, pose), mHits(hits)	{}

		virtual	bool	reportHit(const PrunerPayload& payload)
		{
			// In this example we only return touched objects but we don't go deeper. For triangle meshes we could
			// here go to the triangle-level and return all touched triangles of a mesh, if needed. To do that we'd
			// fetch the PxTriangleMeshGeometry from the payload and use PxMeshQuery::findOverlapTriangleMesh. That
			// call is part of the regular PxMeshQuery API though so it's beyond the scope of this snippet, which is
			// about the Gu-level query system. Instead here we just retrieve & output the touched object's index:
			mHits.pushBack(getObjectIndexFromPayload(payload));
			return true;	// Continue query, call us again for next touched object
		}

		PX_NOCOPY(CustomOverlapMultipleCallback)
	};
}

// Gather-touched-objects overlap. The query returns all objects touched by query volume, in an array.
// This is an arbitrary choice in this snippet, it would be equally easy to report each object individually
// to a user-callback, etc.
bool CustomScene::overlapMultiple(const PxGeometry& geom, const PxTransform& pose, PxArray<PxU32>& hits) const
{
	CustomOverlapMultipleCallback pcb(hits, gFilterCallback, gCachedFuncs.mCachedOverlapFuncs, geom, pose);

	const ShapeData queryVolume(geom, pose, 0.0f);
	mQuerySystem->overlap(queryVolume, pcb, NULL);

	return hits.size()!=0;
}

///////////////////////////////////////////////////////////////////////////////

void CustomScene::runQueries()
{
	if(!mQuerySystem)
		return;

	// Reset all touched flags & colors
	const PxVec3 touchedColor(0.25f, 0.5f, 1.0f);
	for(PxU32 i=0;i<mNbObjects;i++)
	{
		mObjects[i].mTouched = false;
		mObjects[i].mTouchedColor = touchedColor;
	}

	// This is optional, the SIMD guard can be ommited if your app already
	// setups the FPU/SIMD control word per thread. If not, try to use one
	// PX_SIMD_GUARD for many queries, rather than one for each query.
	PX_SIMD_GUARD

	if(0)
		mQuerySystem->commitUpdates();

	switch(gSceneIndex)
	{
		case RAYCAST_CLOSEST:
		{
			const PxVec3 origin(0.0f, 10.0f, 0.0f);
			const PxVec3 unitDir(0.0f, -1.0f, 0.0f);
			const float maxDist = 20.0f;

			CustomRaycastHit hit;
			const bool hasHit = raycastClosest(origin, unitDir, maxDist, hit);
#ifdef RENDER_SNIPPET
			if(hasHit)
			{
				DrawLine(origin, hit.position, PxVec3(1.0f));
				DrawLine(hit.position, hit.position + hit.normal, PxVec3(1.0f, 1.0f, 0.0f));
				DrawFrame(hit.position, 0.5f);
				mObjects[hit.mObjectIndex].mTouched = true;
			}
			else
			{
				DrawLine(origin, origin + unitDir * maxDist, PxVec3(1.0f));
			}
#else
			PX_UNUSED(hasHit);
#endif
		}
		break;

		case RAYCAST_ANY:
		{
			const PxVec3 origin(0.0f, 10.0f, 0.0f);
			const PxVec3 unitDir(0.0f, -1.0f, 0.0f);
			const float maxDist = 20.0f;

			const bool hasHit = raycastAny(origin, unitDir, maxDist);
#ifdef RENDER_SNIPPET
			if(hasHit)
				DrawLine(origin, origin + unitDir * maxDist, PxVec3(1.0f, 0.0f, 0.0f));
			else
				DrawLine(origin, origin + unitDir * maxDist, PxVec3(0.0f, 1.0f, 0.0f));
#else
			PX_UNUSED(hasHit);
#endif
		}
		break;

		case RAYCAST_MULTIPLE:
		{
			const PxVec3 origin(0.0f, 10.0f, 0.0f);
			const PxVec3 unitDir(0.0f, -1.0f, 0.0f);
			const float maxDist = 20.0f;

			PxArray<CustomRaycastHit> hits;
			const bool hasHit = raycastMultiple(origin, unitDir, maxDist, hits);

#ifdef RENDER_SNIPPET
			if(hasHit)
			{
				DrawLine(origin, origin + unitDir * maxDist, PxVec3(0.5f));

				const PxU32 nbHits = hits.size();
				for(PxU32 i=0;i<nbHits;i++)
				{
					const CustomRaycastHit& hit = hits[i];
					DrawLine(hit.position, hit.position + hit.normal, PxVec3(1.0f, 1.0f, 0.0f));
					DrawFrame(hit.position, 0.5f);
					mObjects[hit.mObjectIndex].mTouched = true;
				}
			}
			else
			{
				DrawLine(origin, origin + unitDir * maxDist, PxVec3(1.0f));
			}
#else
			PX_UNUSED(hasHit);
#endif
		}
		break;

		case SWEEP_CLOSEST:
		{
			const PxVec3 origin(0.0f, 10.0f, 0.0f);
			const PxVec3 unitDir(0.0f, -1.0f, 0.0f);
			const float maxDist = 20.0f;

			const PxSphereGeometry sweptGeom(0.5f);
			//const PxCapsuleGeometry sweptGeom(0.5f, 0.5f);
			const PxTransform pose(origin);

			CustomSweepHit hit;
			const bool hasHit = sweepClosest(sweptGeom, pose, unitDir, maxDist, hit);

#ifdef RENDER_SNIPPET
			const PxGeometryHolder gh(sweptGeom);
			if(hasHit)
			{
				const PxVec3 sweptPos = origin + unitDir * hit.distance;
				DrawLine(origin, sweptPos, PxVec3(1.0f));

				renderGeoms(1, &gh, &pose, false, PxVec3(0.0f, 1.0f, 0.0f));

				const PxTransform impactPose(sweptPos);
				renderGeoms(1, &gh, &impactPose, false, PxVec3(1.0f, 0.0f, 0.0f));

				DrawLine(hit.position, hit.position + hit.normal*2.0f, PxVec3(1.0f, 1.0f, 0.0f));
				DrawFrame(hit.position, 2.0f);
				mObjects[hit.mObjectIndex].mTouched = true;
			}
			else
			{
				const PxVec3 sweptPos = origin + unitDir * maxDist;
				DrawLine(origin, sweptPos, PxVec3(1.0f));

				renderGeoms(1, &gh, &pose, false, PxVec3(0.0f, 1.0f, 0.0f));
				const PxTransform impactPose(sweptPos);
				renderGeoms(1, &gh, &impactPose, false, PxVec3(0.0f, 1.0f, 0.0f));
			}
#else
			PX_UNUSED(hasHit);
#endif
		}
		break;

		case SWEEP_ANY:
		{
			const PxVec3 origin(0.0f, 10.0f, 0.0f);
			const PxVec3 unitDir(0.0f, -1.0f, 0.0f);
			const float maxDist = 20.0f;

			const PxBoxGeometry sweptGeom(PxVec3(0.5f));
			PxQuat q(1.1f, 0.1f, 0.8f, 1.4f);
			q.normalize();
			const PxTransform pose(origin, q);

			const bool hasHit = sweepAny(sweptGeom, pose, unitDir, maxDist);

#ifdef RENDER_SNIPPET
			const PxGeometryHolder gh(sweptGeom);
			{
				// We only have a boolean result so we're just going to draw a sequence of geoms along
				// the sweep direction in the appropriate color.

				const PxVec3 color = hasHit ? PxVec3(1.0f, 0.0f, 0.0f) : PxVec3(0.0f, 1.0f, 0.0f);

				const PxU32 nb = 20;
				for(PxU32 i=0;i<nb;i++)
				{
					const float coeff = float(i)/float(nb-1);

					const PxVec3 sweptPos = origin + unitDir * coeff * maxDist;

					const PxTransform impactPose(sweptPos, q);
					renderGeoms(1, &gh, &impactPose, false, color);
				}
			}
#else
			PX_UNUSED(hasHit);
#endif
		}
		break;

		case SWEEP_MULTIPLE:
		{
			const PxVec3 origin(0.0f, 10.0f, 0.0f);
			const PxVec3 unitDir(0.0f, -1.0f, 0.0f);
			const float maxDist = 20.0f;

			const PxCapsuleGeometry sweptGeom(0.5f, 0.5f);
			const PxTransform pose(origin);

			PxArray<CustomSweepHit> hits;
			const bool hasHit = sweepMultiple(sweptGeom, pose, unitDir, maxDist, hits);

#ifdef RENDER_SNIPPET
			const PxGeometryHolder gh(sweptGeom);
			renderGeoms(1, &gh, &pose, false, PxVec3(0.0f, 1.0f, 0.0f));
			if(hasHit)
			{
				{
					const PxVec3 sweptPos = origin + unitDir * maxDist;
					DrawLine(origin, sweptPos, PxVec3(0.5f));
				}

				// It can be difficult to see what is touching what so we use different colors to make it clearer.
				const PxVec3 touchedColors[] = {
					PxVec3(1.0f, 0.0f, 0.0f),
					PxVec3(0.0f, 0.0f, 1.0f),
					PxVec3(1.0f, 0.0f, 1.0f),
					PxVec3(0.0f, 1.0f, 1.0f),
					PxVec3(1.0f, 1.0f, 0.0f),
					PxVec3(1.0f, 1.0f, 1.0f),
					PxVec3(0.5f, 0.5f, 0.5f),
				};

				const PxU32 nbHits = hits.size();
				for(PxU32 i=0;i<nbHits;i++)
				{
					const PxVec3& shapeTouchedColor = touchedColors[i];

					const CustomSweepHit& hit = hits[i];
					const PxVec3 sweptPos = origin + unitDir * hit.distance;
					const PxTransform impactPose(sweptPos);
					renderGeoms(1, &gh, &impactPose, false, shapeTouchedColor);

					DrawLine(hit.position, hit.position + hit.normal*2.0f, PxVec3(1.0f, 1.0f, 0.0f));
					DrawFrame(hit.position, 2.0f);

					mObjects[hit.mObjectIndex].mTouched = true;
					mObjects[hit.mObjectIndex].mTouchedColor = shapeTouchedColor;
				}
			}
			else
			{
				const PxVec3 sweptPos = origin + unitDir * maxDist;
				DrawLine(origin, sweptPos, PxVec3(1.0f));
			}
#else
			PX_UNUSED(hasHit);
#endif
		}
		break;

		case OVERLAP_ANY:
		{
			const PxVec3 origin(0.0f, 4.0f, 0.0f);

			const PxSphereGeometry queryGeom(0.5f);
			const PxTransform pose(origin);

			const bool hasHit = overlapAny(queryGeom, pose);

#ifdef RENDER_SNIPPET
			const PxGeometryHolder gh(queryGeom);
			{
				const PxVec3 color = hasHit ? PxVec3(1.0f, 0.0f, 0.0f) : PxVec3(0.0f, 1.0f, 0.0f);

				renderGeoms(1, &gh, &pose, false, color);
			}
#else
			PX_UNUSED(hasHit);
#endif
		}
		break;

		case OVERLAP_MULTIPLE:
		{
			const PxVec3 origin(0.0f, 4.0f, 0.0f);

			const PxSphereGeometry queryGeom(0.5f);
			const PxTransform pose(origin);

			PxArray<PxU32> hits;
			const bool hasHit = overlapMultiple(queryGeom, pose, hits);

#ifdef RENDER_SNIPPET
			const PxGeometryHolder gh(queryGeom);
			{
				const PxVec3 color = hasHit ? PxVec3(1.0f, 0.0f, 0.0f) : PxVec3(0.0f, 1.0f, 0.0f);

				renderGeoms(1, &gh, &pose, false, color);

				for(PxU32 i=0;i<hits.size();i++)
					mObjects[hits[i]].mTouched = true;
			}
#else
			PX_UNUSED(hasHit);
#endif
		}
		break;

		case NB_SCENES:	// Blame pedantic compilers
		{
		}
		break;
	}
}

void CustomScene::render()
{
	updateObjects();

	runQueries();

#ifdef RENDER_SNIPPET
	const PxVec3 color(1.0f, 0.5f, 0.25f);

	const PxU32 nbObjects = mNbObjects;
	for(PxU32 i=0;i<nbObjects;i++)
	{
		const Object& obj = mObjects[i];

		PrunerPayloadData ppd;
		mQuerySystem->getPayloadData(obj.mData, &ppd);

		//DrawBounds(*bounds);

		const PxVec3& objectColor = obj.mTouched ? obj.mTouchedColor : color;

		renderGeoms(1, &obj.mGeom, ppd.mTransform, false, objectColor);
	}

	//mQuerySystem->visualize(true, true, PxRenderOutput)
#endif
}

}

static CustomScene* gScene = NULL;

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	const PxTolerancesScale scale;
	PxCookingParams params(scale);
	params.midphaseDesc.setToDefault(PxMeshMidPhase::eBVH34);
//	params.midphaseDesc.mBVH34Desc.quantized = false;
	params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
	params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;

	{
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
			gConvexMesh = PxCreateConvexMesh(params, convexDesc);
		}

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
	}

	gScene = new CustomScene;
}

void renderScene()
{
	if(gScene)
		gScene->render();

#ifdef RENDER_SNIPPET
	Snippets::print("Press F1 to F8 to select a scenario.");
	switch(PxU32(gSceneIndex))
	{
		case RAYCAST_CLOSEST:	{ Snippets::print("Current scenario: raycast closest");		}break;
		case RAYCAST_ANY:		{ Snippets::print("Current scenario: raycast any");			}break;
		case RAYCAST_MULTIPLE:	{ Snippets::print("Current scenario: raycast multiple");	}break;
		case SWEEP_CLOSEST:		{ Snippets::print("Current scenario: sweep closest");		}break;
		case SWEEP_ANY:			{ Snippets::print("Current scenario: sweep any");			}break;
		case SWEEP_MULTIPLE:	{ Snippets::print("Current scenario: sweep multiple");		}break;
		case OVERLAP_ANY:		{ Snippets::print("Current scenario: overlap any");			}break;
		case OVERLAP_MULTIPLE:	{ Snippets::print("Current scenario: overlap multiple");	}break;
	}
#endif
}

void stepPhysics(bool /*interactive*/)
{
}

void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gConvexMesh);
	PX_RELEASE(gFoundation);

	printf("SnippetQuerySystemAllQueries done.\n");
}

void keyPress(unsigned char key, const PxTransform& /*camera*/)
{
	if(gScene)
	{
		if(key=='p' || key=='P')
		{
			gPause = !gPause;
		}
		else if(key=='o' || key=='O')
		{
			gPause = true;
			gOneFrame = true;
		}
		else
		{
			if(key>=1 && key<=NB_SCENES)
			{
				gSceneIndex = QueryScenario(key-1);
				PX_RELEASE(gScene);
				gScene = new CustomScene;
			}
		}
	}
}

int snippetMain(int, const char*const*)
{
	printf("Query System All Queries snippet.\n");
	printf("Press F1 to F8 to select a scene:\n");
	printf(" F1......raycast closest\n");
	printf(" F2..........raycast any\n");
	printf(" F3.....raycast multiple\n");
	printf(" F4........sweep closest\n");
	printf(" F5............sweep any\n");
	printf(" F6.......sweep multiple\n");
	printf(" F7..........overlap any\n");
	printf(" F8.....overlap multiple\n");
	printf("\n");
	printf("Press P to Pause.\n");
	printf("Press O to step the simulation One frame.\n");
	printf("Press the cursor keys to move the camera.\n");
	printf("Use the mouse/left mouse button to rotate the camera.\n");

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

