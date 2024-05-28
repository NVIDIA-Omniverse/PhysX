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
// This snippet shows how to use a custom scene query system with N pruners
// instead of the traditional two in PxScene. This is mainly useful for large
// world with a lot of actors and a lot of updates. The snippet creates a
// "worst case scenario" involving hundreds of thousands of actors, which are
// all artificially updated each frame to put a lot of pressure on the query
// structures. This is not entirely realistic but it makes the gains from the
// custom query system more obvious. There is a virtual "player" moving around
// in that world and regions are added and removed at runtime according to the
// player's position. Each region contains thousands of actors, which stresses
// the tree building code, the tree refit code, the build step code, and many
// parts of the SQ update pipeline. Pruners can be updated in parallel, which
// is more useful with N pruners than it was with the two PxScene build-in pruners.
//
// Rendering is disabled by default since it can be quite slow for so many
// actors.
//
// Note that the cost of actual scene queries (raycasts, etc) might go up when
// using multiple pruners. However the cost of updating the SQ structures can be
// much higher than the cost of the scene queries themselves, so this can be a
// good trade-off.
// ****************************************************************************

#include <ctype.h>
#include <vector>
#include "PxPhysicsAPI.h"
#include "foundation/PxArray.h"
#include "foundation/PxTime.h"
#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"
#ifdef RENDER_SNIPPET
	#include "../snippetrender/SnippetCamera.h"
	#include "../snippetrender/SnippetRender.h"
#endif

using namespace physx;

// Define this to use a custom pruner. Undefine to use the default PxScene code.
#define USE_CUSTOM_PRUNER

// This number of threads is used both for the default PhysX CPU dispatcher, and
// for the custom concurrent build steps when a custom pruner is used. It does not
// have much of an impact when USE_CUSTOM_PRUNER is undefined.
#define	NUM_WORKER_THREADS	8
//#define	NUM_WORKER_THREADS	4
//#define	NUM_WORKER_THREADS	2
//#define	NUM_WORKER_THREADS	1

#ifdef RENDER_SNIPPET
	// Enable or disable rendering. Disabled by default, since the default scene uses 648081 actors.
	// There is a custom piece of code that renders the scene as a single mesh though, so it should
	// still be usable on fast PCs when enabled.
	static const bool gEnableRendering = false;
#endif

// Number of frames to simulate, only used when gEnableRendering == false
static const PxU32 gNbFramesToSimulate = 100;

// The PhysX tree rebuild rate hint. It is usually a *bad idea* to decrease it to 10 (the default
// value is 100), but people do this, and it puts more stress on the build code, which fits the
// worst case scenario we're looking for in this snippet. But do note that in most cases it should
// really be left to "100", or actually increased in large scenes.
static PxU32 gDynamicTreeRebuildRateHint = 10;

// How many objects are added in each region. This has a direct impact on performance in all parts
// of the system.
//static const PxU32 gNbObjectsPerRegion = 400;
//static const PxU32 gNbObjectsPerRegion = 800;
//static const PxU32 gNbObjectsPerRegion = 2000;
//static const PxU32 gNbObjectsPerRegion = 4000;
static const PxU32 gNbObjectsPerRegion = 8000;

static const float gGlobalScale = 1.0f;

// Size of added objects.
static const float gObjectScale = 0.01f * gGlobalScale;

// This controls whether objects are artificially updated each frame. This resets the objects' positions
// to what they already are, no nothing is moving but internally it forces all trees to be refit & rebuilt
// constantly. This has a big impact on performance.
//
// Using "false" here means that:
// - if USE_CUSTOM_PRUNER is NOT defined, the new objects are added to a unique tree in PxScene, which triggers
//   a rebuild. The refit operation is not necessary and skipped.
// - if USE_CUSTOM_PRUNER is defined, the new objects are added to a per-region pruner in each region. There is
//   no rebuild necessary, and no refit either.
//
// Using "true" here means that all involved trees are constantly refit & rebuilt over a number of frames.
static const bool gUpdateObjectsInRegion = true;

// Range of player's motion
static const float gRange = 10.0f * gGlobalScale;

#ifdef RENDER_SNIPPET
	// Size of player
	static const float gPlayerSize = 0.1f * gGlobalScale;
#endif

// Speed of player. If you increase it too much the player might leave a region before its tree gets rebuilt,
// which means some parts of the update pipeline are never executed.
static const float gPlayerSpeed = 0.1f;
//static const float gPlayerSpeed = 0.01f;

// Size of active area. The world is effectively infinite but only this active area is considered by the
// streaming code. The active area is a square whose edge size is gActiveAreaSize*2.
static const float gActiveAreaSize = 5.0f * gGlobalScale;

// Number of cells per side == number of regions per side.
static const PxU32 gNbCellsPerSide = 8;

#ifdef USE_CUSTOM_PRUNER
	// Number of pruners in the system
	static const PxU32 gNbPruners = (gNbCellsPerSide+1)*(gNbCellsPerSide+1);

	// Use true to update all pruners in parallel, false to update them sequentially
	static const bool gUseConcurrentBuildSteps = true;

	// Use tree of pruners or not. This is mainly useful if you have a large number of pruners in
	// the system. There is a small cost associated with maintaining that extra tree but since the
	// number of pruners should still be vastly smaller than the total number of objects, this is
	// usually quite cheap. You can profile the ratcast cost by modifying the code at the end of
	// this snippet and see how using a tree of pruners improves performance.
	static const bool gUseTreeOfPruners = false;
#endif

static float gGlobalTime = 0.0f;
static SnippetUtils::BasicRandom gRandom(42);

static const PxVec3 gYellow(1.0f, 1.0f, 0.0f);
static const PxVec3 gRed(1.0f, 0.0f, 0.0f);
static const PxVec3 gGreen(0.0f, 1.0f, 0.0f);

static PxVec3 computePlayerPos(float globalTime)
{
	const float Amplitude = gRange;
	const float t = globalTime * gPlayerSpeed;
	const float x = sinf(t*2.17f) * sinf(t) * Amplitude;
	const float z = sinf(t*0.77f) * cosf(t) * Amplitude;
	return PxVec3(x, 0.0f, z);
}

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation*			gFoundation = NULL;
static PxPhysics*				gPhysics	= NULL;
static PxDefaultCpuDispatcher*	gDispatcher = NULL;
static PxScene*					gScene		= NULL;
static PxMaterial*				gMaterial	= NULL;
static PxPvd*					gPvd        = NULL;

#define INVALID_ID	0xffffffff

#ifdef RENDER_SNIPPET
static PxU8 gBoxIndices[] = {
	0,2,1,	0,3,2,
	1,6,5,	1,2,6,
	5,7,4,	5,6,7,
	4,3,0,	4,7,3,
	3,6,2,	3,7,6,
	5,0,1,	5,4,0
};
#endif

namespace
{
#ifdef USE_CUSTOM_PRUNER
	struct PrunerData
	{
		PrunerData() : mPrunerIndex(INVALID_ID), mNbObjects(0)	{}

		PxU32	mPrunerIndex;
		PxU32	mNbObjects;
	};
#endif

	struct RegionData
	{
		PxArray<PxRigidStatic*>	mObjects;
#ifdef USE_CUSTOM_PRUNER
		PrunerData*				mPrunerData;
#endif
#ifdef RENDER_SNIPPET
		PxU32					mNbVerts, mNbTris;
		PxVec3*					mVerts;
		PxU32*					mIndices;
#endif
	};

#ifdef USE_CUSTOM_PRUNER
	// This adapter class will be used by the PxCustomSceneQuerySystem to map each new actor/shape to a user-defined pruner
	class SnippetCustomSceneQuerySystemAdapter : public PxCustomSceneQuerySystemAdapter
	{
		public:

		PrunerData	mPrunerData[gNbPruners];

		SnippetCustomSceneQuerySystemAdapter()
		{
		}

		void	createPruners(PxCustomSceneQuerySystem* customSQ)
		{
			// We create a pool of pruners, large enough to provide one pruner per region. This is an arbitrary choice, we could also
			// map multiple regions to the same pruner (as long as these regions are close to each other it's just fine).
			if(customSQ)
			{
				for(PxU32 i=0;i<gNbPruners;i++)
					mPrunerData[i].mPrunerIndex = customSQ->addPruner(PxPruningStructureType::eDYNAMIC_AABB_TREE, PxDynamicTreeSecondaryPruner::eINCREMENTAL);
					//mPrunerData[i].mPrunerIndex = customSQ->addPruner(PxPruningStructureType::eDYNAMIC_AABB_TREE, PxDynamicTreeSecondaryPruner::eBVH);
			}
		}

		// This is called by the streaming code to assign a pruner to a region
		PrunerData*	findFreePruner()
		{
			for(PxU32 i=0;i<gNbPruners;i++)
			{
				if(mPrunerData[i].mNbObjects==0)
					return &mPrunerData[i];
			}
			PX_ASSERT(0);
			return NULL;
		}

		// This is called by the streaming code to release a pruner when a region is deleted
		void		releasePruner(PxU32 index)
		{
			PX_ASSERT(mPrunerData[index].mNbObjects==0);
			mPrunerData[index].mNbObjects=0;
		}

		// This is called by PxCustomSceneQuerySystem to assign a pruner index to a new actor/shape
		virtual	PxU32	getPrunerIndex(const PxRigidActor& actor, const PxShape& /*shape*/)	const
		{
			const PrunerData* prunerData = reinterpret_cast<const PrunerData*>(actor.userData);
			return prunerData->mPrunerIndex;
		}

		// This is called by PxCustomSceneQuerySystem to validate a pruner for scene queries
		virtual	bool	processPruner(PxU32 /*prunerIndex*/, const PxQueryThreadContext* /*context*/, const PxQueryFilterData& /*filterData*/, PxQueryFilterCallback* /*filterCall*/)	const
		{
			// We could filter out empty pruners here if we have some, but for now we don't bother
			return true;
		}
	};
#endif

	struct StreamRegion
	{
		PX_FORCE_INLINE	StreamRegion() : mKey(0), mTimestamp(INVALID_ID), mRegionData(NULL)	{}

		PxU64		mKey;
		PxU32		mTimestamp;
		PxBounds3	mCellBounds;
		RegionData*	mRegionData;
	};

	typedef PxHashMap<PxU64, StreamRegion>	StreamingCache;

	class Streamer
	{
						PX_NOCOPY(Streamer)
		public:
						Streamer(float activeAreaSize, PxU32 nbCellsPerSide);
						~Streamer();

		void			update(const PxVec3& playerPos);
		void			renderDebug();
		void			render();

		PxBounds3		mStreamingBounds;

		StreamingCache	mStreamingCache;
		PxU32			mTimestamp;

		const float		mActiveAreaSize;
		const PxU32		mNbCellsPerSide;

		void			addRegion(StreamRegion& region);
		void			updateRegion(StreamRegion& region);
		void			removeRegion(StreamRegion& region);
	};
}

#ifdef USE_CUSTOM_PRUNER
static SnippetCustomSceneQuerySystemAdapter gAdapter;
#endif

Streamer::Streamer(float activeAreaSize, PxU32 nbCellsPerSide) : mTimestamp(0), mActiveAreaSize(activeAreaSize), mNbCellsPerSide(nbCellsPerSide)
{
	mStreamingBounds.setEmpty();
}

Streamer::~Streamer()
{
}

void Streamer::addRegion(StreamRegion& region)
{
	PX_ASSERT(region.mRegionData==NULL);

	// We disable the simulation flag to measure the cost of SQ structures exclusively
	const PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE;

	PxVec3 extents = region.mCellBounds.getExtents();
	extents.x -= 0.01f * gGlobalScale;
	extents.z -= 0.01f * gGlobalScale;
	extents.y = 0.1f * gGlobalScale;

	PxVec3 center = region.mCellBounds.getCenter();
	center.y = -0.1f * gGlobalScale;

	// In each region we create one "ground" shape and a number of extra smaller objects on it. We use
	// static objects to make sure the timings aren't polluted by dynamics-related costs. Dynamic actors
	// can also move to neighboring regions, which in theory means they should be transferred to another
	// pruner to avoid unpleasant visual artefacts when removing an entire region. This is beyond the
	// scope of this snippet so, static actors it is.
	//
	// The number of static actors in the world is usually much larger than the number of dynamic actors.
	// Dynamic actors move constantly, so they usually always trigger a refit & rebuild of the corresponding
	// trees. It is therefore a good idea to separate static and dynamic actors, to avoid the cost of
	// refit & rebuild for the static parts. In the context of a streaming world it is sometimes good enough
	// to put static actors in separate pruners (like we do here) but still stuff all dynamic actors in a
	// unique separate pruner (i.e. the same for the whole world). It avoids the aforementioned issues with
	// dynamic actors moving to other regions, and if the total number of dynamic actors remains small a
	// single structure for dynamic actors is often enough.
	PxShape* groundShape = gPhysics->createShape(PxBoxGeometry(extents), *gMaterial, false, shapeFlags);
	PxRigidStatic* ground = PxCreateStatic(*gPhysics, PxTransform(center), *groundShape);

	RegionData* regionData = new RegionData;
	regionData->mObjects.pushBack(ground);
	region.mRegionData = regionData;

#ifdef USE_CUSTOM_PRUNER
	regionData->mPrunerData = gAdapter.findFreePruner();
	ground->userData = regionData->mPrunerData;
	regionData->mPrunerData->mNbObjects++;
#endif

	gScene->addActor(*ground);

	if(gNbObjectsPerRegion)
	{
		const PxU32 nbExtraObjects = gNbObjectsPerRegion;
		const float objectScale = gObjectScale;
		const float coeffY = objectScale * 20.0f;
		center.y = objectScale;
		const PxBoxGeometry boxGeom(objectScale, objectScale, objectScale);
		PxShape* shape = gPhysics->createShape(boxGeom, *gMaterial, false, shapeFlags);

		PxRigidActor* actors[nbExtraObjects];
		for(PxU32 j=0;j<nbExtraObjects;j++)
		{
			PxVec3 c = center;
			c.x += gRandom.randomFloat() * extents.x * (2.0f - objectScale);
			c.y += fabsf(gRandom.randomFloat()) * coeffY;
			c.z += gRandom.randomFloat() * extents.z * (2.0f - objectScale);

			PxRigidStatic* actor = PxCreateStatic(*gPhysics, PxTransform(c), *shape);
			actors[j] = actor;
			regionData->mObjects.pushBack(actor);

#ifdef USE_CUSTOM_PRUNER
			actor->userData = regionData->mPrunerData;
			regionData->mPrunerData->mNbObjects++;
#endif
		}

/*		if(0)
		{
			PxPruningStructure*	ps = physics.createPruningStructure(actors, nbDynamicObjects);
			scene.addActors(*ps);
		}
		else*/
		{
			gScene->addActors(reinterpret_cast<PxActor**>(actors), nbExtraObjects);
		}
	}

#ifdef RENDER_SNIPPET
	// Precompute single render mesh for this region (rendering them as individual actors is too
	// slow). This is also only possible because we used static actors.
	{
		const PxU32 nbActors = regionData->mObjects.size();
		const PxU32 nbVerts = nbActors*8;
		const PxU32 nbTris = nbActors*12;
		PxVec3* pts = new PxVec3[nbVerts];
		PxVec3* dstPts = pts;
		PxU32* indices = new PxU32[nbTris*3];
		PxU32* dstIndices = indices;
		PxU32 baseIndex = 0;
		for(PxU32 i=0;i<nbActors;i++)
		{
			const PxVec3 c = regionData->mObjects[i]->getGlobalPose().p;

			PxShape* shape;
			regionData->mObjects[i]->getShapes(&shape, 1);

			const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(shape->getGeometry());

			const PxVec3 minimum = c - boxGeom.halfExtents;
			const PxVec3 maximum = c + boxGeom.halfExtents;

			dstPts[0] = PxVec3(minimum.x, minimum.y, minimum.z);
			dstPts[1] = PxVec3(maximum.x, minimum.y, minimum.z);
			dstPts[2] = PxVec3(maximum.x, maximum.y, minimum.z);
			dstPts[3] = PxVec3(minimum.x, maximum.y, minimum.z);
			dstPts[4] = PxVec3(minimum.x, minimum.y, maximum.z);
			dstPts[5] = PxVec3(maximum.x, minimum.y, maximum.z);
			dstPts[6] = PxVec3(maximum.x, maximum.y, maximum.z);
			dstPts[7] = PxVec3(minimum.x, maximum.y, maximum.z);
			dstPts += 8;

			for(PxU32 j=0;j<12*3;j++)
				dstIndices[j] = gBoxIndices[j] + baseIndex;
			dstIndices += 12*3;
			baseIndex += 8;
		}
		regionData->mVerts = pts;
		regionData->mIndices = indices;
		regionData->mNbVerts = nbVerts;
		regionData->mNbTris = nbTris;
	}
#endif
}

void Streamer::updateRegion(StreamRegion& region)
{
	RegionData* regionData = region.mRegionData;

	if(gUpdateObjectsInRegion)
	{
		// Artificial update to trigger the tree refit & rebuild code
		const PxU32 nbObjects = regionData->mObjects.size();
		for(PxU32 i=0;i<nbObjects;i++)
		{
			const PxTransform pose = regionData->mObjects[i]->getGlobalPose();
			regionData->mObjects[i]->setGlobalPose(pose);
		}
	}
}

void Streamer::removeRegion(StreamRegion& region)
{
	PX_ASSERT(region.mRegionData);
	RegionData* regionData = region.mRegionData;

#ifdef RENDER_SNIPPET
	delete [] regionData->mIndices;
	delete [] regionData->mVerts;
#endif

	// Because we used static actors only we can just release all actors, they didn't move to other regions
	const PxU32 nbObjects = regionData->mObjects.size();
	for(PxU32 i=0;i<nbObjects;i++)
		regionData->mObjects[i]->release();

#ifdef USE_CUSTOM_PRUNER
	PX_ASSERT(regionData->mPrunerData);
	regionData->mPrunerData->mNbObjects-=nbObjects;
	PX_ASSERT(regionData->mPrunerData->mNbObjects==0);
	gAdapter.releasePruner(regionData->mPrunerData->mPrunerIndex);
#endif

	delete region.mRegionData;
	region.mRegionData = NULL;
}

void Streamer::update(const PxVec3& playerPos)
{
	const float activeAreaSize = mActiveAreaSize;
	mStreamingBounds = PxBounds3::centerExtents(playerPos, PxVec3(activeAreaSize));

	const float worldSize = activeAreaSize*2.0f;
	const PxU32 nbCellsPerSide = mNbCellsPerSide;
	const float cellSize = worldSize/float(nbCellsPerSide);

	const float cellSizeX = cellSize;
	const float cellSizeZ = cellSize;

	const PxI32 x0 = PxI32(floorf(mStreamingBounds.minimum.x/cellSizeX));
	const PxI32 z0 = PxI32(floorf(mStreamingBounds.minimum.z/cellSizeZ));
	const PxI32 x1 = PxI32(ceilf(mStreamingBounds.maximum.x/cellSizeX));
	const PxI32 z1 = PxI32(ceilf(mStreamingBounds.maximum.z/cellSizeZ));

	// Generally speaking when streaming objects in and out of the game world, we want to first remove
	// old objects then add new objects (in this order) to give the system a chance to recycle removed
	// entries and use less resources overall. That's why we split the loop to add objects in two parts.
	// The first part below finds currently touched regions and updates their timestamp.
	for(PxI32 j=z0;j<z1;j++)
	{
		for(PxI32 i=x0;i<x1;i++)
		{
			const PxU64 Key = (PxU64(i)<<32)|PxU64(PxU32(j));

			StreamRegion& region = mStreamingCache[Key];
			if(region.mTimestamp!=INVALID_ID)
			{
				// This region was already active => update its timestamp.
				PX_ASSERT(region.mKey==Key);
				region.mTimestamp = mTimestamp;
				updateRegion(region);
			}
		}
	}

	// This loop checks all regions in the system and removes the ones that are neither new
	// (mTimestamp==INVALID_ID) nor persistent (mTimestamp==current timestamp).
	{
		PxArray<PxU64> toRemove;	// Delayed removal to avoid touching the hashmap while we're iterating it
		for(StreamingCache::Iterator iter = mStreamingCache.getIterator(); !iter.done(); ++iter)
		{
			if(iter->second.mTimestamp!=mTimestamp && iter->second.mTimestamp!=INVALID_ID)
			{
				removeRegion(iter->second);
				toRemove.pushBack(iter->second.mKey);
			}
		}

		const PxU32 nbToGo = toRemove.size();
		for(PxU32 i=0;i<nbToGo;i++)
		{
			bool b = mStreamingCache.erase(toRemove[i]);
			PX_ASSERT(b);
			PX_UNUSED(b);
		}
	}

	// Finally we do our initial loop again looking for new regions (mTimestamp==INVALID_ID) and actually add them.
	for(PxI32 j=z0;j<z1;j++)
	{
		for(PxI32 i=x0;i<x1;i++)
		{
			const PxU64 Key = (PxU64(i)<<32)|PxU64(PxU32(j));

			StreamRegion& region = mStreamingCache[Key];
			if(region.mTimestamp==INVALID_ID)
			{
				// New entry
				region.mKey = Key;
				region.mTimestamp = mTimestamp;
				region.mCellBounds.minimum = PxVec3(float(i)*cellSizeX, 0.0f, float(j)*cellSizeZ);
				region.mCellBounds.maximum = PxVec3(float(i+1)*cellSizeX, 0.0f, float(j+1)*cellSizeZ);
				addRegion(region);
			}
		}
	}

	mTimestamp++;
}

void Streamer::renderDebug()
{
#ifdef RENDER_SNIPPET
	Snippets::DrawBounds(mStreamingBounds, gGreen);

	for(StreamingCache::Iterator iter = mStreamingCache.getIterator(); !iter.done(); ++iter)
		Snippets::DrawBounds(iter->second.mCellBounds, gRed);
#endif
}

void Streamer::render()
{
#ifdef RENDER_SNIPPET
	for(StreamingCache::Iterator iter = mStreamingCache.getIterator(); !iter.done(); ++iter)
	{
		const RegionData* data = iter->second.mRegionData;
		Snippets::renderMesh(data->mNbVerts, data->mVerts, data->mNbTris, data->mIndices, PxVec3(0.1f, 0.2f, 0.3f));
	}
#endif
}

static Streamer* gStreamer = NULL;
#ifdef USE_CUSTOM_PRUNER
static PxCustomSceneQuerySystem* gCustomSQ = NULL;
#endif

static bool gHasRaycastHit;
static PxRaycastHit gRaycastHit;
static PxVec3 gOrigin;

void renderScene()
{
#ifdef RENDER_SNIPPET
	if(0)	// Disabled, this is too slow
	{
		PxScene* scene;
		PxGetPhysics().getScenes(&scene,1);
		PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
		if(nbActors)
		{
			//printf("Rendering %d actors\n", nbActors);
			std::vector<PxRigidActor*> actors(nbActors);
			scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
			Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), false, PxVec3(0.1f, 0.2f, 0.3f), NULL, true, false);
		}
	}
	else
	{
		if(gStreamer)
			gStreamer->render();
	}

	if(gHasRaycastHit)
	{
		Snippets::DrawLine(gOrigin, gRaycastHit.position, gYellow);
		Snippets::DrawFrame(gRaycastHit.position, 1.0f);
	}
	else
		Snippets::DrawLine(gOrigin, gOrigin - PxVec3(0.0f, 100.0f, 0.0f), gYellow);

	const PxVec3 playerPos = computePlayerPos(gGlobalTime);
	const PxBounds3 playerBounds = PxBounds3::centerExtents(playerPos, PxVec3(gPlayerSize));
	Snippets::DrawBounds(playerBounds, gYellow);

//	const PxBounds3 activeAreaBounds = PxBounds3::centerExtents(playerPos, PxVec3(gActiveAreaSize));
//	Snippets::DrawBounds(activeAreaBounds, gGreen);

	if(gStreamer)
		gStreamer->renderDebug();
#endif
}

void initPhysics(bool /*interactive*/)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	if(1)
	{
		gPvd = PxCreatePvd(*gFoundation);
		PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
		//gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);
		gPvd->connect(*transport,PxPvdInstrumentationFlag::ePROFILE);
	}

	gPhysics	= PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), false, gPvd);
	gDispatcher	= PxDefaultCpuDispatcherCreate(NUM_WORKER_THREADS);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity						= PxVec3(0.0f, -9.81f, 0.0f);
	sceneDesc.cpuDispatcher					= gDispatcher;
	sceneDesc.filterShader					= PxDefaultSimulationFilterShader;
	sceneDesc.staticStructure				= PxPruningStructureType::eDYNAMIC_AABB_TREE;
	sceneDesc.dynamicStructure				= PxPruningStructureType::eDYNAMIC_AABB_TREE;
	sceneDesc.dynamicTreeRebuildRateHint	= gDynamicTreeRebuildRateHint;

#ifdef USE_CUSTOM_PRUNER
	// For concurrent build steps we're going to use the new custom API in PxCustomSceneQuerySystem so we tell the system
	// to disable the built-in build-step & commit functions (which are otherwise executed in fetchResults).
	sceneDesc.sceneQueryUpdateMode			= gUseConcurrentBuildSteps ? PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED : PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_ENABLED;

	// Create our custom scene query system and tell PxSceneDesc about it
	PxCustomSceneQuerySystem* customSQ = PxCreateCustomSceneQuerySystem(sceneDesc.sceneQueryUpdateMode, 0x0102030405060708, gAdapter, gUseTreeOfPruners);
	if(customSQ)
	{
		gAdapter.createPruners(customSQ);
		sceneDesc.sceneQuerySystem = customSQ;
		gCustomSQ = customSQ;
	}
#endif
	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	gStreamer = new Streamer(gActiveAreaSize, gNbCellsPerSide);
}

#ifdef USE_CUSTOM_PRUNER
class TaskBuildStep : public PxLightCpuTask
{
public:
	TaskBuildStep() : PxLightCpuTask(), mIndex(INVALID_ID)	{}

	virtual void run()
	{
		PX_SIMD_GUARD
		gCustomSQ->customBuildstep(mIndex);
	}

	virtual const char* getName() const { return "TaskBuildStep"; }

	PxU32	mIndex;
};

class TaskWait: public PxLightCpuTask
{
public:

	TaskWait(SnippetUtils::Sync* syncHandle) : PxLightCpuTask(), mSyncHandle(syncHandle)	{}

	virtual void run()	{}

	PX_INLINE void release()
	{
		PxLightCpuTask::release();
		SnippetUtils::syncSet(mSyncHandle);
	}

	virtual const char* getName() const { return "TaskWait"; }

private:

	SnippetUtils::Sync* mSyncHandle;
};

static void concurrentBuildSteps()
{
	const PxU32 nbPruners = gCustomSQ->startCustomBuildstep();
	PX_UNUSED(nbPruners);
	{
		PX_ASSERT(nbPruners==gNbPruners);

		SnippetUtils::Sync* buildStepsComplete = SnippetUtils::syncCreate();
		SnippetUtils::syncReset(buildStepsComplete);

		TaskWait taskWait(buildStepsComplete);
		TaskBuildStep taskBuildStep[gNbPruners];
		for(PxU32 i=0; i<gNbPruners; i++)
			taskBuildStep[i].mIndex = i;

		PxTaskManager* tm = gScene->getTaskManager();
		tm->resetDependencies();
		tm->startSimulation();

		taskWait.setContinuation(*tm, NULL);
		for(PxU32 i=0; i<gNbPruners; i++)
			taskBuildStep[i].setContinuation(&taskWait);

		taskWait.removeReference();
		for(PxU32 i=0; i<gNbPruners; i++)
			taskBuildStep[i].removeReference();

		SnippetUtils::syncWait(buildStepsComplete);
		SnippetUtils::syncRelease(buildStepsComplete);
	}
	gCustomSQ->finishCustomBuildstep();
}
#endif

static PxTime gTime;

void stepPhysics(bool /*interactive*/)
{
	if(gStreamer)
	{
		const PxVec3 playerPos = computePlayerPos(gGlobalTime);
		gStreamer->update(playerPos);
		gOrigin = playerPos+PxVec3(0.0f, 10.0f, 0.0f);
	}

	const PxU32 nbActors = gScene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);

	const float dt = 1.0f/60.0f;
	gTime.getElapsedSeconds();
	{
		gScene->simulate(dt);
		gScene->fetchResults(true);
#ifdef USE_CUSTOM_PRUNER
		if(gUseConcurrentBuildSteps)
			concurrentBuildSteps();
#endif
	}
	PxTime::Second time = gTime.getElapsedSeconds()*1000.0;

	// Ignore first frames to skip the cost of creating all the initial regions
	static PxU32 nbIgnored = 16;
	static PxU32 nbCalls = 0;
	static PxF64 totalTime = 0;
	static PxF64 peakTime = 0;

	if(nbIgnored)
		nbIgnored--;
	else
	{
		nbCalls++;
		totalTime+=time;
		if(time>peakTime)
			peakTime = time;

		if(1)
			printf("%d: time: %f ms | avg: %f ms | peak: %f ms | %d actors\n", nbCalls, time, totalTime/PxU64(nbCalls), peakTime, nbActors);
	}

	gTime.getElapsedSeconds();
	{
		PxRaycastBuffer buf;
		gScene->raycast(gOrigin, PxVec3(0.0f, -1.0f, 0.0f), 100.0f, buf);
		gHasRaycastHit = buf.hasBlock;
		if(buf.hasBlock)
			gRaycastHit = buf.block;
	}
	time = gTime.getElapsedSeconds()*1000.0;
	if(0)
		printf("raycast time: %f us\n", time*1000.0);

	gGlobalTime += dt;
}

void cleanupPhysics(bool /*interactive*/)
{
	delete gStreamer;
	PX_RELEASE(gMaterial);
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
	
	printf("SnippetMultiPruners done.\n");
}

void keyPress(unsigned char /*key*/, const PxTransform& /*camera*/)
{
}

static void runWithoutRendering()
{
	static const PxU32 frameCount = gNbFramesToSimulate;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
}

int snippetMain(int, const char*const*)
{
	printf("Multi Pruners snippet.\n");

#ifdef RENDER_SNIPPET
	if(gEnableRendering)
	{
		extern void renderLoop();
		renderLoop();
	}
	else
		runWithoutRendering();
#else
	runWithoutRendering();
#endif

	return 0;
}

