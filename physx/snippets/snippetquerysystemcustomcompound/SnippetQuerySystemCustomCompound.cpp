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
// This snippet demonstrates how to re-implement a 'compound pruner' in
// Gu::QuerySystem using PxCustomGeometry objects.
//
// Please get yourself familiar with SnippetQuerySystemAllQueries first.
//
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

#define MAX_SHAPES_PER_COMPOUND	8

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

static void fromLocalToGlobalSpace(PxLocationHit& hit, const PxTransform& pose)
{
	hit.position = pose.transform(hit.position);
	hit.normal = pose.rotate(hit.normal);
}

///////////////////////////////////////////////////////////////////////////////

// The following functions determine how we use the pruner payloads in this snippet

static PX_FORCE_INLINE void setupPayload(PrunerPayload& payload, PxU32 objectIndex, const PxGeometry* geom)
{
	payload.data[0] = objectIndex;
	payload.data[1] = size_t(geom);
}

static PX_FORCE_INLINE PxU32 getObjectIndexFromPayload(const PrunerPayload& payload)
{
	return PxU32(payload.data[0]);
}

static PX_FORCE_INLINE const PxGeometry& getGeometryFromPayload(const PrunerPayload& payload)
{
	return *reinterpret_cast<const PxGeometry*>(payload.data[1]);
}

///////////////////////////////////////////////////////////////////////////////

static CachedFuncs	gCachedFuncs;

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

	// We now derive CustomScene from PxCustomGeometry::Callbacks.
	class CustomScene : public Adapter, public PxCustomGeometry::Callbacks
	{
		public:
			CustomScene();
			~CustomScene()	{}

			// Adapter
			virtual	const PxGeometry&	getGeometry(const PrunerPayload& payload)	const;
			//~Adapter


			void	release();
			void	addCompound(const PxTransform& pose, bool isDynamic);
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

			// We derive our objects from PxCustomGeometry, to get easy access to the data in the PxCustomGeometry::Callbacks
			// functions. This generally wouldn't work in the regular PxScene / PhysX code since the geometries are copied around and the
			// system doesn't know about potential user-data surrounding the PxCustomGeometry objects. But the Gu::QuerySystem only
			// passes PxGeometry pointers around so it works here.
			// A more traditional usage of PxCustomGeometry would be to derive our Object from PxCustomGeometry::Callbacks (see e.g.
			// SnippetCustomConvex). Both approaches would work here.
			struct Object : public PxCustomGeometry
			{
				// Shape-related data
				// In this simple snippet we just keep all shapes in linear C-style arrays. A more efficient version could use a PxBVH here.
				PxBounds3			mCompoundLocalBounds;
				PxGeometryHolder	mShapeGeoms[MAX_SHAPES_PER_COMPOUND];
				PxTransform			mLocalPoses[MAX_SHAPES_PER_COMPOUND];
				PxU32				mNbShapes;

				// Compound/actor data. Note how mData is Gu::QuerySystem handle for the compound. We don't have
				// ActorShapeData values for each sub-shape, the Gu::QuerySystem doesn't know about these.
				ActorShapeData		mData;
				PxVec3				mTouchedColor;
				bool				mTouched;
			};

			PxU32			mNbObjects;
			Object			mObjects[MAX_NB_OBJECTS];
			QuerySystem*	mQuerySystem;
			PxU32			mPrunerIndex;

			DECLARE_CUSTOM_GEOMETRY_TYPE

			// PxCustomGeometry::Callbacks

			virtual PxBounds3 getLocalBounds(const PxGeometry& geom) const
			{
				// In this snippet we only fill the QuerySystem with our PxCustomGeometry-based objects. In a more complex app
				// we could have to test the type and cast to the appropriate type if needed.
				PX_ASSERT(geom.getType()==PxGeometryType::eCUSTOM);
				// Because our internal Objects are derived from custom geometries we have easy access to mCompoundLocalBounds:
				const Object& obj = static_cast<const Object&>(geom);
				return obj.mCompoundLocalBounds;
			}

			virtual bool generateContacts(const PxGeometry&, const PxGeometry&, const PxTransform&, const PxTransform&, const PxReal, const PxReal, const PxReal, PxContactBuffer&) const
			{
				// This is for contact generation, we don't use that here.
				return false;
			}

			virtual PxU32 raycast(const PxVec3& origin, const PxVec3& unitDir, const PxGeometry& geom, const PxTransform& pose,
									PxReal maxDist, PxHitFlags hitFlags, PxU32 maxHits, PxGeomRaycastHit* rayHits, PxU32 stride, PxRaycastThreadContext* context)	const
			{
				// There are many ways to implement this callback. This is just one example.

				// We retrieve our object, same as in "getLocalBounds" above.
				PX_ASSERT(geom.getType()==PxGeometryType::eCUSTOM);
				const Object& obj = static_cast<const Object&>(geom);

				// From the point-of-view of the QuerySystem a custom geometry is a single object, but from the point-of-view
				// of the user it can be anything, including a sub-scene (or as in this snippet, a compound).
				// The old API flags must be adapted to this new scenario, and we need to re-interpret what they all mean.

				// As discussed in SnippetQuerySystemAllQueries we don't use PxHitFlag::eMESH_MULTIPLE in the snippets, i.e.
				// from the point-of-view of the Gu::QuerySystem this raycast function against a unique (custom) geometry
				// should not return multiple hits. And indeed, "maxHits" is always 1 here.
				PX_UNUSED(stride);
				PX_UNUSED(maxHits);
				PX_ASSERT(maxHits==1);

				// The new flag PxHitFlag::eANY_HIT tells the system to report any hit from any geometry that contains more
				// than one primitive. This is equivalent to the old PxHitFlag::eMESH_ANY flag, but this is now also applicable
				// to PxCustomGeometry objects, not just triangle meshes.
				const bool anyHit = hitFlags & PxHitFlag::eANY_HIT;

				// We derive the object's index from the object's pointer. This is the counterpart of 'getObjectIndexFromPayload'
				// for regular geometries. We don't have a payload here since our sub-shapes are part of our compound
				// and hidden from the Gu::QuerySystem. Note that this is only used to highlight touched objects in the
				// render code, so this is all custom for this snippet and not mandatory in any way.
				const PxU32 objectIndex = PxU32(&obj - mObjects);

				// The ray is in world space, but the compound's shapes are in local space. We transform the ray from world space
				// to the compound's local space to do the raycast queries against these sub-shapes.
				const PxVec3 localOrigin = pose.transformInv(origin);
				const PxVec3 localDir = pose.q.rotateInv(unitDir);

				PxGeomRaycastHit localHit;
				PxGeomRaycastHit bestLocalHit;
				bestLocalHit.distance = maxDist;
				bool hasLocalHit = false;
				for(PxU32 i=0;i<obj.mNbShapes;i++)
				{
					// We need to replicate for our compound/sub-scene what was done in the calling code for a regular leaf node.
					// In particular that leaf code called 'validatePayload' (implemented later in the snippet) to retrieve the
					// geometry from a payload. We don't need to do that here since we have direct access to the geometries.
					// However we would still need to implement a preFilter function if needed, to replicate the second part of
					// 'validatePayload'. We aren't using a preFilter in this snippet though so this is missing here.
					const PxGeometry& currentGeom = obj.mShapeGeoms[i].any();
					const PxTransform& currentPose = obj.mLocalPoses[i];

					// We now raycast() against a sub-shape. Note how we only ask for one hit here, because eMESH_MULTIPLE was
					// not used in the snippet. We could call PxGeometryQuery::raycast here but it is more efficient to go through
					// the Gu-level function pointers directly, since it bypasses the PX_SIMD_GUARD entirely. We added one of these
					// when the query started (in CustomScene::runQueries()) so we can ignore them all in subsequent raycast calls.
					const RaycastFunc func = gCachedFuncs.mCachedRaycastFuncs[currentGeom.getType()];
					const PxU32 nbHits = func(currentGeom, currentPose, localOrigin, localDir, bestLocalHit.distance, hitFlags, 1, &localHit, sizeof(PxGeomRaycastHit), context);

					if(nbHits && localHit.distance<bestLocalHit.distance)
					{
						// We detected a valid hit. To simplify the code we decided to reuse the 'reportHits' function from our
						// own pruner raycast callback, which are passed to us by the query system as the context parameter. These
						// are the callbacks passed to 'mQuerySystem->raycast' later in this snippet. Specifically they will be
						// either DefaultPrunerRaycastClosestCallback, DefaultPrunerRaycastAnyCallback, or our own 
						// CustomRaycastMultipleCallback objects. All of them are DefaultPrunerRaycastCallback.
						PX_ASSERT(context);
						DefaultPrunerRaycastCallback* raycastCB = static_cast<DefaultPrunerRaycastCallback*>(context);

						// We moved the ray to the compound's local space so the hit is in this local compound space. We
						// need to transform the data back into world space. We cannot immediately tell in which raycast mode we are so
						// we do this conversion immediately. This is a bit less efficient than what we did in SnippetQuerySystemAllQueries,
						// where that conversion was delayed in the raycastClosest / raycastAny modes. We could be more efficient here
						// as well but it would make the code more complicated.
						fromLocalToGlobalSpace(localHit, pose);

						// We need a payload to call the 'reportHits' function so we create an artificial one here:
						PrunerPayload payload;
						setupPayload(payload, objectIndex, &currentGeom);

						// Reusing the reportHits function is mainly an easy way for us to report multiple hits from here. As
						// we mentioned above 'maxHits' is 1 and we cannot write out multiple hits to the 'rayHits' buffer, so
						// this is one alternative. In the 'multiple hits' codepath our own reportHits function will return false.
						// In that case we don't touch 'hasLocalHit' and we don't update 'bestLocalHit', so the code will use the same
						// (non shrunk) distance for next raycast calls in this loop, and we will return 0 from the query in
						// the end. The calling code doesn't need to know that we kept all the hits: we tell it that we
						// didn't find hits and it doesn't have to do any further processing.
						if(raycastCB->reportHits(payload, 1, &localHit))
						{
							// This is the single-hit codepath. We still need to know if we're coming from 'raycastAny' or from
							// 'raycastClosest'.
							if(anyHit)
							{
								// In 'raycastAny' mode we just write out the current hit and early exit.
								*rayHits = localHit;
								return 1;
							}

							// Otherwise in 'raycastClosest' mode we update the best hit, which will shrink the current distance,
							// and go on. We delay writing out the best hit (we don't have it yet).
							bestLocalHit = localHit;
							hasLocalHit = true;
						}
					}
				}

				// Last part of 'raycastClosest' mode, process best hit.
				if(hasLocalHit)
				{
					// In SnippetQuerySystemAllQueries this is where we'd convert the best hit back to world-space. In this
					// snippet we already did that above, so we only need to write out the best hit.
					*rayHits = bestLocalHit;
				}
				return hasLocalHit ? 1 : 0;
			}

			virtual bool overlap(const PxGeometry& geom0, const PxTransform& pose0, const PxGeometry& geom1, const PxTransform& pose1, PxOverlapThreadContext* context) const
			{
				// Bits similar to what we covered in the raycast callback above are not commented anymore here.
				PX_ASSERT(geom0.getType()==PxGeometryType::eCUSTOM);
				const Object& obj = static_cast<const Object&>(geom0);
				const PxU32 objectIndex = PxU32(&obj - mObjects);

				// This is the geom we passed to the OVERLAP_ANY / OVERLAP_MULTIPLE codepaths later in this snippet, i.e.
				// this is our own query volume.
				const PxGeometry& queryGeom = geom1;

				// Similar to what we did for the ray in the raycast callback, we need to convert the world-space query volume to our
				// compound's local space. Note our we convert the whole PxTransform here, not just the position (contrary to what we
				// did for raycasts).
				const PxTransform queryLocalPose(pose0.transformInv(pose1));

				for(PxU32 i=0;i<obj.mNbShapes;i++)
				{
					const PxGeometry& currentGeom = obj.mShapeGeoms[i].any();
					const PxTransform& currentPose = obj.mLocalPoses[i];

					if(Gu::overlap(queryGeom, queryLocalPose, currentGeom, currentPose, gCachedFuncs.mCachedOverlapFuncs, context))
					{
						PrunerPayload payload;
						setupPayload(payload, objectIndex, &currentGeom);

						// We use the same approach as for the raycast callback above. This time the context will be
						// either CustomOverlapAnyCallback or CustomOverlapMultipleCallback. Either way they are
						// DefaultPrunerOverlapCallback objects.
						PX_ASSERT(context);
						DefaultPrunerOverlapCallback* overlapCB = static_cast<DefaultPrunerOverlapCallback*>(context);

						// The 'overlapAny' case will return false, in which case we don't need to go through the
						// remaining sub-shapes.
						if(!overlapCB->reportHit(payload))
							return true;
					}
				}
				return false;
			}

			virtual bool sweep(const PxVec3& unitDir, const PxReal maxDist,
				const PxGeometry& geom0, const PxTransform& pose0, const PxGeometry& geom1, const PxTransform& pose1,
				PxGeomSweepHit& sweepHit, PxHitFlags hitFlags, const PxReal inflation, PxSweepThreadContext* context) const
			{
				// Sweeps are a combination of raycast (for the unitDir / maxDist parameters) and overlaps (for the
				// query-volume-related parameters). Bits already covered in the raycast and overlap callbacks above
				// will not be commented again here.
				PX_UNUSED(inflation);

				PX_ASSERT(geom0.getType()==PxGeometryType::eCUSTOM);
				const Object& obj = static_cast<const Object&>(geom0);
				const PxU32 objectIndex = PxU32(&obj - mObjects);
				const bool anyHit = hitFlags & PxHitFlag::eANY_HIT;

				// Bit subtle here: the internal system converts swept spheres to swept capsules (there is no internal
				// codepath for spheres to make the code smaller). So if we use a PxSphereGeometry in the SWEEP_CLOSEST
				// SWEEP_ANY / SWEEP_MULTIPLE codepaths later in this snippet, we actually receive it as a PxCapsuleGeometry
				// whose halfHeight = 0 here. This is not important in this snippet because we will use PxGeometryQuery::sweep
				// below, but it would be important if we'd decide to use the cached Gu-level functions (like we did for raycasts
				// and overlaps).
				const PxGeometry& queryGeom = geom1;

				// Convert both the query volume & ray to local space.
				const PxTransform queryLocalPose(pose0.transformInv(pose1));
				const PxVec3 localDir = pose0.q.rotateInv(unitDir);

				PxGeomSweepHit localHit;
				PxGeomSweepHit bestLocalHit;
				bestLocalHit.distance = maxDist;
				bool hasLocalHit = false;
				for(PxU32 i=0;i<obj.mNbShapes;i++)
				{
					const PxGeometry& currentGeom = obj.mShapeGeoms[i].any();
					const PxTransform& currentPose = obj.mLocalPoses[i];

					// Bit subtle here: we don't want to replicate the whole PxGeometryQuery::sweep() function directly
					// here so contrary to what we previously did, we do not use the gCachedFuncs sweep pointers directly.
					// We can still pass PxGeometryQueryFlag::Enum(0) to the system to tell it we already took care of the
					// SIMD guards. This optimization is not as important for sweeps as it was for raycasts, because the
					// sweeps themselves are generally much more expensive than raycasts, so the relative cost of the SIMD
					// guard is not as high.
					const PxU32 retVal = PxGeometryQuery::sweep(localDir, bestLocalHit.distance, queryGeom, queryLocalPose, currentGeom, currentPose, localHit, hitFlags, 0.0f, PxGeometryQueryFlag::Enum(0), context);
					if(retVal && localHit.distance<bestLocalHit.distance)
					{
						fromLocalToGlobalSpace(localHit, pose0);

						PrunerPayload payload;
						setupPayload(payload, objectIndex, &currentGeom);

						// Same approach as before, this time involving our own sweep callbacks.
						PX_ASSERT(context);
						DefaultPrunerSweepCallback* sweepCB = static_cast<DefaultPrunerSweepCallback*>(context);

						if(sweepCB->reportHit(payload, localHit))
						{
							if(anyHit)
							{
								sweepHit = localHit;
								return 1;
							}

							bestLocalHit = localHit;
							hasLocalHit = true;
						}
					}
				}

				if(hasLocalHit)
				{
					sweepHit = bestLocalHit;
				}
				return hasLocalHit ? 1 : 0;
			}

			virtual void visualize(const PxGeometry&, PxRenderOutput&, const PxTransform&, const PxBounds3&) const
			{
			}

			virtual void computeMassProperties(const PxGeometry&, PxMassProperties&) const
			{
			}

			virtual bool usePersistentContactManifold(const PxGeometry&, PxReal&) const
			{
				return false;
			}

			//~PxCustomGeometry::Callbacks

	};
	IMPLEMENT_CUSTOM_GEOMETRY_TYPE(CustomScene)

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
		addCompound(PxTransform(PxVec3(0.0f, 0.0f, 0.0f)), true);
	}

#ifdef RENDER_SNIPPET
	Camera* camera = getCamera();
	camera->setPose(gCamPos[gSceneIndex], gCamDir[gSceneIndex]);
#endif
}

void CustomScene::addCompound(const PxTransform& pose, bool isDynamic)
{
	PX_ASSERT(mQuerySystem);

	Object& obj = mObjects[mNbObjects];
	obj.callbacks = this;

	PrunerPayload payload;
	setupPayload(payload, mNbObjects, static_cast<const PxCustomGeometry*>(&obj));

	{
		const PxBoxGeometry boxGeom(PxVec3(1.0f, 2.0f, 0.5f));
		const PxTransform boxGeomPose(PxVec3(0.0f, 0.0f, 0.0f));

		const PxSphereGeometry sphereGeom(1.5f);
		const PxTransform sphereGeomPose(PxVec3(3.0f, 0.0f, 0.0f));

		const PxCapsuleGeometry capsuleGeom(1.0f, 1.0f);
		const PxTransform capsuleGeomPose(PxVec3(-3.0f, 0.0f, 0.0f));

		const PxConvexMeshGeometry convexGeom(gConvexMesh);
		const PxTransform convexGeomPose(PxVec3(0.0f, 0.0f, 3.0f));

		const PxTriangleMeshGeometry meshGeom(gTriangleMesh);
		const PxTransform meshGeomPose(PxVec3(0.0f, 0.0f, -3.0f));

		obj.mShapeGeoms[0].storeAny(boxGeom);
		obj.mShapeGeoms[1].storeAny(sphereGeom);
		obj.mShapeGeoms[2].storeAny(capsuleGeom);
		obj.mShapeGeoms[3].storeAny(convexGeom);
		obj.mShapeGeoms[4].storeAny(meshGeom);

		obj.mLocalPoses[0] = boxGeomPose;
		obj.mLocalPoses[1] = sphereGeomPose;
		obj.mLocalPoses[2] = capsuleGeomPose;
		obj.mLocalPoses[3] = convexGeomPose;
		obj.mLocalPoses[4] = meshGeomPose;

		obj.mNbShapes = 5;

		// Precompute local bounds for our compound
		PxBounds3 localCompoundBounds = PxBounds3::empty();
		for(PxU32 i=0;i<obj.mNbShapes;i++)
		{
			PxBounds3 localShapeBounds;
			PxGeometryQuery::computeGeomBounds(localShapeBounds, obj.mShapeGeoms[i].any(), obj.mLocalPoses[i]);

			localCompoundBounds.include(localShapeBounds);
		}
		obj.mCompoundLocalBounds = localCompoundBounds;
	}

	if(gManualBoundsComputation)
	{
		PxBounds3 bounds;
		PxGeometryQuery::computeGeomBounds(bounds, obj, pose, 0.0f, 1.0f + gBoundsInflation);
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
			PxGeometryQuery::computeGeomBounds(bounds, obj, pose, 0.0f, 1.0f + gBoundsInflation);
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

///////////////////////////////////////////////////////////////////////////////

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

bool CustomScene::raycastAny(const PxVec3& origin, const PxVec3& unitDir, float maxDist) const
{
	DefaultPrunerRaycastAnyCallback CB(gFilterCallback, gCachedFuncs.mCachedRaycastFuncs, origin, unitDir, maxDist);

	mQuerySystem->raycast(origin, unitDir, maxDist, CB, NULL);

	return CB.mFoundHit;
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	struct CustomRaycastMultipleCallback : public DefaultPrunerRaycastCallback
	{
		PxGeomRaycastHit			mLocalHit;
		PxArray<CustomRaycastHit>&	mHits;

		CustomRaycastMultipleCallback(PxArray<CustomRaycastHit>& hits, PrunerFilterCallback& filterCB, const GeomRaycastTable& funcs, const PxVec3& origin, const PxVec3& dir, float distance) :
			DefaultPrunerRaycastCallback(filterCB, funcs, origin, dir, distance, 1, &mLocalHit, PxHitFlag::eDEFAULT, false),
			mHits						(hits)	{}

		virtual	bool	reportHits(const PrunerPayload& payload, PxU32 nbHits, PxGeomRaycastHit* hits)
		{
			PX_ASSERT(nbHits==1);
			PX_UNUSED(nbHits);

			CustomRaycastHit customHit;
			static_cast<PxGeomRaycastHit&>(customHit) = hits[0];
			customHit.mObjectIndex = getObjectIndexFromPayload(payload);

			mHits.pushBack(customHit);

			return false;
		}

		PX_NOCOPY(CustomRaycastMultipleCallback)
	};
}

bool CustomScene::raycastMultiple(const PxVec3& origin, const PxVec3& unitDir, float maxDist, PxArray<CustomRaycastHit>& hits) const
{
	CustomRaycastMultipleCallback CB(hits, gFilterCallback, gCachedFuncs.mCachedRaycastFuncs, origin, unitDir, maxDist);

	mQuerySystem->raycast(origin, unitDir, maxDist, CB, NULL);

	return hits.size()!=0;
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	template<class CallbackT>
	static bool _sweepClosestT(CustomSweepHit& hit, const CustomScene& cs, const PxGeometry& geom, const PxTransform& pose, const PxVec3& unitDir, float maxDist)
	{
		const ShapeData queryVolume(geom, pose, 0.0f);

		CallbackT pcb(gFilterCallback, gCachedFuncs.mCachedSweepFuncs, geom, pose, queryVolume, unitDir, maxDist, PxHitFlag::eDEFAULT | PxHitFlag::ePRECISE_SWEEP, false);

		cs.mQuerySystem->sweep(queryVolume, unitDir, pcb.mClosestHit.distance, pcb, NULL);

		if(pcb.mFoundHit)
		{
			static_cast<PxGeomSweepHit&>(hit) = pcb.mClosestHit;
			hit.mObjectIndex = getObjectIndexFromPayload(pcb.mClosestPayload);
		}
		return pcb.mFoundHit;
	}
}

bool CustomScene::sweepClosest(const PxGeometry& geom, const PxTransform& pose, const PxVec3& unitDir, float maxDist, CustomSweepHit& hit) const
{
	switch(PxU32(geom.getType()))
	{
		case PxGeometryType::eSPHERE:			{ return _sweepClosestT<DefaultPrunerSphereSweepCallback>(hit, *this, geom, pose, unitDir, maxDist);	}
		case PxGeometryType::eCAPSULE:			{ return _sweepClosestT<DefaultPrunerCapsuleSweepCallback>(hit, *this, geom, pose, unitDir, maxDist);	}
		case PxGeometryType::eBOX:				{ return _sweepClosestT<DefaultPrunerBoxSweepCallback>(hit, *this, geom, pose, unitDir, maxDist);		}
		case PxGeometryType::eCONVEXMESH:		{ return _sweepClosestT<DefaultPrunerConvexSweepCallback>(hit, *this, geom, pose, unitDir, maxDist);	}
		default:								{ PX_ASSERT(0); return false;																			}
	}
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
	template<class CallbackT>
	static bool _sweepAnyT(const CustomScene& cs, const PxGeometry& geom, const PxTransform& pose, const PxVec3& unitDir, float maxDist)
	{
		const ShapeData queryVolume(geom, pose, 0.0f);

		CallbackT pcb(gFilterCallback, gCachedFuncs.mCachedSweepFuncs, geom, pose, queryVolume, unitDir, maxDist, PxHitFlag::eANY_HIT | PxHitFlag::ePRECISE_SWEEP, true);

		cs.mQuerySystem->sweep(queryVolume, unitDir, pcb.mClosestHit.distance, pcb, NULL);

		return pcb.mFoundHit;
	}
}

bool CustomScene::sweepAny(const PxGeometry& geom, const PxTransform& pose, const PxVec3& unitDir, float maxDist) const
{
	switch(PxU32(geom.getType()))
	{
		case PxGeometryType::eSPHERE:			{ return _sweepAnyT<DefaultPrunerSphereSweepCallback>(*this, geom, pose, unitDir, maxDist);		}
		case PxGeometryType::eCAPSULE:			{ return _sweepAnyT<DefaultPrunerCapsuleSweepCallback>(*this, geom, pose, unitDir, maxDist);	}
		case PxGeometryType::eBOX:				{ return _sweepAnyT<DefaultPrunerBoxSweepCallback>(*this, geom, pose, unitDir, maxDist);		}
		case PxGeometryType::eCONVEXMESH:		{ return _sweepAnyT<DefaultPrunerConvexSweepCallback>(*this, geom, pose, unitDir, maxDist);		}
		default:								{ PX_ASSERT(0); return false;																	}
	}
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
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

	template<class CallbackT>
	static bool _sweepMultipleT(PxArray<CustomSweepHit>& hits, const CustomScene& cs, const PxGeometry& geom, const PxTransform& pose, const PxVec3& unitDir, float maxDist)
	{
		const ShapeData queryVolume(geom, pose, 0.0f);

		CustomSweepMultipleCallback<CallbackT> pcb(hits, gFilterCallback, gCachedFuncs.mCachedSweepFuncs, geom, pose, queryVolume, unitDir, maxDist);

		cs.mQuerySystem->sweep(queryVolume, unitDir, pcb.mClosestHit.distance, pcb, NULL);

		return hits.size()!=0;
	}
}

bool CustomScene::sweepMultiple(const PxGeometry& geom, const PxTransform& pose, const PxVec3& unitDir, float maxDist, PxArray<CustomSweepHit>& hits) const
{
	switch(PxU32(geom.getType()))
	{
		case PxGeometryType::eSPHERE:			{ return _sweepMultipleT<DefaultPrunerSphereSweepCallback>(hits, *this, geom, pose, unitDir, maxDist);	}
		case PxGeometryType::eCAPSULE:			{ return _sweepMultipleT<DefaultPrunerCapsuleSweepCallback>(hits, *this, geom, pose, unitDir, maxDist);	}
		case PxGeometryType::eBOX:				{ return _sweepMultipleT<DefaultPrunerBoxSweepCallback>(hits, *this, geom, pose, unitDir, maxDist);		}
		case PxGeometryType::eCONVEXMESH:		{ return _sweepMultipleT<DefaultPrunerConvexSweepCallback>(hits, *this, geom, pose, unitDir, maxDist);	}
		default:								{ PX_ASSERT(0); return false;																			}
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
			return false;
		}
	};
}

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
			mHits.pushBack(getObjectIndexFromPayload(payload));
			return true;
		}

		PX_NOCOPY(CustomOverlapMultipleCallback)
	};
}

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

	const PxVec3 touchedColor(0.25f, 0.5f, 1.0f);
	for(PxU32 i=0;i<mNbObjects;i++)
	{
		mObjects[i].mTouched = false;
		mObjects[i].mTouchedColor = touchedColor;
	}

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

			//const PxSphereGeometry sweptGeom(0.5f);
			const PxCapsuleGeometry sweptGeom(0.5f, 0.5f);
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
			//const PxSphereGeometry sweptGeom(0.5f);
			PxQuat q(1.1f, 0.1f, 0.8f, 1.4f);
			q.normalize();
			const PxTransform pose(origin, q);

			const bool hasHit = sweepAny(sweptGeom, pose, unitDir, maxDist);

#ifdef RENDER_SNIPPET
			const PxGeometryHolder gh(sweptGeom);
			{
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

//			const PxCapsuleGeometry sweptGeom(0.5f, 0.5f);
			const PxSphereGeometry sweptGeom(0.5f);
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

			//const PxSphereGeometry queryGeom(0.5f);
			//const PxCapsuleGeometry queryGeom(0.5f, 0.5f);
			const PxBoxGeometry queryGeom(1.0f, 0.25f, 0.5f);
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

//			const PxSphereGeometry queryGeom(0.5f);
			const PxCapsuleGeometry queryGeom(0.5f, 0.5f);
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

		//DrawBounds(*ppd.mBounds);

		const PxVec3& objectColor = obj.mTouched ? obj.mTouchedColor : color;

		// renderGeoms doesn't support PxCustomGeometry so we deal with this here:
		PxTransform shapeGlobalPoses[MAX_SHAPES_PER_COMPOUND];
		for(PxU32 j=0;j<obj.mNbShapes;j++)
		{
			// This is basically PxShapeExt::getGlobalPose with the actor's global pose == *ppd.mTransform
			shapeGlobalPoses[j] = (*ppd.mTransform) * obj.mLocalPoses[j];
		}
		renderGeoms(obj.mNbShapes, obj.mShapeGeoms, shapeGlobalPoses, false, objectColor);
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

	printf("SnippetQuerySystemCustomCompound done.\n");
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
	printf("Query System Custom Compound snippet.\n");
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

