// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * Umbrella header for the parse library entry points. Each parseX function
 * is the surface form of a per-concept requirement — entry-point existence
 * and source-agnostic signature are demonstrated here; behavioural ACs
 * (byte-equivalence, edge-case handling, fallback) are covered by the
 * corresponding implementation files.
 *
 * @implements REQ-PARSE-CORE-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-PARSE-MAT-001
 * @covers AC-4
 *
 * @implements REQ-PARSE-BODY-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-BODY-002
 * @covers AC-1
 *
 * @implements REQ-PARSE-MASS-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-JOINT-001
 * @covers AC-1 AC-3
 *
 * @implements REQ-PARSE-JOINT-002
 * @covers AC-1
 *
 * @implements REQ-PARSE-ART-001
 * @covers AC-3
 *
 * @implements REQ-PARSE-COL-001
 * @covers AC-3
 *
 * @implements REQ-PARSE-COL-002
 * @covers AC-1 AC-2
 *
 * @implements REQ-PARSE-COL-003
 * @covers AC-1
 *
 * @implements REQ-PARSE-SCENE-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-SHAPE-001
 * @covers AC-1 AC-3
 *
 * @implements REQ-PARSE-SHAPE-002
 * @covers AC-1 AC-3
 *
 * @implements REQ-PARSE-FILTER-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-COLGROUP-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-ATTACH-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-DEF-001
 * @covers AC-1
 */

#pragma once

#include "Descriptors.h"
#include "ParseContext.h"

namespace omni::physics::parse
{

// ---------------------------------------------------------------------------
// Per-concept parse entry points.
//
// Each function reads attribute data from ctx.source() and returns a populated
// descriptor. The returned descriptor is owned by the caller.
// ---------------------------------------------------------------------------

DescPtr<PhysxMaterialDesc> parseMaterial(ParseContext& ctx, ObjectKey key);

// PBD particle-material reader. Reads `PhysxSchemaPhysxPBDMaterialAPI`
// attribute values onto a freshly-allocated PBDMaterialDesc when the
// API is applied; returns nullptr otherwise.
DescPtr<PBDMaterialDesc> parsePBDMaterial(ParseContext& ctx, ObjectKey key);

// Particle system reader. Reads the PhysxParticleSystem prim's offsets (with
// metersPerUnit-aware autocompletion + schema lower-limit enforcement), solver/
// neighborhood/wind/lockedAxis config, sub-API enable bools, and scene/filtered
// cross-refs (as source ObjectKeys). Runtime ObjectId resolution for material /
// collisionGroup stays consumer-side.
DescPtr<ParticleSystemDesc> parseParticleSystem(ParseContext& ctx, ObjectKey key);

// Particle-system sub-API readers (Anisotropy / Smoothing / Isosurface APIs
// applied on the PhysxParticleSystem prim). Each returns nullptr when its API
// is not applied. parseParticleIsosurface takes the system's resolved
// fluidRestOffset for the grid-spacing / surface-distance autocompletion.
DescPtr<ParticleAnisotropyDesc> parseParticleAnisotropy(ParseContext& ctx, ObjectKey key);
DescPtr<ParticleSmoothingDesc>  parseParticleSmoothing(ParseContext& ctx, ObjectKey key);
DescPtr<ParticleIsosurfaceDesc> parseParticleIsosurface(ParseContext& ctx, ObjectKey key, float fluidRestOffset);

// Particle set reader. Reads a PhysxParticleSetAPI-bearing UsdGeomPointBased /
// PointInstancer prim — points/velocities/simulationPoints arrays, flags,
// maxParticles, the diffuse-particles sub-API (inline), the particleSystem rel
// (as a source key), and the scene/rest-offset fields copied from its system.
// Returns nullptr when the API is not applied.
DescPtr<ParticleSetDesc> parseParticleSet(ParseContext& ctx, ObjectKey key);

// Particle sampler reader. Reads a PhysxParticleSamplingAPI-bearing UsdGeomMesh
// — samplingDistance (autocompleted from the referenced set's system point
// width), maxSamples, sampleVolume, and the particles rel (as a source key).
// Returns nullptr when not a sampling-API mesh.
DescPtr<ParticleSamplingDesc> parseParticleSampling(ParseContext& ctx, ObjectKey key);

// Deformable-material reader (overlay). Reads
// `OmniPhysicsBaseMaterialAPI` (dynamicFriction / staticFriction /
// density) and `OmniPhysicsDeformableMaterialAPI` (youngsModulus /
// poissonsRatio) onto `desc` when the matching API is applied,
// leaving fields untouched otherwise. Caller is responsible for
// setToDefault before calling.
void parseDeformableMaterial(ParseContext& ctx, ObjectKey key, PhysxDeformableMaterialDesc& desc);

// Surface-deformable-material reader (overlay). Reads
// `OmniPhysicsSurfaceDeformableMaterialAPI` attrs onto `desc` (after
// the deformable-material overlay).
void parseSurfaceDeformableMaterial(ParseContext& ctx, ObjectKey key, PhysxSurfaceDeformableMaterialDesc& desc);

// ---------------------------------------------------------------------------
// Rigid body parsing.
//
// setToDefault populates *all* PhysxRigidBodyAPI-extension fields with the
// units-aware defaults the runtime expects. Parsers below ALWAYS call
// setToDefault first, then override fields when the corresponding source
// schema is applied. Callers that consume the descriptor without going
// through these parsers must invoke setToDefault themselves.
// ---------------------------------------------------------------------------

void setToDefault(DynamicPhysxRigidBodyDesc& desc, const SourceUnits& units);

DescPtr<DynamicPhysxRigidBodyDesc> parseDynamicBody(ParseContext& ctx, ObjectKey key);
DescPtr<StaticPhysxRigidBodyDesc>  parseStaticBody(ParseContext& ctx, ObjectKey key);

// ---------------------------------------------------------------------------
// Scene parsing.
//
// Gravity direction/magnitude resolution and the quasistaticActors collection
// expansion are USD-specific work (the schema parser bakes the up-axis-derived
// default gravity into UsdPhysicsScene's GfVec3f, and `UsdCollectionAPI` walks
// the IncludesRel + UsdTraverseInstanceProxies to enumerate prims). The
// consumer hands us the resolved bits in `SceneInfo`; the parse library reads
// PhysxSceneAPI / PhysxSceneQuasistaticAPI / Newton fallbacks on top and writes
// the result into a passed-in `PhysxSceneDesc`. The four nested material
// descriptors (defaultMaterialDesc, defaultPBDMaterialDesc, etc.) are NOT
// touched — the consumer's material-binding parsers populate those before
// this runs.
// ---------------------------------------------------------------------------

struct SceneInfo
{
    // From the schema parser (UsdPhysicsScene): gravity direction + magnitude.
    // Already carries up-axis-derived defaults if no value was authored.
    carb::Float3 gravityDirection = { 0.0f, 0.0f, -1.0f };
    float gravityMagnitude = 9.81f;

    // PhysxSceneQuasistaticAPI's quasistaticActors collection, pre-resolved to
    // ObjectKeys by the routing site. Empty when the API isn't applied or the
    // collection has no included paths.
    std::vector<ObjectKey> quasistaticActors;
};

// Sets units-aware defaults on `desc`. Does NOT initialise the four
// nested material descriptors — those are populated by the consumer's
// material-binding parsers (or by separate calls to setToDefault for
// the per-material-desc types).
void setToDefault(PhysxSceneDesc& desc, const SourceUnits& units);

// Allocates a synthetic default scene descriptor for the case where a source
// has no authored PhysicsScene. Applies setToDefault(units) and marks the result
// `synthetic = true` (so the consumer can skip source-existence/ownership gates
// that assume a real prim). The caller assigns `primKey`. An unscoped OVStage
// scan uses this when no scene exists. Scoped scans emit no fallback because
// they cannot prove the whole stage has no scene; `LoadStage` owns the separate
// scene-less initial-load fallback.
// Returns a null DescPtr if allocation fails.
DescPtr<PhysxSceneDesc> makeDefaultSceneDesc(IDescriptorAllocator& allocator, const SourceUnits& units);

// Reads PhysxSceneAPI / PhysxSceneQuasistaticAPI / Newton fallbacks onto `desc`.
// `desc` is expected to be initialised via setToDefault first; gravity is then
// overwritten from `info`. Nested material descriptors are left untouched.
void parseScene(ParseContext& ctx, ObjectKey key, const SceneInfo& info, PhysxSceneDesc& desc);

// ---------------------------------------------------------------------------
// Mass parsing.
//
// MassApiData carries the directly-authored UsdPhysicsMassAPI fields —
// a partial mass description. The full RigidBodyMass aggregation
// (combining authored mass with mass derived from child collision
// shapes via cooking) is the consumer's job — it depends on PhysX
// cooking and isn't part of the parse library.
// ---------------------------------------------------------------------------

struct MassApiData
{
    float mass = -1.0f;
    float density = -1.0f;
    bool hasInertia = false;
    carb::Float3 diagonalInertia = { 1.0f, 1.0f, 1.0f };
    bool hasCenterOfMass = false;
    carb::Float3 centerOfMass = { 0.0f, 0.0f, 0.0f };
    bool hasPrincipalAxes = false;
    carb::Float4 principalAxes = { 0.0f, 0.0f, 0.0f, 1.0f };
};

MassApiData parseMassApi(ParseContext& ctx, ObjectKey key);

// ---------------------------------------------------------------------------
// Joint parsing.
//
// Joint resolution (type, body relationships, local poses, per-axis limits
// and drives) is USD-specific work performed by Pixar's UsdPhysicsLoad-
// StageFromPrimRange. Rather than reimplement it, we let the consumer pass
// the already-resolved info as `JointInfo`, then this layer reads PhysX
// extension fields on top. Non-USD sources construct JointInfo natively.
// ---------------------------------------------------------------------------

struct JointLimitInfo
{
    bool enabled = false;
    // For angular limits authored in degrees the consumer translates into
    // these fields via the union semantics of `JointLimitInfo`.
    float lower = 0.0f;
    float upper = 0.0f;
};

struct JointDriveInfo
{
    bool enabled = false;
    float targetPosition = 0.0f;
    float targetVelocity = 0.0f;
    float forceLimit = FLT_MAX;
    float stiffness = 0.0f;
    float damping = 0.0f;
    bool acceleration = false;
};

struct JointInfo
{
    // ObjectType — same enum already in Descriptors.h. Not all joint types
    // carry per-axis info; this struct stores the union of fields and the
    // type tells the parser which are populated.
    ObjectType type = eUndefined;

    // Body relationship targets (resolved bodies) and the path that authored
    // the relationship. rel0/rel1 may differ from body0/body1 when the
    // relationship target is non-rigid (e.g. articulation root link).
    ObjectKey body0;
    ObjectKey body1;
    ObjectKey rel0;
    ObjectKey rel1;

    carb::Float3 localPose0Position = { 0.0f, 0.0f, 0.0f };
    carb::Float4 localPose0Orientation = { 0.0f, 0.0f, 0.0f, 1.0f };
    carb::Float3 localPose1Position = { 0.0f, 0.0f, 0.0f };
    carb::Float4 localPose1Orientation = { 0.0f, 0.0f, 0.0f, 1.0f };

    bool jointEnabled = true;
    float breakForce = 0.0f;        // consumer fills in (default FLT_MAX)
    float breakTorque = 0.0f;
    bool excludeFromArticulation = false;
    bool collisionEnabled = false;

    // For typed joints (Spherical/Revolute/Prismatic) — the schema parser
    // pre-resolves the active axis (eX/eY/eZ). D6 joints carry per-axis
    // limit/drive lists below instead.
    Axis axis = eX;

    // Spherical/Revolute/Prismatic carry a single limit; D6 fills jointLimits.
    JointLimitInfo limit;

    // Prismatic/Revolute carry a single drive; D6 fills jointDrives.
    JointDriveInfo drive;

    // D6-specific: per-axis limits and drives.
    std::vector<std::pair<JointAxis, JointLimitInfo>> jointLimits;
    std::vector<std::pair<JointAxis, JointDriveInfo>>  jointDrives;

    // Distance-specific: separate flags for min/max enabled.
    bool minEnabled = false;
    bool maxEnabled = false;
};

DescPtr<PhysxJointDesc> parseJoint(ParseContext& ctx, ObjectKey key, const JointInfo& info);

// ---------------------------------------------------------------------------
// Articulation parsing.
//
// Articulation enumeration (root prims, articulated bodies, articulated joints,
// filtered collisions) is USD-specific work the schema parser handles. This
// layer reads the units-aware defaults and the PhysxArticulationAPI extension
// fields onto each PhysxArticulationDesc.
// ---------------------------------------------------------------------------

struct ArticulationFields
{
    int  solverPositionIterationCount = 32;
    int  solverVelocityIterationCount = 1;
    float sleepThreshold = 0.0f;          // populated from SourceUnits + extensions
    float stabilizationThreshold = 0.0f;  // populated from SourceUnits + extensions
    bool selfCollision = true;
    bool articulationEnabled = true;
};

// Sets units-aware defaults on the units-dependent fields.
void setToDefault(ArticulationFields& fields, const SourceUnits& units);

// Reads PhysxArticulationAPI overrides (and Newton's selfCollisionEnabled
// fallback) on top of `fields`. `fields` is expected to be initialized
// via setToDefault first.
void parseArticulation(ParseContext& ctx, ObjectKey key, ArticulationFields& fields);

// ---------------------------------------------------------------------------
// Collision extension parsing.
//
// Shape geometry, mesh data, cooking, and the contactOffset/restOffset
// inf-sentinel + time-sample-registration interplay stay in the
// USD-coupled walker path. This layer reads the simple PhysxCollisionAPI
// extension fields that have no time-sample dependency.
// ---------------------------------------------------------------------------

struct CollisionExtFields
{
    // Default 0.0f; populated from the PhysxCollisionAPI authored value.
    float torsionalPatchRadius = 0.0f;
    float minTorsionalPatchRadius = 0.0f;

    // PhysxTriggerAPI / PhysxTriggerStateAPI presence flags — only ever
    // set to true (never reset). Caller passes the current outDesc value
    // in and the parser ORs in the per-prim schema presence.
    bool isTrigger = false;
    bool isTriggerUsdOutput = false;

    // contactOffset / restOffset reads from PhysxCollisionAPI. Caller
    // passes the current outDesc values in; the parser applies the
    // inf-sentinel + range validation and writes back, but only when
    // the attribute was *authored* (HasAuthoredValue, not just HasValue
    // — schema fallback `-inf` must NOT trigger the inf-sentinel write).
    //
    // The {contactOffset, restOffset}Authored bits surface to the
    // routing site so the Newton fallback (newton:contactGap /
    // newton:contactMargin) can gate on them.
    //
    // The cross-validation logic — contactOffset >= restOffset /
    // restOffset < contactOffset — is applied internally before writing
    // back.
    float contactOffset = 0.0f;
    float restOffset = 0.0f;
    bool contactOffsetAuthored = false;
    bool restOffsetAuthored = false;
};

void parseCollisionExt(ParseContext& ctx, ObjectKey key, CollisionExtFields& fields);

// ---------------------------------------------------------------------------
// Shape parsing (simple shapes: Sphere/Box/Capsule/Cylinder/Cone/Plane/
//                SpherePoints/Custom).
//
// Mesh shapes — and their cooking-service invocation — live in the
// consumer-side walker; this header covers only the simple-shape parsers.
// The schema parser pre-resolves the shape type, geometry parameters
// (radius/halfExtents/axis), and the common ShapeInfo bits (transform,
// owners, collision group); each per-type parser below produces the
// matching `*PhysxShapeDesc` and overlays PhysX extension reads.
// ---------------------------------------------------------------------------

struct ShapeInfo
{
    bool collisionEnabled = true;
    ObjectKey rigidBody;
    ObjectKey sourceGprim;
    ObjectId collisionGroup = kInvalidObjectId;
    carb::Float3 localPos     = { 0.0f, 0.0f, 0.0f };
    carb::Float4 localRot     = { 0.0f, 0.0f, 0.0f, 1.0f };
    carb::Float3 localScale   = { 1.0f, 1.0f, 1.0f };

    // Schema-resolved scene owners. parseShape resolves these to
    // `desc.sceneIds` via ParseContext::objects()::findEntry(owner, eScene);
    // a non-empty `simulationOwners` with no resolvable scenes causes
    // parseShape to return nullptr.
    std::vector<ObjectKey> simulationOwners;
};

// Sets units-aware shape defaults. `desc.type` must already be set
// (ePlaneShape gets a different contactOffset default than other shapes).
void setToDefault(PhysxShapeDesc& desc, const SourceUnits& units);

// Per-shape-type parsers. Each:
//   1. Allocates the typed descriptor.
//   2. Calls setToDefault for units-aware contactOffset/restOffset/etc.
//   3. Copies common fields from `info`.
//   4. Sets type-specific fields from the trailing parameters.
//   5. Runs `parseCollisionExt` to overlay PhysX extension fields and
//      Newton contactMargin/contactGap fallbacks.
//   6. Resolves `info.simulationOwners` to scene ObjectIds.
// Returns nullptr when `info.simulationOwners` is non-empty but no owner
// resolved to a scene.
DescPtr<SpherePhysxShapeDesc>       parseSphereShape  (ParseContext& ctx, ObjectKey key, const ShapeInfo& info, float radius);
DescPtr<BoxPhysxShapeDesc>          parseBoxShape     (ParseContext& ctx, ObjectKey key, const ShapeInfo& info, carb::Float3 halfExtents);
DescPtr<CapsulePhysxShapeDesc>      parseCapsuleShape (ParseContext& ctx, ObjectKey key, const ShapeInfo& info, float radius, float halfHeight, Axis axis);
DescPtr<CylinderPhysxShapeDesc>     parseCylinderShape(ParseContext& ctx, ObjectKey key, const ShapeInfo& info, float radius, float halfHeight, Axis axis);
DescPtr<ConePhysxShapeDesc>         parseConeShape    (ParseContext& ctx, ObjectKey key, const ShapeInfo& info, float radius, float halfHeight, Axis axis);
DescPtr<PlanePhysxShapeDesc>        parsePlaneShape   (ParseContext& ctx, ObjectKey key, const ShapeInfo& info, Axis axis);
DescPtr<SpherePointsPhysxShapeDesc> parseSpherePointsShape(ParseContext& ctx, ObjectKey key, const ShapeInfo& info, std::vector<SpherePhysxPoint> spheres);
DescPtr<CustomPhysxShapeDesc>       parseCustomShape  (ParseContext& ctx, ObjectKey key, const ShapeInfo& info, size_t customGeometryTokenHash);

// Allocate a scaled-points working buffer. Reads the points referenced by
// `points` (expected to be an `BufferElemType::eVec3` payload of `carb::Float3`),
// multiplies each component by `meshScale`, writes into `out`. This is
// the per-axis component scale applied before handing points to the
// PhysX OBB/bounding-sphere compute. Returns silently on invalid handle
// — `out` is left empty.
void scaleMeshPoints(ParseContext& ctx, BufferHandle points,
                     carb::Float3 meshScale, std::vector<carb::Float3>& out);

// Bake an instance-proxy's world scale into a shape descriptor's geometry
// fields in place. Per-shape-type radius/halfExtents/meshScale
// adjustment; does NOT touch localPos / localScale (those depend on the
// USD-stage / xfCache state the parse-lib doesn't own — the caller
// sets them).
//
// Used by `scanStage::emitShape` when `inDesc.masterDesc=true` so each
// instance proxy gets a descriptor with the instance's world scale baked
// into the geometry (REQ-PARSE-CONSUMER-001 AC-13).
void scaleShapeDescByInstance(PhysxShapeDesc& desc, carb::Float3 instanceScale);

// Construct a BoundingSpherePhysxShapeDesc from already-computed bounds.
// The PhysX cooking-driven OBB / bounding-sphere compute (`PxCreateConvexMesh`)
// stays at the routing site since it requires the full PhysX SDK; the
// parse-lib only owns the descriptor-construction shell + common shape fill.
// Returns nullptr when `info.simulationOwners` cannot resolve to scenes
// (matches `parseSphereShape` etc.).
DescPtr<BoundingSpherePhysxShapeDesc> makeBoundingSphereShape(
    ParseContext& ctx, ObjectKey key, const ShapeInfo& info,
    carb::Float3 sphereCenter, float radius);

// Construct a BoundingBoxPhysxShapeDesc from already-computed OBB params.
DescPtr<BoundingBoxPhysxShapeDesc> makeBoundingBoxShape(
    ParseContext& ctx, ObjectKey key, const ShapeInfo& info,
    carb::Float3 halfExtents, carb::Float3 offsetPos, carb::Float4 offsetRot);

// Mesh-shape approximation discriminator. Read from
// `UsdPhysicsMeshCollisionAPI:physics:approximation` (schema name
// `PhysicsMeshCollisionAPI`). When the API is not applied or the
// attribute is unauthored, the schema fallback is `none`.
//
// Token mapping:
//   "none" -> eNone               (UsdPhysicsTokens)
//   "convexHull" -> eConvexHull         (UsdPhysicsTokens)
//   "boundingSphere" -> eBoundingSphere     (UsdPhysicsTokens)
//   "boundingCube" -> eBoundingCube       (UsdPhysicsTokens)
//   "meshSimplification" -> eMeshSimplification (UsdPhysicsTokens)
//   "convexDecomposition"-> eConvexDecomposition(PhysxSchemaTokens)
//   "sphereFill" -> eSphereFill         (PhysxSchemaTokens)
//   "sdf" -> eSdf                (PhysxSchemaTokens)
// Any other token value (or no API) falls through to eNone — the
// consumer's fallback path treats that as a triangle mesh.
enum class MeshApproximation : uint32_t
{
    eNone = 0,            // triangle mesh, original triangles
    eConvexHull,
    eBoundingSphere,
    eBoundingCube,
    eMeshSimplification,
    eConvexDecomposition,
    eSphereFill,
    eSdf,
};

MeshApproximation parseMeshApproximation(ParseContext& ctx, ObjectKey key);

// Resolve a mesh-typed prim's raw geometry into BufferHandles + scalars.
// Wraps IPhysicsSource::getMeshAttributes; the source does the heavy
// lifting (registering VtArrays as buffers etc.). The returned
// `BufferHandle`s carry the same 128-bit fnv content hash the cooking
// service uses internally — see Sh1 for the registry shape. Use
// ParseContext::getBuffer<T> to access the typed bytes.
MeshGeometry parseMeshGeometry(ParseContext& ctx, ObjectKey key);

// Per-shape-type cooking-API extension readers. These overlay schema-
// authored cooking knobs on top of `params`, which the caller seeds
// with cooking-defaults or current desc state. The actual cooking call
// (e.g. cookingService->requestConvexMeshCookedData) stays on the
// consumer side — only the schema-read side lives here.
void parseConvexHullCookingExt(ParseContext& ctx, ObjectKey key, ConvexMeshCookingParams& params);
void parseConvexDecompositionCookingExt(ParseContext& ctx, ObjectKey key, ConvexDecompositionCookingParams& params);
void parseSphereFillCookingExt(ParseContext& ctx, ObjectKey key, SphereFillCookingParams& params);

// Reads `PhysxTriangleMeshCollisionAPI:weldTolerance` (no schema-applied
// gate). After the read, applies a NaN → -FLT_MAX guard.
void parseTriangleMeshCookingExt(ParseContext& ctx, ObjectKey key, TriangleMeshCookingParams& params);

// Reads `PhysxTriangleMeshSimplificationCollisionAPI:metric` and
// `:weldTolerance` (also no schema-applied gate). Applies the same NaN
// guard on weldTolerance.
void parseTriangleMeshSimplificationCookingExt(ParseContext& ctx, ObjectKey key, TriangleMeshCookingParams& params);

// Reads `PhysxSDFMeshCollisionAPI` cooking knobs into `params`. Gated:
// only writes fields if the schema is applied AND `sdfResolution > 0`.
// Returns true when the SDF would be considered valid for cooking; the
// caller uses this to choose between SDF and triangle-mesh cooking.
bool parseSdfMeshCookingExt(ParseContext& ctx, ObjectKey key, SdfMeshCookingParams& params);

// ---------------------------------------------------------------------------
// Filtered pairs.
//
// `PhysicsFilteredPairsAPI` is a multi-applicable schema attached to bodies,
// collision shapes, articulation roots, and deformable bodies; it lists per-
// prim pairs to filter out of contact generation via the
// `physics:filteredPairs` relationship. Each consumer descriptor stores the
// per-prim filter as a flat ObjectKey list; this helper centralises the read.
//
// Returns an empty vector when the API is not applied or the relationship is
// empty.
// ---------------------------------------------------------------------------

std::vector<ObjectKey> parseFilteredPairs(ParseContext& ctx, ObjectKey key);

// ---------------------------------------------------------------------------
// Collision groups.
//
// `UsdPhysicsCollisionGroup` is a typed prim with two pieces of data:
//   - the `physics:filteredGroups` relationship — paths to OTHER collision-
//     group prims that this group should filter contacts against.
//   - the `colliders` collection — UsdCollectionAPI naming the prims that
//     belong to this group. Pre-resolved at parse time to a flat
//     ObjectKey list (the source backend handles the include/exclude /
//     traversal semantics).
//
// Returns the filtered groups + members as source-agnostic handle lists.
// ---------------------------------------------------------------------------

struct CollisionGroupInfo
{
    std::vector<ObjectKey> filteredGroups;
    std::vector<ObjectKey> members;
};

CollisionGroupInfo parseCollisionGroup(ParseContext& ctx, ObjectKey key);

// ---------------------------------------------------------------------------
// Attachments and element collision filters.
//
// Each typed attachment prim (VtxVtx / VtxTri / VtxTet / VtxCrv / VtxXform /
// TetXform / TriTri) shares the same set of base attributes:
// `omniphysics:attachmentEnabled`, `omniphysics:damping`,
// `omniphysics:stiffness`, plus `omniphysics:src0` and `omniphysics:src1`
// relationships (single target each).
//
// `parseAttachment` allocates and populates a `PhysxDeformableAttachmentDesc`
// of the given parse-lib `eAttachment*` type. The caller is responsible for
// classifying the prim before calling (so the parser doesn't have to detect
// which of the seven typed schemas applies; same pattern as parseJoint).
//
// Returns `nullptr` when either src0 or src1 relationship has != 1 target.
// Caller owns the returned pointer.
//
// `parseElementCollisionFilter` is the same pattern for the typed
// collision-filter prim, populating a `PhysxDeformableCollisionFilterDesc`.
// ---------------------------------------------------------------------------

DescPtr<PhysxDeformableAttachmentDesc>       parseAttachment(ParseContext& ctx, ObjectKey key, ObjectType type);
DescPtr<PhysxDeformableCollisionFilterDesc> parseElementCollisionFilter(ParseContext& ctx, ObjectKey key);

// ---------------------------------------------------------------------------
// Deformable body — per-prim attribute + relationship reader.
//
// The schema-side parser does substantial USD-specific orchestration:
// hierarchy walk to classify sim mesh / collision geom / skin geom, material-
// binding resolution per-prim, xform-stack-reset validation, multi-apply
// pose-purpose token discovery. None of those are USD-free operations.
//
// `parseDeformableBody` here covers ONLY the per-prim attribute and
// relationship reads on the deformable body's root prim:
//   - `OmniPhysicsDeformableBodyAPI` — bodyEnabled flag + mass scalar.
//   - `OmniPhysicsBodyAPI` — kinematicEnabled, startsAsleep flags +
//     simulationOwner relationship targets.
//   - `PhysicsFilteredPairsAPI` — per-body filter pairs (delegates to
//     parseFilteredPairs).
//
// The hierarchy walk + sim-mesh / collision / skin classification + material
// binding resolution stay in the consumer until the parse-library's USD
// backend grows the equivalent helpers (xform stack reset, material binding,
// multi-apply schema-instance iteration). Tracked under REQ-PARSE-DEF-001
// as an explicit gap.
// ---------------------------------------------------------------------------

struct DeformableBodyParse
{
    bool  bodyEnabled    = false;
    bool  kinematicBody  = false;
    bool  startsAsleep   = false;
    float mass           = 0.0f;
    std::vector<ObjectKey> simulationOwners;
    std::vector<ObjectKey> filteredCollisions;
};

DeformableBodyParse parseDeformableBody(ParseContext& ctx, ObjectKey key);

// Stage-aware defaults for the deformable-body descriptor family.
// Fields scaled by `metersPerUnit` (sleepThreshold / settlingThreshold /
// maxDepenetrationVelocity / restOffset) take their scaled defaults
// here.
//
// Call before reading USD attributes onto the descriptor: the
// `emitDeformableBody` overlay only writes back when an attribute is
// authored (or its parent API applied), so the descriptor must enter the
// overlay phase with valid defaults.
void setToDefault(PhysxDeformableBodyDesc& desc, const SourceUnits& units);
void setToDefault(PhysxVolumeDeformableBodyDesc& desc, const SourceUnits& units);
void setToDefault(PhysxSurfaceDeformableBodyDesc& desc, const SourceUnits& units);

// ---------------------------------------------------------------------------
// Character controller.
//
// `parseCct` produces a `CapsuleCctDesc` for a prim that has
// `PhysxCharacterControllerAPI` applied. Capsule geometry and world pose
// are pre-resolved by the walker (USD `UsdGeomCapsule` + the source's
// world-transform query) and passed in via `CctInfo`; the parser reads the
// CCT-API-specific attribute (`physxCharacterController:slopeLimit`) and
// stores the simulation-owner ObjectKey for the consumer to resolve.
//
// Legacy semantics (mirrors `usdLoad/Cct.cpp::parseCct`):
//   - `slopeLimit` defaults to `0.0f` when the attribute is not authored.
//   - More than one `simulationOwner` target logs an error; the first
//     target is stored.
//   - Scene-resolution + drop-on-unresolved happens consumer-side via
//     `CctDesc::sourceSimulationOwner` (mirrors the rigid-body pattern;
//     the ObjectDatabase isn't populated during `scanStage`).
// ---------------------------------------------------------------------------

struct CctInfo
{
    float  radius     = 0.1f;
    float  halfHeight = 0.1f;
    carb::Float3 pos   = { 0.0f, 0.0f, 0.0f };
    carb::Float3 scale = { 1.0f, 1.0f, 1.0f };
    std::vector<ObjectKey> simulationOwners;
};

DescPtr<CapsuleCctDesc> parseCct(ParseContext& ctx, ObjectKey key, const CctInfo& info);

// ---------------------------------------------------------------------------
// Mimic joints.
//
// `parseMimicJoints` reads the `PhysxMimicJointAPI:<axis>` multi-apply
// schemas on a joint and appends one `MimicJointDesc` per applied axis
// (rotX / rotY / rotZ). `parseNewtonMimicJoints` reads the single-apply
// `NewtonMimicAPI` and appends 0 or 1 desc. Both parsers perform the
// joint-degree-of-freedom checks that look at limit-API state on D6
// joints and the finite-limit check on revolute joints.
//
// Walker-pre-resolved: joint kind (revolute / prismatic / spherical /
// fixed / D6 / etc.) and the boolean flags `jointEnabled`,
// `excludeFromArticulation`. The parser uses these to short-circuit
// the cases that don't need parsing. The reference-joint kind is
// resolved by the caller via `jointTypeOf` - a small lookup the
// walker builds once from the emitted joints - and falls back to the
// source type name when partial resync scans reference a joint outside
// the scanned subtree.
//
// Path resolution: parse-lib stores `ObjectKey`s on
// `MimicJointDesc::{mimicJointKey, referenceJointKey}` and the
// consumer (`processScannedDescs`) converts via `ScannedStage::pathFor()`.
// ---------------------------------------------------------------------------

struct MimicJointParseInfo
{
    ObjectType jointType = eUndefined;      // resolved by walker
    bool jointEnabled = true;
    bool excludeFromArticulation = false;
};

using JointTypeLookup = std::function<ObjectType(ObjectKey)>;

// Per-axis limit lookup for a joint, supplied by the walker from the scanned
// JointLimitInfo. Returns nullptr when the joint has no limit authored on that
// axis (which reads as not locked). Lets the mimic D6 validation detect locked
// translation axes (lower > upper) from parsed data instead of re-reading raw
// UsdPhysicsLimitAPI attributes, which the source does not surface for an
// arbitrary reference joint.
using JointLimitLookup = std::function<const JointLimitInfo*(ObjectKey, JointAxis)>;

void parseMimicJoints(ParseContext& ctx, ObjectKey jointKey,
                     const MimicJointParseInfo& info,
                     const JointTypeLookup& jointTypeOf,
                     const JointLimitLookup& jointLimitOf,
                     std::vector<DescPtr<MimicJointDesc>>& out);

void parseNewtonMimicJoints(ParseContext& ctx, ObjectKey jointKey,
                            const MimicJointParseInfo& info,
                            const JointTypeLookup& jointTypeOf,
                            std::vector<DescPtr<MimicJointDesc>>& out);

// ---------------------------------------------------------------------------
// Spatial tendons.
//
// `parseSpatialTendons` reads the three multi-apply schemas that compose a
// spatial tendon hierarchy on a rigid-body / xformable prim:
//   - `PhysxTendonAttachmentRootAPI:<inst>` → root attachment (PhysxTendonSpatialDesc)
//   - `PhysxTendonAttachmentAPI:<inst>` → intermediate attachment (PhysxTendonAttachmentDesc)
//   - `PhysxTendonAttachmentLeafAPI:<inst>` → leaf attachment (PhysxTendonAttachmentLeafDesc)
//
// Multi-apply instance iteration is via `IPhysicsSource::forEachMultiApplyInstance`.
// Per-instance attributes live under `physxTendon:<inst>:<attr>`; the parent
// relationship is `physxTendon:<inst>:parentLink` and the parent instance
// token is `physxTendon:<inst>:parentAttachment`.
//
// Walker-pre-resolved: the link prim's world-scale (the parser
// multiplies it into the authored `localPos`).
//
// Hierarchy resolution stays consumer-side: the parser emits a flat list
// of attachments, the consumer builds the parent→children map from
// `parentLink` + `parentAttachment` and walks the tree at engine-object
// creation time.
// ---------------------------------------------------------------------------

struct SpatialTendonParseInfo
{
    carb::Float3 linkWorldScale = { 1.0f, 1.0f, 1.0f };
};

void parseSpatialTendons(ParseContext& ctx, ObjectKey linkKey,
                         const SpatialTendonParseInfo& info,
                         std::vector<DescPtr<PhysxTendonAttachmentDesc>>& out);

// ---------------------------------------------------------------------------
// Fixed tendons.
//
// `parseFixedTendons` reads the two multi-apply schemas that compose a
// fixed tendon on a single-DOF joint:
//   - `PhysxTendonAxisRootAPI:<inst>` → emits one PhysxTendonAxisDesc (the
//     root axis) AND one PhysxTendonFixedDesc (the tendon itself).
//   - `PhysxTendonAxisAPI:<inst>` → emits one PhysxTendonAxisDesc per
//     intermediate axis (excluding root instances).
//
// `gearing` and `forceCoefficient` are VtFloatArray attributes; only
// the first element is read. For revolute joints, gearing is converted
// deg→rad (or clamped to ±FLT_MAX past ±180°). Joint kind (revolute /
// prismatic) is walker-resolved; everything else is rejected with a
// warning.
//
// `body0` / `body1` are pre-resolved by the walker from the joint's
// `physics:body0` / `physics:body1` relationships; the parser stores
// them on the axis descriptor as `link0` / `link1`.
//
// Cross-reference: each `PhysxTendonAxisRootAPI:<inst>` produces both an
// axis and a tendon descriptor sharing the same `jointKey` +
// `instanceToken`. The consumer wires `PhysxTendonFixedDesc::rootAxis`
// to the matching axis via this pair.
// ---------------------------------------------------------------------------

struct FixedTendonParseInfo
{
    ObjectType jointType = eUndefined;  // walker-resolved
    ObjectKey body0;
    ObjectKey body1;
};

void parseFixedTendons(ParseContext& ctx, ObjectKey jointKey,
                       const FixedTendonParseInfo& info,
                       std::vector<DescPtr<PhysxTendonAxisDesc>>& outAxes,
                       std::vector<DescPtr<PhysxTendonFixedDesc>>& outTendons);

// ---------------------------------------------------------------------------
// Vehicle — TireFrictionTable + VehicleContextAPI.
//
// Standalone single-prim parsers with no cross-references into the
// vehicle component graph.
//
// `parseTireFrictionTable` — `PhysxVehicleTireFrictionTable`-typed prim.
//   Reads `defaultFrictionValue` (clamped >= 0) and validates that
//   each `groundMaterials` rel target has `PhysicsMaterialAPI` applied.
//   The friction-values float[] and the rel targets are pre-resolved
//   by the walker via direct USD APIs and passed in via `info`
//   (`IPhysicsSource` does not yet expose a float-array accessor).
//
// `parseVehicleContext` — `PhysxVehicleContextAPI` applied on a
//   `UsdPhysicsScene` typed prim. Reads update mode + vertical /
//   longitudinal axis tokens; falls back to the deprecated up /
//   forward axis Vec3s when the token attribute resolves to
//   "undefined". The "applied on a scene" check stays consumer-side
//   (parse-lib has no prim-type query).
// ---------------------------------------------------------------------------

struct TireFrictionTableInfo
{
    float defaultFrictionValue = 0.0f;
    std::vector<ObjectKey> materialPaths;   // walker pre-resolved rel
    std::vector<float>     frictionValues;  // walker pre-read float[]
};

DescPtr<TireFrictionTableDesc> parseTireFrictionTable(ParseContext& ctx, ObjectKey key,
                                                      const TireFrictionTableInfo& info);

DescPtr<VehicleContextDesc> parseVehicleContext(ParseContext& ctx, ObjectKey key);

// ---------------------------------------------------------------------------
// Vehicle — Wheel + Tire + Suspension shareable components.
//
// Per-component parsers for the three shareable vehicle component types
// (referenced by N vehicles via the WheelAttachment rels). Each runs
// once per applied API + each emits one descriptor; dedup across
// vehicles is the consumer adapter's job (path-keyed map fed into the
// VehicleComponentTracker before loadVehicle).
//
// `parseWheel` — PhysxVehicleWheelAPI; reads radius/width/mass/
//                              moi/dampingRate + deprecated brake/steer/toe
//                              attrs with deprecation warnings.
// `parseTire` — PhysxVehicleTireAPI; reads the lateral/
//                              longitudinal/camber stiffness with the
//                              deprecation fallback chains; takes
//                              walker pre-resolved `frictionVsSlipGraph`
//                              (3 carb::Float2s) + `frictionTableKey`
//                              ObjectKey via `TireInfo`.
// `parseSuspension` — PhysxVehicleSuspensionAPI; travel distance
//                              with deprecated maxCompression/maxDroop
//                              fallback; camber angles read with half-pi
//                              range checks.
//
// Mass scale (1 / kilogramsPerUnit) is walker-pre-resolved and passed via
// `TireInfo::massScale` since IPhysicsSource has no stage-units kg accessor.
// ---------------------------------------------------------------------------

struct TireInfo
{
    float massScale = 1.0f;
    bool hasFrictionVsSlipGraph = false;
    carb::Float2 frictionVsSlipGraph[3] = { {0.0f, 1.0f}, {0.1f, 1.0f}, {1.0f, 1.0f} };
    ObjectKey frictionTableKey;        // empty when no rel target
    bool frictionTableTooMany = false;  // walker saw >1 rel target — reject
};

DescPtr<WheelDesc>      parseWheel     (ParseContext& ctx, ObjectKey key);
DescPtr<TireDesc>       parseTire      (ParseContext& ctx, ObjectKey key, const TireInfo& info);
DescPtr<SuspensionDesc> parseSuspension(ParseContext& ctx, ObjectKey key);

// ---------------------------------------------------------------------------
// Vehicle — Engine + Gears + Clutch drivetrain components.
//
// Same shareable-component shape as Wheel/Tire/Suspension. AutoGearBox
// lives with the drive group because it carries a parent-Gears
// dependency (its ratio count is computed from
// `forwardGearCount = gears->ratios.size() - 1`).
//
// kgmsScale (mass scale × length scale²) is walker-pre-resolved and
// passed via Info structs since IPhysicsSource has no stage-units
// accessor for kilogramsPerUnit (same gap as Tire massScale).
//
// torqueCurve (8-Float2 array on EngineDesc) and ratios (VtArray<float>
// on GearsDesc) are walker-pre-read and passed via EngineInfo /
// GearsInfo since `IPhysicsSource::AttrValue` covers only scalar +
// small-vector kinds, not arbitrary-length float / float2 arrays.
// ---------------------------------------------------------------------------

struct EngineInfo
{
    float kgmsScale = 1.0f;
    bool hasTorqueCurve = false;
    carb::Float2 torqueCurve[8] = {};   // EngineDesc::maxNumberOfTorqueCurvePoints
    uint32_t torqueCurvePointCount = 0;
};

struct GearsInfo
{
    bool hasRatios = false;
    std::vector<float> ratios;
};

DescPtr<EngineDesc> parseEngine(ParseContext& ctx, ObjectKey key, const EngineInfo& info);
DescPtr<GearsDesc>  parseGears (ParseContext& ctx, ObjectKey key, const GearsInfo& info);
DescPtr<ClutchDesc> parseClutch(ParseContext& ctx, ObjectKey key, float kgmsScale);

// ---------------------------------------------------------------------------
// Vehicle — DriveBasic + Differential variants + AutoGearBox.
//
// `parseDriveBasic` — PhysxVehicleDriveBasicAPI; peakTorque
//                            with kgms-scale default at sentinel -1.
//                            `nonlinearCmdResponse` is wired by the
//                            consumer adapter from the multi-apply
//                            NonlinearCmdResponseAPI emit.
//
// `parseMultiWheelDifferential` / `parseTankDifferential`
//                          — single-apply with the Tank variant a
//                            subclass. Walker picks at emit time;
//                            tank takes both DifferentialInfo (base
//                            fields) and TankDifferentialInfo
//                            (subclass fields). Path side-table on
//                            ScannedStage carries the prim
//                            ObjectKey since DifferentialDesc has no
//                            path field of its own.
//
// `parseAutoGearBox` — walker pre-reads upRatios / downRatios.
//                            Validation against the parent
//                            DriveStandard's gears.ratios.size() is
//                            deferred to vehicle-create time (the
//                            parent is not visible at parse-lib).
//                            Each ratio must be non-negative.
// ---------------------------------------------------------------------------

struct AutoGearBoxInfo
{
    bool hasUpRatios = false;
    bool hasDownRatios = false;
    std::vector<float> upRatios;
    std::vector<float> downRatios;
};

struct DifferentialInfo
{
    std::vector<int>   wheels;                   // walker-pre-read; may be empty
    bool               hasTorqueRatios = false;
    std::vector<float> torqueRatios;
    bool               hasAverageWheelSpeedRatios = false;
    std::vector<float> averageWheelSpeedRatios;
};

struct TankDifferentialInfo
{
    bool             hasNumberOfWheelsPerTrack = false;
    std::vector<int> numberOfWheelsPerTrack;
    bool             hasThrustIndexPerTrack = false;
    std::vector<int> thrustIndexPerTrack;
    bool             hasWheelIndicesInTrackOrder = false;
    std::vector<int> wheelIndicesInTrackOrder;
    bool             hasTrackToWheelIndices = false;
    std::vector<int> trackToWheelIndices;
};

DescPtr<DriveBasicDesc>             parseDriveBasic            (ParseContext& ctx, ObjectKey key, float kgmsScale);
DescPtr<MultiWheelDifferentialDesc> parseMultiWheelDifferential(ParseContext& ctx, ObjectKey key, const DifferentialInfo& info);
DescPtr<TankDifferentialDesc>       parseTankDifferential      (ParseContext& ctx, ObjectKey key, const DifferentialInfo& info, const TankDifferentialInfo& tankInfo);
DescPtr<AutoGearBoxDesc>            parseAutoGearBox           (ParseContext& ctx, ObjectKey key, const AutoGearBoxInfo& info);

// ---------------------------------------------------------------------------
// Vehicle — Brakes (multi-apply) + Steering basic + Ackermann.
//
// `parseBrakes` — per multi-apply instance ("brakes0" /
//                            "brakes1" are the only two honored names).
//                            Walker pre-reads wheels (VtArray<int>) +
//                            torqueMultipliers (VtArray<float>) and
//                            passes via BrakesInfo. Walker tags each
//                            emit with its brakesIndex (0 or 1) and
//                            the instance token.
//
// `parseSteeringBasic` — PhysxVehicleSteeringAPI single-apply.
//                            Walker pre-reads wheels + angleMultipliers.
//
// `parseSteeringAckermann` — PhysxVehicleAckermannSteeringAPI single-
//                            apply. All scalar reads.
//
// Both Brakes and Steering descriptors carry a NonlinearCmdResponse-
// Desc pointer; this parser leaves it nullptr. The consumer adapter
// wires the pointer from the matching NonlinearCmdResponse emit.
// ---------------------------------------------------------------------------

struct BrakesInfo
{
    std::vector<int>   wheels;
    bool               hasTorqueMultipliers = false;
    std::vector<float> torqueMultipliers;
};

struct SteeringBasicInfo
{
    std::vector<int>   wheels;
    bool               hasAngleMultipliers = false;
    std::vector<float> angleMultipliers;
};

DescPtr<BrakesDesc>             parseBrakes            (ParseContext& ctx, ObjectKey key,
                                                        std::string_view instanceName, uint8_t brakesIndex,
                                                        const BrakesInfo& info);
DescPtr<SteeringBasicDesc>      parseSteeringBasic     (ParseContext& ctx, ObjectKey key, const SteeringBasicInfo& info);
DescPtr<SteeringAckermannDesc>  parseSteeringAckermann (ParseContext& ctx, ObjectKey key);

// ---------------------------------------------------------------------------
// Vehicle — NonlinearCmdResponse (multi-apply per command).
//
// `parseNonlinearCmdResponse` consumes pre-read arrays (commandValues,
// speedResponsesPerCommandValue, speedResponses) and produces one
// NonlinearCmdResponseDesc per multi-apply instance. Instance names
// drive the attach-target lookup in the consumer adapter:
//   - "drive" → DriveBasicDesc / DriveStandardDesc on the same prim
//   - "steer" → SteeringDesc on the same prim
//   - "brakes0" / "brakes1" → BrakesDesc with matching brakesIndex
//     on the same prim
// ---------------------------------------------------------------------------

struct NonlinearCmdResponseInfo
{
    bool                       hasCommandValues = false;
    std::vector<float>         commandValues;
    bool                       hasSpeedResponsesPerCommandValue = false;
    std::vector<int>           speedResponsesPerCommandValue;
    bool                       hasSpeedResponses = false;
    std::vector<carb::Float2>  speedResponses;
};

DescPtr<NonlinearCmdResponseDesc> parseNonlinearCmdResponse(
    ParseContext& ctx, ObjectKey ownerKey,
    std::string_view instanceName, const NonlinearCmdResponseInfo& info);

// ---------------------------------------------------------------------------
// Vehicle — DriveStandard.
//
// DriveStandardDesc carries pointer cross-refs to Engine / Gears /
// AutoGearBox / Clutch. Walker pre-resolves the four rel-or-API
// paths (prefer rel target; fall back to API on the same prim) and
// passes them via DriveStandardInfo. Consumer adapter resolves each
// ObjectKey → engine-side descriptor pointer at processScannedDescs
// time.
// ---------------------------------------------------------------------------

struct DriveStandardInfo
{
    ObjectKey engineKey;
    ObjectKey gearsKey;
    ObjectKey autoGearBoxKey;  // optional — empty when none applied
    ObjectKey clutchKey;
};

DescPtr<DriveStandardDesc> parseDriveStandard(ParseContext& ctx, ObjectKey key,
                                              const DriveStandardInfo& info);

// ---------------------------------------------------------------------------
// Vehicle — WheelAttachment + SuspensionCompliance.
//
// `parseSuspensionCompliance` — PhysxVehicleSuspensionComplianceAPI;
//                            walker pre-reads four optional VtArrays
//                            (wheelToeAngle / wheelCamberAngle as
//                            Vec2; suspensionForceAppPoint /
//                            tireForceAppPoint as Vec4). Validation
//
// `parseWheelAttachment` — walker pre-reads ~10 scalar / Vec3 /
//                            Quat attrs + pre-resolves three rel-or-
//                            API cross-refs (wheel / tire /
//                            suspension) + pre-walks descendants for
//                            the collision-shape detection (max one
//                            direct child with CollisionAPI; nested
//                            colliders rejected). Parse-lib
//                            validates index range [−1, max).
//
// Both leave cross-ref pointer fields null; the consumer adapter
// resolves to engine-side pointers via the tracker maps.
//
// `parseSuspensionCompliance` validates per-list cap of 3 entries plus
// jounce-range, monotonicity, and angle-range checks.
// ---------------------------------------------------------------------------

struct SuspensionComplianceInfo
{
    std::vector<carb::Float2> wheelToeAngles;
    std::vector<carb::Float2> wheelCamberAngles;
    std::vector<carb::Float4> suspensionForceAppPoints;
    std::vector<carb::Float4> tireForceAppPoints;
};

struct WheelAttachmentInfo
{
    // Walker-resolved state flags (eMANAGE_TRANSFORMS / eHAS_SHAPE /
    // eHAS_WHEEL_COM_OFFSET / eHAS_SUSP_FORCE_APP_POINT /
    // eHAS_TIRE_FORCE_APP_POINT / eHAS_SUSPENSION_FRAME).
    uint8_t state = 0;

    // Vector / quaternion attribute values pre-read by the walker.
    carb::Float3 suspensionTravelDirection     = { 0.0f, 0.0f, 0.0f };
    carb::Float3 suspensionForceAppPointOffset = { 0.0f, 0.0f, 0.0f };
    carb::Float3 wheelCenterOfMassOffset       = { 0.0f, 0.0f, 0.0f };
    carb::Float3 tireForceAppPointOffset       = { 0.0f, 0.0f, 0.0f };
    carb::Float3 suspensionFramePosition       = { 0.0f, 0.0f, 0.0f };
    carb::Float4 suspensionFrameOrientation    = { 0.0f, 0.0f, 0.0f, 1.0f };
    carb::Float3 wheelFramePosition            = { 0.0f, 0.0f, 0.0f };
    carb::Float4 wheelFrameOrientation         = { 0.0f, 0.0f, 0.0f, 1.0f };
    bool         driven = false;
    int          index  = 0;

    // Cross-references pre-resolved by the walker via rel-or-API
    // semantics; consumer adapter resolves to engine-side pointers.
    ObjectKey wheelKey;
    ObjectKey tireKey;
    ObjectKey suspensionKey;
    ObjectKey collisionGroupKey;  // optional
    ObjectKey shapeKey;           // walker-discovered (eHAS_SHAPE)

    // Owning vehicle: nearest ancestor prim with PhysxVehicleAPI applied
    // (walker-resolved). Lets the consumer group wheel attachments by vehicle
    // without re-walking USD descendants. Invalid when the attachment has no
    // vehicle ancestor (a malformed configuration).
    ObjectKey vehicleKey;
};

DescPtr<SuspensionComplianceDesc> parseSuspensionCompliance(ParseContext& ctx, ObjectKey key, const SuspensionComplianceInfo& info);
DescPtr<WheelAttachmentDesc>      parseWheelAttachment      (ParseContext& ctx, ObjectKey key, const WheelAttachmentInfo& info);

// ---------------------------------------------------------------------------
// Vehicle — chassis root (parseVehicle scalar-attr reads).
//
// Per-vehicle scalar / bool / token attribute reader. ~25 fields
// covering substep config, slip denominators, sticky-tire thresholds,
// query mode, vehicle-enabled flags.
//
// Walker pre-resolves length scale (1 / metersPerUnit), total xform
// scale via xfCache, the suspensionLineQueryType token, and the
// referenceFrameIsCenterOfMass custom-metadata bool — these are
// USD-coupled and can't go through the parse-lib's typed AttrValue.
//
// Cross-refs (drive / differential / steering / brakes) plus the
// wheel-attachment vector + controller initial state stay walker-side
// / consumer-side respectively. This parser only fills the scalar
// chassis fields; the consumer adapter wires the rest.
// ---------------------------------------------------------------------------

struct VehicleInfo
{
    float        lengthScale = 1.0f;                // 1 / metersPerUnit
    carb::Float3 scale       = { 1.0f, 1.0f, 1.0f }; // walker-resolved total xform scale
    bool         referenceFrameIsCenterOfMass = true;
    uint8_t      queryType   = 0;                    // 0 = raycast, 1 = sweep
};

DescPtr<VehicleDesc> parseVehicle(ParseContext& ctx, ObjectKey key, const VehicleInfo& info);

} // namespace omni::physics::parse
