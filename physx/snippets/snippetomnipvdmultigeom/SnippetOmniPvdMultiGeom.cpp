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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.

// ****************************************************************************
// A single OmniPvd capture that exercises every renderable geometry type
// currently supported by the PVD3/OVD viewer in one scene:
//   * static ground plane
//   * dynamic box, sphere, capsule
//   * dynamic convex mesh (non-uniform scale, offset local pose)
//   * static triangle mesh terrain (tucked off to the side)
//   * static heightfield positioned directly beneath the falling objects so
//     the capture contains a non-trivial set of contact pairs
//   * 6-link capsule articulation chain hanging from a fixed anchor
//   * static box tub beneath the particle spawn so PBD particles pool
//   * D6 spherical-joint pendulum (PxJoint gizmo, all-locked baseline)
//   * D6 with cone-limited swings, a limited linear-X axis, and an active
//     twist drive (PxJointLimitCone / PxJointLinearLimitPair / PxD6JointDrive)
//   * Two-D6 chain (anchor -> bob1 -> bob2) for multi-joint gizmo layering
//   * FEM tetrahedral deformable volumes (sphere + cube + cone stack) plus a
//     fourth small volume dropping onto the cloth sheet (vol/surface contact)
//   * triangle-mesh deformable surface (cloth sheet draping over rigids and
//     receiving an FEM volume from above)
//   * scheduled diagnostic messages with Info/Warning/Error severity levels
//
// Contact reporting is enabled via a custom filter shader so PxScene's
// pairsContact* arrays are populated each frame.
//
// Usage: SnippetOmniPvdMultiGeom --omnipvdfile=<path/to/out.ovd>
// ****************************************************************************

#include "PxPhysicsAPI.h"
#include "../snippetutils/SnippetUtils.h"
#include "omnipvd/PxOmniPvd.h"
#include "extensions/PxParticleExt.h"
#include "extensions/PxCudaHelpersExt.h"
#include "extensions/PxRemeshingExt.h"
#include "extensions/PxTetMakerExt.h"
#include "extensions/PxDeformableVolumeExt.h"
#include "extensions/PxDeformableSurfaceExt.h"

#if PX_SUPPORT_OMNI_PVD
#include "../pvdruntime/include/OmniPvdWriter.h"
#include "../pvdruntime/include/OmniPvdFileWriteStream.h"
#include "../snippetdeformablevolume/MeshGenerator.h"
#include "../snippetdeformablevolume/SnippetDeformableVolume.h"
#include "../snippetdeformablesurface/SnippetDeformableSurface.h"

using namespace physx;
using namespace ExtGpu;

static PxDefaultAllocator                       gAllocator;
static PxDefaultErrorCallback                   gErrorCallback;
static PxFoundation*                            gFoundation = NULL;
static PxPhysics*                               gPhysics = NULL;
static PxDefaultCpuDispatcher*                  gDispatcher = NULL;
static PxCudaContextManager*                    gCudaContextManager = NULL;
static PxScene*                                 gScene = NULL;
static PxMaterial*                              gMaterial = NULL;
static PxArticulationReducedCoordinate*         gArticulation = NULL;
static PxPBDParticleSystem*                     gParticleSystem = NULL;
static PxParticleBuffer*                        gParticleBuffer = NULL;
static PxArray<DeformableVolume>                gDeformableVolumes;
static PxArray<TestSurface>                     gDeformableSurfaces;

static PxOmniPvd*                               gOmniPvd = NULL;
static const char*                              gOmniPvdPath = NULL;

// Enables per-point contact notifications so OmniPvdPxSampler::streamSceneContacts
// populates PxScene's pairsContact* arrays (consumed by the viewer's contact gizmo).
static PxFilterFlags contactReportFilterShader(
    PxFilterObjectAttributes, PxFilterData,
    PxFilterObjectAttributes, PxFilterData,
    PxPairFlags& pairFlags, const void*, PxU32)
{
    pairFlags = PxPairFlag::eSOLVE_CONTACT | PxPairFlag::eDETECT_DISCRETE_CONTACT
              | PxPairFlag::eNOTIFY_TOUCH_FOUND
              | PxPairFlag::eNOTIFY_TOUCH_PERSISTS
              | PxPairFlag::eNOTIFY_CONTACT_POINTS;
    return PxFilterFlag::eDEFAULT;
}

// Deterministic pseudo-random so every capture is byte-identical for a given build.
static PxU32 gRandState = 0x13579BDFu;
static float randf(float lo, float hi)
{
    gRandState = gRandState * 1664525u + 1013904223u;
    const float u = float(gRandState) / float(0xFFFFFFFFu);
    return lo + u * (hi - lo);
}

static PxConvexMesh* createTetrahedronConvex()
{
    const PxU32 numVerts = 32;
    PxVec3 verts[numVerts];
    for (PxU32 i = 0; i < numVerts; ++i)
        verts[i] = PxVec3(randf(-1.0f, 1.0f), randf(-1.0f, 1.0f), randf(-1.0f, 1.0f));

    PxConvexMeshDesc desc;
    desc.points.count = numVerts;
    desc.points.stride = sizeof(PxVec3);
    desc.points.data = verts;
    desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
    desc.vertexLimit = 64;

    PxTolerancesScale tol;
    PxCookingParams params(tol);
    params.buildGPUData = true;
    return PxCreateConvexMesh(params, desc, gPhysics->getPhysicsInsertionCallback());
}

static PxTriangleMesh* createGridTriangleMesh()
{
    const PxU32 nx = 9, nz = 9;
    const PxU32 numVerts = nx * nz;
    const PxU32 numTris = (nx - 1) * (nz - 1) * 2;

    PxVec3 verts[numVerts];
    PxU32 indices[numTris * 3];

    for (PxU32 z = 0; z < nz; ++z)
        for (PxU32 x = 0; x < nx; ++x)
            verts[z * nx + x] = PxVec3(float(x), randf(-0.2f, 0.2f), float(z));

    PxU32 t = 0;
    for (PxU32 z = 0; z < nz - 1; ++z)
        for (PxU32 x = 0; x < nx - 1; ++x)
        {
            const PxU32 a = z * nx + x;
            const PxU32 b = a + 1;
            const PxU32 c = a + nx;
            const PxU32 d = c + 1;
            indices[t++] = a; indices[t++] = c; indices[t++] = b;
            indices[t++] = b; indices[t++] = c; indices[t++] = d;
        }

    PxTriangleMeshDesc desc;
    desc.points.count = numVerts;
    desc.points.stride = sizeof(PxVec3);
    desc.points.data = verts;
    desc.triangles.count = numTris;
    desc.triangles.stride = 3 * sizeof(PxU32);
    desc.triangles.data = indices;

    // SDF so the GPU particle system can cull/collide silently with this mesh.
    PxSDFDesc sdfDesc;
    sdfDesc.spacing = 0.1f;
    sdfDesc.subgridSize = 6;
    sdfDesc.bitsPerSubgridPixel = PxSdfBitsPerSubgridPixel::e16_BIT_PER_PIXEL;
    sdfDesc.numThreadsForSdfConstruction = 4;
    desc.sdfDesc = &sdfDesc;

    PxTolerancesScale tol;
    PxCookingParams params(tol);
    params.buildGPUData = true;
    return PxCreateTriangleMesh(params, desc, gPhysics->getPhysicsInsertionCallback());
}

static PxHeightField* createPlateHeightField()
{
    const PxU32 nbRows = 8, nbCols = 8;
    const PxU32 nbSamples = nbRows * nbCols;

    PxHeightFieldSample samples[nbSamples];
    for (PxU32 i = 0; i < nbSamples; ++i)
    {
        samples[i].height = PxI16(randf(-300.0f, 300.0f));
        samples[i].materialIndex0 = 0;
        samples[i].materialIndex1 = 0;
        samples[i].clearTessFlag();
    }

    PxHeightFieldDesc desc;
    desc.nbRows = nbRows;
    desc.nbColumns = nbCols;
    desc.samples.data = samples;
    desc.samples.stride = sizeof(PxHeightFieldSample);

    return PxCreateHeightField(desc, gPhysics->getPhysicsInsertionCallback());
}

// 6-link capsule chain dangling from a fixed anchor. Revolute pins at every
// joint except the middle (prismatic slider) so the viewer exercises every
// articulation-joint type it is likely to draw. The base is pre-rotated so
// that its local +X coincides with world +Y -- keeping every link's local X
// parallel means each revolute-swing joint only has to resolve a tilt about
// its swing axis, not a 90-degree mismatch that would collapse the chain.
static void createArticulationChain()
{
    gArticulation = gPhysics->createArticulationReducedCoordinate();
    gArticulation->setSolverIterationCounts(16);
    gArticulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);

    const PxReal capRadius = 0.08f;
    const PxReal capHalf   = 0.30f;                             // capsule half-height along local X
    const PxReal overlap   = 0.02f;
    const PxReal jointOffs = capRadius + capHalf - overlap * 0.5f;
    const PxReal baseHalfX = 0.06f;                             // anchor-box half-extent along local X

    const PxVec3 anchor(-4.0f, 6.5f, -2.0f);
    const PxQuat vertical(PxPi * 0.5f, PxVec3(0, 0, 1));        // local +X  ->  world +Y
    // A small tilt about world X gives the solver an off-equilibrium start so
    // the chain swings under gravity rather than hanging perfectly still.
    const PxQuat seedTilt(PxPi * 0.18f, PxVec3(1, 0, 0));
    const PxQuat baseRot  = seedTilt * vertical;

    PxArticulationLink* prev = gArticulation->createLink(NULL, PxTransform(anchor, baseRot));
    PxRigidActorExt::createExclusiveShape(*prev,
        PxBoxGeometry(baseHalfX, 0.14f, 0.14f), *gMaterial);
    PxRigidBodyExt::updateMassAndInertia(*prev, 5.0f);

    // Walk down the chain: each child sits directly below the previous joint
    // anchor in world Y. Because every link shares the same orientation, the
    // solver's initial state is already consistent and the chain doesn't
    // "snap" into place on frame 1.
    PxReal topWorldY = anchor.y - baseHalfX;
    const PxU32 numLinks = 6;
    for (PxU32 i = 0; i < numLinks; ++i)
    {
        const PxReal centreY = topWorldY - jointOffs;
        const PxVec3 linkOrigin(anchor.x, centreY, anchor.z);

        PxArticulationLink* link = gArticulation->createLink(prev, PxTransform(linkOrigin, baseRot));
        PxRigidActorExt::createExclusiveShape(*link, PxCapsuleGeometry(capRadius, capHalf), *gMaterial);
        PxRigidBodyExt::updateMassAndInertia(*link, 1.0f);

        PxArticulationJointReducedCoordinate* j = link->getInboundJoint();
        const PxReal parentOffsX = (i == 0) ? -baseHalfX : -jointOffs;
        j->setParentPose(PxTransform(PxVec3(parentOffsX, 0.0f, 0.0f)));
        j->setChildPose (PxTransform(PxVec3(+jointOffs,   0.0f, 0.0f)));

        if (i == 3)
        {
            j->setJointType(PxArticulationJointType::ePRISMATIC);
            j->setMotion(PxArticulationAxis::eX, PxArticulationMotion::eLIMITED);
            j->setLimitParams(PxArticulationAxis::eX, PxArticulationLimit(-0.08f, 0.08f));
        }
        else
        {
            j->setJointType(PxArticulationJointType::eREVOLUTE);
            j->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
        }

        prev      = link;
        topWorldY = centreY - jointOffs;
    }

    gScene->addArticulation(*gArticulation);
}

// ~100 miscellaneous dynamic rigids raining from above so the capture
// contains enough actors to stress the renderer's bulk-dispatch paths.
static void createCloudOfActors(PxU32 count)
{
    for (PxU32 i = 0; i < count; ++i)
    {
        const PxVec3 p(randf(-7.0f, 10.0f), randf(6.0f, 16.0f), randf(-5.0f, 10.0f));
        const PxQuat q(randf(-PxPi, PxPi), PxVec3(0.0f, 1.0f, 0.0f));
        const PxU32 kind = i & 3;
        PxRigidDynamic* a = NULL;
        if (kind == 0)
            a = PxCreateDynamic(*gPhysics, PxTransform(p, q),
                PxBoxGeometry(randf(0.08f, 0.20f), randf(0.08f, 0.20f), randf(0.08f, 0.20f)),
                *gMaterial, 2.0f);
        else if (kind == 1)
            a = PxCreateDynamic(*gPhysics, PxTransform(p, q),
                PxSphereGeometry(randf(0.09f, 0.18f)),
                *gMaterial, 2.0f);
        else if (kind == 2)
            a = PxCreateDynamic(*gPhysics, PxTransform(p, q),
                PxCapsuleGeometry(randf(0.06f, 0.11f), randf(0.09f, 0.18f)),
                *gMaterial, 2.0f);
        else
        {
            PxConvexMeshGeometry geom(createTetrahedronConvex());
            geom.scale = PxMeshScale(PxVec3(randf(0.25f, 0.45f), randf(0.25f, 0.45f), randf(0.25f, 0.45f)));
            a = PxCreateDynamic(*gPhysics, PxTransform(p, q), geom, *gMaterial, 2.0f);
        }
        if (a)
            gScene->addActor(*a);
    }
}

// Open-top box container. Walls are short (~0.5 m) so the pile of fluid
// inside stays visible over the rim while still catching the stream.
static void createParticleCascade()
{
    const PxReal cx = 4.0f, cz = 2.0f, cy = 2.0f;
    const PxReal halfSide   = 1.0f;
    const PxReal wallHalfH  = 0.25f;   // wall height = 0.5 m
    const PxReal thick      = 0.025f;

    // Floor.
    {
        PxRigidStatic* a = gPhysics->createRigidStatic(PxTransform(PxVec3(cx, cy, cz)));
        PxShape* s = gPhysics->createShape(
            PxBoxGeometry(halfSide, thick, halfSide), *gMaterial, true);
        a->attachShape(*s);
        gScene->addActor(*a);
        s->release();
    }

    const PxReal wallY = cy + wallHalfH;
    struct Wall { PxVec3 pos; PxVec3 half; };
    const Wall walls[] = {
        { PxVec3(cx, wallY, cz + halfSide), PxVec3(halfSide, wallHalfH, thick) }, // +Z
        { PxVec3(cx, wallY, cz - halfSide), PxVec3(halfSide, wallHalfH, thick) }, // -Z
        { PxVec3(cx + halfSide, wallY, cz), PxVec3(thick, wallHalfH, halfSide) }, // +X
        { PxVec3(cx - halfSide, wallY, cz), PxVec3(thick, wallHalfH, halfSide) }, // -X
    };
    for (const Wall& w : walls)
    {
        PxRigidStatic* a = gPhysics->createRigidStatic(PxTransform(w.pos));
        PxShape* s = gPhysics->createShape(PxBoxGeometry(w.half), *gMaterial, true);
        a->attachShape(*s);
        gScene->addActor(*a);
        s->release();
    }
}

// A small PBD fluid blob dropped into the scene so particles show up in the OVD.
static void createParticles()
{
    if (!gCudaContextManager)
        return;

    // Narrow tall column so the fluid reads as a pour onto the top tier of
    // the cascade rather than a dumped cube.
    const PxU32 numX = 10, numY = 38, numZ = 10;
    const PxU32 maxParticles = numX * numY * numZ;   // 3800 particles
    const PxReal particleSpacing = 0.10f;
    const PxReal fluidDensity = 1000.0f;
    const PxReal restOffset = 0.5f * particleSpacing / 0.6f;

    PxPBDMaterial* mat = gPhysics->createPBDMaterial(0.05f, 0.05f, 0.0f, 0.001f, 0.5f, 0.005f, 0.01f, 0.0f, 0.0f);
    mat->setViscosity(0.001f);
    mat->setSurfaceTension(0.00704f);
    mat->setCohesion(0.0704f);

    gParticleSystem = gPhysics->createPBDParticleSystem(*gCudaContextManager, 96);
    const PxReal fluidRestOffset = restOffset * 0.6f;
    const PxReal particleMass = fluidDensity * 1.333f * 3.14159f * particleSpacing * particleSpacing * particleSpacing;
    gParticleSystem->setRestOffset(restOffset);
    gParticleSystem->setContactOffset(restOffset + 0.01f);
    gParticleSystem->setParticleContactOffset(fluidRestOffset / 0.6f);
    gParticleSystem->setSolidRestOffset(restOffset);
    gParticleSystem->setFluidRestOffset(fluidRestOffset);
    gParticleSystem->setMaxLinearVelocity(restOffset * 100.0f);
    gScene->addActor(*gParticleSystem);

    const PxU32 phaseId = gParticleSystem->createPhase(mat,
        PxParticlePhaseFlags(PxParticlePhaseFlag::eParticlePhaseFluid | PxParticlePhaseFlag::eParticlePhaseSelfCollide));

    PxU32*  phase    = PX_EXT_PINNED_MEMORY_ALLOC(PxU32,  *gCudaContextManager, maxParticles);
    PxVec4* positionsInvMassM  = PX_EXT_PINNED_MEMORY_ALLOC(PxVec4, *gCudaContextManager, maxParticles);
    PxVec4* velocity = PX_EXT_PINNED_MEMORY_ALLOC(PxVec4, *gCudaContextManager, maxParticles);

    // Stream origin sits directly above the top cascade plate (y=5.0); a
    // narrow X/Z footprint plus the tall Y count gives us a fountain-style
    // pour that keeps feeding the top tier for most of the 2 s capture.
    const PxVec3 origin(3.55f, 5.6f, 1.55f);
    for (PxU32 i = 0; i < numX; ++i)
    for (PxU32 j = 0; j < numY; ++j)
    for (PxU32 k = 0; k < numZ; ++k)
    {
        const PxU32 idx = i * (numY * numZ) + j * numZ + k;
        phase[idx]    = phaseId;
        positionsInvMassM[idx]  = PxVec4(origin.x + i * particleSpacing,
                               origin.y + j * particleSpacing,
                               origin.z + k * particleSpacing,
                               1.0f / particleMass);
        velocity[idx] = PxVec4(0.0f);
    }

    ExtGpu::PxParticleBufferDesc bufferDesc;
    bufferDesc.maxParticles      = maxParticles;
    bufferDesc.numActiveParticles = maxParticles;
    bufferDesc.positions         = positionsInvMassM;
    bufferDesc.velocities        = velocity;
    bufferDesc.phases            = phase;

    gParticleBuffer = ExtGpu::PxCreateAndPopulateParticleBuffer(bufferDesc, gCudaContextManager);
    gParticleSystem->addParticleBuffer(gParticleBuffer);

    PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, positionsInvMassM);
    PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, velocity);
    PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, phase);
}

// Two-body D6 joint with all translations + twist locked, both swings free,
// so the viewer's PxJoint gizmo has a non-trivial constraint to render.
// Static anchor at (7, 5, -5) holds a dynamic bob hanging 1.2 m below; a
// sideways kick at spawn makes it pendulum-swing under gravity.
static void createJointedPendulum()
{
    const PxVec3 anchorPos(7.0f, 5.0f, -5.0f);

    PxRigidStatic* root = gPhysics->createRigidStatic(PxTransform(anchorPos));
    PxRigidActorExt::createExclusiveShape(*root, PxBoxGeometry(0.1f, 0.1f, 0.1f), *gMaterial);
    gScene->addActor(*root);

    PxRigidDynamic* bob = PxCreateDynamic(*gPhysics,
        PxTransform(PxVec3(anchorPos.x, anchorPos.y - 1.2f, anchorPos.z)),
        PxSphereGeometry(0.3f), *gMaterial, 5.0f);
    bob->setLinearVelocity(PxVec3(0.0f, 0.0f, 3.0f));
    gScene->addActor(*bob);

    PxD6Joint* j = PxD6JointCreate(*gPhysics,
        root, PxTransform(PxIdentity),
        bob,  PxTransform(PxVec3(0.0f, 1.2f, 0.0f)));
    j->setMotion(PxD6Axis::eX,      PxD6Motion::eLOCKED);
    j->setMotion(PxD6Axis::eY,      PxD6Motion::eLOCKED);
    j->setMotion(PxD6Axis::eZ,      PxD6Motion::eLOCKED);
    j->setMotion(PxD6Axis::eTWIST,  PxD6Motion::eLOCKED);
    j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
    j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
}

// A second D6 with a richer constraint mix than the all-locked pendulum:
// bounded swing cone (PxJointLimitCone), a limited linear-X axis with a
// linear-limit pair (PxJointLinearLimitPair so the bob can slide a short
// distance along the joint axis), and an active twist drive
// (PxD6JointDrive) keeping the body spinning. Exercises the viewer's
// property-panel paths for limits and drives that the basic pendulum's
// all-locked / all-free axes skip entirely.
static void createDrivenD6()
{
    const PxVec3 anchorPos(10.0f, 5.0f, -5.0f);

    PxRigidStatic* anchor = gPhysics->createRigidStatic(PxTransform(anchorPos));
    PxRigidActorExt::createExclusiveShape(*anchor, PxBoxGeometry(0.1f, 0.1f, 0.1f), *gMaterial);
    gScene->addActor(*anchor);

    PxRigidDynamic* bob = PxCreateDynamic(*gPhysics,
        PxTransform(PxVec3(anchorPos.x, anchorPos.y - 1.0f, anchorPos.z)),
        PxBoxGeometry(0.25f, 0.25f, 0.5f), *gMaterial, 4.0f);
    gScene->addActor(*bob);

    PxD6Joint* j = PxD6JointCreate(*gPhysics,
        anchor, PxTransform(PxIdentity),
        bob,    PxTransform(PxVec3(0.0f, 1.0f, 0.0f)));

    // Linear: X is a short bounded slide, Y and Z locked.
    j->setMotion(PxD6Axis::eX, PxD6Motion::eLIMITED);
    j->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
    j->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);
    j->setLinearLimit(PxD6Axis::eX,
        PxJointLinearLimitPair(gPhysics->getTolerancesScale(), -0.25f, 0.25f));

    // Angular: free twist (driven below), swings bounded by a cone.
    j->setMotion(PxD6Axis::eTWIST,  PxD6Motion::eFREE);
    j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
    j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
    j->setSwingLimit(PxJointLimitCone(PxPi / 6.0f, PxPi / 4.0f));

    // Twist drive: angular-velocity target around the joint X axis. Force
    // limit left at MAX so the bob actually reaches the target rate rather
    // than asymptoting partway.
    j->setDrive(PxD6Drive::eTWIST, PxD6JointDrive(0.0f, 1.0e3f, PX_MAX_F32, false));
    j->setDriveVelocity(PxVec3(0.0f), PxVec3(4.0f, 0.0f, 0.0f));
}

// Two D6 joints in series (anchor -> bob1 -> bob2). Each link locks all
// translations + twist and cone-limits both swings to ~60 deg, so the chain
// settles into a soft hanging fold under gravity rather than rotating
// freely. Verifies the viewer's joint gizmo layering when multiple
// PxJoints share an actor -- bob1 is both the child of the upper joint
// and the parent of the lower one.
static void createD6Chain()
{
    const PxVec3 anchorPos(4.0f, 7.0f, -7.0f);
    const PxReal linkLen = 1.0f;

    PxRigidStatic* anchor = gPhysics->createRigidStatic(PxTransform(anchorPos));
    PxRigidActorExt::createExclusiveShape(*anchor, PxBoxGeometry(0.1f, 0.1f, 0.1f), *gMaterial);
    gScene->addActor(*anchor);

    PxRigidDynamic* bob1 = PxCreateDynamic(*gPhysics,
        PxTransform(PxVec3(anchorPos.x, anchorPos.y - linkLen, anchorPos.z)),
        PxSphereGeometry(0.2f), *gMaterial, 2.0f);
    gScene->addActor(*bob1);

    PxRigidDynamic* bob2 = PxCreateDynamic(*gPhysics,
        PxTransform(PxVec3(anchorPos.x, anchorPos.y - 2.0f * linkLen, anchorPos.z)),
        PxSphereGeometry(0.2f), *gMaterial, 2.0f);
    bob2->setLinearVelocity(PxVec3(0.0f, 0.0f, 2.0f));  // initial kick so the chain swings
    gScene->addActor(*bob2);

    auto configureLink = [](PxD6Joint* j)
    {
        j->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
        j->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
        j->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);
        j->setMotion(PxD6Axis::eTWIST,  PxD6Motion::eLOCKED);
        j->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
        j->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
        j->setSwingLimit(PxJointLimitCone(PxPi / 3.0f, PxPi / 3.0f));
    };

    // anchor frame at its origin (4, 7, -7); bob1 frame offset +linkLen up
    // its local Y so its world position lands at the anchor's origin.
    PxD6Joint* j1 = PxD6JointCreate(*gPhysics,
        anchor, PxTransform(PxIdentity),
        bob1,   PxTransform(PxVec3(0.0f, linkLen, 0.0f)));
    configureLink(j1);

    // Second joint pivots about bob1's centre; bob2's frame offset puts the
    // constraint point at bob1's world position so the chain links cleanly.
    PxD6Joint* j2 = PxD6JointCreate(*gPhysics,
        bob1, PxTransform(PxIdentity),
        bob2, PxTransform(PxVec3(0.0f, linkLen, 0.0f)));
    configureLink(j2);
}

// Allocates host-mirror buffers, transforms+masses the tet mesh, uploads to
// GPU, and registers the volume so stepPhysics can copy deformed verts back
// for OmniPvd streaming. Adapted from SnippetDeformableVolume's addDeformableVolume.
static void registerDeformableVolume(PxDeformableVolume* dv, const PxTransform& t, PxReal density, PxReal scale)
{
    PxVec4 *simPos, *simVel, *collPos, *restPos;
    PxDeformableVolumeExt::allocateAndInitializeHostMirror(*dv, gCudaContextManager,
        simPos, simVel, collPos, restPos);
    PxDeformableVolumeExt::transform(*dv, t, scale, simPos, simVel, collPos, restPos);
    PxDeformableVolumeExt::updateMass(*dv, density, 50.f, simPos);
    PxDeformableVolumeExt::copyToDevice(*dv, PxDeformableVolumeDataFlag::eALL,
        simPos, simVel, collPos, restPos);

    gDeformableVolumes.pushBack(DeformableVolume(dv, gCudaContextManager));

    PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, simPos);
    PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, simVel);
    PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, collPos);
    PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, restPos);
}

// Cooks a tet mesh from a closed surface mesh, creates a deformable volume,
// attaches shapes + sim mesh, and registers it. Adapted from
// SnippetDeformableVolume::createDeformableVolume.
static void createDeformableVolumeFromMesh(const PxCookingParams& params,
    const PxArray<PxVec3>& triVerts, const PxArray<PxU32>& triIndices)
{
    PxSimpleTriangleMesh surfaceMesh;
    surfaceMesh.points.count    = triVerts.size();
    surfaceMesh.points.data     = triVerts.begin();
    surfaceMesh.triangles.count = triIndices.size() / 3;
    surfaceMesh.triangles.data  = triIndices.begin();

    PxDeformableVolumeMesh* mesh = PxDeformableVolumeExt::createDeformableVolumeMesh(
        params, surfaceMesh, 8, gPhysics->getPhysicsInsertionCallback());
    if (!mesh) return;

    PxDeformableVolume* dv = gPhysics->createDeformableVolume(*gCudaContextManager);
    if (!dv) return;

    PxDeformableVolumeMaterial* mat = PxGetPhysics().createDeformableVolumeMaterial(2.e+5f, 0.3f, 0.1f);
    PxTetrahedronMeshGeometry geom(mesh->getCollisionMesh());
    const PxShapeFlags sFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE;
    PxShape* shape = gPhysics->createShape(geom, &mat, 1, true, sFlags);
    if (!shape) return;
    dv->attachShape(*shape);
    shape->setSimulationFilterData(PxFilterData(0, 0, 2, 0));
    dv->attachSimulationMesh(*mesh->getSimulationMesh(), *mesh->getDeformableVolumeAuxData());
    gScene->addActor(*dv);

    registerDeformableVolume(dv, PxTransform(PxIdentity), 100.f, 1.0f);
    dv->setDeformableBodyFlag(PxDeformableBodyFlag::eDISABLE_SELF_COLLISION, true);
    dv->setSolverIterationCounts(30);
    shape->release();
}

// Three FEM deformable volumes (sphere, cube, cone) stacked vertically so
// they land on each other under gravity -- matches SnippetDeformableVolume's
// "createDeformableVolumes" demo so the OVD capture exercises volume-volume
// interaction, not just volume-static contact.
static void createDeformableVolumes(const PxCookingParams& params)
{
    if (!gCudaContextManager) return;

    const PxReal maxEdgeLength = 0.4f;
    const PxVec3 column(3.0f, 0.0f, -3.0f); // isolated from cloth + cloud landing zone
    PxArray<PxVec3> triVerts;
    PxArray<PxU32>  triIndices;

    // Bottom: sphere that the others will land on.
    meshgenerator::createSphere(triVerts, triIndices, column + PxVec3(0, 3.0f, 0), 0.8f, maxEdgeLength);
    createDeformableVolumeFromMesh(params, triVerts, triIndices);

    // Middle: cube landing on the sphere.
    meshgenerator::createCube(triVerts, triIndices, column + PxVec3(0, 5.5f, 0), 0.8f);
    PxRemeshingExt::limitMaxEdgeLength(triIndices, triVerts, maxEdgeLength);
    createDeformableVolumeFromMesh(params, triVerts, triIndices);

    // Top: cone landing on the cube. Slight X offset so it doesn't rest
    // perfectly flat and the pile folds sideways.
    meshgenerator::createConeY(triVerts, triIndices, column + PxVec3(0.05f, 8.0f, 0), 0.6f, 1.2f);
    PxRemeshingExt::limitMaxEdgeLength(triIndices, triVerts, maxEdgeLength);
    createDeformableVolumeFromMesh(params, triVerts, triIndices);

    // Cloth-interaction probe: small sphere positioned directly above the
    // cloth's draping centre at (-6, 8, 3). It falls onto the sheet, giving
    // the capture a deformable-volume vs deformable-surface contact pair
    // (the stack column above only tests volume-volume; the static obstacle
    // beneath the cloth tests cloth-vs-rigid).
    meshgenerator::createSphere(triVerts, triIndices, PxVec3(-6.0f, 11.0f, 3.0f), 0.35f, maxEdgeLength);
    createDeformableVolumeFromMesh(params, triVerts, triIndices);
}

// Cloth sheet -- 24x24 quad mesh hung at (-6, 8, 3), draping under gravity.
// Uses a bespoke tri-mesh grid rather than meshgenerator because the surface
// path wants an explicit quad-tessellated patch.
static PX_FORCE_INLINE PxU32 clothIdx(PxU32 x, PxU32 y, PxU32 numY) { return x * numY + y; }
static PX_FORCE_INLINE PxReal clothTriMass(const PxU32* t, const PxVec3* v, PxReal thickness, PxReal density)
{
    const PxReal area = 0.5f * (v[t[1]] - v[t[0]]).cross(v[t[2]] - v[t[0]]).magnitude();
    return area * thickness * density;
}
static void createDeformableCloth(const PxCookingParams& params)
{
    if (!gCudaContextManager) return;

    const PxReal thickness       = 0.01f;
    const PxReal bendingStiffness = 0.00001f;
    const PxReal size             = 2.0f;
    const PxU32  numX = 24, numZ = 24;
    const PxReal density          = 500.0f;
    const PxTransform transform(PxVec3(-6.0f - 0.5f * size, 8.0f, 3.0f - 0.5f * size));

    // Static sphere obstacle under the cloth's center so the sheet drapes
    // over it rather than pancaking on the ground. Radius < cloth half-size
    // gives the fabric room to fold around the sides.
    PxRigidStatic* clothObstacle = gPhysics->createRigidStatic(PxTransform(PxVec3(-6.0f, 6.5f, 3.0f)));
    PxRigidActorExt::createExclusiveShape(*clothObstacle, PxSphereGeometry(0.5f), *gMaterial);
    gScene->addActor(*clothObstacle);

    PxDeformableSurfaceMaterial* mat = gPhysics->createDeformableSurfaceMaterial(
        1.e10f, 0.3f, 0.5f, thickness, bendingStiffness);

    PxArray<PxVec3> verts;       verts.reserve(numX * numZ);
    PxArray<PxU32>  tris;        tris.reserve(3 * 2 * (numX - 1) * (numZ - 1));
    PxArray<PxVec3> velocity;    velocity.reserve(numX * numZ);
    PxArray<PxReal> triMasses;   triMasses.reserve(2 * (numX - 1) * (numZ - 1));

    const PxReal sx = size / (numX - 1);
    const PxReal sz = size / (numZ - 1);
    for (PxU32 i = 0; i < numX; ++i)
        for (PxU32 j = 0; j < numZ; ++j)
        {
            verts.pushBack(PxVec3(i * sx, 0.0f, j * sz));
            velocity.pushBack(PxVec3(0.0f));
        }
    for (PxU32 i = 1; i < numX; ++i)
        for (PxU32 j = 1; j < numZ; ++j)
        {
            tris.pushBack(clothIdx(i - 1, j - 1, numZ));
            tris.pushBack(clothIdx(i,     j - 1, numZ));
            tris.pushBack(clothIdx(i - 1, j,     numZ));
            triMasses.pushBack(clothTriMass(&tris[tris.size() - 3], verts.begin(), thickness, density));
            tris.pushBack(clothIdx(i - 1, j,     numZ));
            tris.pushBack(clothIdx(i,     j - 1, numZ));
            tris.pushBack(clothIdx(i,     j,     numZ));
            triMasses.pushBack(clothTriMass(&tris[tris.size() - 3], verts.begin(), thickness, density));
        }

    PxTriangleMeshDesc desc;
    desc.points.count       = verts.size();
    desc.points.stride      = sizeof(PxVec3);
    desc.points.data        = verts.begin();
    desc.triangles.count    = tris.size() / 3;
    desc.triangles.stride   = 3 * sizeof(PxU32);
    desc.triangles.data     = tris.begin();

    PxTriangleMesh* triMesh = PxCreateTriangleMesh(params, desc, gPhysics->getPhysicsInsertionCallback());
    if (!triMesh) return;

    PxDeformableSurface* ds = gPhysics->createDeformableSurface(*gCudaContextManager);
    if (!ds) return;

    const PxShapeFlags sFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE;
    PxTriangleMeshGeometry geom(triMesh);
    PxShape* shape = gPhysics->createShape(geom, &mat, 1, true, sFlags);
    if (!shape) return;
    ds->attachShape(*shape);
    gScene->addActor(*ds);

    PxVec4 *positionsInvMass, *vel, *rest;
    PxDeformableSurfaceExt::allocateAndInitializeHostMirror(*ds,
        verts.begin(), velocity.begin(), verts.begin(), 0.5f,
        transform, gCudaContextManager, positionsInvMass, vel, rest);
    PxDeformableSurfaceExt::distributeTriangleMassToVertices(*ds, triMasses.begin(), positionsInvMass);

    PxShape* sh = ds->getShape();
    sh->setContactOffset(2.0f * thickness);
    sh->setRestOffset(thickness);
    sh->setDeformableSurfaceMaterials(&mat, 1);

    PxDeformableSurfaceExt::copyToDevice(*ds, PxDeformableSurfaceDataFlag::eALL,
        verts.size(), positionsInvMass, vel, rest);
    gDeformableSurfaces.pushBack(TestSurface(ds, gCudaContextManager));

    PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, positionsInvMass);
    PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, vel);
    PX_EXT_PINNED_MEMORY_FREE(*gCudaContextManager, rest);

    ds->setSelfCollisionFilterDistance(thickness * 2.5f);
    ds->setLinearDamping(0.f);
    ds->setMaxLinearVelocity(1000.0f);
    ds->setNbCollisionPairUpdatesPerTimestep(1);
    ds->setNbCollisionSubsteps(1);
    shape->release();
}

static void buildMultiGeomScene()
{
    PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
    gDispatcher = PxDefaultCpuDispatcherCreate(2);
    sceneDesc.cpuDispatcher = gDispatcher;
    sceneDesc.filterShader = contactReportFilterShader;
    sceneDesc.cudaContextManager = gCudaContextManager;
    if (gCudaContextManager)
    {
        sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;
        sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
        sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
        sceneDesc.solverType = PxSolverType::eTGS;
        sceneDesc.gpuDynamicsConfig.maxRigidContactCount  = 1024 * 1024;
        sceneDesc.gpuDynamicsConfig.maxRigidPatchCount    = 1024 * 160;
        sceneDesc.gpuDynamicsConfig.foundLostPairsCapacity = 512 * 1024;
        sceneDesc.gpuDynamicsConfig.maxParticleContacts   = 4 * 1024 * 1024;
        sceneDesc.gpuDynamicsConfig.heapCapacity          = 256 * 1024 * 1024;
        sceneDesc.gpuDynamicsConfig.collisionStackSize    = 128 * 1024 * 1024;
    }
    gScene = gPhysics->createScene(sceneDesc);

    gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.5f);

    // Ground plane.
    PxRigidStatic* ground = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
    gScene->addActor(*ground);

    // A non-identity local pose reused on mesh shapes so localPos/localQuat columns are non-trivial.
    const PxTransform shapeLocal(
        PxVec3(0.25f, 0.0f, -0.10f),
        PxQuat(PxPi * 0.125f, PxVec3(0.0f, 1.0f, 0.0f)));

    // Box -- dynamic.
    {
        PxRigidDynamic* box = PxCreateDynamic(*gPhysics,
            PxTransform(PxVec3(-2.0f, 8.0f, 0.0f)),
            PxBoxGeometry(0.5f, 0.5f, 0.5f), *gMaterial, 10.0f);
        gScene->addActor(*box);
    }

    // Sphere -- dynamic.
    {
        PxRigidDynamic* sphere = PxCreateDynamic(*gPhysics,
            PxTransform(PxVec3(0.0f, 10.0f, 0.0f)),
            PxSphereGeometry(0.5f), *gMaterial, 10.0f);
        gScene->addActor(*sphere);
    }

    // Capsule -- dynamic, tipped sideways so you can see it's a capsule.
    {
        PxRigidDynamic* cap = PxCreateDynamic(*gPhysics,
            PxTransform(PxVec3(2.0f, 9.0f, 0.0f), PxQuat(PxPi * 0.5f, PxVec3(0, 0, 1))),
            PxCapsuleGeometry(0.3f, 0.6f), *gMaterial, 10.0f);
        gScene->addActor(*cap);
    }

    // Convex mesh -- dynamic, non-uniform scale.
    {
        PxConvexMesh* convex = createTetrahedronConvex();
        PxConvexMeshGeometry geom(convex);
        geom.scale = PxMeshScale(PxVec3(1.3f, 0.8f, 1.0f));

        PxRigidDynamic* actor = gPhysics->createRigidDynamic(PxTransform(PxVec3(-4.0f, 6.0f, 0.0f)));
        PxShape* shape = gPhysics->createShape(geom, *gMaterial, true);
        shape->setLocalPose(shapeLocal);
        actor->attachShape(*shape);
        PxRigidBodyExt::updateMassAndInertia(*actor, 10.0f);
        gScene->addActor(*actor);
        shape->release();
    }

    // Triangle mesh -- static, tucked into the far corner so it does not
    // overlap the heightfield that sits beneath the dynamic objects.
    {
        PxTriangleMesh* tri = createGridTriangleMesh();
        PxTriangleMeshGeometry geom(tri);
        geom.scale = PxMeshScale(PxVec3(0.5f, 2.0f, 0.5f));

        PxRigidStatic* actor = gPhysics->createRigidStatic(PxTransform(PxVec3(8.0f, 0.0f, 8.0f)));
        PxShape* shape = gPhysics->createShape(geom, *gMaterial, true);
        shape->setLocalPose(shapeLocal);
        actor->attachShape(*shape);
        gScene->addActor(*actor);
        shape->release();
    }

    // Heightfield -- static, positioned directly beneath the falling cluster
    // (dynamic box/sphere/capsule/convex and the particle blob) so the
    // capture exercises contact reporting on an interesting surface.
    // 8x8 samples * 2.0 row/col scale = 16m square; origin at (-8, 0, -6)
    // covers X=[-8, 8], Z=[-6, 10].  heightScale 0.01 with int16 samples in
    // [-300, 300] gives a ~3m relief so falling bodies actually hit features.
    {
        PxHeightField* hf = createPlateHeightField();
        PxHeightFieldGeometry geom(hf, PxMeshGeometryFlags(), 0.01f, 2.0f, 2.0f);

        PxRigidStatic* actor = gPhysics->createRigidStatic(PxTransform(PxVec3(-8.0f, 0.0f, -6.0f)));
        PxShape* shape = gPhysics->createShape(geom, *gMaterial, true);
        actor->attachShape(*shape);
        gScene->addActor(*actor);
        shape->release();
    }

    createArticulationChain();
    createCloudOfActors(75);
    createParticleCascade();
    createParticles();
    createJointedPendulum();
    createDrivenD6();
    createD6Chain();

    // Deformables need their own cooking path: tet-mesh building wants GPU
    // data, and the surface path wants vertex-mapping enabled for the tri
    // mesh cooker. Built lazily here so the existing rigid/particle setup
    // stays unchanged on CPU-only builds.
    if (gCudaContextManager)
    {
        PxCookingParams volumeParams(gPhysics->getTolerancesScale());
        volumeParams.meshWeldTolerance = 0.001f;
        volumeParams.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES);
        volumeParams.buildTriangleAdjacencies = false;
        volumeParams.buildGPUData = true;

        PxCookingParams surfaceParams = volumeParams;
        surfaceParams.midphaseDesc = PxMeshMidPhase::eBVH34;
        surfaceParams.meshPreprocessParams |= PxMeshPreprocessingFlag::eENABLE_VERT_MAPPING;

        createDeformableVolumes(volumeParams);
        createDeformableCloth(surfaceParams);
    }
}

static void initPhysicsWithOmniPvd()
{
    gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
    if (!gFoundation) { printf("Error: PxCreateFoundation\n"); return; }

    gOmniPvd = PxCreateOmniPvd(*gFoundation);
    if (!gOmniPvd) { printf("Error: PxCreateOmniPvd\n"); return; }

    OmniPvdWriter* writer = gOmniPvd->getWriter();
    OmniPvdFileWriteStream* fs = gOmniPvd->getFileWriteStream();
    if (!writer || !fs) { printf("Error: OmniPvd writer/stream\n"); return; }
    fs->setFileName(gOmniPvdPath);
    writer->setWriteStream(static_cast<OmniPvdWriteStream&>(*fs));

    gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, NULL, gOmniPvd);
    if (!gPhysics) { printf("Error: PxCreatePhysics\n"); return; }

    if (!gPhysics->getOmniPvd() || !gPhysics->getOmniPvd()->startSampling())
    {
        printf("Error: could not start OmniPvd sampling to file(%s)\n", gOmniPvdPath);
        return;
    }

    PxCudaContextManagerDesc cudaDesc;
    gCudaContextManager = PxCreateCudaContextManager(*gFoundation, cudaDesc, PxGetProfilerCallback());
    if (gCudaContextManager && !gCudaContextManager->contextIsValid())
    {
        PX_RELEASE(gCudaContextManager);
        printf("Warning: CUDA context invalid, particles will be skipped.\n");
    }

    buildMultiGeomScene();
}

static void cleanupPhysics()
{
    for (PxU32 i = 0; i < gDeformableVolumes.size(); ++i)
        gDeformableVolumes[i].release();
    gDeformableVolumes.reset();
    for (PxU32 i = 0; i < gDeformableSurfaces.size(); ++i)
        gDeformableSurfaces[i].release();
    gDeformableSurfaces.reset();

    if (gParticleBuffer) { gParticleBuffer->release(); gParticleBuffer = NULL; }
    PX_RELEASE(gArticulation);
    PX_RELEASE(gScene);
    PX_RELEASE(gDispatcher);
    PX_RELEASE(gCudaContextManager);
    PX_RELEASE(gPhysics);
    PX_RELEASE(gOmniPvd);
    PX_RELEASE(gFoundation);
}

static bool parseOmniPvdOutputFile(int argc, const char* const* argv)
{
    const char* const prefix = "--omnipvdfile=";
    const size_t prefixLen = strlen(prefix);
    if (argc != 2 || strncmp(argv[1], prefix, prefixLen) != 0 || argv[1][prefixLen] == '\0')
    {
        printf("SnippetOmniPvdMultiGeom usage:\n"
               "SnippetOmniPvdMultiGeom --omnipvdfile=<path to output OmniPvd file>\n");
        return false;
    }
    gOmniPvdPath = argv[1] + prefixLen;
    return true;
}

static void stepPhysics()
{
    gScene->simulate(1.0f / 60.0f);
    gScene->fetchResults(true);

    // Deformable position buffers live on device; copy-back per frame so
    // OmniPvd sampling sees the deformed state for the viewer.
    for (PxU32 i = 0; i < gDeformableVolumes.size(); ++i)
        gDeformableVolumes[i].copyDeformedVerticesFromGPU();
    for (PxU32 i = 0; i < gDeformableSurfaces.size(); ++i)
        gDeformableSurfaces[i].copyDeformedVerticesFromGPU();
}

// Emit a handful of diagnostic messages with varied severity levels so the viewer's
// Messages panel filter dropdown exercises all three severity buckets
// (Info/Warning/Error). The OmniPvd writer attaches each message to the frame
// active at emission time and surfaces them as eRECORD_MESSAGE entries.
static void emitScheduledMessage(PxU32 frameIdx)
{
    if (frameIdx == 0)
        gFoundation->error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__,
            "multi-geom scene built: 6 rigid geoms, 6-link articulation, ~3800 PBD particles, "
            "D6 pendulum + driven-limit D6 + 2-link D6 chain, 4 FEM volumes (3-stack + cloth probe), cloth sheet");
    else if (frameIdx == 20)
        gFoundation->error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__,
            "solver iteration count is high for convex mesh actor");
    else if (frameIdx == 50)
        gFoundation->error(PxErrorCode::ePERF_WARNING, __FILE__, __LINE__,
            "contact pair count approaching GPU budget (~80%% of maxRigidContactCount)");
    else if (frameIdx == 80)
        gFoundation->error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__,
            "attempted to set negative linear damping on dynamic actor (clamped to 0)");
    else if (frameIdx == 100)
        gFoundation->error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__,
            "particle buffer steady-state reached: 3800 active particles");
}
#endif  // PX_SUPPORT_OMNI_PVD

int snippetMain(int argc, const char* const* argv)
{
#if PX_SUPPORT_OMNI_PVD
    if (!parseOmniPvdOutputFile(argc, argv))
        return 1;

    initPhysicsWithOmniPvd();
    const PxU32 frameCount = 120;
    for (PxU32 i = 0; i < frameCount; ++i)
    {
        emitScheduledMessage(i);
        stepPhysics();
    }
    cleanupPhysics();
    printf("SnippetOmniPvdMultiGeom wrote %u frames to %s\n", frameCount, gOmniPvdPath);
#else
    PX_UNUSED(argc);
    PX_UNUSED(argv);
    printf("OmniPvd is not supported in release build configuration.\n");
#endif
    return 0;
}
