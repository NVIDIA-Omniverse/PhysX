// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// ****************************************************************************
// This snippet records an OmniPvd capture containing one actor of every
// mesh-backed geometry type (convex, triangle, heightfield) plus a sphere
// for reference.  Each shape uses a non-identity local pose and each
// mesh-geometry uses a non-uniform scale, so a downstream OVD reader can
// verify the full actor -> shape -> geom -> mesh chain including localPose
// and geometry scale.
//
// Usage: SnippetOmniPvdMesh --omnipvdfile=<path/to/out.ovd>
// ****************************************************************************

#include <stdio.h>
#include "PxPhysicsAPI.h"
#include "../snippetutils/SnippetUtils.h"
#include "omnipvd/PxOmniPvd.h"

#if PX_SUPPORT_OMNI_PVD
#include "../pvdruntime/include/OmniPvdLibraryFunctions.h"
#include "../pvdruntime/include/OmniPvdWriter.h"
#include "../pvdruntime/include/OmniPvdFileWriteStream.h"

using namespace physx;

static PxDefaultAllocator       gAllocator;
static PxDefaultErrorCallback   gErrorCallback;
static PxFoundation*            gFoundation = NULL;
static PxPhysics*               gPhysics = NULL;
static PxDefaultCpuDispatcher*  gDispatcher = NULL;
static PxScene*                 gScene = NULL;
static PxMaterial*              gMaterial = NULL;

static PxOmniPvd*               gOmniPvd = NULL;
static OmniPvdFileWriteStream*  gOmniPvdFileStream = NULL;
static const char*              gOmniPvdPath = NULL;

// Deterministic pseudo-random for reproducible captures.
static PxU32 gRandState = 0x13579BDFu;
static float randf(float lo, float hi)
{
    gRandState = gRandState * 1664525u + 1013904223u;
    const float u = float(gRandState) / float(0xFFFFFFFFu);
    return lo + u * (hi - lo);
}

static PxConvexMesh* createTetrahedronConvex()
{
    // A 32-vertex tetrahedron-ish cloud.  QuickHull will reduce to the hull.
    const PxU32 numVerts = 32;
    PxVec3 verts[numVerts];
    for (PxU32 i = 0; i < numVerts; ++i)
        verts[i] = PxVec3(randf(-1.0f, 1.0f), randf(-1.0f, 1.0f), randf(-1.0f, 1.0f));

    PxConvexMeshDesc desc;
    desc.points.count = numVerts;
    desc.points.stride = sizeof(PxVec3);
    desc.points.data = verts;
    desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

    PxTolerancesScale tol;
    PxCookingParams params(tol);
    return PxCreateConvexMesh(params, desc, gPhysics->getPhysicsInsertionCallback());
}

static PxTriangleMesh* createGridTriangleMesh()
{
    // 8x8 grid terrain, slight height variation.
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

    PxTolerancesScale tol;
    PxCookingParams params(tol);
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

static void buildMeshScene()
{
    PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
    gDispatcher = PxDefaultCpuDispatcherCreate(2);
    sceneDesc.cpuDispatcher = gDispatcher;
    sceneDesc.filterShader = PxDefaultSimulationFilterShader;
    gScene = gPhysics->createScene(sceneDesc);

    gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.5f);

    // Ground plane -- gives a recognizable static reference in the capture.
    PxRigidStatic* ground = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
    gScene->addActor(*ground);

    // Offset local pose used on every mesh shape so the snapshot's
    // localPos/localQuat columns are non-trivial.
    const PxTransform shapeLocal(
        PxVec3(0.25f, 0.0f, -0.10f),
        PxQuat(PxPi * 0.125f, PxVec3(0.0f, 1.0f, 0.0f)));

    // Convex mesh actor -- dynamic, so it settles under gravity.
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

    // Triangle mesh actor -- static, as PhysX disallows triangle meshes on
    // non-kinematic dynamics.
    {
        PxTriangleMesh* tri = createGridTriangleMesh();
        PxTriangleMeshGeometry geom(tri);
        geom.scale = PxMeshScale(PxVec3(0.5f, 2.0f, 0.5f));

        PxRigidStatic* actor = gPhysics->createRigidStatic(PxTransform(PxVec3(4.0f, 0.0f, -4.0f)));
        PxShape* shape = gPhysics->createShape(geom, *gMaterial, true);
        shape->setLocalPose(shapeLocal);
        actor->attachShape(*shape);
        gScene->addActor(*actor);
        shape->release();
    }

    // Heightfield actor -- static, with non-uniform XYZ scaling.
    {
        PxHeightField* hf = createPlateHeightField();
        PxHeightFieldGeometry geom(hf, PxMeshGeometryFlags(), 0.01f, 2.0f, 2.0f);

        PxRigidStatic* actor = gPhysics->createRigidStatic(PxTransform(PxVec3(0.0f, 0.0f, 6.0f)));
        PxShape* shape = gPhysics->createShape(geom, *gMaterial, true);
        shape->setLocalPose(shapeLocal);
        actor->attachShape(*shape);
        gScene->addActor(*actor);
        shape->release();
    }

    // A bouncing sphere for a familiar non-mesh actor in the snapshot.
    {
        PxRigidDynamic* sphere = PxCreateDynamic(*gPhysics,
            PxTransform(PxVec3(0.0f, 10.0f, 0.0f)),
            PxSphereGeometry(0.5f), *gMaterial, 10.0f);
        sphere->setLinearVelocity(PxVec3(0.0f, 0.0f, 0.0f));
        gScene->addActor(*sphere);
    }
}

static bool initPhysicsWithOmniPvd()
{
    gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
    if (!gFoundation) { printf("Error: PxCreateFoundation\n"); return false; }

    gOmniPvd = PxCreateOmniPvd(*gFoundation);
    if (!gOmniPvd) { printf("Error: PxCreateOmniPvd\n"); return false; }

    OmniPvdWriter* writer = gOmniPvd->getWriter();
    if (!writer) { printf("Error: OmniPvd writer\n"); return false; }
    gOmniPvdFileStream = createOmniPvdFileWriteStream();
    if (!gOmniPvdFileStream) { printf("Error: OmniPvd file stream\n"); return false; }
    gOmniPvdFileStream->setFileName(gOmniPvdPath);
    // Validate the output path before binding; the writer's lazy open is idempotent.
    if (!gOmniPvdFileStream->openStream())
    {
        printf("Error: could not open OmniPvd output file stream for '%s'\n", gOmniPvdPath);
        return false;
    }
    writer->setWriteStream(*gOmniPvdFileStream);

    gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, NULL, gOmniPvd);
    if (!gPhysics) { printf("Error: PxCreatePhysics\n"); return false; }

    if (!gPhysics->getOmniPvd() || !gPhysics->getOmniPvd()->startSampling())
    {
        printf("Error: could not start OmniPvd sampling to file(%s)\n", gOmniPvdPath);
        return false;
    }

    buildMeshScene();
    return true;
}

static bool cleanupPhysics()
{
    bool streamClosed = true;
    PX_RELEASE(gScene);
    PX_RELEASE(gDispatcher);
    PX_RELEASE(gPhysics);
    PX_RELEASE(gOmniPvd);
    if (gOmniPvdFileStream)
    {
        streamClosed = gOmniPvdFileStream->closeStream();
        destroyOmniPvdFileWriteStream(*gOmniPvdFileStream);
        gOmniPvdFileStream = NULL;
    }
    PX_RELEASE(gFoundation);
    return streamClosed;
}

static bool parseOmniPvdOutputFile(int argc, const char* const* argv)
{
    const char* const prefix = "--omnipvdfile=";
    const size_t prefixLen = strlen(prefix);
    if (argc != 2 || strncmp(argv[1], prefix, prefixLen) != 0 || argv[1][prefixLen] == '\0')
    {
        printf("SnippetOmniPvdMesh usage:\n"
               "SnippetOmniPvdMesh --omnipvdfile=<path to output OmniPvd file>\n");
        return false;
    }
    gOmniPvdPath = argv[1] + prefixLen;
    return true;
}

static void stepPhysics()
{
    gScene->simulate(1.0f / 60.0f);
    gScene->fetchResults(true);
}
#endif  // PX_SUPPORT_OMNI_PVD

int snippetMain(int argc, const char* const* argv)
{
#if PX_SUPPORT_OMNI_PVD
    if (!parseOmniPvdOutputFile(argc, argv))
        return 1;

    if (!initPhysicsWithOmniPvd())
    {
        cleanupPhysics();
        return 1;
    }
    const PxU32 frameCount = 60;
    for (PxU32 i = 0; i < frameCount; ++i)
        stepPhysics();
    if (!cleanupPhysics())
    {
        fprintf(stderr, "Error: could not finalize OmniPvd output file '%s'\n", gOmniPvdPath);
        return 1;
    }
    printf("SnippetOmniPvdMesh wrote %u frames to %s\n", frameCount, gOmniPvdPath);
#else
    PX_UNUSED(argc);
    PX_UNUSED(argv);
    printf("OmniPvd is not supported in release build configuration.\n");
#endif
    return 0;
}
