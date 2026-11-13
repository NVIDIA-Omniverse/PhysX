// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Generate a sample .ovd file for testing PVD3 and pvddom.
// Simulates a simple PhysX-like scene: 1 scene, 2 rigid dynamic boxes falling,
// 1 static ground plane, with transforms updating over 10 frames.

#include <cstdio>
#include <cstring>
#include <cmath>
#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

#include "OmniPvdWriter.h"
#include "OmniPvdFileWriteStream.h"
#include "OmniPvdLibraryFunctions.h"
#include "OmniPvdDefines.h"
#include "OmniPvdCommands.h"
#include "numeric_fixture.h"

struct Vec3 { float x, y, z; };
struct Quat { float x, y, z, w; };

static bool truncateFile(const char* path, long bytes)
{
    FILE* file = fopen(path, "r+b");
    if (!file) return false;
#ifdef _WIN32
    const bool truncated = _chsize_s(_fileno(file), static_cast<unsigned long long>(bytes)) == 0;
#else
    const bool truncated = ftruncate(fileno(file), bytes) == 0;
#endif
    return fclose(file) == 0 && truncated;
}

int main(int argc, char** argv)
{
    const char* outputPath = "test_scene.ovd";
    if (argc > 1) outputPath = argv[1];
    const bool unknownDestroy = argc == 3 && std::strcmp(argv[2], "--unknown-destroy") == 0;
    const bool missingOptionalRef = argc == 3 && std::strcmp(argv[2], "--missing-optional-ref") == 0;
    const bool manyActors = argc == 3 && std::strcmp(argv[2], "--many-actors") == 0;
    const bool unsupportedVersion = argc == 3 && std::strcmp(argv[2], "--unsupported-version") == 0;
    const bool metricVariant = argc == 3 && std::strcmp(argv[2], "--metric-variant") == 0;
    const bool truncatedTail = argc == 3 && std::strcmp(argv[2], "--truncated-tail") == 0;
    const bool truncatedFirst = argc == 3 && std::strcmp(argv[2], "--truncated-first") == 0;
    const bool largeTail = argc == 3 && std::strcmp(argv[2], "--truncated-large-payload") == 0;
    const bool deepPath = argc == 3 && std::strcmp(argv[2], "--deep-path") == 0;
    const bool numeric = argc == 3 && PvdDomTest::numericFixtureOption(argv[2]);
    if (argc > 3 || (argc == 3 && !unknownDestroy && !missingOptionalRef && !manyActors &&
        !unsupportedVersion && !metricVariant && !truncatedTail && !truncatedFirst && !largeTail && !deepPath && !numeric)) return 2;

    OmniPvdWriter* writer = createOmniPvdWriter();
    OmniPvdFileWriteStream* fileStream = createOmniPvdFileWriteStream();

    if (!writer || !fileStream)
    {
        printf("Failed to create OmniPVD writer or file stream\n");
        if (fileStream) destroyOmniPvdFileWriteStream(*fileStream);
        if (writer) destroyOmniPvdWriter(*writer);
        return 1;
    }

    fileStream->setFileName(const_cast<char*>(outputPath));
    if (!fileStream->openStream())
    {
        printf("Failed to open %s for writing\n", outputPath);
        destroyOmniPvdWriter(*writer);
        destroyOmniPvdFileWriteStream(*fileStream);
        return 1;
    }

    writer->setWriteStream(*fileStream);

    if (numeric)
    {
        PvdDomTest::writeNumericFixture(*writer, argv[2]);
        destroyOmniPvdWriter(*writer);
        const bool closed = fileStream->closeStream();
        destroyOmniPvdFileWriteStream(*fileStream);
        return closed ? 0 : 1;
    }

    const OmniPvdContextHandle ctx = 1;

    // ---- Register classes (mimicking PhysX OmniPVD output) ----

    // Actor type enum
    OmniPvdClassHandle actorTypeEnum = writer->registerClass("PxActorType");
    writer->registerEnumValue(actorTypeEnum, "eRIGID_STATIC", 0);
    writer->registerEnumValue(actorTypeEnum, "eRIGID_DYNAMIC", 1);

    // Scene
    OmniPvdClassHandle sceneClass = writer->registerClass("PxScene");
    OmniPvdAttributeHandle sceneActorsAttr = writer->registerUniqueListAttribute(sceneClass, "actors", OmniPvdDataType::eOBJECT_HANDLE);

    // Actor base
    OmniPvdClassHandle actorClass = writer->registerClass("PxActor");
    OmniPvdAttributeHandle actorTypeAttr = writer->registerAttribute(actorClass, "type", OmniPvdDataType::eUINT32, 1);
    OmniPvdAttributeHandle actorShapesAttr = writer->registerUniqueListAttribute(actorClass, "shapes", OmniPvdDataType::eOBJECT_HANDLE);

    // RigidActor (inherits from Actor)
    OmniPvdClassHandle rigidActorClass = writer->registerClass("PxRigidActor", actorClass);
    OmniPvdAttributeHandle globalPoseAttr = writer->registerAttribute(rigidActorClass, "globalPose", OmniPvdDataType::eFLOAT32, 7); // quat(4) + pos(3)

    // RigidDynamic (inherits from RigidActor)
    OmniPvdClassHandle rigidDynamicClass = writer->registerClass("PxRigidDynamic", rigidActorClass);
    OmniPvdAttributeHandle linearVelAttr = writer->registerAttribute(rigidDynamicClass, "linearVelocity", OmniPvdDataType::eFLOAT32, 3);
    OmniPvdAttributeHandle debugVectorAttr = 0;
    if (manyActors || metricVariant)
    {
        debugVectorAttr = writer->registerAttribute(rigidDynamicClass, "debugVector", OmniPvdDataType::eFLOAT32, 3);
        writer->registerAttribute(rigidDynamicClass, "reservedVector", OmniPvdDataType::eFLOAT32, 3);
    }

    // RigidStatic (inherits from RigidActor)
    OmniPvdClassHandle rigidStaticClass = writer->registerClass("PxRigidStatic", rigidActorClass);

    // Shape
    OmniPvdClassHandle shapeClass = writer->registerClass("PxShape");
    OmniPvdAttributeHandle shapeLocalPoseAttr = writer->registerAttribute(shapeClass, "localPose", OmniPvdDataType::eFLOAT32, 7);
    OmniPvdAttributeHandle shapeIsExclusiveAttr = writer->registerAttribute(shapeClass, "isExclusive", OmniPvdDataType::eUINT8, 1);

    // Box geometry
    OmniPvdClassHandle boxGeomClass = writer->registerClass("PxBoxGeometry");
    OmniPvdAttributeHandle boxHalfExtentsAttr = writer->registerAttribute(boxGeomClass, "halfExtents", OmniPvdDataType::eFLOAT32, 3);

    // Plane geometry
    OmniPvdClassHandle planeGeomClass = writer->registerClass("PxPlaneGeometry");

    // Material
    OmniPvdClassHandle materialClass = writer->registerClass("PxMaterial");
    OmniPvdAttributeHandle staticFrictionAttr = writer->registerAttribute(materialClass, "staticFriction", OmniPvdDataType::eFLOAT32, 1);
    OmniPvdAttributeHandle dynamicFrictionAttr = writer->registerAttribute(materialClass, "dynamicFriction", OmniPvdDataType::eFLOAT32, 1);
    OmniPvdAttributeHandle restitutionAttr = writer->registerAttribute(materialClass, "restitution", OmniPvdDataType::eFLOAT32, 1);

    // ---- Create objects ----

    // Scene
    const OmniPvdObjectHandle sceneHandle = 100;
    writer->createObject(ctx, sceneClass, sceneHandle, "DefaultScene");

    // Material
    const OmniPvdObjectHandle materialHandle = 200;
    writer->createObject(ctx, materialClass, materialHandle, "DefaultMaterial");
    float staticFriction = 0.5f, dynamicFriction = 0.5f, restitution = 0.6f;
    writer->setAttribute(ctx, materialHandle, staticFrictionAttr, reinterpret_cast<const uint8_t*>(&staticFriction), sizeof(float));
    writer->setAttribute(ctx, materialHandle, dynamicFrictionAttr, reinterpret_cast<const uint8_t*>(&dynamicFriction), sizeof(float));
    writer->setAttribute(ctx, materialHandle, restitutionAttr, reinterpret_cast<const uint8_t*>(&restitution), sizeof(float));

    // Ground plane (rigid static)
    const OmniPvdObjectHandle groundHandle = 300;
    writer->createObject(ctx, rigidStaticClass, groundHandle, "GroundPlane");
    uint32_t staticType = 0;
    writer->setAttribute(ctx, groundHandle, actorTypeAttr, reinterpret_cast<const uint8_t*>(&staticType), sizeof(uint32_t));
    float groundPose[7] = { 0, 0, 0, 1, 0, 0, 0 }; // identity quat + pos
    writer->setAttribute(ctx, groundHandle, globalPoseAttr, reinterpret_cast<const uint8_t*>(groundPose), sizeof(groundPose));

    // Ground shape
    const OmniPvdObjectHandle groundShapeHandle = 301;
    writer->createObject(ctx, shapeClass, groundShapeHandle, "GroundShape");
    uint8_t isExclusive = 1;
    writer->setAttribute(ctx, groundShapeHandle, shapeIsExclusiveAttr, &isExclusive, sizeof(uint8_t));
    float shapeIdentityPose[7] = { 0, 0, 0, 1, 0, 0, 0 };
    writer->setAttribute(ctx, groundShapeHandle, shapeLocalPoseAttr, reinterpret_cast<const uint8_t*>(shapeIdentityPose), sizeof(shapeIdentityPose));

    // Add ground to scene
    writer->addToUniqueListAttribute(ctx, sceneHandle, sceneActorsAttr,
        reinterpret_cast<const uint8_t*>(&groundHandle), sizeof(OmniPvdObjectHandle));
    // Add shape to ground
    writer->addToUniqueListAttribute(ctx, groundHandle, actorShapesAttr,
        reinterpret_cast<const uint8_t*>(&groundShapeHandle), sizeof(OmniPvdObjectHandle));

    // Two falling boxes
    struct BoxActor {
        OmniPvdObjectHandle actorHandle;
        OmniPvdObjectHandle shapeHandle;
        OmniPvdObjectHandle geomHandle;
        float startX, startY;
    };

    BoxActor boxes[2] = {
        { 400, 401, 402, -2.0f, 10.0f },
        { 500, 501, 502,  2.0f, 15.0f },
    };

    for (int i = 0; i < 2; i++)
    {
        auto& box = boxes[i];
        char name[64];
        snprintf(name, sizeof(name), "FallingBox_%d", i);
        writer->createObject(ctx, rigidDynamicClass, box.actorHandle, name);
        uint32_t dynamicType = 1;
        writer->setAttribute(ctx, box.actorHandle, actorTypeAttr,
            reinterpret_cast<const uint8_t*>(&dynamicType), sizeof(uint32_t));

        // Initial pose
        float pose[7] = { 0, 0, 0, 1, box.startX, box.startY, 0.0f };
        writer->setAttribute(ctx, box.actorHandle, globalPoseAttr,
            reinterpret_cast<const uint8_t*>(pose), sizeof(pose));

        Vec3 vel = { 0, 0, 0 };
        writer->setAttribute(ctx, box.actorHandle, linearVelAttr,
            reinterpret_cast<const uint8_t*>(&vel), sizeof(vel));

        // Shape
        snprintf(name, sizeof(name), "BoxShape_%d", i);
        writer->createObject(ctx, shapeClass, box.shapeHandle, name);
        writer->setAttribute(ctx, box.shapeHandle, shapeIsExclusiveAttr, &isExclusive, sizeof(uint8_t));
        writer->setAttribute(ctx, box.shapeHandle, shapeLocalPoseAttr,
            reinterpret_cast<const uint8_t*>(shapeIdentityPose), sizeof(shapeIdentityPose));

        // Geometry
        snprintf(name, sizeof(name), "BoxGeom_%d", i);
        writer->createObject(ctx, boxGeomClass, box.geomHandle, name);
        float halfExtents[3] = { 0.5f, 0.5f, 0.5f };
        writer->setAttribute(ctx, box.geomHandle, boxHalfExtentsAttr,
            reinterpret_cast<const uint8_t*>(halfExtents), sizeof(halfExtents));

        // Add actor to scene
        writer->addToUniqueListAttribute(ctx, sceneHandle, sceneActorsAttr,
            reinterpret_cast<const uint8_t*>(&box.actorHandle), sizeof(OmniPvdObjectHandle));
        // Add shape to actor
        writer->addToUniqueListAttribute(ctx, box.actorHandle, actorShapesAttr,
            reinterpret_cast<const uint8_t*>(&box.shapeHandle), sizeof(OmniPvdObjectHandle));
    }
    if (manyActors || metricVariant)
    {
        for (int i = 0; i < 120; ++i)
        {
            const OmniPvdObjectHandle extraHandle = OmniPvdObjectHandle(1000 + i);
            char name[64];
            snprintf(name, sizeof(name), "ExtraActor_%03d", i);
            writer->createObject(ctx, rigidDynamicClass, extraHandle, name);
            const float debugVector[3] = {float(i + (metricVariant && i == 100 ? 3 : 0)),
                metricVariant && i == 100 ? 5.0f : 1.0f, 2.0f};
            writer->setAttribute(ctx, extraHandle, debugVectorAttr,
                reinterpret_cast<const uint8_t*>(debugVector), sizeof(debugVector));
            writer->addToUniqueListAttribute(ctx, sceneHandle, sceneActorsAttr,
                reinterpret_cast<const uint8_t*>(&extraHandle), sizeof(extraHandle));
        }
        const auto lateList = writer->registerUniqueListAttribute(rigidDynamicClass, "debugList", OmniPvdDataType::eOBJECT_HANDLE);
        const OmniPvdObjectHandle target = 401;
        writer->removeFromUniqueListAttribute(ctx, 1100, lateList, reinterpret_cast<const uint8_t*>(&target), sizeof(target));
        writer->addToUniqueListAttribute(ctx, 1101, lateList, reinterpret_cast<const uint8_t*>(&target), sizeof(target));
        writer->removeFromUniqueListAttribute(ctx, 1101, lateList, reinterpret_cast<const uint8_t*>(&target), sizeof(target));
    }

    // ---- Simulate 10 frames ----
    const float dt = 1.0f / 60.0f;
    const float gravity = -9.81f;

    float positions[2][3] = {
        { boxes[0].startX, boxes[0].startY, 0.0f },
        { boxes[1].startX, boxes[1].startY, 0.0f },
    };
    float velocities[2][3] = {
        { 0, 0, 0 },
        { 0, 0, 0 },
    };

    for (int frame = 0; frame < 10; frame++)
    {
        uint64_t timestamp = frame + 1;
        writer->startFrame(sceneHandle, timestamp);
        if ((manyActors || metricVariant) && frame == 0)
        {
            const float debugVector[3] = {metricVariant ? 103.0f : 100.0f,
                metricVariant ? 5.0f : 1.0f, 2.0f};
            writer->setAttribute(ctx, 1100, debugVectorAttr,
                reinterpret_cast<const uint8_t*>(debugVector), sizeof(debugVector));
        }

        for (int i = 0; i < 2; i++)
        {
            // Simple Euler integration
            velocities[i][1] += gravity * dt;
            positions[i][0] += velocities[i][0] * dt;
            positions[i][1] += velocities[i][1] * dt;
            positions[i][2] += velocities[i][2] * dt;

            // Ground collision at y=0.5 (half extent)
            if (positions[i][1] < 0.5f)
            {
                positions[i][1] = 0.5f;
                velocities[i][1] = -velocities[i][1] * restitution; // bounce
            }

            float pose[7] = {
                0, 0, 0, 1,
                positions[i][0], positions[i][1], positions[i][2]
            };
            if (metricVariant && i == 0)
            {
                pose[2] = 0.7071067811865476f;
                pose[3] = 0.7071067811865476f;
                pose[4] += 3.0f;
                pose[5] += 4.0f;
            }
            writer->setAttribute(ctx, boxes[i].actorHandle, globalPoseAttr,
                reinterpret_cast<const uint8_t*>(pose), sizeof(pose));

            writer->setAttribute(ctx, boxes[i].actorHandle, linearVelAttr,
                reinterpret_cast<const uint8_t*>(velocities[i]), sizeof(float) * 3);
        }

        writer->stopFrame(sceneHandle, timestamp);
    }

    if (missingOptionalRef)
    {
        const OmniPvdObjectHandle missingShape = 9999;
        for (const auto& box : boxes)
            writer->addToUniqueListAttribute(ctx, box.actorHandle, actorShapesAttr,
                reinterpret_cast<const uint8_t*>(&missingShape), sizeof(missingShape));
    }
    if (unknownDestroy) writer->destroyObject(ctx, 9999);

    // ---- Cleanup ----
    destroyOmniPvdWriter(*writer);
    const bool streamClosed = fileStream->closeStream();
    destroyOmniPvdFileWriteStream(*fileStream);

    if (!streamClosed)
    {
        fprintf(stderr, "Failed to finalize %s\n", outputPath);
        return 1;
    }
    if (unsupportedVersion)
    {
        FILE* file = fopen(outputPath, "r+b");
        if (!file) return 1;
        const bool changed = fputc(1, file) != EOF;
        const bool closed = fclose(file) == 0;
        if (!changed || !closed) return 1;
    }
    if (largeTail || deepPath)
    {
        FILE* file = fopen(outputPath, "ab");
        if (!file) return 1;
        const auto put = [file](const auto& value) { return fwrite(&value, sizeof(value), 1, file) == 1; };
        bool written = put(uint8_t(OmniPvdCommand::eSET_ATTRIBUTE)) && put(uint64_t(1)) && put(uint64_t(1)) &&
            put(uint8_t(deepPath ? 33 : 1));
        if (largeTail)
        {
            const uint8_t residual[10] = {};
            written = written && put(uint32_t(0)) && put(uint32_t(100000000)) &&
                fwrite(residual, 1, sizeof(residual), file) == sizeof(residual);
        }
        const bool closed = fclose(file) == 0;
        if (!written || !closed) return 1;
    }
    if (truncatedTail || truncatedFirst)
    {
        FILE* file = fopen(outputPath, "rb");
        if (!file || fseek(file, 0, SEEK_END) != 0) { if (file) fclose(file); return 1; }
        const long size = ftell(file);
        const bool closed = fclose(file) == 0;
        const long newSize = truncatedFirst ? long(sizeof(OmniPvdVersionType) * 3 + 1) : size - 1;
        if (!closed || size <= newSize || !truncateFile(outputPath, newSize)) return 1;
    }
    printf("Generated %s successfully\n", outputPath);
    return 0;
}
