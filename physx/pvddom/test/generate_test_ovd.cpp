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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.


// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Generate a sample .ovd file for testing PVD3 and pvddom.
// Simulates a simple PhysX-like scene: 1 scene, 2 rigid dynamic boxes falling,
// 1 static ground plane, with transforms updating over 10 frames.

#include <cstdio>
#include <cstring>
#include <cmath>

#include "OmniPvdWriter.h"
#include "OmniPvdFileWriteStream.h"
#include "OmniPvdDefines.h"

extern "C" {
    OmniPvdWriter* OMNI_PVD_CALL createOmniPvdWriter();
    void OMNI_PVD_CALL destroyOmniPvdWriter(OmniPvdWriter& writer);
    OmniPvdFileWriteStream* OMNI_PVD_CALL createOmniPvdFileWriteStream();
    void OMNI_PVD_CALL destroyOmniPvdFileWriteStream(OmniPvdFileWriteStream& stream);
}

struct Vec3 { float x, y, z; };
struct Quat { float x, y, z, w; };

int main(int argc, char** argv)
{
    const char* outputPath = "test_scene.ovd";
    if (argc > 1) outputPath = argv[1];

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
    if (!fileStream->openFile())
    {
        printf("Failed to open %s for writing\n", outputPath);
        destroyOmniPvdFileWriteStream(*fileStream);
        destroyOmniPvdWriter(*writer);
        return 1;
    }

    writer->setWriteStream(*fileStream);

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
    OmniPvdAttributeHandle globalPoseAttr = writer->registerAttribute(rigidActorClass, "globalPose", OmniPvdDataType::eFLOAT32, 7); // pos(3) + quat(4)

    // RigidDynamic (inherits from RigidActor)
    OmniPvdClassHandle rigidDynamicClass = writer->registerClass("PxRigidDynamic", rigidActorClass);
    OmniPvdAttributeHandle linearVelAttr = writer->registerAttribute(rigidDynamicClass, "linearVelocity", OmniPvdDataType::eFLOAT32, 3);

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
    float groundPose[7] = { 0, 0, 0, 0, 0, 0, 1 }; // pos + identity quat
    writer->setAttribute(ctx, groundHandle, globalPoseAttr, reinterpret_cast<const uint8_t*>(groundPose), sizeof(groundPose));

    // Ground shape
    const OmniPvdObjectHandle groundShapeHandle = 301;
    writer->createObject(ctx, shapeClass, groundShapeHandle, "GroundShape");
    uint8_t isExclusive = 1;
    writer->setAttribute(ctx, groundShapeHandle, shapeIsExclusiveAttr, &isExclusive, sizeof(uint8_t));
    float shapeIdentityPose[7] = { 0, 0, 0, 0, 0, 0, 1 };
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
        float pose[7] = { box.startX, box.startY, 0.0f, 0, 0, 0, 1 };
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
        writer->startFrame(ctx, timestamp);

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
                positions[i][0], positions[i][1], positions[i][2],
                0, 0, 0, 1
            };
            writer->setAttribute(ctx, boxes[i].actorHandle, globalPoseAttr,
                reinterpret_cast<const uint8_t*>(pose), sizeof(pose));

            writer->setAttribute(ctx, boxes[i].actorHandle, linearVelAttr,
                reinterpret_cast<const uint8_t*>(velocities[i]), sizeof(float) * 3);
        }

        writer->stopFrame(ctx, timestamp);
    }

    // ---- Cleanup ----
    fileStream->closeFile();
    destroyOmniPvdFileWriteStream(*fileStream);
    destroyOmniPvdWriter(*writer);

    printf("Generated %s successfully\n", outputPath);
    return 0;
}
