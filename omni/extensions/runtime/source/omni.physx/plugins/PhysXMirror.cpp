// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/Defines.h>

#include "PhysXMirror.h"

#include <extensions/PxCollectionExt.h>

using namespace ::physx;

namespace omni
{
namespace physx
{
namespace
{

void addMeshToSharedCollection(const PxShape& shape, PxCollection& sharedCollection)
{
    const PxGeometry& geom = shape.getGeometry();
    const PxGeometryType::Enum geomType = geom.getType();
    if (geomType == PxGeometryType::eCONVEXMESH)
    {
        const PxConvexMeshGeometry& cg = static_cast<const PxConvexMeshGeometry&>(geom);
        if(cg.convexMesh != NULL)
        {
            sharedCollection.add(*cg.convexMesh);
        }
    }
    else if (geomType == PxGeometryType::eTRIANGLEMESH)
    {
        const PxTriangleMeshGeometry& tg = static_cast<const PxTriangleMeshGeometry&>(geom);
        if (tg.triangleMesh != NULL)
        {
            sharedCollection.add(*tg.triangleMesh);
        }
    }
}

}

void mirrorActor(::physx::PxActor& actor, void*& outMemory, uint32_t& memSize, ::physx::PxSerializationRegistry& sr, ::physx::PxCollection*& outSharedCol)
{
    PxCollection* collection = PxCreateCollection();
    PxCollection* sharedCollection = PxCreateCollection();

    // share materials and meshes
    {
        PxRigidActor* rbo = actor.is<PxRigidActor>();
        if (rbo)
        {
            PxShape* shape = nullptr;
            PxMaterial* material = nullptr;
            const PxU32 numShapes = rbo->getNbShapes();
            for (PxU32 i = 0; i < numShapes; i++)
            {
                rbo->getShapes(&shape, 1, i);

                // share meshes
                addMeshToSharedCollection(*shape, *sharedCollection);

                // share materials
                const PxU32 nbMaterials = shape->getNbMaterials();
                for (PxU32 j = 0; j < nbMaterials; j++)
                {
                    shape->getMaterials(&material, 1, j);
                    sharedCollection->add(*material);
                }
            }
        }
    }

    PxSerialization::complete(*sharedCollection, sr);
    PxSerialization::createSerialObjectIds(*sharedCollection, PxSerialObjectId(77));

    collection->add(actor);

    PxSerialization::complete(*collection, sr, sharedCollection);

    PxDefaultMemoryOutputStream outStream;

    PxSerialization::serializeCollectionToBinary(outStream, *collection, sr, sharedCollection);
    collection->release();

    PxDefaultMemoryInputData inputStream(outStream.getData(), outStream.getSize());
    void* memory = malloc(outStream.getSize() + PX_SERIAL_FILE_ALIGN);
    void* memoryA = (void*)((size_t(memory) + PX_SERIAL_FILE_ALIGN) & ~(PX_SERIAL_FILE_ALIGN - 1));
    inputStream.read(memoryA, inputStream.getLength());

    memSize = outStream.getSize();

    outMemory = memory;

    outSharedCol = sharedCollection;
}

::physx::PxActor* instantiateMirrorActor(void* nonalignedBlock, ::physx::PxSerializationRegistry& sr, ::physx::PxScene& scene,
    ::physx::PxCollection*& collectionOut, const ::physx::PxCollection* sharedCollection)
{
    void* memoryA = (void*)((size_t(nonalignedBlock) + PX_SERIAL_FILE_ALIGN) & ~(PX_SERIAL_FILE_ALIGN - 1));

    PxActor* retActor = nullptr;

    PxCollection* collection1 = PxSerialization::createCollectionFromBinary(memoryA, sr, sharedCollection);

    // Add collection to the scene.
    scene.addCollection(*collection1);

    PxBase* basePtr = nullptr;
    const PxU32 nbPtrs = collection1->getNbObjects();
    for (PxU32 i = 0; i < nbPtrs; i++)
    {
        collection1->getObjects(&basePtr, 1, i);
        if (basePtr->getConcreteType() == PxConcreteType::eRIGID_STATIC)
        {
            retActor = static_cast<PxActor*>(basePtr);
            break;
        }
        else if (basePtr->getConcreteType() == PxConcreteType::eRIGID_DYNAMIC)
        {
            retActor = static_cast<PxActor*>(basePtr);
            break;
        }
    }

    collectionOut = collection1;

    return retActor;
}

void mirrorHierarchy(::physx::PxScene& scene, void*& outMemory, uint32_t& memSize, ::physx::PxSerializationRegistry& sr, ::physx::PxCollection*& outSharedCol, const std::unordered_set<void*>& inPtrs)
{
    PxCollection* collection = PxCreateCollection();
    PxCollection* sharedCollection = PxCreateCollection();

    // Dynamic/static bodies
    {
        const uint32_t nbActors = scene.getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC);
        for (uint32_t i = 0; i < nbActors; i++)
        {
            PxActor* actor = nullptr;
            scene.getActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1, i);
            // share materials and meshes
            {
                PxRigidActor* rbo = actor->is<PxRigidActor>();
                if (rbo && inPtrs.find(rbo) != inPtrs.end())
                {
                    PxShape* shape = nullptr;
                    PxMaterial* material = nullptr;
                    const PxU32 numShapes = rbo->getNbShapes();
                    for (PxU32 i = 0; i < numShapes; i++)
                    {
                        rbo->getShapes(&shape, 1, i);

                        // share meshes
                        addMeshToSharedCollection(*shape, *sharedCollection);

                        // share materials
                        const PxU32 nbMaterials = shape->getNbMaterials();
                        for (PxU32 j = 0; j < nbMaterials; j++)
                        {
                            shape->getMaterials(&material, 1, j);
                            // if material is not part of the hierarchy its shared
                            if (material && inPtrs.find(material) == inPtrs.end())
                            {
                                sharedCollection->add(*material);
                            }
                        }
                    }
                    collection->add(*actor);
                }
            }
        }
    }
    // articulations
    const uint32_t nbArticulations = scene.getNbArticulations();
    for (uint32_t i = 0; i < nbArticulations; i++)
    {
        PxArticulationReducedCoordinate* articulation = nullptr;
        scene.getArticulations(&articulation, 1, i);
        if (inPtrs.find(articulation) != inPtrs.end())
        {
            PxArticulationLink* link = nullptr;
            const uint32_t nbLinks = articulation->getNbLinks();
            for (uint32_t i = 0; i < nbLinks; i++)
            {
                articulation->getLinks(&link, 1, i);
                // share materials and meshes
                PxShape* shape = nullptr;
                PxMaterial* material = nullptr;
                const PxU32 numShapes = link->getNbShapes();
                for (PxU32 i = 0; i < numShapes; i++)
                {
                    link->getShapes(&shape, 1, i);

                    // share meshes
                    addMeshToSharedCollection(*shape, *sharedCollection);

                    // share materials
                    const PxU32 nbMaterials = shape->getNbMaterials();
                    for (PxU32 j = 0; j < nbMaterials; j++)
                    {
                        shape->getMaterials(&material, 1, j);
                        sharedCollection->add(*material);
                    }
                }
            }
            collection->add(*articulation);
        }
    }
    // aggregates
    const uint32_t nbAggregates = scene.getNbAggregates();
    for (uint32_t i = 0; i < nbAggregates; i++)
    {
        PxAggregate* aggregate = nullptr;
        scene.getAggregates(&aggregate, 1, i);
        if (inPtrs.find(aggregate) != inPtrs.end())
        {
            collection->add(*aggregate);
        }
    }
    // constraints
    const uint32_t nbConstraints = scene.getNbConstraints();
    for (uint32_t i = 0; i < nbConstraints; i++)
    {
        PxConstraint* constraint = nullptr;
        scene.getConstraints(&constraint, 1, i);
        if (inPtrs.find(constraint) != inPtrs.end())
        {
            PxU32 typeId;
            PxJoint* joint = reinterpret_cast<PxJoint*>(constraint->getExternalReference(typeId));
            if (joint && (typeId == PxConstraintExtIDs::eJOINT))
            {
                collection->add(*joint);
            }
        }
    }

    PxSerialization::complete(*sharedCollection, sr);
    PxSerialization::createSerialObjectIds(*sharedCollection, PxSerialObjectId(77));


    PxSerialization::complete(*collection, sr, sharedCollection);

    PxDefaultMemoryOutputStream outStream;

    PxSerialization::serializeCollectionToBinary(outStream, *collection, sr, sharedCollection);
    collection->release();

    PxDefaultMemoryInputData inputStream(outStream.getData(), outStream.getSize());
    void* memory = malloc(outStream.getSize() + PX_SERIAL_FILE_ALIGN);
    void* memoryA = (void*)((size_t(memory) + PX_SERIAL_FILE_ALIGN) & ~(PX_SERIAL_FILE_ALIGN - 1));
    inputStream.read(memoryA, inputStream.getLength());

    memSize = outStream.getSize();

    outMemory = memory;

    outSharedCol = sharedCollection;
}

::physx::PxCollection* instantiateHierarchy(void* alignedBlock, ::physx::PxSerializationRegistry& sr, ::physx::PxScene& scene, const ::physx::PxCollection* sharedCollection, const ProcessObjectFn& proccesObjectFn)
{
    PxCollection* collection1 = PxSerialization::createCollectionFromBinary(alignedBlock, sr, sharedCollection);

    const uint32_t nbObjects = collection1->getNbObjects();
    for (uint32_t o = 0; o < nbObjects; o++)
    {
        PxBase& object = collection1->getObject(o);
        proccesObjectFn(object);
    }

    // Add collection to the scene.
    scene.addCollection(*collection1);
    return collection1;
}

}
}
