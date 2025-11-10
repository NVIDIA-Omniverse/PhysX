// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "OmniPhysX.h"
#include "Setup.h"
#include "PhysXUnitTestsIface.h"

#include "usdLoad/LoadUsd.h"

#include "internal/InternalScene.h"
#include "ObjectDataQuery.h"

#include <PxPhysicsAPI.h>
#include <carb/logging/Log.h>
#include <carb/logging/StandardLogger.h>

using namespace ::physx;
using namespace pxr;
using namespace carb;

namespace omni
{
namespace physx
{
PhysicsStats getPhysicsStats()
{
    const PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    const PhysXScenesMap& physxScenes = physxSetup.getPhysXScenes();

    PhysicsStats stats;
    if (physxScenes.empty())
        return stats;

    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        PxSimulationStatistics simStats;
        const PxScene* scene = ref.second->getScene();
        if (!scene)
            continue;
        scene->getSimulationStatistics(simStats);

        // bodies
        stats.numDynamicRigids += simStats.nbDynamicBodies;
        stats.numStaticRigids += simStats.nbStaticBodies;
        stats.numKinematicBodies += simStats.nbKinematicBodies;

        // articulations
        stats.numArticulations += simStats.nbArticulations;

        // shapes
        stats.numBoxShapes += simStats.nbShapes[PxGeometryType::eBOX];
        stats.numSphereShapes += simStats.nbShapes[PxGeometryType::eSPHERE];
        stats.numCapsuleShapes += simStats.nbShapes[PxGeometryType::eCAPSULE];
        stats.numConvexShapes += simStats.nbShapes[PxGeometryType::eCONVEXMESH];
        stats.numTriMeshShapes += simStats.nbShapes[PxGeometryType::eTRIANGLEMESH];
        stats.numPlaneShapes += simStats.nbShapes[PxGeometryType::ePLANE];

        stats.numConstraints += simStats.nbActiveConstraints;

        // We don't currently return this value but use it to limit how many actors and shapes that must be iterated.
        uint32_t convexCoreShapesToCheck = simStats.nbShapes[PxGeometryType::eCONVEXCORE];

        const uint32_t nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC);
        
        for (uint32_t i = 0; i < nbActors && convexCoreShapesToCheck > 0; i++)
        {
            PxActor* actor = nullptr;
            scene->getActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1, i);
            {
                PxRigidActor* rbo = actor->is<PxRigidActor>();
                if (rbo)
                {
                    PxShape* shape = nullptr;
                    const PxU32 numShapes = rbo->getNbShapes();
                    for (PxU32 i = 0; i < numShapes; i++)
                    {
                        rbo->getShapes(&shape, 1, i);

                        const PxGeometry& geom = shape->getGeometry();
                        if (geom.getType() == PxGeometryType::eCONVEXCORE)
                        {
                            convexCoreShapesToCheck--;
                            const PxConvexCoreGeometry& convex_core_geometry = static_cast<const PxConvexCoreGeometry&>(geom);
                            if(convex_core_geometry.getCoreType() == PxConvexCore::eCYLINDER)
                            {
                                stats.numCylinderShapes++;
                            }
                            else if(convex_core_geometry.getCoreType() == PxConvexCore::eCONE)
                            {
                                stats.numConeShapes++;
                            }
                        }
                    }
                }
            }
        }
    }

    const usdparser::Axis axis[] = { usdparser::eX , usdparser::eY ,usdparser::eZ };
    for (int i = 0; i < sizeof(axis)/sizeof(axis[0]); i++)
    {
        {
            PxConvexMesh* mesh = physxSetup.getCylinderConvexMesh(axis[i]);
            if (mesh && mesh->getReferenceCount() > 1)
            {
                stats.numCylinderShapes += mesh->getReferenceCount() - 1;
                stats.numConvexShapes -= mesh->getReferenceCount() - 1;
            }
        }
        {
            PxConvexMesh* mesh = physxSetup.getConeConvexMesh(axis[i]);
            if (mesh && mesh->getReferenceCount() > 1)
            {
                stats.numConeShapes += mesh->getReferenceCount() - 1;
                stats.numConvexShapes -= mesh->getReferenceCount() - 1;
            }
        }
    }
    return stats;
}

float getMassInformation(const char* path, Float3& inertia, Float3& com)
{
    float mass = -1.0f;
    const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    SdfPath rbPath(path);

    const uint64_t stageId = OmniPhysX::getInstance().getStageId();
    const usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (!attachedStage)
        return mass;

    OmniPhysX::getInstance().getInternalPhysXDatabase().updateDirtyMassActors();

    PxRigidActor* actor = (PxRigidActor*)getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(rbPath, ePTActor, db, *attachedStage);
    if (!actor)
    {
        actor = (PxRigidActor*)getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(rbPath, ePTLink, db, *attachedStage);
    }

    if (!actor)
    {
        CARB_LOG_WARN("Mass information not found on given path %s (information available only in runtime)", path);
        return mass;
    }

    PxRigidBody* rb = actor->is<PxRigidBody>();
    if (rb)
    {
        mass = rb->getMass();
        const PxVec3 rbInertia = rb->getMassSpaceInertiaTensor();
        const PxVec3 rbCom = rb->getCMassLocalPose().p;
        inertia = { rbInertia.x, rbInertia.y, rbInertia.z };
        com = { rbCom.x, rbCom.y, rbCom.z };
        return mass;
    }
    return mass;
}

void getMaterialsPaths(const pxr::SdfPath& path, std::vector<pxr::SdfPath>& materialPaths)
{
    const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    std::vector<PxMaterial*> materials;

    const uint64_t stageId = OmniPhysX::getInstance().getStageId();
    const usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (!attachedStage)
        return;

    const PxShape* shape = static_cast<const PxShape*>((void*)(getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(path, ePTShape, db, *attachedStage)));
    if (shape != nullptr)
    {
        const PxU32 nbMaterials = shape->getNbMaterials();
        if (nbMaterials)
        {
            materials.resize(size_t(nbMaterials));
            shape->getMaterials(materials.data(), nbMaterials);
        }
    }
    else
    {
        const PhysXCompoundShape* shape = static_cast<const PhysXCompoundShape*>((void*)(getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(path, ePTCompoundShape, db, *attachedStage)));
        if (shape)
        {
            for (size_t i = 0; i < shape->getShapes().size(); i++)
            {
                const PxShape* pxShape = (const PxShape*)shape->getShapes()[i];
                const PxU32 nbMaterials = pxShape->getNbMaterials();
                PxMaterial* material = nullptr;
                for (PxU32 j = 0; j < nbMaterials; j++)
                {
                    pxShape->getMaterials(&material, 1, j);
                    if (std::find(materials.begin(), materials.end(), material) == materials.end())
                    {
                        materials.push_back(material);
                    }
                }
            }
        }
    }

    for (size_t i = 0; i < materials.size(); i++)
    {
        const PxMaterial* material = materials[i];
        size_t index = (size_t)material->userData;
        const internal::InternalDatabase::Record* record = db.getFullTypedRecord(ePTMaterial, index);
        if (record)
        {
            materialPaths.push_back(record->mPath);
        }
    }
}

struct CheckLogger : public logging::Logger
{
    CheckLogger()
    {
        handleMessage = msg;
        reset();
    }

    struct Message
    {
        std::string mText;
        bool mFound;
    };

    std::vector<Message> mMessages;
    bool mExpectedResult;
    bool mExpectAll;
    bool mPartialStringMatch;

    void reset()
    {
        mMessages.clear();
        mExpectedResult = true;
        mExpectAll = false;
        mPartialStringMatch = false;
    }

    void start(std::vector<std::string>& messages, bool expectedResult, bool expectAll, bool partialStringMatch)
    {
        reset();
        useLogger(true);
        mExpectedResult = expectedResult;
        mExpectAll = expectAll;
        mPartialStringMatch = partialStringMatch;

        for (auto& m : messages)
        {
            mMessages.push_back({ m, false });
        }
    }

    void start(const char* message, bool expectedResult, bool partialStringMatch)
    {
        reset();
        useLogger(true);
        mExpectedResult = expectedResult;
        mPartialStringMatch = partialStringMatch;

        mMessages.push_back({ std::string(message), false });
    }

    bool end()
    {
        useLogger(false);

        if (mExpectAll)
        {
            for (auto&m : mMessages)
            {
                if (m.mFound != mExpectedResult)
                    return false;
            }

            return true;
        }
        else
        {
            for (auto& m : mMessages)
            {
                if (m.mFound == mExpectedResult)
                    return true;
            }

            return false;
        }
    }

    static void msg(Logger* logger,
                    const char* source,
                    int32_t level,
                    const char* filename,
                    const char* functionName,
                    int lineNumber,
                    const char* message)
    {
        auto& self = *static_cast<CheckLogger*>(logger);

        bool found = false;

        for (auto& m : self.mMessages)
        {
            bool match = self.mPartialStringMatch ? strstr(message, m.mText.c_str()) != nullptr : m.mText.compare(message) == 0;
            if (match)
            {
                found = true;
                m.mFound = true;
            }
        }

        if (!found)
        {  
            logging::Logger* dl = logging::getLogging()->getDefaultLogger()->getLogger();
            dl->handleMessage(dl, source, level, filename, functionName, lineNumber, message);
        }
    }

    void useLogger(bool enable)
    {
        auto* lg = logging::getLogging();

        if (enable)
        {
            lg->removeLogger(lg->getDefaultLogger()->getLogger());
            lg->addLogger(this);
        }
        else
        {
            lg->removeLogger(this);
            lg->removeLogger(lg->getDefaultLogger()->getLogger());
        }
    }
};

static CheckLogger sLogger;

void startLoggerCheck(const char* message, bool expectedResult, bool partialStringMatch)
{
    sLogger.start(message, expectedResult, partialStringMatch);
}

void startLoggerCheckForMultiple(std::vector<std::string>& messages, bool expectedResult, bool expectAll, bool partialStringMatch)
{
    sLogger.start(messages, expectedResult, expectAll, partialStringMatch);
}

bool endLoggerCheck()
{
    return sLogger.end();
}

bool isCudaLibPresent()
{
    omni::physx::OmniPhysX& physx = omni::physx::OmniPhysX::getInstance();
    return physx.isCudaLibPresent();
}

} // namespace physx
} // namespace omni
