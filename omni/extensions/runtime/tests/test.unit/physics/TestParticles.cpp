// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"
#include "../../common/PhysicsChangeTemplate.h"

#include <common/utilities/CudaHelpers.h>

#include "PhysicsTools.h"

#include <PxPhysicsAPI.h>

#include <omni/fabric/IFabric.h>
#include <omni/fabric/SimStageWithHistory.h>

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxPrivate.h>
#include <private/omni/physx/IPhysxTests.h>
#include <omni/physx/IPhysxSimulation.h>

#include <cuda.h>

#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;
using namespace carb;
using namespace omni::fabric;

namespace
{
#define CHECK_CU(code) cudaHelpers::checkCu(code, __FILE__, __LINE__)

#if USE_PHYSX_GPU
    template<typename T>
    T* memHostAlloc(PxCudaContext* cudaContext, uint32_t numElements)
    {
        void* ptr = NULL;
        cudaContext->memHostAlloc(&ptr, sizeof(T) * numElements, CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
        return reinterpret_cast<T*>(ptr);
    }

    void setMassApi(UsdPrim prim, float mass)
    {
        UsdPhysicsMassAPI massApi = UsdPhysicsMassAPI::Apply(prim);
        massApi.CreateMassAttr().Set(mass);
    }

    PhysxSchemaPhysxParticleSystem createDefaultParticleSystem(UsdStageRefPtr stage, SdfPath path, SdfPath physicsScenePath)
    {
        float particleSpacing = 0.2f;
        float restOffset = particleSpacing * 0.9f;
        float solidRestOffset = restOffset;
        float fluidRestOffset = restOffset * 0.6f;
        float particleContactOffset = std::max(solidRestOffset + 0.001f, fluidRestOffset / 0.6f);
        float contactOffset = restOffset + 0.001f;
        int solverPositionIterations = 4;
        GfVec3f wind(0, 0, 0);

        PhysxSchemaPhysxParticleSystem particleSystem = PhysxSchemaPhysxParticleSystem::Define(stage, path);

        if (particleSystem)
        {
            particleSystem.CreateContactOffsetAttr().Set(contactOffset);
            particleSystem.CreateRestOffsetAttr().Set(restOffset);
            particleSystem.CreateParticleContactOffsetAttr().Set(particleContactOffset);
            particleSystem.CreateSolidRestOffsetAttr().Set(solidRestOffset);
            particleSystem.CreateFluidRestOffsetAttr().Set(fluidRestOffset);
            particleSystem.CreateSolverPositionIterationCountAttr().Set(solverPositionIterations);
            particleSystem.CreateWindAttr().Set(wind);

            particleSystem.GetSimulationOwnerRel().SetTargets(SdfPathVector({physicsScenePath}));
        }
        return particleSystem;
    }

    PhysxSchemaPhysxParticleSetAPI createDefaultPointsParticleSet(UsdStageRefPtr stage, SdfPath path, SdfPath particleSystemPath)
    {
        UsdGeomPoints points = UsdGeomPoints::Define(stage, path);
        PhysxSchemaPhysxParticleSetAPI particleSet;
        if (points)
        {
            bool selfCollision = true;
            bool fluid = false;
            int particleGroup = 0;
            particleSet = PhysxSchemaPhysxParticleSetAPI::Apply(points.GetPrim());
            PhysxSchemaPhysxParticleAPI particleApi(particleSet);
            particleApi.CreateSelfCollisionAttr().Set(selfCollision);
            particleSet.CreateFluidAttr().Set(fluid);
            particleApi.CreateParticleGroupAttr().Set(particleGroup);
            particleApi.GetParticleSystemRel().SetTargets(SdfPathVector({ particleSystemPath }));
        }
        return particleSet;
    }

    PhysxSchemaPhysxParticleSetAPI createDefaultInstancerParticleSet(UsdStageRefPtr stage, SdfPath path, SdfPath particleSystemPath)
    {
        UsdGeomPointInstancer instancer = UsdGeomPointInstancer::Define(stage, path);
        PhysxSchemaPhysxParticleSetAPI particleSet;
        if (instancer)
        {
            const SdfPath sphereActorPath = path.AppendChild(TfToken("sphereActor"));
            UsdGeomSphere sphereGeom = UsdGeomSphere::Define(stage, sphereActorPath);
            instancer.GetPrototypesRel().AddTarget(sphereActorPath);
            double radius = 0.1f / 2.0f;
            sphereGeom.GetRadiusAttr().Set(radius);

            bool selfCollision = true;
            bool fluid = false;
            int particleGroup = 0;
            particleSet = PhysxSchemaPhysxParticleSetAPI::Apply(instancer.GetPrim());
            PhysxSchemaPhysxParticleAPI particleApi(particleSet);
            particleApi.CreateSelfCollisionAttr().Set(selfCollision);
            particleSet.CreateFluidAttr().Set(fluid);
            particleApi.CreateParticleGroupAttr().Set(particleGroup);
            particleApi.GetParticleSystemRel().SetTargets(SdfPathVector({particleSystemPath}));
        }
        return particleSet;
    }

    void addParticles(PhysxSchemaPhysxParticleSetAPI particleSet, const VtArray<GfVec3f>& positions, const VtArray<GfVec3f>& velocities)
    {
        UsdGeomPoints points(particleSet.GetPrim());
        UsdGeomPointInstancer instancer(particleSet.GetPrim());

        VtArray<GfVec3f> usdPositions;
        VtArray<GfVec3f> usdVelocities;
        if (points)
        {
            VtArray<float> usdWidths;
            points.GetPointsAttr().Get(&usdPositions);
            points.GetVelocitiesAttr().Get(&usdVelocities);
            points.GetWidthsAttr().Get(&usdWidths);

            for (size_t i = 0; i < positions.size(); ++i)
            {
                usdPositions.push_back(positions[i]);
                usdVelocities.push_back(velocities[i]);
                usdWidths.push_back(0.1f);
            }

            points.GetPointsAttr().Set(usdPositions);
            points.GetVelocitiesAttr().Set(usdVelocities);
            points.GetWidthsAttr().Set(usdWidths);
        }
        else if (instancer)
        {
            VtArray<int> usdProtoIndices;
            instancer.GetPositionsAttr().Get(&usdPositions);
            instancer.GetVelocitiesAttr().Get(&usdVelocities);
            instancer.GetProtoIndicesAttr().Get(&usdProtoIndices);

            for (size_t i = 0; i < positions.size(); ++i)
            {
                usdPositions.push_back(positions[i]);
                usdVelocities.push_back(velocities[i]);
                usdProtoIndices.push_back(0);
            }

            instancer.GetPositionsAttr().Set(usdPositions);
            instancer.GetVelocitiesAttr().Set(usdVelocities);
            instancer.GetProtoIndicesAttr().Set(usdProtoIndices);
        }
        else
        {
            return;
        }

        if (!particleSet.GetPrim().HasAPI<UsdPhysicsMassAPI>())
        {
            float particleMass = 1;
            setMassApi(particleSet.GetPrim(), particleMass * usdPositions.size());
        }
    }

    void addSingleParticle(PhysxSchemaPhysxParticleSetAPI particleSet)
    {
        VtArray<GfVec3f> positions = { GfVec3f(0, 0, 0) };
        VtArray<GfVec3f> velocities = { GfVec3f(0, 0, 0) };
        addParticles(particleSet, positions, velocities);
    }

    PhysxSchemaPhysxParticleClothAPI createDefaultParticleCloth(UsdStageRefPtr stage, SdfPath path, SdfPath particleSystemPath)
    {
        PhysxSchemaPhysxParticleClothAPI particleCloth;
        UsdGeomMesh mesh = UsdGeomMesh::Define(stage, path);
        if (mesh)
        {
            VtArray<GfVec3f> points = { GfVec3f(1, 1, 0), GfVec3f(-1, 1, 0), GfVec3f(-1, -1, 0), GfVec3f(1, -1, 0) };
            VtArray<GfVec3f> normals = { GfVec3f(0, 0, 1), GfVec3f(0, 0, 1), GfVec3f(0, 0, 1), GfVec3f(0, 0, 1) };
            VtArray<GfVec3f> velocities = { GfVec3f(0, 0, 0), GfVec3f(0, 0, 0), GfVec3f(0, 0, 0), GfVec3f(0, 0, 0) };
            VtArray<int> faceVertexCounts = { 3, 3 };
            VtArray<int> faceVertexIndices = { 0, 1, 2, 0, 2, 3 };

            mesh.CreatePointsAttr().Set(points);
            mesh.CreateNormalsAttr().Set(normals);
            mesh.CreateFaceVertexCountsAttr().Set(faceVertexCounts);
            mesh.CreateFaceVertexIndicesAttr().Set(faceVertexIndices);
            mesh.CreateDoubleSidedAttr().Set(true);
            mesh.CreateVelocitiesAttr().Set(velocities);

            particleCloth = PhysxSchemaPhysxParticleClothAPI::Apply(mesh.GetPrim());
            PhysxSchemaPhysxParticleAPI particleApi(particleCloth);
            particleApi.CreateParticleGroupAttr().Set(0);
            particleApi.GetParticleSystemRel().SetTargets(SdfPathVector({ particleSystemPath }));

            PhysxSchemaPhysxAutoParticleClothAPI autoParticleClothApi =
                PhysxSchemaPhysxAutoParticleClothAPI::Apply(mesh.GetPrim());

            setMassApi(mesh.GetPrim(), 0.1f * points.size());
        }
        return particleCloth;
    }
#endif
}

//-----------------------------------------------------------------------------
// Init test
TEST_CASE("Particles Tests",
          "[omniphysics]"
          "[component=OmniPhysics][owner=vreutskyy][priority=mandatory]")
{
    // constants for setup
    const GfVec3f gravityDirection(0.0f, 0.0f, -1.0f); // use z-up
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const float gravityMagnitude = 10.0f / metersPerStageUnit;
    const float density = 1000.f * metersPerStageUnit * metersPerStageUnit * metersPerStageUnit;

    // setup common to all subcases
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);
    scene.CreateGravityMagnitudeAttr().Set(gravityMagnitude);
    scene.CreateGravityDirectionAttr().Set(gravityDirection);

#if USE_PHYSX_GPU
    SUBCASE("Particle System")
    {
        const SdfPath particleSystemPath = defaultPrimPath.AppendChild(TfToken("particleSystem0"));

        float particleSpacing = 0.2f;
        float restOffset = particleSpacing * 0.9f;
        float solidRestOffset = restOffset;
        float fluidRestOffset = restOffset * 0.6f;
        float particleContactOffset = std::max(solidRestOffset + 0.001f, fluidRestOffset / 0.6f);
        float contactOffset = restOffset + 0.001f;
        int solverPositionIterations = 4;
        GfVec3f wind(0, 0, 0);

        PhysxSchemaPhysxParticleSystem particleSystem = PhysxSchemaPhysxParticleSystem::Define(stage, particleSystemPath);
        const SdfPath pointsSetPath = defaultPrimPath.AppendChild(TfToken("particles0"));

        if (particleSystem)
        {
            particleSystem.CreateContactOffsetAttr().Set(contactOffset);
            particleSystem.CreateRestOffsetAttr().Set(restOffset);
            particleSystem.CreateParticleContactOffsetAttr().Set(particleContactOffset);
            particleSystem.CreateSolidRestOffsetAttr().Set(solidRestOffset);
            particleSystem.CreateFluidRestOffsetAttr().Set(fluidRestOffset);
            particleSystem.CreateSolverPositionIterationCountAttr().Set(solverPositionIterations);
            particleSystem.CreateWindAttr().Set(wind);

            particleSystem.GetSimulationOwnerRel().SetTargets(SdfPathVector({ physicsScenePath }));

            UsdGeomPoints pointsSet = UsdGeomPoints::Define(stage, pointsSetPath);
            if (pointsSet)
            {
                bool selfCollision = true;
                bool fluid = false;
                int particleGroup = 0;
                float particleMass = 1;
                PhysxSchemaPhysxParticleSetAPI particleSet = PhysxSchemaPhysxParticleSetAPI::Apply(pointsSet.GetPrim());
                PhysxSchemaPhysxParticleAPI particleApi(particleSet);
                particleApi.CreateSelfCollisionAttr().Set(selfCollision);
                particleSet.CreateFluidAttr().Set(fluid);
                particleApi.CreateParticleGroupAttr().Set(particleGroup);

                particleApi.GetParticleSystemRel().SetTargets(SdfPathVector({ particleSystemPath }));

                addSingleParticle(particleSet);
            }
        }

        // parse
        physxSim->attachStage(stageId);

        // sanity check
        PxPBDParticleSystem* physxParticleSystem = getPhysxBaseDerivedFromPathChecked<PxPBDParticleSystem>(particleSystemPath, PhysXType::ePTParticleSystem);

        // check particle system
        CHECK_EQ(contactOffset, physxParticleSystem->getContactOffset());
        CHECK_EQ(restOffset, physxParticleSystem->getRestOffset());
        CHECK_EQ(particleContactOffset, physxParticleSystem->getParticleContactOffset());
        CHECK_EQ(solidRestOffset, physxParticleSystem->getSolidRestOffset());
        CHECK_EQ(fluidRestOffset, physxParticleSystem->getFluidRestOffset());
        PxU32 posIt, velIt; physxParticleSystem->getSolverIterationCounts(posIt, velIt);
        CHECK_EQ(solverPositionIterations, posIt);
        CHECK_EQ(1, velIt);
        CHECK_EQ(wind[0], physxParticleSystem->getWind().x);
        CHECK_EQ(wind[1], physxParticleSystem->getWind().y);
        CHECK_EQ(wind[2], physxParticleSystem->getWind().z);

        // modify parameters
        restOffset = particleSpacing * 0.8f;
        solidRestOffset = restOffset;
        fluidRestOffset = restOffset * 0.7f;
        particleContactOffset = std::max(solidRestOffset + 0.002f, fluidRestOffset / 0.65f);
        contactOffset = restOffset + 0.002f;
        solverPositionIterations = 5;
        wind[0] = 0.1f;

        if (particleSystem)
        {
            particleSystem.GetContactOffsetAttr().Set(contactOffset);
            particleSystem.GetRestOffsetAttr().Set(restOffset);
            particleSystem.GetParticleContactOffsetAttr().Set(particleContactOffset);
            particleSystem.GetSolidRestOffsetAttr().Set(solidRestOffset);
            particleSystem.GetFluidRestOffsetAttr().Set(fluidRestOffset);
            particleSystem.GetSolverPositionIterationCountAttr().Set(solverPositionIterations);
            particleSystem.GetWindAttr().Set(wind);
        }

        // step
        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // check particle system
        CHECK_EQ(contactOffset, physxParticleSystem->getContactOffset());
        CHECK_EQ(restOffset, physxParticleSystem->getRestOffset());
        CHECK_EQ(particleContactOffset, physxParticleSystem->getParticleContactOffset());
        CHECK_EQ(solidRestOffset, physxParticleSystem->getSolidRestOffset());
        CHECK_EQ(fluidRestOffset, physxParticleSystem->getFluidRestOffset());
        physxParticleSystem->getSolverIterationCounts(posIt, velIt);
        CHECK_EQ(solverPositionIterations, posIt);
        CHECK_EQ(1, velIt);
        CHECK_EQ(wind[0], physxParticleSystem->getWind().x);
        CHECK_EQ(wind[1], physxParticleSystem->getWind().y);
        CHECK_EQ(wind[2], physxParticleSystem->getWind().z);

        // check particle buffer
        PxParticleAndDiffuseBuffer* particleBuffer = getPhysxPtrFromPathUnchecked<PxParticleAndDiffuseBuffer>(pointsSetPath, PhysXType::ePTParticleSet);
        CHECK_EQ(1, particleBuffer->getNbActiveParticles());
    }

    SUBCASE("ParticleSystem lockedFlags")
    {
        const SdfPath particleSystemPath = defaultPrimPath.AppendChild(TfToken("particleSystem0"));

        float particleSpacing = 0.2f;
        float particleContactOffset = particleSpacing*0.9f*0.6f;

        PhysxSchemaPhysxParticleSystem particleSystem = PhysxSchemaPhysxParticleSystem::Define(stage, particleSystemPath);

        if (particleSystem)
        {
            particleSystem.CreateParticleContactOffsetAttr().Set(particleContactOffset);
            particleSystem.GetSimulationOwnerRel().SetTargets(SdfPathVector({ physicsScenePath }));
            particleSystem.GetPrim().CreateAttribute(TfToken("lockedAxis"), SdfValueTypeNames->Int).Set((1 << 2));
        }

        // parse
        physxSim->attachStage(stageId);

        // sanity check
        PxPBDParticleSystem* physxParticleSystem = getPhysxBaseDerivedFromPathChecked<PxPBDParticleSystem>(particleSystemPath, PhysXType::ePTParticleSystem);

        // check locked flags
        PxParticleLockFlags particleLockFlags = physxParticleSystem->getParticleLockFlags();
        CHECK_EQ(PxParticleLockFlags(PxParticleLockFlag::eLOCK_Z), particleLockFlags);
    }

    SUBCASE("PBD Material")
    {
        float friction = 0.2f;
        float particleFrictionScale = 0.5f;
        float damping = 0.3f;
        float drag = 0.4f;
        float lift = 0.5f;
        float surfaceTension = 0.7f;
        float cohesion = 0.8f;
        float adhesion = 0.5f;
        float particleAdhesionScale = 1.2f;
        float adhesionOffsetScale = 1.2f;
        float viscosity = 0.9f;
        float gravityScale = 0.5f;
        float cflCoefficient = 1.2f;

        UsdShadeMaterial mat = UsdShadeMaterial::Define(stage, defaultPrimPath.AppendChild(TfToken("pbdMaterial")));
        PhysxSchemaPhysxPBDMaterialAPI pbdMatAPI = PhysxSchemaPhysxPBDMaterialAPI::Apply(mat.GetPrim());
        pbdMatAPI.CreateFrictionAttr().Set(friction);
        pbdMatAPI.CreateParticleFrictionScaleAttr().Set(particleFrictionScale);
        pbdMatAPI.CreateDampingAttr().Set(damping);
        pbdMatAPI.CreateDragAttr().Set(drag);
        pbdMatAPI.CreateLiftAttr().Set(lift);
        pbdMatAPI.CreateSurfaceTensionAttr().Set(surfaceTension);
        pbdMatAPI.CreateCohesionAttr().Set(cohesion);
        pbdMatAPI.CreateAdhesionAttr().Set(adhesion);
        pbdMatAPI.CreateParticleAdhesionScaleAttr().Set(particleAdhesionScale);
        pbdMatAPI.CreateAdhesionOffsetScaleAttr().Set(adhesionOffsetScale);
        pbdMatAPI.CreateViscosityAttr().Set(viscosity);
        pbdMatAPI.CreateGravityScaleAttr().Set(gravityScale);
        pbdMatAPI.CreateCflCoefficientAttr().Set(cflCoefficient);

        // parse
        physxSim->attachStage(stageId);

        // sanity check
        PxPBDMaterial* physxPBDMaterial = getPhysxBaseDerivedFromPathChecked<PxPBDMaterial>(mat.GetPath(), PhysXType::ePTPBDMaterial);
        REQUIRE(physxPBDMaterial);

        CHECK_EQ(friction, physxPBDMaterial->getFriction());
        CHECK_EQ(particleFrictionScale, physxPBDMaterial->getParticleFrictionScale());
        CHECK_EQ(damping, physxPBDMaterial->getDamping());
        CHECK_EQ(drag, physxPBDMaterial->getDrag());
        CHECK_EQ(lift, physxPBDMaterial->getLift());
        CHECK_EQ(surfaceTension, physxPBDMaterial->getSurfaceTension());
        CHECK_EQ(cohesion, physxPBDMaterial->getCohesion());
        CHECK_EQ(adhesion, physxPBDMaterial->getAdhesion());
        CHECK_EQ(particleAdhesionScale, physxPBDMaterial->getParticleAdhesionScale());
        CHECK_EQ(adhesionOffsetScale, physxPBDMaterial->getAdhesionRadiusScale());
        CHECK_EQ(viscosity, physxPBDMaterial->getViscosity());
        CHECK_EQ(gravityScale, physxPBDMaterial->getGravityScale());
        CHECK_EQ(cflCoefficient, physxPBDMaterial->getCFLCoefficient());

        // update params:
        friction = friction - 0.1f;
        particleFrictionScale = particleFrictionScale - 0.1f;
        damping = damping - 0.1f;
        drag = drag - 0.1f;
        lift = lift - 0.1f;
        surfaceTension = surfaceTension - 0.1f;
        cohesion = cohesion - 0.1f;
        adhesion = adhesion - 0.1f;
        particleAdhesionScale = particleAdhesionScale - 0.1f;
        adhesionOffsetScale = adhesionOffsetScale - 0.1f;
        viscosity = viscosity - 0.1f;
        gravityScale = gravityScale - 0.1f;
        cflCoefficient = cflCoefficient + 0.1f;

        pbdMatAPI.CreateFrictionAttr().Set(friction);
        pbdMatAPI.CreateParticleFrictionScaleAttr().Set(particleFrictionScale);
        pbdMatAPI.CreateDampingAttr().Set(damping);
        pbdMatAPI.CreateDragAttr().Set(drag);
        pbdMatAPI.CreateLiftAttr().Set(lift);
        pbdMatAPI.CreateSurfaceTensionAttr().Set(surfaceTension);
        pbdMatAPI.CreateCohesionAttr().Set(cohesion);
        pbdMatAPI.CreateAdhesionAttr().Set(adhesion);
        pbdMatAPI.CreateParticleAdhesionScaleAttr().Set(particleAdhesionScale);
        pbdMatAPI.CreateAdhesionOffsetScaleAttr().Set(adhesionOffsetScale);
        pbdMatAPI.CreateViscosityAttr().Set(viscosity);
        pbdMatAPI.CreateGravityScaleAttr().Set(gravityScale);
        pbdMatAPI.CreateCflCoefficientAttr().Set(cflCoefficient);

        // step
        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // check material changes
        CHECK_EQ(friction, physxPBDMaterial->getFriction());
        CHECK_EQ(particleFrictionScale, physxPBDMaterial->getParticleFrictionScale());
        CHECK_EQ(damping, physxPBDMaterial->getDamping());
        CHECK_EQ(drag, physxPBDMaterial->getDrag());
        CHECK_EQ(lift, physxPBDMaterial->getLift());
        CHECK_EQ(surfaceTension, physxPBDMaterial->getSurfaceTension());
        CHECK_EQ(cohesion, physxPBDMaterial->getCohesion());
        CHECK_EQ(adhesion, physxPBDMaterial->getAdhesion());
        CHECK_EQ(particleAdhesionScale, physxPBDMaterial->getParticleAdhesionScale());
        CHECK_EQ(adhesionOffsetScale, physxPBDMaterial->getAdhesionRadiusScale());
        CHECK_EQ(viscosity, physxPBDMaterial->getViscosity());
        CHECK_EQ(gravityScale, physxPBDMaterial->getGravityScale());
        CHECK_EQ(cflCoefficient, physxPBDMaterial->getCFLCoefficient());
    }


    SUBCASE("Material Precedence")
    {
        for (bool useParticleCloth : { false, true })
        {
            // Create particle system
            const SdfPath particleSystemPath = defaultPrimPath.AppendChild(TfToken("particleSystem0"));
            PhysxSchemaPhysxParticleSystem particleSystem =
                createDefaultParticleSystem(stage, particleSystemPath, physicsScenePath);

            // Define Particle Object - otherwise no phase gets created and no material assigned to physx
            if (useParticleCloth)
            {
                const SdfPath particleClothPath = defaultPrimPath.AppendChild(TfToken("particleCloth"));
                createDefaultParticleCloth(stage, particleClothPath, particleSystemPath);
            }
            else
            {
                const SdfPath pointsSetPath = defaultPrimPath.AppendChild(TfToken("particleSet"));
                createDefaultPointsParticleSet(stage, pointsSetPath, particleSystemPath);
            }

            // Create four PBD materials
            SdfPath pbdMaterialPath[4];
            UsdShadeMaterial pbdMaterial[4];
            for (uint32_t i = 0; i < 4; ++i)
            {
                std::string name(std::string("pbdMaterial_") + std::to_string(i));
                pbdMaterialPath[i] = defaultPrimPath.AppendChild(TfToken(name.c_str()));
                pbdMaterial[i] = UsdShadeMaterial::Define(stage, pbdMaterialPath[i]);
                PhysxSchemaPhysxPBDMaterialAPI::Apply(pbdMaterial[i].GetPrim());
            }

            // The following SUBCASE structure makes sure we are going through assinging materials
            // at ever increasing precedence.
            uint32_t caseCounter = 0;
            SUBCASE("Default Material"){}
            SUBCASE("Scene Material Binding")
            {
                UsdShadeMaterialBindingAPI sceneBindAPI = UsdShadeMaterialBindingAPI::Apply(scene.GetPrim());
                sceneBindAPI.Bind(pbdMaterial[0], UsdShadeTokens->fallbackStrength, TfToken("physics"));
                caseCounter++;
                SUBCASE("No Further Bindings"){}
                SUBCASE("Scene Physics Material Binding")
                {
                    UsdRelationship sceneRel =
                        scene.GetPrim().CreateRelationship(TfToken("material:binding:physics"), false);
                    sceneRel.SetTargets(SdfPathVector({ pbdMaterialPath[1] }));
                    caseCounter++;
                    SUBCASE("No Further Bindings"){}
                    SUBCASE("Particle System Material Binding")
                    {
                        UsdShadeMaterialBindingAPI psBindAPI =
                            UsdShadeMaterialBindingAPI::Apply(particleSystem.GetPrim());
                        psBindAPI.Bind(pbdMaterial[2], UsdShadeTokens->fallbackStrength, TfToken("physics"));
                        caseCounter++;
                        SUBCASE("No Further Bindings"){}
                        SUBCASE("Particle System Physics Material Binding")
                        {
                            UsdRelationship psRel =
                                particleSystem.GetPrim().CreateRelationship(TfToken("material:binding:physics"), false);
                            psRel.SetTargets({ pbdMaterialPath[3] });
                            caseCounter++;
                        }
                    }
                }
            }

            physxSim->detachStage();
            physxSim->attachStage(stageId);

            // Materials should be there
            PxBase* baseMaterialPtrs[4];
            for (uint32_t i = 0; i < 4; ++i)
            {
                baseMaterialPtrs[i] = reinterpret_cast<PxBase*>(physx->getPhysXPtr(pbdMaterialPath[i], ePTPBDMaterial));
                CHECK(baseMaterialPtrs[i] != nullptr);
                CHECK(baseMaterialPtrs[i]->is<PxPBDMaterial>());
            }

            // Particle system should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(particleSystemPath, ePTParticleSystem));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxPBDParticleSystem>());

            PxPBDParticleSystem* ps = basePtr->is<PxPBDParticleSystem>();

            // Check material bindings

            // Either default material or one of the assigned materials
            REQUIRE(ps->getNbParticleMaterials() == 1);

            if (caseCounter > 0)
            {
                PxParticleMaterial* particleMaterial;
                REQUIRE(ps->getParticleMaterials(&particleMaterial, 1) == 1);
                REQUIRE(particleMaterial == baseMaterialPtrs[caseCounter - 1]);
            }
        }
    }


    SUBCASE("Material Density")
    {
        // Create base physics and PBD materials
        const SdfPath basePhysicsMaterialPath = defaultPrimPath.AppendChild(TfToken("basePhysicsMaterial"));
        UsdShadeMaterial basePhysicsMaterial = UsdShadeMaterial::Define(stage, basePhysicsMaterialPath);
        UsdPhysicsMaterialAPI::Apply(basePhysicsMaterial.GetPrim());

        UsdShadeMaterial basePBDMaterial = UsdShadeMaterial::Define(stage, basePhysicsMaterialPath);
        PhysxSchemaPhysxPBDMaterialAPI materialAPI = PhysxSchemaPhysxPBDMaterialAPI::Apply(basePBDMaterial.GetPrim());

        // TOFIX: Querying of materials from particle system is not available in the SDK
        // The alternative is to query from PxPhysics. Switch over to querying from particle system once the feature is implemented
        // Need to use the right index when querying material from getPBDMaterials
        int materialIndex = -1;

        SUBCASE("DefaultScene")
        {
            UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(scene.GetPrim());
            bindingAPI.Bind(basePhysicsMaterial, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));

            materialIndex = 0;
        }

        // Create particle system
        const SdfPath particleSystemPath = defaultPrimPath.AppendChild(TfToken("particleSystem0"));
        PhysxSchemaPhysxParticleSystem particleSystem = createDefaultParticleSystem(stage, particleSystemPath, physicsScenePath);

        SUBCASE("ParticleSystem")
        {
            UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(particleSystem.GetPrim());
            bindingAPI.Bind(basePhysicsMaterial, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));

            materialIndex = 1;
        }

        const SdfPath pointsSetPath = defaultPrimPath.AppendChild(TfToken("particles0"));
        UsdGeomPoints pointsSet = UsdGeomPoints::Define(stage, pointsSetPath);
        PhysxSchemaPhysxParticleSetAPI particleSet = PhysxSchemaPhysxParticleSetAPI::Apply(pointsSet.GetPrim());
        PhysxSchemaPhysxParticleAPI particleApi(particleSet);
        particleApi.CreateSelfCollisionAttr().Set(true);
        particleSet.CreateFluidAttr().Set(false);
        particleApi.CreateParticleGroupAttr().Set(0);
        particleApi.GetParticleSystemRel().SetTargets(SdfPathVector({ particleSystemPath }));
        addSingleParticle(particleSet);

        setMassApi(particleSet.GetPrim(), 0.0f);
        materialAPI.CreateDensityAttr().Set(1000.0f);

        physxSim->attachStage(stageId);

        // material should be there
        PxBase* baseMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(basePhysicsMaterialPath, ePTPBDMaterial));
        CHECK(baseMaterialPtr != nullptr);
        CHECK(baseMaterialPtr->is<PxPBDMaterial>());

        // particle system should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(particleSystemPath, ePTParticleSystem));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxPBDParticleSystem>());

        PxPBDParticleSystem* ps = basePtr->is<PxPBDParticleSystem>();
        REQUIRE(ps->getNbParticleMaterials() == 1);

        // TOFIX: Querying of materials from particle system is not available in the SDK
        // The alternative is to query from PxPhysics. Switch over to querying from particle system once the feature is implemented
        PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
        const PxPhysics& pxPhysics = pxScene->getPhysics();
        PxPBDMaterial* material;
        pxPhysics.getPBDMaterials(&material, 1, materialIndex);

        CHECK(material->userData != nullptr);
        CHECK(material == baseMaterialPtr->is<PxPBDMaterial>());

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        PxParticleAndDiffuseBuffer* pointsBuffer = getPhysxPtrFromPathUnchecked<PxParticleAndDiffuseBuffer>(pointsSetPath, PhysXType::ePTParticleSet);
        PxU32 numParticles = pointsBuffer->getNbActiveParticles();

        // allocate some pinned memory to get back the data
        PxScene* scene = getPhysxPtrFromPathUnchecked<PxScene>(physicsScenePath, PhysXType::ePTScene);
        PxCudaContextManager* cudaContextManager = scene->getCudaContextManager();
        PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

        cudaContextManager->acquireContext();
        PxVec4* positionInvMass = memHostAlloc<PxVec4>(cudaContext, numParticles);
        REQUIRE(positionInvMass);

        // memcpy
        cudaContext->memcpyDtoHAsync(positionInvMass, CUdeviceptr(pointsBuffer->getPositionInvMasses()), numParticles * sizeof(PxVec4), 0);

        // synchronize
        cudaContext->streamSynchronize(0);
        cudaContextManager->releaseContext();

        float initialMass = 1.0f / positionInvMass[0].w;

        // Change material density
        materialAPI.CreateDensityAttr().Set(100.0f);

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // Check mass
        cudaContextManager->acquireContext();
        // memcpy
        cudaContext->memcpyDtoHAsync(positionInvMass, CUdeviceptr(pointsBuffer->getPositionInvMasses()), numParticles * sizeof(PxVec4), 0);

        // synchronize
        cudaContext->streamSynchronize(0);
        cudaContextManager->releaseContext();

        float finalMass = 1.0f / positionInvMass[0].w;
        CHECK_LT(finalMass, initialMass);

        // Change mass instead of density
        setMassApi(particleSet.GetPrim(), 1.0f);

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // Check mass
        cudaContextManager->acquireContext();
        // memcpy
        cudaContext->memcpyDtoHAsync(positionInvMass, CUdeviceptr(pointsBuffer->getPositionInvMasses()), numParticles * sizeof(PxVec4), 0);

        // synchronize
        cudaContext->streamSynchronize(0);
        cudaContextManager->releaseContext();

        float mass = 1.0f / positionInvMass[0].w;
        CHECK_EQ(mass, 1.0f);
    }


    SUBCASE("Particle Cloth")
    {
        const SdfPath particleSystemPath = defaultPrimPath.AppendChild(TfToken("particleSystem0"));
        PhysxSchemaPhysxParticleSystem particleSystem = createDefaultParticleSystem(stage, particleSystemPath, physicsScenePath);

        const SdfPath meshPath = defaultPrimPath.AppendChild(TfToken("mesh0"));
        if (particleSystem)
        {
            PhysxSchemaPhysxParticleClothAPI particleCloth = createDefaultParticleCloth(stage, meshPath, particleSystemPath);
        }

        // parse
        physxSim->attachStage(stageId);

        // need to step for things to get set up.
        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // get the cloth buffer
        PxParticleClothBuffer* clothBuffer = getPhysxPtrFromPathUnchecked<PxParticleClothBuffer>(meshPath, PhysXType::ePTParticleClothDeprecated);
        CHECK_EQ(2, clothBuffer->getNbTriangles());
        CHECK_EQ(4, clothBuffer->getNbActiveParticles());
        CHECK_EQ(1, clothBuffer->getNbParticleVolumes());
    }
    
    
    SUBCASE("Runtime Add and Expand Particle Sets")
    {
        const SdfPath particleSystemPath = defaultPrimPath.AppendChild(TfToken("particleSystem0"));
        PhysxSchemaPhysxParticleSystem particleSystem = createDefaultParticleSystem(stage, particleSystemPath, physicsScenePath);

        // parse
        physxSim->attachStage(stageId);

        // add a UsdGeomPoints/UsdGeomPointInstancer with 1 particle
        const SdfPath particleSetPath = defaultPrimPath.AppendChild(TfToken("particleSet"));
        PhysxSchemaPhysxParticleSetAPI particleSet;
        SUBCASE("UsdGeomPoints")
        {
            particleSet = createDefaultPointsParticleSet(stage, particleSetPath, particleSystemPath);
        }
        SUBCASE("UsdGeomPointInstancer")
        {
            particleSet = createDefaultInstancerParticleSet(stage, particleSetPath, particleSystemPath);
        }
        REQUIRE(particleSet);
        addSingleParticle(particleSet);

        //set max particles hint
        const int maxParticles = 2;
        UsdAttribute maxParticlesAttr = particleSet.GetPrim().CreateAttribute(TfToken("physxParticle:maxParticles"), SdfValueTypeNames->Int);
        maxParticlesAttr.Set(maxParticles);

        // step
        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // get the particle buffer & check particle count
        PxParticleAndDiffuseBuffer* bufferParticleSet = getPhysxPtrFromPathUnchecked<PxParticleAndDiffuseBuffer>(particleSetPath, PhysXType::ePTParticleSet);
        CHECK_EQ(1, bufferParticleSet->getNbActiveParticles());
        CHECK_EQ(maxParticles, bufferParticleSet->getMaxParticles());

        // add another UsdGeomPoints/UsdGeomPointInstancer with each 1 particle
        const SdfPath particleSetPath1 = defaultPrimPath.AppendChild(TfToken("particleSet1"));
        PhysxSchemaPhysxParticleSetAPI particleSet1;
        if (particleSet.GetPrim().IsA<UsdGeomPoints>())
        {
            particleSet1 = createDefaultPointsParticleSet(stage, particleSetPath1, particleSystemPath);
        }
        else
        {
            particleSet1 = createDefaultInstancerParticleSet(stage, particleSetPath1, particleSystemPath);
        }
        REQUIRE(particleSet1);
        addSingleParticle(particleSet1);

        // step
        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // check particle counts of the runtime added sets
        PxParticleAndDiffuseBuffer* bufferParticleSet1 = getPhysxPtrFromPathUnchecked<PxParticleAndDiffuseBuffer>(particleSetPath1, PhysXType::ePTParticleSet);
        CHECK_EQ(1, bufferParticleSet1->getNbActiveParticles());
        CHECK_EQ(1, bufferParticleSet1->getMaxParticles());

        // add a particle to the existing UsdGeomPoints/UsdGeomPointInstancer
        addSingleParticle(particleSet);

        // step
        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // check particle count - this is the buffer for the same set, but potentially has been resized so we need to get the pointer again
        PxParticleAndDiffuseBuffer* bufferParticleSet2 = getPhysxPtrFromPathUnchecked<PxParticleAndDiffuseBuffer>(particleSetPath, PhysXType::ePTParticleSet);
        CHECK_EQ(2, bufferParticleSet2->getNbActiveParticles());
        CHECK_EQ(maxParticles, bufferParticleSet2->getMaxParticles());

        // add yet another particle, which should increase the internal capacity
        addSingleParticle(particleSet);

        // step
        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // check particle count - this is the buffer for the same set, but potentially has been resized so we need to get the pointer again
        PxParticleAndDiffuseBuffer* bufferParticleSet3 = getPhysxPtrFromPathUnchecked<PxParticleAndDiffuseBuffer>(particleSetPath, PhysXType::ePTParticleSet);
        CHECK_EQ(3, bufferParticleSet3->getNbActiveParticles());
        CHECK_GE(maxParticles+1, bufferParticleSet3->getMaxParticles());
    }
    
    
    SUBCASE("Runtime Update Particle Sets")
    {
        const SdfPath particleSystemPath = defaultPrimPath.AppendChild(TfToken("particleSystem0"));
        PhysxSchemaPhysxParticleSystem particleSystem = createDefaultParticleSystem(stage, particleSystemPath, physicsScenePath);

        scene.CreateGravityMagnitudeAttr().Set(0.f);

        // add UsdGeomPoints
        const SdfPath pointsSetPath = defaultPrimPath.AppendChild(TfToken("pointsSet"));
        UsdGeomPoints pointsSet;
        const int numParticlesPointsSet = 5;
        {
            PhysxSchemaPhysxParticleSetAPI particleSet = createDefaultPointsParticleSet(stage, pointsSetPath, particleSystemPath);
            REQUIRE(particleSet);
            VtArray<GfVec3f> positions;
            VtArray<GfVec3f> velocities;
            for (int i = 0; i < numParticlesPointsSet; i++)
            {
                positions.push_back(GfVec3f(i * 10.0f, 0.0f, 10.0f));
                velocities.push_back(GfVec3f(0, 0, 0));
            }
            addParticles(particleSet, positions, velocities);
            pointsSet = UsdGeomPoints(particleSet);
        }

        // add UsdGeomPointInstancer
        const SdfPath instancerSetPath = defaultPrimPath.AppendChild(TfToken("instancerSet"));
        UsdGeomPointInstancer instancerSet;
        const int numParticlesInstancerSet = 10;
        {
            PhysxSchemaPhysxParticleSetAPI particleSet = createDefaultInstancerParticleSet(stage, instancerSetPath, particleSystemPath);
            REQUIRE(particleSet);
            VtArray<GfVec3f> positions;
            VtArray<GfVec3f> velocities;
            for (int i = 0; i < numParticlesInstancerSet; i++)
            {
                positions.push_back(GfVec3f(i * 10.0f, 0.0f, 20.0f));
                velocities.push_back(GfVec3f(0, 0, 0));
            }
            addParticles(particleSet, positions, velocities);
            instancerSet = UsdGeomPointInstancer(particleSet);
        }

        // parse
        physxSim->attachStage(stageId);

        // step
        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // sanity check
        int numParticles = numParticlesPointsSet + numParticlesInstancerSet;

        PxParticleAndDiffuseBuffer* pointsBuffer = getPhysxPtrFromPathUnchecked<PxParticleAndDiffuseBuffer>(pointsSetPath, PhysXType::ePTParticleSet);
        PxParticleAndDiffuseBuffer* instancerBuffer = getPhysxPtrFromPathUnchecked<PxParticleAndDiffuseBuffer>(instancerSetPath, PhysXType::ePTParticleSet);
        CHECK_EQ(numParticles, pointsBuffer->getNbActiveParticles() + instancerBuffer->getNbActiveParticles());

        // update positions
        const float toleranceEpsilon = 1e-4f;

        VtArray<GfVec3f> initialPointsPositions;
        VtArray<GfVec3f> initialPointsVelocities;

        if (pointsSet)
        {
            for (int i = 0; i < numParticlesPointsSet; i++)
            {
                initialPointsPositions.push_back(GfVec3f(i * 10.0f, i * 10.0f, 10.0f));
                initialPointsVelocities.push_back(GfVec3f(0, i * 10.0f, 0));
            }
            pointsSet.GetPointsAttr().Set(initialPointsPositions);
            pointsSet.GetVelocitiesAttr().Set(initialPointsVelocities);
        }

        VtArray<GfVec3f> initialInstancerPositions;
        VtArray<GfVec3f> initialInstancerVelocities;

        if (instancerSet)
        {
            for (int i = 0; i < numParticlesInstancerSet; i++)
            {
                initialInstancerPositions.push_back(GfVec3f(i * 10.0f, i * 10.0f, 20.0f));
                initialInstancerVelocities.push_back(GfVec3f(i * 10.0f, 0, i * 10.0f));
            }
            instancerSet.GetPositionsAttr().Set(initialInstancerPositions);
            instancerSet.GetVelocitiesAttr().Set(initialInstancerVelocities);
        }

        float stepSize = 0.017f;
        physxSim->simulate(stepSize, 0.0f);
        physxSim->fetchResults();

        pointsBuffer = getPhysxPtrFromPathUnchecked<PxParticleAndDiffuseBuffer>(pointsSetPath, PhysXType::ePTParticleSet);
        instancerBuffer = getPhysxPtrFromPathUnchecked<PxParticleAndDiffuseBuffer>(instancerSetPath, PhysXType::ePTParticleSet);

        PxScene* scene = getPhysxPtrFromPathUnchecked<PxScene>(physicsScenePath, PhysXType::ePTScene);
        PxCudaContextManager* cudaContextManager = scene->getCudaContextManager();
        PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

        // allocate some pinned memory to get back the data
        cudaContextManager->acquireContext();

        PxVec4* pointsPoints = memHostAlloc<PxVec4>(cudaContext, numParticlesPointsSet);
        REQUIRE(pointsPoints);
        
        PxVec4* pointsVels = memHostAlloc<PxVec4>(cudaContext, numParticlesPointsSet);
        REQUIRE(pointsVels);

        PxVec4* instancerPoints = memHostAlloc<PxVec4>(cudaContext, numParticlesInstancerSet);
        REQUIRE(instancerPoints);

        PxVec4* instancerVels = memHostAlloc<PxVec4>(cudaContext, numParticlesInstancerSet);
        REQUIRE(instancerVels);

        // memcpy
        cudaContext->memcpyDtoHAsync(pointsPoints, CUdeviceptr(pointsBuffer->getPositionInvMasses()), numParticlesPointsSet * sizeof(PxVec4), 0);
        cudaContext->memcpyDtoHAsync(pointsVels, CUdeviceptr(pointsBuffer->getVelocities()), numParticlesPointsSet * sizeof(PxVec4), 0);
        cudaContext->memcpyDtoHAsync(instancerPoints, CUdeviceptr(instancerBuffer->getPositionInvMasses()), numParticlesInstancerSet * sizeof(PxVec4), 0);
        cudaContext->memcpyDtoHAsync(instancerVels, CUdeviceptr(instancerBuffer->getVelocities()), numParticlesInstancerSet * sizeof(PxVec4), 0);

        // synchronize
        cudaContext->streamSynchronize(0);
        cudaContextManager->releaseContext();
    
        // compare
        if (pointsSet)
        {
            for (int i = 0; i < numParticlesPointsSet; i++)
            {
                // even though the particles don't interact, we need to advect them here because we set a velocity.
                GfVec3f advectedPoint = initialPointsPositions[i] + stepSize * initialPointsVelocities[i];
                compare(pointsPoints[i].getXYZ(), PxVec3(advectedPoint[0], advectedPoint[1], advectedPoint[2]), toleranceEpsilon);
                compare(pointsVels[i].getXYZ(), (const PxVec3&)(initialPointsVelocities[i]), toleranceEpsilon);
            }
        }

        if (instancerSet)
        {
            for (int i = 0; i < numParticlesInstancerSet; i++)
            {
                GfVec3f advectedPoint = initialInstancerPositions[i] + stepSize * initialInstancerVelocities[i];
                compare(instancerPoints[i].getXYZ(), PxVec3(advectedPoint[0], advectedPoint[1], advectedPoint[2]), toleranceEpsilon);
                compare(instancerVels[i].getXYZ(), (const PxVec3&)(initialInstancerVelocities[i]), toleranceEpsilon);
            }
        }

        cudaContextManager->acquireContext();

        cudaContext->memFreeHost((void*)pointsPoints);
        cudaContext->memFreeHost((void*)pointsVels);
        cudaContext->memFreeHost((void*)instancerPoints);
        cudaContext->memFreeHost((void*)instancerVels);

        cudaContextManager->releaseContext();
    }

    // reset gravity for further tests.
    scene.CreateGravityMagnitudeAttr().Set(gravityMagnitude);


    SUBCASE("Fabric")
    if (0) // Skip Fabric unit test because it is failing on TC although it passes locally. Suspected driver related issues.
    {
        IPhysxPrivate* physxPriv = physicsTests.acquirePhysxPrivateInterface();
        REQUIRE(physxPriv);

        const SdfPath particleSystemPath = defaultPrimPath.AppendChild(TfToken("particleSystem0"));
        PhysxSchemaPhysxParticleSystem particleSystem = createDefaultParticleSystem(stage, particleSystemPath, physicsScenePath);

        static const TfToken fabricToken{ "physxParticle:fabric" };
        static const TfToken maxParticlesToken{ "physxParticle:maxParticles" };

        // add UsdGeomPoints
        const SdfPath pointsSetPath = defaultPrimPath.AppendChild(TfToken("pointsSet"));
        PhysxSchemaPhysxParticleSetAPI particleSet = createDefaultPointsParticleSet(stage, pointsSetPath, particleSystemPath);
        REQUIRE(particleSet);
        size_t numParticlesPointsSet = 5;
        size_t maxHostBufferSize = numParticlesPointsSet * 2; // over allocate host buffer so that we do not have to reallocate when gpu fabric increases in size
        float particleMass = 1.0f;
        float invParticleMass = 1.0f / particleMass;

        {
#if 0
            VtArray<GfVec3f> positions;
            VtArray<GfVec3f> velocities;
            for (int i = 0; i < numParticlesPointsSet; i++)
            {
                positions.push_back(GfVec3f(i * 10.0f, 0.0f, 10.0f));
                velocities.push_back(GfVec3f(0, 0, 0));
            }
            addParticles(particleSet, positions, velocities);
#endif
            const UsdAttribute fabricAttr = particleSet.GetPrim().CreateAttribute(fabricToken, SdfValueTypeNames->Bool);
            fabricAttr.Set(true);

            size_t maxParticles = numParticlesPointsSet;
            const UsdAttribute maxParticlesAttr = particleSet.GetPrim().CreateAttribute(maxParticlesToken, SdfValueTypeNames->Int);
            maxParticlesAttr.Set(maxParticles);
        }

        FabricChange fabricTemplate;
        fabricTemplate.init(stageId, physicsTests.getApp()->getFramework());

        const omni::fabric::Type typeAppliedSchema(BaseDataType::eTag, 1, 0, AttributeRole::eAppliedSchema);
        const omni::fabric::Type float4ArrayType(BaseDataType::eFloat, 4, 1);

        const omni::fabric::Token tokenParticleSet("PhysxParticleSetAPI");
        const omni::fabric::Token tokenPositionInvMasses("_positionInvMasses");
        const omni::fabric::Token tokenVelocities("_velocitiesFloat4");

        StageReaderWriter srw = fabricTemplate.iStageReaderWriter->get(fabricTemplate.mStageId);

        omni::fabric::PathC primPath = omni::fabric::asInt(pointsSetPath);

        fabricTemplate.iStageReaderWriter->createAttribute(srw.getId(), primPath, tokenPositionInvMasses, (omni::fabric::TypeC)float4ArrayType);
        fabricTemplate.iStageReaderWriter->createAttribute(srw.getId(), primPath, tokenVelocities, (omni::fabric::TypeC)float4ArrayType);

        // parse
        physxSim->attachStage(stageId);

        PxScene* scene = getPhysxPtrFromPathUnchecked<PxScene>(physicsScenePath, PhysXType::ePTScene);
        PxCudaContextManager* cudaContextManager = scene->getCudaContextManager();
        PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

        // allocate some pinned memory to get back the data
        cudaContextManager->acquireContext();

        // allocate host memory
        PxVec4* positionsHost = memHostAlloc<PxVec4>(cudaContext, static_cast<uint32_t>(maxHostBufferSize));
        REQUIRE(positionsHost);

        PxVec4* velocitiesHost = memHostAlloc<PxVec4>(cudaContext, static_cast<uint32_t>(maxHostBufferSize));
        REQUIRE(velocitiesHost);

        fabricTemplate.iStageReaderWriter->setArrayAttributeSize(srw.getId(), primPath, tokenPositionInvMasses, numParticlesPointsSet);
        fabricTemplate.iStageReaderWriter->setArrayAttributeSize(srw.getId(), primPath, tokenVelocities, numParticlesPointsSet);

        const omni::fabric::set<AttrNameAndType_v2> requiredAll = { AttrNameAndType_v2(typeAppliedSchema, tokenParticleSet) };
        const omni::fabric::set<AttrNameAndType_v2> requiredAny = { AttrNameAndType_v2(float4ArrayType, tokenPositionInvMasses),
                                                                 AttrNameAndType_v2(float4ArrayType, tokenVelocities) };

        PrimBucketList primBuckets = srw.findPrims(requiredAll, requiredAny);
        size_t bucketCount = primBuckets.bucketCount();
        for (size_t i = 0; i < bucketCount; i++)
        {
            SpanWithTypeC positionsGpu = fabricTemplate.iStageReaderWriter->getAttributeWrGpu(srw.getId(), primPath, tokenPositionInvMasses);
            SpanWithTypeC velocitiesGpu = fabricTemplate.iStageReaderWriter->getAttributeWrGpu(srw.getId(), primPath, tokenVelocities);

            for (size_t j = 0; j < numParticlesPointsSet; j++)
            {
                float pos = 10.0f * (j + 1);
                positionsHost[j] = PxVec4(pos, pos, pos, invParticleMass);

                float vel = 1.0f * (j + 1);
                velocitiesHost[j] = PxVec4(vel, vel, vel, 0.0f);
            }

            // Native cuda api can be called instead of using PhysX cuda context manager
            // Using async mem copies are more efficient but you must remember to synchronize
            CHECK(CHECK_CU(cuMemcpyHtoDAsync(CUdeviceptr(*reinterpret_cast<carb::Float4**>(positionsGpu.ptr)), positionsHost, numParticlesPointsSet * sizeof(carb::Float4), 0)));
            CHECK(CHECK_CU(cuMemcpyHtoDAsync(CUdeviceptr(*reinterpret_cast<carb::Float4**>(velocitiesGpu.ptr)), velocitiesHost, numParticlesPointsSet * sizeof(carb::Float4), 0)));
        }
        cudaContext->streamSynchronize(0);

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();

        const float toleranceEpsilon = 1e-4f;

        // sanity check
        PxParticleAndDiffuseBuffer* particleBuffer = getPhysxPtrFromPathUnchecked<PxParticleAndDiffuseBuffer>(pointsSetPath, PhysXType::ePTParticleSet);
        CHECK_EQ(numParticlesPointsSet, particleBuffer->getNbActiveParticles());

        PxVec4* positionsPhysx = memHostAlloc<PxVec4>(cudaContext, static_cast<uint32_t>(maxHostBufferSize));
        REQUIRE(positionsPhysx);

        PxVec4* velocitiesPhysx = memHostAlloc<PxVec4>(cudaContext, static_cast<uint32_t>(maxHostBufferSize));
        REQUIRE(velocitiesPhysx);

        // device to host mem copies
        cudaContext->memcpyDtoHAsync(positionsPhysx, CUdeviceptr(particleBuffer->getPositionInvMasses()), numParticlesPointsSet * sizeof(PxVec4), 0);
        cudaContext->memcpyDtoHAsync(velocitiesPhysx, CUdeviceptr(particleBuffer->getVelocities()), numParticlesPointsSet * sizeof(PxVec4), 0);

        for (size_t i = 0; i < bucketCount; i++)
        {
            ConstSpanWithTypeC positionsGpu = fabricTemplate.iStageReaderWriter->getAttributeRdGpu(srw.getId(), primPath, tokenPositionInvMasses);
            ConstSpanWithTypeC velocitiesGpu = fabricTemplate.iStageReaderWriter->getAttributeRdGpu(srw.getId(), primPath, tokenVelocities);

            CHECK(CHECK_CU(cuMemcpyDtoHAsync(positionsHost, CUdeviceptr(*((carb::Float4**)positionsGpu.ptr)), numParticlesPointsSet * sizeof(carb::Float4), 0)));
            CHECK(CHECK_CU(cuMemcpyDtoHAsync(velocitiesHost, CUdeviceptr(*((carb::Float4**)velocitiesGpu.ptr)), numParticlesPointsSet * sizeof(carb::Float4), 0)));
        }
        cudaContext->streamSynchronize(0);

        // Check and verify
        for (size_t i = 0; i < bucketCount; i++)
        {
            for (size_t j = 0; j < numParticlesPointsSet; j++)
            {
                compare(positionsPhysx[i].getXYZ(), positionsHost[i].getXYZ(), toleranceEpsilon);
                compare(velocitiesPhysx[i].getXYZ(), velocitiesHost[i].getXYZ(), toleranceEpsilon);
            }
        }

        // overwrite the particle positions and velocities (using the intial data)
        // if the number of active particles are the same, there is no need to call setArrayAttributeSize()
        for (size_t i = 0; i < bucketCount; i++)
        {
            SpanWithTypeC positionsGpu = fabricTemplate.iStageReaderWriter->getAttributeWrGpu(srw.getId(), primPath, tokenPositionInvMasses);
            SpanWithTypeC velocitiesGpu = fabricTemplate.iStageReaderWriter->getAttributeWrGpu(srw.getId(), primPath, tokenVelocities);

            for (size_t j = 0; j < numParticlesPointsSet; j++)
            {
                float pos = 10.0f * (j + 1);
                positionsHost[j] = PxVec4(pos, pos, pos, invParticleMass);

                float vel = 1.0f * (j + 1);
                velocitiesHost[j] = PxVec4(vel, vel, vel, 0.0f);
            }

            CHECK(CHECK_CU(cuMemcpyHtoDAsync(CUdeviceptr(*reinterpret_cast<carb::Float4**>(positionsGpu.ptr)), positionsHost, numParticlesPointsSet * sizeof(carb::Float4), 0)));
            CHECK(CHECK_CU(cuMemcpyHtoDAsync(CUdeviceptr(*reinterpret_cast<carb::Float4**>(velocitiesGpu.ptr)), velocitiesHost, numParticlesPointsSet * sizeof(carb::Float4), 0)));
        }
        cudaContext->streamSynchronize(0);

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();

        for (size_t i = 0; i < bucketCount; i++)
        {
            ConstSpanWithTypeC positionsGpu = fabricTemplate.iStageReaderWriter->getAttributeRdGpu(srw.getId(), primPath, tokenPositionInvMasses);
            ConstSpanWithTypeC velocitiesGpu = fabricTemplate.iStageReaderWriter->getAttributeRdGpu(srw.getId(), primPath, tokenVelocities);

            CHECK(CHECK_CU(cuMemcpyDtoHAsync(positionsHost, CUdeviceptr(*((carb::Float4**)positionsGpu.ptr)), numParticlesPointsSet * sizeof(carb::Float4), 0)));
            CHECK(CHECK_CU(cuMemcpyDtoHAsync(velocitiesHost, CUdeviceptr(*((carb::Float4**)velocitiesGpu.ptr)), numParticlesPointsSet * sizeof(carb::Float4), 0)));
        }
        cudaContext->streamSynchronize(0);

        // Check and verify
        for (size_t i = 0; i < bucketCount; i++)
        {
            for (size_t j = 0; j < numParticlesPointsSet; j++)
            {
                compare(positionsPhysx[i].getXYZ(), positionsHost[i].getXYZ(), toleranceEpsilon);
                compare(velocitiesPhysx[i].getXYZ(), velocitiesHost[i].getXYZ(), toleranceEpsilon);
            }
        }

        // resize fabric
        numParticlesPointsSet++;
        fabricTemplate.iStageReaderWriter->setArrayAttributeSize(srw.getId(), primPath, tokenPositionInvMasses, numParticlesPointsSet);
        fabricTemplate.iStageReaderWriter->setArrayAttributeSize(srw.getId(), primPath, tokenVelocities, numParticlesPointsSet);

        for (size_t i = 0; i < bucketCount; i++)
        {
            SpanWithTypeC positionsGpu = fabricTemplate.iStageReaderWriter->getAttributeWrGpu(srw.getId(), primPath, tokenPositionInvMasses);
            SpanWithTypeC velocitiesGpu = fabricTemplate.iStageReaderWriter->getAttributeWrGpu(srw.getId(), primPath, tokenVelocities);

            for (size_t j = 0; j < numParticlesPointsSet; j++)
            {
                float pos = -10.0f * (j + 1);
                positionsHost[j] = PxVec4(pos, pos, pos, invParticleMass);

                float vel = -1.0f * (j + 1);
                velocitiesHost[j] = PxVec4(vel, vel, vel, 0.0f);
            }

            CHECK(CHECK_CU(cuMemcpyHtoDAsync(CUdeviceptr(*reinterpret_cast<carb::Float4**>(positionsGpu.ptr)), positionsHost, numParticlesPointsSet * sizeof(carb::Float4), 0)));
            CHECK(CHECK_CU(cuMemcpyHtoDAsync(CUdeviceptr(*reinterpret_cast<carb::Float4**>(velocitiesGpu.ptr)), velocitiesHost, numParticlesPointsSet * sizeof(carb::Float4), 0)));
        }
        cuStreamSynchronize(0);

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();

        // if number of active particles exceed max particles, PhysX particle buffer will be automatically expanded
        particleBuffer = getPhysxPtrFromPathUnchecked<PxParticleAndDiffuseBuffer>(pointsSetPath, PhysXType::ePTParticleSet);
        CHECK_EQ(numParticlesPointsSet, particleBuffer->getNbActiveParticles());

        // device to host mem copies
        cudaContext->memcpyDtoHAsync(positionsPhysx, CUdeviceptr(particleBuffer->getPositionInvMasses()), numParticlesPointsSet * sizeof(PxVec4), 0);
        cudaContext->memcpyDtoHAsync(velocitiesPhysx, CUdeviceptr(particleBuffer->getVelocities()), numParticlesPointsSet * sizeof(PxVec4), 0);

        for (size_t i = 0; i < bucketCount; i++)
        {
            ConstSpanWithTypeC positionsGpu = fabricTemplate.iStageReaderWriter->getAttributeRdGpu(srw.getId(), primPath, tokenPositionInvMasses);
            ConstSpanWithTypeC velocitiesGpu = fabricTemplate.iStageReaderWriter->getAttributeRdGpu(srw.getId(), primPath, tokenVelocities);

            CHECK(CHECK_CU(cuMemcpyDtoHAsync(positionsHost, CUdeviceptr(*((carb::Float4**)positionsGpu.ptr)), numParticlesPointsSet * sizeof(carb::Float4), 0)));
            CHECK(CHECK_CU(cuMemcpyDtoHAsync(velocitiesHost, CUdeviceptr(*((carb::Float4**)velocitiesGpu.ptr)), numParticlesPointsSet * sizeof(carb::Float4), 0)));
        }
        cudaContext->streamSynchronize(0);

        // Check and verify
        for (size_t i = 0; i < bucketCount; i++)
        {
            for (size_t j = 0; j < numParticlesPointsSet; j++)
            {
                compare(positionsPhysx[i].getXYZ(), positionsHost[i].getXYZ(), toleranceEpsilon);
                compare(velocitiesPhysx[i].getXYZ(), velocitiesHost[i].getXYZ(), toleranceEpsilon);
            }
        }

        // simulate for a few more frames
        for (size_t i = 0; i < 5; i++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
            physxSim->fetchResults();
        }

        // device to host mem copies
        cudaContext->memcpyDtoHAsync(positionsPhysx, CUdeviceptr(particleBuffer->getPositionInvMasses()), numParticlesPointsSet * sizeof(PxVec4), 0);
        cudaContext->memcpyDtoHAsync(velocitiesPhysx, CUdeviceptr(particleBuffer->getVelocities()), numParticlesPointsSet * sizeof(PxVec4), 0);

        for (size_t i = 0; i < bucketCount; i++)
        {
            ConstSpanWithTypeC positionsGpu = fabricTemplate.iStageReaderWriter->getAttributeRdGpu(srw.getId(), primPath, tokenPositionInvMasses);
            ConstSpanWithTypeC velocitiesGpu = fabricTemplate.iStageReaderWriter->getAttributeRdGpu(srw.getId(), primPath, tokenVelocities);

            CHECK(CHECK_CU(cuMemcpyDtoHAsync(positionsHost, CUdeviceptr(*((carb::Float4**)positionsGpu.ptr)), numParticlesPointsSet * sizeof(carb::Float4), 0)));
            CHECK(CHECK_CU(cuMemcpyDtoHAsync(velocitiesHost, CUdeviceptr(*((carb::Float4**)velocitiesGpu.ptr)), numParticlesPointsSet * sizeof(carb::Float4), 0)));
        }
        cudaContext->streamSynchronize(0);

        // Check and verify
        for (size_t i = 0; i < bucketCount; i++)
        {
            for (size_t j = 0; j < numParticlesPointsSet; j++)
            {
                compare(positionsPhysx[i].getXYZ(), positionsHost[i].getXYZ(), toleranceEpsilon);
                compare(velocitiesPhysx[i].getXYZ(), velocitiesHost[i].getXYZ(), toleranceEpsilon);
            }
        }

        cudaContextManager->releaseContext();
    }
#endif // USE_PHYSX_GPU

#if 0
    // debugging
    stage->Export("ParticlesDebug.usda");
#endif

    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

