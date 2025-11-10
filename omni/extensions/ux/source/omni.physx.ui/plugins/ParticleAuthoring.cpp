// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxParticlesPrivate.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>

#include "ParticleAuthoring.h"

#include <carb/profiler/Profile.h>
#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>
#include <private/omni/physx/PhysxUsd.h>

#include <usdrt/scenegraph/usd/usd/stage.h>

using namespace omni::physx;
using namespace omni::physx::ui;
using namespace pxr;
using namespace carb;

extern UsdStageRefPtr gStage;
extern omni::physx::IPhysxParticlesPrivate* gPhysXParticlesPrivate;
extern carb::settings::ISettings* gSettings;

namespace
{
    SdfPath getStageNextFreePath(SdfPath inPath)
    {
        if (inPath.IsEmpty())
        {
            return inPath;
        }

        std::string s = inPath.GetString();

        // find and increment the digit in the name until a free path is found
        // similar to the python implementation, but in c++
        // not optimized, because not expected to be used very often
        while (gStage->GetPrimAtPath(SdfPath(s)))
        {
            std::smatch m;
            std::regex_search(s, m, std::regex("_(\\d+)$")); // find pattern like _21
            if (m.size() > 0)
            {
                size_t size = m[0].str().size();
                std::string partial = m[0].str().substr(1, size - 1); // extract only digits
                int newNum = std::stoi(partial) + 1; // increment
                std::string numString = (newNum < 10) ? "_0" + std::to_string(newNum) : "_" + std::to_string(newNum); // create string from new digit
                s = std::regex_replace(s, std::regex("_(\\d+)$"), numString); // replace matched pattern from above
            }
            else
            {
                s = s + std::string("_01");
            }
        }
        return SdfPath(s);
    }

    SdfPath getDefaultParticleSystemPath()
    {
        SdfPath defaultPath;
        if (gStage)
        {
            defaultPath = gStage->GetDefaultPrim().GetPath().AppendChild(TfToken("ParticleSystem"));
        }
        return defaultPath;
    }

    SdfPath getDefaultParticleSetPath()
    {
        SdfPath defaultPath;
        if (gStage)
        {
            defaultPath = gStage->GetDefaultPrim().GetPath().AppendChild(TfToken("ParticleSet"));
        }
        return defaultPath;
    }

    uint32_t getPostFlags(bool anisotropy, bool smoothing, bool isosurface)
    {
        uint32_t flags = ParticlePostFlag::eNone;
        flags |= anisotropy ? ParticlePostFlag::eAnisotropy : 0;
        flags |= smoothing ? ParticlePostFlag::eSmoothing : 0;
        flags |= isosurface ? ParticlePostFlag::eIsosurface : 0;
        return flags;
    }

    uint32_t getPostFlagsFromParticleSystem(const SdfPath& particleSystemPath)
    {
        UsdPrim prim = gStage->GetPrimAtPath(particleSystemPath);
        if (!prim)
        {
            return ParticlePostFlag::eNone;
        }

        bool hasAnisotropy = false;
        bool hasSmoothing = false;
        bool hasIsosurface = false;

        if (prim.HasAPI<PhysxSchemaPhysxParticleAnisotropyAPI>())
        {
            PhysxSchemaPhysxParticleAnisotropyAPI(prim).GetParticleAnisotropyEnabledAttr().Get(&hasAnisotropy);
        }

        if (prim.HasAPI<PhysxSchemaPhysxParticleSmoothingAPI>())
        {
            PhysxSchemaPhysxParticleSmoothingAPI(prim).GetParticleSmoothingEnabledAttr().Get(&hasSmoothing);
        }

        if (prim.HasAPI<PhysxSchemaPhysxParticleIsosurfaceAPI>())
        {
            PhysxSchemaPhysxParticleIsosurfaceAPI(prim).GetIsosurfaceEnabledAttr().Get(&hasIsosurface);
        }

        return getPostFlags(hasAnisotropy, hasSmoothing, hasIsosurface);
    }
}

ParticleAuthoring::ParticleAuthoring(bool enable) :
    mFirstUpdate(true), 
    mSceneIsPlaying(false),
    mSceneIsPaused(false),
    mIsEnabled(enable)
{ }

ParticleAuthoring::~ParticleAuthoring()
{
    release();
}

void ParticleAuthoring::parseStage()
{
    CARB_PROFILE_ZONE(0, "ParticleAuthoring::parseStage");

    if (!mIsEnabled)
    {
        return;
    }

    PXR_NS::UsdStageCache& cache = PXR_NS::UsdUtilsStageCache::Get();
    omni::fabric::UsdStageId stageId = { static_cast<uint64_t>(cache.GetId(gStage).ToLongInt()) };
    omni::fabric::IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(stageId);
    usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(stageId, stageInProgress);

    bool needToParse = false;
    {
        const std::vector<usdrt::SdfPath> paths = usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxParticleSamplingAPI"));
        if (!paths.empty())
            needToParse = true;
    }
    if (!needToParse)
    {
        const std::vector<usdrt::SdfPath> paths = usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxParticleSetAPI"));
        if (!paths.empty())
            needToParse = true;
    }
    if (!needToParse)
    {
        const std::vector<usdrt::SdfPath> paths = usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("PhysxParticleSystem"));
        if (!paths.empty())
            needToParse = true;
    }

    if (needToParse)
        handlePrimResync(gStage->GetPseudoRoot().GetPath());
}

void ParticleAuthoring::handlePrimResync(const SdfPath path)
{
    CARB_PROFILE_ZONE(0, "ParticleAuthoring::handlePrimResync");

    if (mSceneIsPlaying && !mSceneIsPaused)
    {
        return;
    }

    UsdPrimRange range(gStage->GetPrimAtPath(path));
    for(UsdPrimRange::const_iterator cit = range.begin(); cit != range.end(); ++cit)
    {
        const UsdPrim& prim = *cit;
        if (!prim)
            continue;

        const SdfPath primPath = prim.GetPath();

        //don't process samplers while simulation runs, even in pause mode
        if (!mSceneIsPaused)
        {
            bool hasAPI = prim.HasAPI<PhysxSchemaPhysxParticleSamplingAPI>();
            if (mSamplers.count(primPath))
            {
                if (!hasAPI)
                    mSamplerPathsToRemove.insert(primPath);
                else if (mActiveSamplers.count(primPath))
                    mSamplerPathsToUpdate.insert(primPath);
            }
            else if (hasAPI)
            {
                if (prim.IsA<UsdGeomMesh>())
                {
                    mNewSamplers.insert(prim.GetPath());
                }
            }
        }

        if (prim.IsA<PhysxSchemaPhysxParticleSystem>())
        {
            handleAddParticleSystem(primPath);
        }
        else if (prim.HasAPI<PhysxSchemaPhysxParticleSetAPI>())
        {
            handleAddParticleSet(primPath);
        }
        else if (mParticleSetToSystem.count(primPath))
        {
            // particle set API removal
            handleRemoveParticleSet(primPath);
        }
    }
}


void ParticleAuthoring::handlePrimRemove(const SdfPath path)
{
    //don't process samplers while simulation runs
    if (!mSceneIsPaused && !mSceneIsPlaying) //for good measure: playing should always true if paused
    {
        {
            const auto iteratorPair = mSamplerTable.FindSubtreeRange(path);
            for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
            {
                if (mSamplers.count(it->first))
                {
                    mSamplerPathsToRemove.insert(it->first);
                }
            }
        }
    }

    {
        const auto iteratorPair = mParticleSystemsTable.FindSubtreeRange(path);
        for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
        {
            if (mParticleSystemToSet.count(it->first))
            {
                mParticleSystemPathsToRemove.insert(it->first);
            }
        }
    }

    {
        const auto iteratorPair = mParticleSetsTable.FindSubtreeRange(path);
        for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
        {
            if (mParticleSetToSystem.count(it->first))
            {
                mParticleSetPathsToRemove.insert(it->first);
            }
        }
    }
}

void ParticleAuthoring::handleAttributeChange(const SdfPath path, const TfToken attributeName, const bool isXform)
{
    if (mSceneIsPlaying && !mSceneIsPaused)
    {
        return;
    }

    //don't process samplers while simulation runs, not even in pause mode
    if (!mSceneIsPaused)
    {
        // if fluid/solid changed on the particle set
        if (attributeName == PhysxSchemaTokens->physxParticleFluid)
        {
            // check if it has some samplers attached to it
            const UsdPrim particlePrim = gStage->GetPrimAtPath(path);
            SdfPathSet samplers = mParticlesToSampler[path];
            for (SdfPathSet::const_iterator i = samplers.cbegin(); i != samplers.cend(); ++i)
            {
                SdfPath samplerPath = *i;
                mSamplerPathsToUpdate.insert(samplerPath);
            }

            //update postprocess with changed fluid property
            mPostSetsToRemove.insert(path);
            mPostSetsToAdd.insert(path);
            const SdfPath particleSystemPath = getParticleSystemPath(path);
            mPostsToUpdate.insert(particleSystemPath);
            return;
        }
        else if (attributeName == PhysxSchemaTokens->particleContactOffset ||
                 attributeName == PhysxSchemaTokens->fluidRestOffset ||
                 attributeName == PhysxSchemaTokens->solidRestOffset)
        {
            auto cit = mParticleSystemToSet.find(path);
            if (cit != mParticleSystemToSet.end())
            {
                const SdfPathSet& particleSetPaths = cit->second;
                for (SdfPath particleSetPath : particleSetPaths)
                {
                    const UsdPrim particlePrim = gStage->GetPrimAtPath(particleSetPath);
                    SdfPathSet samplers = mParticlesToSampler[particleSetPath];
                    for (SdfPathSet::const_iterator i = samplers.cbegin(); i != samplers.cend(); ++i)
                    {
                        SdfPath samplerPath = *i;

                        PhysxSchemaPhysxParticleSetAPI particleAPI(particlePrim);
                        if (particleAPI)
                        {
                            bool fluid = true;
                            particleAPI.GetFluidAttr().Get(&fluid);

                            if ((fluid && attributeName == PhysxSchemaTokens->fluidRestOffset) || 
                                (!fluid && attributeName == PhysxSchemaTokens->solidRestOffset) ||
                                (attributeName == PhysxSchemaTokens->particleContactOffset)) // always listen to particleContactOffset, since it could affect autocomputed solid/fluidRestOffset
                            {
                                //check if autocomputed sampling distance
                                PhysxSchemaPhysxParticleSamplingAPI samplingAPI(gStage->GetPrimAtPath(samplerPath));
                                if (samplingAPI)
                                {
                                    float samplingDistance;
                                    samplingAPI.GetSamplingDistanceAttr().Get(&samplingDistance);
                                    if (samplingDistance == 0.0f) // autocomputed distance value
                                    {
                                        mSamplerPathsToUpdate.insert(samplerPath);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            return;
        }

        if (isXform)
        {
            const auto iteratorPair = mSamplerTable.FindSubtreeRange(path);
            for (auto it = iteratorPair.first; it != iteratorPair.second; it++)
            {
                if (mSamplers.count(it->first))
                {
                    mSamplerPathsToTranslate.insert(it->first);
                    return;
                }
            }
        }

        if (mSamplers.count(path))
        {
            if (attributeName == PhysxSchemaTokens->physxParticleSamplingParticles)
            {
                // make sure the associatedParticles has the right value.
                auto cit = mSamplers.find(path.GetPrimPath());
                if (cit != mSamplers.end())
                {
                    PhysxSchemaPhysxParticleSamplingAPI samplingAPI(gStage->GetPrimAtPath(path));
                    SdfPathVector particlesPaths;
                    samplingAPI.GetParticlesRel().GetTargets(&particlesPaths);

                    SdfPath newPath = (particlesPaths.size() > 0) ? particlesPaths[0] : SdfPath();

                    SdfPath oldPath = cit->second;
                    cit->second = newPath;

                    SdfPathSet samplers = mParticlesToSampler[oldPath];
                    samplers.erase(cit->first);

                    if (oldPath != SdfPath())
                    {
                        mSamplerPathsToReset.insert(path);
                    }
                       
                    if (newPath != SdfPath())
                    {
                        mSamplerPathsToUpdate.insert(path);
                    }
                }
            }

            // resample for these attributes.
            if (attributeName == PhysxSchemaTokens->physxParticleSamplingSamplingDistance ||
                attributeName == PhysxSchemaTokens->physxParticleSamplingVolume ||
                attributeName == PhysxSchemaTokens->physxParticleSamplingMaxSamples)
            {
                mSamplerPathsToUpdate.insert(path);
            }
        }
    }

    UsdPrim usdPrim = gStage->GetPrimAtPath(path);
    if (mParticleSystemToSet.count(path))
    {
        if (attributeName == PhysxSchemaTokens->physxParticleAnisotropyParticleAnisotropyEnabled ||
            attributeName == PhysxSchemaTokens->physxParticleSmoothingParticleSmoothingEnabled ||
            attributeName == PhysxSchemaTokens->physxParticleIsosurfaceIsosurfaceEnabled)
        {
            handlePostChanges(path);
        }
        else if (mPosts.count(path))
        {
            if (attributeName == PhysxSchemaTokens->physxParticleAnisotropyScale ||
                attributeName == PhysxSchemaTokens->physxParticleAnisotropyMin ||
                attributeName == PhysxSchemaTokens->physxParticleAnisotropyMax ||
                attributeName == PhysxSchemaTokens->physxParticleSmoothingStrength ||
                attributeName == PhysxSchemaTokens->physxParticleIsosurfaceGridFilteringPasses ||
                attributeName == PhysxSchemaTokens->physxParticleIsosurfaceGridSmoothingRadius ||
                attributeName == PhysxSchemaTokens->physxParticleIsosurfaceSurfaceDistance ||
                attributeName == PhysxSchemaTokens->physxParticleIsosurfaceNumMeshSmoothingPasses ||
                attributeName == PhysxSchemaTokens->physxParticleIsosurfaceNumMeshNormalSmoothingPasses ||
                attributeName == PhysxSchemaTokens->physxParticleIsosurfaceMaxTriangles ||
                attributeName == PhysxSchemaTokens->physxParticleIsosurfaceMaxVertices ||
                attributeName == PhysxSchemaTokens->physxParticleIsosurfaceGridSpacing ||
                attributeName == PhysxSchemaTokens->physxParticleIsosurfaceMaxSubgrids)
            {
                mPostsToUpdate.insert(path);
            }
        }
    }
    else if (mParticleSetToSystem.count(path))
    {
        //PhysxSchemaTokens.->physxParticleFluid is covered above
        if (isXform ||
            attributeName == UsdGeomTokens->points ||
            attributeName == UsdGeomTokens->positions ||
            attributeName == UsdGeomTokens->scales ||
            attributeName == UsdGeomTokens->orientations ||
            attributeName == PhysxSchemaTokens->physxParticleFluid)
        {
            const SdfPath particleSystemPath = getParticleSystemPath(path);
            mPostsToUpdate.insert(particleSystemPath);
        }

        if (attributeName == PhysxSchemaTokens->physxParticleFluid)
        {
            //update postprocess with changed fluid property
            mPostSetsToRemove.insert(path);
            mPostSetsToAdd.insert(path);
            const SdfPath particleSystemPath = getParticleSystemPath(path);
            mPostsToUpdate.insert(particleSystemPath);
        }

        // set-owning particle system changed
        if (attributeName == PhysxSchemaTokens->physxParticleParticleSystem)
        {
            mPostSetsToUpdateParticleSystem.insert(path);
        }
    }
}

void ParticleAuthoring::update()
{
    // AD note because of OM-89182:
    // Unfortunately we cannot distinguish whether a sampler is truly new or if the prim has just been moved in the stage
    // tree. Therefore we cannot allow any new samplers to be added after simulation has started, because the "new" sampler
    // could have been a path-move and we do not have the setup to cleanup properly. We need to be careful and early-out
    // in the stage parsing as well, because we also create new samplers there. That piece of code has to be there because
    // we need to be able to block the sim start if there is still some backhground sampling activity.
    // see Loadstage.cpp::loadFromRange().
    
    if (!mIsEnabled || (mSceneIsPlaying && !mSceneIsPaused))
        return;

    // hack for iso surface mesh rendering
    // for some reason the UsdGeomMesh that is created during the first update
    // is never rendered, even when updating its data later on.
    if (mFirstUpdate)
    {
        mFirstUpdate = false;
        return;
    }

    for (SdfPathSet::const_iterator cit = mNewSamplers.cbegin(); cit != mNewSamplers.cend(); ++cit)
    {
        // insert the prim path into mSamplers and cache the reference to particle prim
        PhysxSchemaPhysxParticleSamplingAPI samplingAPI(gStage->GetPrimAtPath(*cit));

        UsdRelationship rel = samplingAPI.GetParticlesRel();
        SdfPathVector particlesPaths;
        rel.GetTargets(&particlesPaths);
        SdfPath particlePath = particlesPaths.size() > 0 ? particlesPaths[0] : SdfPath();

        if (particlePath.IsEmpty())
        {
            particlePath = getDefaultParticleSet();
            rel.SetTargets(SdfPathVector({ particlePath }));
        }

        mSamplerTable.insert({*cit, 42});
        mSamplers.insert({*cit, particlePath});
        mSamplerPathsToUpdate.insert(*cit);
        mActiveSamplers.insert(*cit);

        auto it = mParticlesToSampler.find(particlePath);
        if (it != mParticlesToSampler.end())
            it->second.insert(*cit);
        else
        {
            mParticlesToSampler.insert({particlePath, SdfPathSet()});
            mParticlesToSampler[particlePath].insert(*cit);
        }

        gPhysXParticlesPrivate->createParticleSampler(*cit, particlePath);
    }

    for (SdfPathSet::const_iterator cit = mSamplerPathsToRemove.cbegin(); cit != mSamplerPathsToRemove.cend(); ++cit)
    {
        auto it = mSamplers.find(*cit);
        if (it != mSamplers.end())
        {
            if (it->second != SdfPath())
            {
                auto it2 = mParticlesToSampler.find(it->second);
                if (it2 != mParticlesToSampler.end())
                {
                    // erase the relationship in the other direction:
                    it2->second.erase(*cit);
                }
            }

            gPhysXParticlesPrivate->removeParticleSampler(*cit, it->second);

            mActiveSamplers.erase(it->first);
            mSamplerTable.erase(it->first);
            mSamplerPathsToTranslate.erase(it->first);
            mSamplerPathsToUpdate.erase(it->first);

            // at the end because iterator will be invalidated by the erase.
            mSamplers.erase(it->first);
        }
    }

    for (SdfPathSet::const_iterator cit = mParticleSystemPathsToRemove.cbegin(); cit != mParticleSystemPathsToRemove.cend(); ++cit)
    {
        handleRemoveParticleSystem(*cit);
    }

    for (SdfPathSet::const_iterator cit = mParticleSetPathsToRemove.cbegin(); cit != mParticleSetPathsToRemove.cend(); ++cit)
    {
        handleRemoveParticleSet(*cit);
    }

    for (SdfPathSet::const_iterator cit = mSamplerPathsToReset.cbegin(); cit != mSamplerPathsToReset.cend(); ++cit)
    {
        // reset: samplers that weren't removed, but some important state changed. We remove them and resample
        // as soon as something changes.
        // Main reason this is here is particles relationship changing.

        auto it = mSamplers.find(*cit);
        if (it != mSamplers.end())
        {
            // passing an empty path here will search the internal lists for the old path.
            gPhysXParticlesPrivate->removeParticleSampler(*cit, SdfPath());

            if (it->second == SdfPath())
            {
                mSamplerPathsToTranslate.erase(*cit);
                mSamplerPathsToUpdate.erase(*cit);
            }
        }

        mActiveSamplers.erase(*cit);
    }

    // needs to happen before conversion to point instancers because the new particle system could have a postprocess with anisotropy
    for (auto it = mPostSetsToUpdateParticleSystem.begin(); it != mPostSetsToUpdateParticleSystem.end(); ++it)
    {
        handleUpdateParticleSystemForParticleSet(*it);
    }

    // handle point instancer conversions for Anisotropy visualization
    for (SdfPathSet::const_iterator cit = mSetsToConvertToPointInstancer.cbegin(); cit != mSetsToConvertToPointInstancer.cend(); ++cit)
    {
        convertToPointInstancer(*cit);
    }

    // handle translations separately to make sure we don't resample.
    for (SdfPathSet::const_iterator cit = mSamplerPathsToTranslate.cbegin(); cit != mSamplerPathsToTranslate.cend(); ++cit)
    {
        // sanity checks before doing any work
        auto it = mSamplerPathsToUpdate.find(*cit);
        if (it != mSamplerPathsToUpdate.end())
        {
            continue;
        }

        bool forceResampling = false;
        if (!checkValidSampler(*cit, forceResampling))
            continue;

        gPhysXParticlesPrivate->updateParticleSampler(*cit, mSamplers[*cit], forceResampling);
    }

    for (SdfPathSet::const_iterator cit = mSamplerPathsToUpdate.cbegin(); cit != mSamplerPathsToUpdate.cend(); ++cit)
    {
        bool forceResampling = false;
        if (!checkValidSampler(*cit, forceResampling))
            continue;

        forceResampling = true;
        gPhysXParticlesPrivate->updateParticleSampler(*cit, mSamplers[*cit], forceResampling);
    }

    for (auto it = mPostsToRelease.begin(); it != mPostsToRelease.end(); ++it)
    {
        gPhysXParticlesPrivate->releasePostprocess(*it);
    }

    for (auto it = mPostsToCreate.begin(); it != mPostsToCreate.end(); ++it)
    {
        uint32_t postFlags = getPostFlagsFromParticleSystem(*it);
        gPhysXParticlesPrivate->createPostprocess(*it, postFlags);

        // add sets
        SdfPathSet& sets = mParticleSystemToSet[*it];
        for (SdfPath set : sets)
        {
            gPhysXParticlesPrivate->addPostprocessParticleSet(*it, set);
        }
        mPostsToUpdate.insert(*it);
    }

    for (auto it = mPostSetsToRemove.begin(); it != mPostSetsToRemove.end(); ++it)
    {
        const SdfPath postPath = getParticleSystemPath(*it);
        gPhysXParticlesPrivate->removePostprocessParticleSet(postPath, *it);
    }

    for (auto it = mPostSetsToAdd.begin(); it != mPostSetsToAdd.end(); ++it)
    {
        const SdfPath postPath = getParticleSystemPath(*it);
        gPhysXParticlesPrivate->addPostprocessParticleSet(postPath, *it);
    }

    for (auto it = mPostsToUpdate.begin(); it != mPostsToUpdate.end(); ++it)
    {
        uint32_t postFlags = getPostFlagsFromParticleSystem(*it);
        gPhysXParticlesPrivate->setPostprocessStages(*it, postFlags);
        updatePost(*it);
    }

    clearBuffers();
}

bool ParticleAuthoring::checkValidSampler(const SdfPath& samplerPath, bool& forceResampling)
{
    forceResampling = false;

    if (mSamplers.count(samplerPath) && !mActiveSamplers.count(samplerPath))
    {
        SdfPath particlePath = mSamplers[samplerPath];

        gPhysXParticlesPrivate->createParticleSampler(samplerPath, particlePath);
        mActiveSamplers.insert(samplerPath);

        auto it = mParticlesToSampler.find(particlePath);
        if (it != mParticlesToSampler.end())
            it->second.insert(samplerPath);

        forceResampling = true;
    }

    // check/recreate pointbased.
    UsdPrim prim = gStage->GetPrimAtPath(samplerPath);
    PhysxSchemaPhysxParticleSamplingAPI samplingApi(prim);

    if (samplingApi)
    {
        SdfPathVector targets;
        samplingApi.GetParticlesRel().GetTargets(&targets);

        if (targets.empty() || targets[0].IsEmpty())
        {
            // if path is null/empty - means it was deleted deliberately. Don't do anything and warn the user
            CARB_LOG_WARN("ParticleSampling:particles relationship is empty, needs to point to a valid UsdGeomPoints or UsdGeomPointInstancer for sampling to work.");
            
            // we still might need to clean things up because sometimes clearing the relationship comes through the primResync path
            SdfPath oldPath = mSamplers[samplerPath];
            mSamplers[samplerPath] = SdfPath();
                        
            // remove this sampler from this list.
            SdfPathSet samplers = mParticlesToSampler[oldPath];
            samplers.erase(samplerPath);

            // need to update all the samplers that were pointing here.
            gPhysXParticlesPrivate->removeParticleSampler(samplerPath, oldPath);
            mActiveSamplers.erase(samplerPath);
            
            return false;
        }

        const UsdPrim particlesPrim = gStage->GetPrimAtPath(targets[0]);
        if (!particlesPrim)
        {
            createNewPointbased(targets[0]);
            mParticlesToSampler.insert({targets[0], {samplerPath}});

            // loop though samplers and add all the ones that have this target
            // inefficient, but we lost that info when deleting the particle prim
            for (auto it: mSamplers)
            {
                if (it.second == targets[0])
                    mSamplerPathsToUpdate.insert(it.first);
            }

            forceResampling = true;
        }
        else
        {
            const auto it2 = mParticleSetToSystem.find(targets[0]);
            if (it2 == mParticleSetToSystem.end())
            {
                CARB_LOG_WARN("ParticleSampling:particles relationship does not point to a valid particle prim, please update");
                return false;
            }
        }

        return true;
    }

    return false;
}

void ParticleAuthoring::createNewPointbased(const SdfPath particlesPath)
{
    if (gStage)
    {
        UsdGeomPoints points = UsdGeomPoints::Define(gStage, particlesPath);
        if (points)
        {
            PhysxSchemaPhysxParticleSetAPI set = PhysxSchemaPhysxParticleSetAPI::Apply(points.GetPrim());
            if (set)
            {
                updateParticleSystemOwner(particlesPath);
                UsdPhysicsMassAPI mass = UsdPhysicsMassAPI::Apply(points.GetPrim());
            }
            else
            {
                gStage->RemovePrim(particlesPath);
            }
        }
    }
}

const SdfPath ParticleAuthoring::getParticleSystemPath(const SdfPath& particlesPath)
{
    const UsdPrim particlePrim = gStage->GetPrimAtPath(particlesPath);
    PhysxSchemaPhysxParticleSetAPI set(particlePrim);
    UsdRelationship rel = PhysxSchemaPhysxParticleAPI(set).GetParticleSystemRel();
    if (rel)
    {
        SdfPathVector particleSystemPaths;
        rel.GetTargets(&particleSystemPaths);
        if (!particleSystemPaths.empty())
        {
            return particleSystemPaths[0];
        }
    }
    return SdfPath();
}

void ParticleAuthoring::updatePost(const SdfPath postPath)
{
    UsdPrim particleSystemPrim = gStage->GetPrimAtPath(postPath);
    if (!particleSystemPrim)
    {
        return;
    }

    gPhysXParticlesPrivate->updatePostprocess(postPath);
}

void ParticleAuthoring::onResume()
{
    mSceneIsPaused = false;
    for (auto it = mPosts.begin(); it != mPosts.end(); ++it)
    {
        if (!mPostsToCreate.count(*it))
            gPhysXParticlesPrivate->releasePostprocess(*it);
    }
    mPosts.clear();
}

void ParticleAuthoring::insertPostsForAllParticleSystems()
{
    for (auto it = mParticleSystemToSet.begin(); it != mParticleSystemToSet.end(); ++it)
    {
        const SdfPath particleSystemPath = it->first;
        if (mPosts.count(particleSystemPath) == 0)
        {
            uint32_t postFlags = getPostFlagsFromParticleSystem(particleSystemPath);
            if (postFlags != ParticlePostFlag::eNone)
            {
                mPosts.insert(particleSystemPath);
                mPostsToCreate.insert(particleSystemPath);
            }
        }
    }
}

void ParticleAuthoring::onPause()
{
    mSceneIsPaused = true;

    // TODO:
    // AD this can now be handled by just calling internalPbdParticleSystem->fetchParticles();
    // and updateTransforms() 

    // only give back control to preview if updateParticlesToUsd an updateToUsd are on,
    // because otherwise the current particle positions are not available because they
    // have never been transferred out of PhysX.
    const bool updateParticlesToUsd = (gSettings->getAsBool(kSettingUpdateParticlesToUsd));
    const bool updateToUsd = (gSettings->getAsBool(kSettingUpdateToUsd));
    if (updateParticlesToUsd && updateToUsd)
    {
        insertPostsForAllParticleSystems();
    }
}

void ParticleAuthoring::onStop()
{
    if (!mSceneIsPaused)
    {
        insertPostsForAllParticleSystems();
    }
    mSceneIsPaused = false;
}

void ParticleAuthoring::clearBuffers()
{
    mSamplerPathsToUpdate.clear();
    mSamplerPathsToTranslate.clear();
    mSamplerPathsToRemove.clear();
    mSamplerPathsToReset.clear();
    mNewSamplers.clear();

    mPostsToCreate.clear();
    mPostsToRelease.clear();
    mPostsToUpdate.clear();
    mPostSetsToAdd.clear();
    mPostSetsToRemove.clear();
    mPostSetsToUpdateParticleSystem.clear();

    mParticleSystemPathsToRemove.clear();
    mParticleSetPathsToRemove.clear();

    mSetsToConvertToPointInstancer.clear();
}

void ParticleAuthoring::release()
{
    for (auto it = mSamplers.begin(); it != mSamplers.end(); ++it)
    {
        gPhysXParticlesPrivate->removeParticleSampler(it->first, it->second);
    }

    mSamplerTable.clear();
    mSamplers.clear();
    mParticlesToSampler.clear();
    mActiveSamplers.clear();

    for (auto it = mPosts.begin(); it != mPosts.end(); ++it)
    {
        gPhysXParticlesPrivate->releasePostprocess(*it);
    }
    mParticleSystemToSet.clear();
    mParticleSetToSystem.clear();
    mParticleSystemsTable.clear();
    mParticleSetsTable.clear();
    mDefaultParticleSystem = SdfPath();
    mDefaultParticleSet = SdfPath();

    mPosts.clear();
    mParticleSystemsWithAnisotropy.clear();

    clearBuffers();

    mFirstUpdate = true;
}

void ParticleAuthoring::handleAddParticleSystem(const SdfPath& particleSystemPath)
{
    if (mDefaultParticleSystem.IsEmpty())
        mDefaultParticleSystem = particleSystemPath;

    auto pit = mParticleSystemToSet.insert({particleSystemPath, SdfPathSet()});
    mParticleSystemsTable.insert({particleSystemPath, 42});
    SdfPathSet& set = pit.first->second;
    for (auto sit = mParticleSetToSystem.begin(); sit != mParticleSetToSystem.end(); ++sit)
    {
        const SdfPath relPath = getParticleSystemPath(sit->first);
        // we use this opportunity to clean up stale relationships
        if (relPath != sit->second)
        {
            mPostSetsToUpdateParticleSystem.insert(sit->first);
        }
        else if (sit->second == relPath && relPath == particleSystemPath)
        {
            set.insert(sit->first);
        }
    }

    const UsdPrim prim = gStage->GetPrimAtPath(particleSystemPath);
    if (prim.HasAPI<PhysxSchemaPhysxParticleAnisotropyAPI>())
    {
        mParticleSystemsWithAnisotropy.insert(particleSystemPath);
        for (auto setit: set)
        {
            if (mParticlesToSampler.count(setit))
                mSetsToConvertToPointInstancer.insert(setit);
        }
    }
    else if (mParticleSystemsWithAnisotropy.count(particleSystemPath))
    {
        mParticleSystemsWithAnisotropy.erase(particleSystemPath);
    }

    handlePostChanges(particleSystemPath);
}

void ParticleAuthoring::handleRemoveParticleSystem(const SdfPath& particleSystemPath)
{
    auto pit = mParticleSystemToSet.find(particleSystemPath);
    if (pit != mParticleSystemToSet.end())
    {
        mParticleSystemsTable.erase(particleSystemPath);
        mPosts.erase(particleSystemPath);
        mPostsToRelease.insert(particleSystemPath);
        mParticleSystemsWithAnisotropy.erase(particleSystemPath);

        // invalidates iterator.
        mParticleSystemToSet.erase(pit);
    }

    if (mDefaultParticleSystem == particleSystemPath)
    {
        mDefaultParticleSystem = SdfPath();
        if (mParticleSystemToSet.size() > 0)
        {
            mDefaultParticleSystem = mParticleSystemToSet.begin()->first;
        }
    }
}

void ParticleAuthoring::handleAddParticleSet(const SdfPath& particleSetPath)
{
    if (mDefaultParticleSet.IsEmpty())
    {
        mDefaultParticleSet = particleSetPath;
    }

    const SdfPath particleSystemPath = getParticleSystemPath(particleSetPath);

    auto setIt = mParticleSetToSystem.find(particleSetPath);
    if (setIt == mParticleSetToSystem.end())
    {
        mParticleSetToSystem.insert({particleSetPath, particleSystemPath});
        mParticleSetsTable.insert({particleSetPath, 42});
    }
    else
    {
        // updating system path - need to make sure we trigger the update
        const SdfPath oldPath = setIt->second;
        if (oldPath != particleSystemPath)
        {
            mPostSetsToUpdateParticleSystem.insert(particleSetPath);            
            return;
        }
    }
    auto pit = mParticleSystemToSet.find(particleSystemPath);
    if (pit != mParticleSystemToSet.end())
    {
        SdfPathSet& set = pit->second;
        set.insert(particleSetPath);

        if (mParticleSystemsWithAnisotropy.count(particleSystemPath))
        {
            if (mParticlesToSampler.count(particleSetPath))
                mSetsToConvertToPointInstancer.insert(particleSetPath);
        }

        if (mPosts.count(particleSystemPath))
        {
            mPostSetsToAdd.insert(particleSetPath);
            mPostsToUpdate.insert(particleSystemPath);
        }
    }
}

void ParticleAuthoring::handleRemoveParticleSet(const SdfPath& particleSetPath)
{
    SdfPathSet& samplers = mParticlesToSampler[particleSetPath];
    for (SdfPath sampler : samplers)
    {
        // this should clean things up internally as well.
        mSamplerPathsToReset.insert(sampler);

        auto sit = mSamplers.find(sampler);
        if (sit != mSamplers.end())
        {
            // don't delete cached relationship to the particles prim, 
            // we might want to recreate it!
            UsdGeomImageable img = UsdGeomImageable::Get(gStage, sit->first);
            if (img)
            {
                img.CreateVisibilityAttr().Set(UsdGeomTokens->inherited);
            }
        }
    }

    // always release it
    mParticlesToSampler.erase(particleSetPath);

    // postProcessing part
    const SdfPath particleSystemPath = mParticleSetToSystem[particleSetPath];
    mParticleSetToSystem.erase(particleSetPath);
    mParticleSetsTable.erase(particleSetPath);
    auto pit = mParticleSystemToSet.find(particleSystemPath);
    if (pit != mParticleSystemToSet.end())
    {
        SdfPathSet& set = pit->second;
        set.erase(particleSetPath);

        if (mPosts.count(particleSystemPath))
        {
            mPostSetsToRemove.insert(particleSetPath);
            mPostsToUpdate.insert(particleSystemPath);
        }
    }

    if (mDefaultParticleSet == particleSetPath)
    {
        mDefaultParticleSet = SdfPath();
        if (mParticleSetToSystem.size() > 0)
        {
            mDefaultParticleSet = mParticleSetToSystem.begin()->first;
        }
    }
}

// AD: added to fix OM-47101
// This all feels a bit redundant and very similar to add/remove particle set handling
// But we need this function because add/remove particle set brute-force removes the cache entries
// but in case of a change of the simulation owner particle system, we need the old relationship
// to clean things up properly.
//
// consider unifying this - would probably also allow us to get rid of all the remove/add pairs.
//
// OM-47122
//
void ParticleAuthoring::handleUpdateParticleSystemForParticleSet(const SdfPath& particleSetPath)
{
    const SdfPath newParticleSystemPath = getParticleSystemPath(particleSetPath);
    auto itOld = mParticleSetToSystem.find(particleSetPath);
    if (itOld != mParticleSetToSystem.end())
    {
        if (itOld->second != newParticleSystemPath) // particle system has changed on this set
        {
            // remove old path and update postprocessing
            if (itOld->second != SdfPath())
            {
                auto pit = mParticleSystemToSet.find(itOld->second); // erase from particlesystem to set table
                if (pit != mParticleSystemToSet.end())
                {
                    SdfPathSet& set = pit->second;
                    set.erase(particleSetPath);
                }

                if (mPosts.count(itOld->second)) // remove from post if there are posts on this particle system
                {
                    gPhysXParticlesPrivate->removePostprocessParticleSet(itOld->second, particleSetPath);
                    mPostsToUpdate.insert(itOld->second); // make sure the post updates.
                }
            }

            // add new path
            mParticleSetToSystem.erase(particleSetPath);
            mParticleSetToSystem.insert({particleSetPath, newParticleSystemPath});

            auto pit = mParticleSystemToSet.find(newParticleSystemPath);
            if (pit != mParticleSystemToSet.end())
            {
                pit->second.insert(particleSetPath);

                if (mParticleSystemsWithAnisotropy.count(newParticleSystemPath))
                {
                    if (mParticlesToSampler.count(particleSetPath))
                        mSetsToConvertToPointInstancer.insert(particleSetPath);
                }

                if (mPosts.count(newParticleSystemPath))
                {
                    gPhysXParticlesPrivate->addPostprocessParticleSet(newParticleSystemPath, particleSetPath);
                    mPostsToUpdate.insert(newParticleSystemPath);
                }     
            }     
        }
    }
}

void ParticleAuthoring::handlePostChanges(const SdfPath& postPath)
{
    uint32_t newPostFlags = getPostFlagsFromParticleSystem(postPath);

    if (newPostFlags == ParticlePostFlag::eNone)
    {
        // no flags are set, release post if exists
        if (mPosts.count(postPath))
        {
            mPostsToRelease.insert(postPath);
        }
        mPosts.erase(postPath);
        mPostsToCreate.erase(postPath);
        mPostsToUpdate.erase(postPath);
    }
    else if (mPosts.count(postPath))
    {
        // post already exists
        uint32_t postFlags = gPhysXParticlesPrivate->getPostprocessStages(postPath);
        if (postFlags != newPostFlags)
        {
            mPostsToUpdate.insert(postPath);
        }
    }
    else
    {
        // create a new post
        mPosts.insert(postPath);
        mPostsToCreate.insert(postPath);
    }
}

void ParticleAuthoring::convertToPointInstancer(const SdfPath& path)
{
    UsdPrim prim = gStage->GetPrimAtPath(path);
    
    if (prim.IsA<UsdGeomPointInstancer>())
        return;

    UsdGeomPointInstancer pointInstancer = UsdGeomPointInstancer::Define(gStage, path);
    
    VtArray<float> widths;
    VtArray<GfVec3f> usdColors;
    VtArray<GfVec3f> positions;
    UsdGeomPoints points(prim);
    points.GetWidthsAttr().Get(&widths);
    points.GetDisplayColorAttr().Get(&usdColors);
    points.GetPointsAttr().Get(&positions);
    const size_t size = widths.size();

    // create the prototype
    UsdRelationship prototypes = pointInstancer.CreatePrototypesRel();
    const SdfPath protoPath = path.AppendElementString("pointPrototype");
    const UsdGeomSphere sphere = UsdGeomSphere::Define(gStage, protoPath);
    if (size > 0)
        sphere.CreateRadiusAttr().Set(0.5 * double(widths[0]));
    if (usdColors.size() > 0)
        sphere.CreateDisplayColorPrimvar().Set(usdColors);
    prototypes.AddTarget(protoPath);

    VtArray<int> indices(size, 0);

    pointInstancer.CreatePositionsAttr().Set(positions);
    pointInstancer.CreateProtoIndicesAttr().Set(indices);

    prim.RemoveProperty(points.GetWidthsAttr().GetName());
    prim.RemoveProperty(points.GetPointsAttr().GetName());
}

SdfPath ParticleAuthoring::getDefaultParticleSystem()
{
    if (mDefaultParticleSystem.IsEmpty() && gStage)
    {
        SdfPath defaultPath = getDefaultParticleSystemPath();
        defaultPath = getStageNextFreePath(defaultPath);
        PhysxSchemaPhysxParticleSystem ps = PhysxSchemaPhysxParticleSystem::Define(gStage, defaultPath);
        if (ps)
        {
            mDefaultParticleSystem = defaultPath;
        }
    }
    return mDefaultParticleSystem;
}

SdfPath ParticleAuthoring::getDefaultParticleSet()
{
    if (mDefaultParticleSet.IsEmpty() && gStage)
    {
        SdfPath defaultPath = getDefaultParticleSetPath();
        defaultPath = getStageNextFreePath(defaultPath);
        createNewPointbased(defaultPath);
        if (gStage->GetPrimAtPath(defaultPath))
        {
            mDefaultParticleSet = defaultPath;
            for (auto it: mSamplers)
            {
                if (it.second == defaultPath)
                    mSamplerPathsToUpdate.insert(it.first);
            }
        }
    }
    return mDefaultParticleSet;
}

void ParticleAuthoring::updateParticleSystemOwner(const SdfPath& particlesPath)
{
    if (gStage)
    {
        PhysxSchemaPhysxParticleAPI particleAPI = PhysxSchemaPhysxParticleAPI::Get(gStage, particlesPath);
        if (particleAPI)
        {
            UsdRelationship rel = particleAPI.GetParticleSystemRel();
            SdfPathVector targets;
            rel.GetTargets(&targets);
            if (targets.empty() || targets[0].IsEmpty())
            {
                SdfPath defaultParticleSystem = getDefaultParticleSystem();
                rel.SetTargets(SdfPathVector({ defaultParticleSystem }));
            }
        }
    }
}
