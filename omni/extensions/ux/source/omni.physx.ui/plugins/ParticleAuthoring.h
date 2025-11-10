// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

namespace omni
{

namespace physx
{

namespace ui
{

class ParticleAuthoring
{
public:
    explicit ParticleAuthoring(bool enabled);
    ~ParticleAuthoring();

    void handlePrimResync(const pxr::SdfPath path);
    void handlePrimRemove(const pxr::SdfPath path);
    void handleAttributeChange(const pxr::SdfPath path, pxr::TfToken attributeName, bool isXform);

    void parseStage();
    void update();
    void release();

    void setEnabled(bool enable)
    {
        mIsEnabled = enable;
    }

    bool isActive()
    {
        return mIsEnabled;
    };

    void setIsPlaying(bool isPlaying)
    {
        mSceneIsPlaying = isPlaying;
    }

    bool isEmpty() const
    {
        return mSamplers.empty() && mParticleSystemToSet.empty();
    }

    void onResume();
    void onPause();
    void onStop();

private:
    bool checkValidSampler(const pxr::SdfPath& samplerPath, bool& forceResampling);
    void createNewPointbased(pxr::SdfPath particlesPath);
    const pxr::SdfPath getParticleSystemPath(const pxr::SdfPath& particlesPath);
    void clearBuffers();
    void updatePost(const pxr::SdfPath postPath);

    void handleAddParticleSystem(const pxr::SdfPath& particleSystemPath);
    void handleRemoveParticleSystem(const pxr::SdfPath& particleSystemPath);
    void handleAddParticleSet(const pxr::SdfPath& particleSetPath);
    void handleRemoveParticleSet(const pxr::SdfPath& particleSetPath);
    void handleUpdateParticleSystemForParticleSet(const pxr::SdfPath& particleSetPath);
    void handlePostChanges(const pxr::SdfPath& postPath);

    void convertToPointInstancer(const pxr::SdfPath& path);
    pxr::SdfPath getDefaultParticleSystem();
    pxr::SdfPath getDefaultParticleSet();
    void updateParticleSystemOwner(const pxr::SdfPath& particles);
    void insertPostsForAllParticleSystems();

    using SdfPathToSdfPathMap = pxr::TfHashMap<pxr::SdfPath, pxr::SdfPath, pxr::SdfPath::Hash>;
    using SdfPathToSdfPathSetMap = pxr::TfHashMap<pxr::SdfPath, pxr::SdfPathSet, pxr::SdfPath::Hash>;

    pxr::SdfPathTable<uint32_t> mSamplerTable;
    pxr::SdfPathTable<uint32_t> mParticleSystemsTable;
    pxr::SdfPathTable<uint32_t> mParticleSetsTable;

    SdfPathToSdfPathMap mSamplers;
    SdfPathToSdfPathSetMap mParticlesToSampler;
    pxr::SdfPathSet mSamplerPathsToUpdate;
    pxr::SdfPathSet mSamplerPathsToTranslate;
    pxr::SdfPathSet mSamplerPathsToRemove;
    pxr::SdfPathSet mNewSamplers;
    pxr::SdfPathSet mActiveSamplers;
    pxr::SdfPathSet mSamplerPathsToReset;

    SdfPathToSdfPathSetMap mParticleSystemToSet;
    SdfPathToSdfPathMap mParticleSetToSystem;

    pxr::SdfPathSet mPosts;

    pxr::SdfPathSet mPostsToCreate;
    pxr::SdfPathSet mPostsToRelease;
    pxr::SdfPathSet mPostsToUpdate;
    pxr::SdfPathSet mPostSetsToAdd;
    pxr::SdfPathSet mPostSetsToRemove;
    pxr::SdfPathSet mPostSetsToUpdateParticleSystem;

    pxr::SdfPathSet mParticleSystemPathsToRemove;
    pxr::SdfPathSet mParticleSetPathsToRemove;

    pxr::SdfPathSet mSetsToConvertToPointInstancer;
    pxr::SdfPathSet mParticleSystemsWithAnisotropy;

    pxr::SdfPath mDefaultParticleSystem;
    pxr::SdfPath mDefaultParticleSet;

    bool mFirstUpdate;
    bool mSceneIsPlaying;
    bool mSceneIsPaused;
    bool mIsEnabled;
};

} // namespace ui
} // namespace physx
} // namespace omni
