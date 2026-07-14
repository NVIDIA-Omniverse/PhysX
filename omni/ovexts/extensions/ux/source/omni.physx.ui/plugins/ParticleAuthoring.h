// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

    void handlePrimResync(const PXR_NS::SdfPath path);
    void handlePrimRemove(const PXR_NS::SdfPath path);
    void handleAttributeChange(const PXR_NS::SdfPath path, PXR_NS::TfToken attributeName, bool isXform);

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
    bool checkValidSampler(const PXR_NS::SdfPath& samplerPath, bool& forceResampling);
    void createNewPointbased(PXR_NS::SdfPath particlesPath);
    const PXR_NS::SdfPath getParticleSystemPath(const PXR_NS::SdfPath& particlesPath);
    void clearBuffers();
    void updatePost(const PXR_NS::SdfPath postPath);

    void handleAddParticleSystem(const PXR_NS::SdfPath& particleSystemPath);
    void handleRemoveParticleSystem(const PXR_NS::SdfPath& particleSystemPath);
    void handleAddParticleSet(const PXR_NS::SdfPath& particleSetPath);
    void handleRemoveParticleSet(const PXR_NS::SdfPath& particleSetPath);
    void handleUpdateParticleSystemForParticleSet(const PXR_NS::SdfPath& particleSetPath);
    void handlePostChanges(const PXR_NS::SdfPath& postPath);

    void convertToPointInstancer(const PXR_NS::SdfPath& path);
    PXR_NS::SdfPath getDefaultParticleSystem();
    PXR_NS::SdfPath getDefaultParticleSet();
    void updateParticleSystemOwner(const PXR_NS::SdfPath& particles);
    void insertPostsForAllParticleSystems();

    using SdfPathToSdfPathMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, PXR_NS::SdfPath, PXR_NS::SdfPath::Hash>;
    using SdfPathToSdfPathSetMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, PXR_NS::SdfPathSet, PXR_NS::SdfPath::Hash>;

    PXR_NS::SdfPathTable<uint32_t> mSamplerTable;
    PXR_NS::SdfPathTable<uint32_t> mParticleSystemsTable;
    PXR_NS::SdfPathTable<uint32_t> mParticleSetsTable;

    SdfPathToSdfPathMap mSamplers;
    SdfPathToSdfPathSetMap mParticlesToSampler;
    PXR_NS::SdfPathSet mSamplerPathsToUpdate;
    PXR_NS::SdfPathSet mSamplerPathsToTranslate;
    PXR_NS::SdfPathSet mSamplerPathsToRemove;
    PXR_NS::SdfPathSet mNewSamplers;
    PXR_NS::SdfPathSet mActiveSamplers;
    PXR_NS::SdfPathSet mSamplerPathsToReset;

    SdfPathToSdfPathSetMap mParticleSystemToSet;
    SdfPathToSdfPathMap mParticleSetToSystem;

    PXR_NS::SdfPathSet mPosts;

    PXR_NS::SdfPathSet mPostsToCreate;
    PXR_NS::SdfPathSet mPostsToRelease;
    PXR_NS::SdfPathSet mPostsToUpdate;
    PXR_NS::SdfPathSet mPostSetsToAdd;
    PXR_NS::SdfPathSet mPostSetsToRemove;
    PXR_NS::SdfPathSet mPostSetsToUpdateParticleSystem;

    PXR_NS::SdfPathSet mParticleSystemPathsToRemove;
    PXR_NS::SdfPathSet mParticleSetPathsToRemove;

    PXR_NS::SdfPathSet mSetsToConvertToPointInstancer;
    PXR_NS::SdfPathSet mParticleSystemsWithAnisotropy;

    PXR_NS::SdfPath mDefaultParticleSystem;
    PXR_NS::SdfPath mDefaultParticleSet;

    bool mFirstUpdate;
    bool mSceneIsPlaying;
    bool mSceneIsPaused;
    bool mIsEnabled;
};

} // namespace ui
} // namespace physx
} // namespace omni
