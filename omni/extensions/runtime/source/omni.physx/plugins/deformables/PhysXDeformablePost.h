// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include "PxPhysicsAPI.h"
#include "PxDeformableSkinning.h"
#include "PxDeformableVolume.h"

#include <internal/Internal.h>

namespace omni
{
namespace physx
{

namespace deformables
{
    struct VolumeDeformableSkinningData
    {
        void packageSkinningData(::physx::PxTetmeshSkinningGpuData& packagedData);

        ::physx::PxDeformableVolume* mDeformableVolume;
        ::physx::PxU32* mGuideTetrahedraD;
        ::physx::PxTetrahedronMeshEmbeddingInfo* mSkinningEmbeddingInfoD;
        ::physx::PxVec3* mAllSkinnedVerticesD;
        ::physx::PxU32 mNumSkinnedVertices;
    };

    struct SurfaceDeformableSkinningData
    {
        void packageSkinningData(::physx::PxTrimeshSkinningGpuData& packagedData);

        ::physx::PxDeformableSurface* mDeformableSurface;
        ::physx::PxU32* mGuideTrianglesD;
        ::physx::PxU32 mNumGuideTriangles;
        ::physx::PxVec3* mGuideNormalsD;
        ::physx::PxU32 mNumGuideVertices;
        ::physx::PxTriangleMeshEmbeddingInfo* mSkinningEmbeddingInfoD;
        ::physx::PxVec3* mSkinnedVerticesD;
        ::physx::PxU32 mNumSkinnedVertices;
        ::physx::PxReal mHalfThickness;
    };

    // Base class for post-solve callback functionality of deformables.
    class DeformablePostSolveCallback : public ::physx::PxPostSolveCallback, public Allocateable
    {
    public:
        DeformablePostSolveCallback(CUstream stream, ::physx::PxCudaContextManager* cudaContextManager, ::physx::PxScene* scene);
        virtual ~DeformablePostSolveCallback() {};

        virtual void copySkinnedVerticesDtoHAsync(size_t deformableIndex, ::physx::PxVec3* mAllSkinnedVerticesH) = 0;
        virtual void onPostSolve(CUevent startEvent) = 0;

        void synchronize();

    protected:
        CUstream mSkinningStream;
        ::physx::PxCudaContextManager* mCudaContextManager;
        ::physx::PxDeformableSkinning* mSkinning;
        ::physx::PxScene* mScene;
        size_t mCurrentSize;        // Current size of the packaged skinning data buffers
        bool mNeedResize;           // For lazy allocation    
        bool isRegistered;          // Flag for registering the callback with the scene
    };

    class VolumeDeformablePostSolveCallback : public DeformablePostSolveCallback
    {
    public:
        VolumeDeformablePostSolveCallback(CUstream stream, ::physx::PxCudaContextManager* cudaContextManager, ::physx::PxScene* scene);
        ~VolumeDeformablePostSolveCallback();

        virtual void copySkinnedVerticesDtoHAsync(size_t deformableIndex, ::physx::PxVec3* mAllSkinnedVerticesH);
        virtual void onPostSolve(CUevent startEvent);

        void addVolumeDeformableSkinningData(const VolumeDeformableSkinningData& skinningData);
        void removeVolumeDeformableSkinningData(const size_t index);

    private:
        void resizeMemory(const size_t newSize);
        void releaseMemory();

        ::physx::PxTetmeshSkinningGpuData* mPackagedSkinningDataH;  // Pinned host memory for packaged skinning data
        ::physx::PxTetmeshSkinningGpuData* mPackagedSkinningDataD;  // Device memory for packaged skinning data

        std::vector<VolumeDeformableSkinningData> mSkinningData;
    };

    class SurfaceDeformablePostSolveCallback : public DeformablePostSolveCallback
    {
    public:
        SurfaceDeformablePostSolveCallback(CUstream stream, ::physx::PxCudaContextManager* cudaContextManager, ::physx::PxScene* scene);
        ~SurfaceDeformablePostSolveCallback();

        virtual void copySkinnedVerticesDtoHAsync(size_t deformableIndex, ::physx::PxVec3* mAllSkinnedVerticesH);
        virtual void onPostSolve(CUevent startEvent);

        void addSurfaceDeformableSkinningData(const SurfaceDeformableSkinningData& skinningData);
        void removeSurfaceDeformableSkinningData(const size_t index);

    private:
        void resizeMemory(const size_t newSize);
        void releaseMemory();

        ::physx::PxTrimeshSkinningGpuData* mPackagedSkinningDataH;  // Pinned host memory for packaged skinning data
        ::physx::PxTrimeshSkinningGpuData* mPackagedSkinningDataD;  // Device memory for packaged skinning data

        std::vector<SurfaceDeformableSkinningData> mSkinningData;
    };
} // namespace deformables
} // namespace physx
} // namespace omni
