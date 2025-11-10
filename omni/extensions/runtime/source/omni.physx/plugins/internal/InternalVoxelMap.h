// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once
#include "UsdPCH.h"
#include <private/omni/physx/PhysxUsd.h>

#include <PxPhysicsAPI.h>

// @@@ VR: Remove this after updating these macros in PhysX SDK
#define DECLARE_CUSTOM_GEOMETRY_TYPE2                                                                                  \
    static ::physx::PxCustomGeometry::Type TYPE();                                                                     \
    virtual ::physx::PxCustomGeometry::Type getCustomType() const;
#define IMPLEMENT_CUSTOM_GEOMETRY_TYPE2(CLASS)                                                                         \
    ::physx::PxCustomGeometry::Type CLASS::TYPE()                                                                      \
    {                                                                                                                  \
        static ::physx::PxCustomGeometry::Type customType;                                                             \
        return customType;                                                                                             \
    }                                                                                                                  \
    ::physx::PxCustomGeometry::Type CLASS::getCustomType() const                                                       \
    {                                                                                                                  \
        return TYPE();                                                                                                 \
    }

namespace omni
{
namespace physx
{
namespace internal
{
enum VoxelCollision
{
    eNoCollision,
    eBoxCollision,
    eConvexMeshCollision,
    eTriangleMeshCollision
};

class InfiniteVoxelMap
{
public:
    struct VoxelType
    {
        int type, subType, geomIndex;
        std::string name, subName;
    };

    InfiniteVoxelMap(::physx::PxScene* scene, ::pxr::UsdStageRefPtr stage, const usdparser::InfiniteVoxelMapDesc& desc);
    ~InfiniteVoxelMap();

    void addVoxelType(const carb::Int2& type,
                      const std::string& name,
                      const std::string& subName,
                      VoxelCollision collision,
                      const std::vector<::physx::PxVec3>& vertices = std::vector<::physx::PxVec3>(),
                      const std::vector<::physx::PxU32>& indices = std::vector<::physx::PxU32>());
    const VoxelType& getVoxelType(const carb::Int2& type) const;

    bool setVoxelRegion(const carb::Int3& regionMin,
                        const carb::Int3& regionMax,
                        const carb::Int2& type,
                        const std::function<void(const carb::Int3&)>& callback = nullptr);
    bool setVoxel(const carb::Int3& coords, const carb::Int2& type);
    carb::Int2 getVoxel(const carb::Int3& coords) const;

    void setVoxels_USD(const carb::Int3& regionMin, const carb::Int3& regionMax, const carb::Int2& type, bool update = true);

    void setUpdateVoxel(int sx, int sy, int sz, int ex, int ey, int ez, int type, int subType, int update);


private:
    struct VoxelChunk : ::physx::PxCustomGeometry::Callbacks
    {
        InfiniteVoxelMap& voxelMap;

        VoxelChunk(InfiniteVoxelMap& voxelMap, const carb::Int3& coords);
        ~VoxelChunk();

        DECLARE_CUSTOM_GEOMETRY_TYPE2
        virtual ::physx::PxBounds3 getLocalBounds(const ::physx::PxGeometry& /*geometry*/) const override;
        virtual bool generateContacts(const ::physx::PxGeometry& geom0,
                                      const ::physx::PxGeometry& geom1,
                                      const ::physx::PxTransform& pose0,
                                      const ::physx::PxTransform& pose1,
                                      const ::physx::PxReal contactDistance,
                                      const ::physx::PxReal meshContactMargin,
                                      const ::physx::PxReal toleranceLength,
                                      ::physx::PxContactBuffer& contactBuffer) const override;
        virtual ::physx::PxU32 raycast(const ::physx::PxVec3& origin,
                                       const ::physx::PxVec3& unitDir,
                                       const ::physx::PxGeometry& geom,
                                       const ::physx::PxTransform& pose,
                                       ::physx::PxReal maxDist,
                                       ::physx::PxHitFlags hitFlags,
                                       ::physx::PxU32 maxHits,
                                       ::physx::PxGeomRaycastHit* rayHits,
                                       ::physx::PxU32 stride,
                                       ::physx::PxRaycastThreadContext* /*threadContext*/) const override;
        virtual bool overlap(const ::physx::PxGeometry& geom0,
                             const ::physx::PxTransform& pose0,
                             const ::physx::PxGeometry& geom1,
                             const ::physx::PxTransform& pose1,
                             ::physx::PxRaycastThreadContext* /*threadContext*/) const override;
        virtual bool sweep(const ::physx::PxVec3& unitDir,
                           const ::physx::PxReal maxDist,
                           const ::physx::PxGeometry& geom0,
                           const ::physx::PxTransform& pose0,
                           const ::physx::PxGeometry& geom1,
                           const ::physx::PxTransform& pose1,
                           ::physx::PxGeomSweepHit& sweepHit,
                           ::physx::PxHitFlags hitFlags,
                           const ::physx::PxReal inflation,
                           ::physx::PxRaycastThreadContext* /*threadContext*/) const override;
        virtual void visualize(const ::physx::PxGeometry& geometry,
                               ::physx::PxRenderOutput& out,
                               const ::physx::PxTransform& absPose,
                               const ::physx::PxBounds3& cullbox) const;
        virtual void computeMassProperties(const ::physx::PxGeometry& geometry,
                                           ::physx::PxMassProperties& massProperties) const
        {
        }
        virtual bool usePersistentContactManifold(const ::physx::PxGeometry&, ::physx::PxReal&) const
        {
            return true;
        }

        bool empty() const
        {
            return mCounter == 0;
        }
        float voxelSizeX() const
        {
            return 1.0f;
        }
        float voxelSizeY() const
        {
            return 1.0f;
        }
        float voxelSizeZ() const
        {
            return 1.0f;
        }
        ::physx::PxVec3 voxelSize() const
        {
            return ::physx::PxVec3(voxelSizeX(), voxelSizeY(), voxelSizeZ());
        }
        ::physx::PxU32 dimX() const
        {
            return voxelMap.mChunkSize.x;
        }
        ::physx::PxU32 dimY() const
        {
            return voxelMap.mChunkSize.y;
        }
        ::physx::PxU32 dimZ() const
        {
            return voxelMap.mChunkSize.z;
        }
        ::physx::PxVec3 extents() const
        {
            return ::physx::PxVec3(dimX() * voxelSizeX(), dimY() * voxelSizeY(), dimZ() * voxelSizeZ());
        }
        ::physx::PxVec3 voxelPos(int x, int y, int z) const;
        void pointCoords(const ::physx::PxVec3& p, int& x, int& y, int& z) const;
        void getVoxelRegion(const ::physx::PxBounds3& b, int& sx, int& sy, int& sz, int& ex, int& ey, int& ez) const;
        bool voxelHasCollision(int x, int y, int z) const;
        bool voxelIsCube(int x, int y, int z) const;
        const ::physx::PxGeometry& voxelGeometry(int x, int y, int z) const;
        bool setVoxel(const carb::Int3& coords, const carb::Int2& type);
        carb::Int2 voxel(const carb::Int3& coords) const;

    private:
        std::vector<::physx::PxU32> mVoxels;
        ::physx::PxRigidStatic* mRigidStatic;
        int mCounter;
    };

    int addMeshGeometry(const ::physx::PxVec3* verts,
                        ::physx::PxU32 vertCount,
                        const ::physx::PxU32* tris,
                        ::physx::PxU32 triCount,
                        float scale);
    int addMeshGeometry(const ::physx::PxVec3* verts, ::physx::PxU32 vertCount, float scale);

    struct Int3_less
    {
        constexpr auto operator()(const carb::Int3& a, const carb::Int3& b) const
        {
            // return memcmp(&a, &b, sizeof(carb::Int3)) < 0;
            return a.z < b.z || (a.z == b.z && a.y < b.y) || (a.z == b.z && a.y == b.y && a.x < b.x);
        }
    };

    void collectChildMeshes(const pxr::UsdPrim& prim,
                            std::vector<::physx::PxVec3>& vertices,
                            std::vector<::physx::PxU32>& indices);
    void queryBlockRegion(const carb::Int3& s,
                          const carb::Int3& e,
                          pxr::VtArray<int>& indices,
                          pxr::VtArray<pxr::GfVec3f>& positions,
                          pxr::SdfPathVector& prototypes) const;

    carb::Int3 mChunkSize;
    std::map<carb::Int3, VoxelChunk*, Int3_less> mChunks;
    std::vector<std::vector<VoxelType*>> mLib;
    std::vector<::physx::PxGeometryHolder> mGeoms;
    std::vector<::physx::PxRefCounted*> mMeshes;
    ::physx::PxMaterial* mDefaultMaterial;
    ::physx::PxScene* mScene;
    pxr::UsdStageRefPtr mStage;
    const pxr::SdfPath mRoot;
    pxr::GfVec3i mUsdChunkSize;
    std::set<carb::Int3, Int3_less> mDirtyUsdChunks;
};

} // namespace internal
} // namespace physx
} // namespace omni
