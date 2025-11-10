// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"
#include "InternalVoxelMap.h"
#include "PxImmediateMode.h"
#include <Setup.h>
#include <OmniPhysX.h>
#include "geomutils/PxContactBuffer.h"

#include <common/PxRenderOutput.h>

#if CARB_PLATFORM_LINUX
#define sscanf_s sscanf
#define sprintf_s sprintf
#endif

using namespace omni::physx::internal;
using namespace ::physx;
using namespace carb;

// Blocks flags from Mineways blockInfo.h
// fills whole block
#define BLF_WHOLE			0x0001
// almost a whole block
#define BLF_ALMOST_WHOLE    0x0002
// stairs
#define BLF_STAIRS			0x0004
// half block, i.e., a slab
#define BLF_HALF			0x0008

IMPLEMENT_CUSTOM_GEOMETRY_TYPE2(omni::physx::internal::InfiniteVoxelMap::VoxelChunk)

InfiniteVoxelMap::InfiniteVoxelMap(::physx::PxScene* scene, ::pxr::UsdStageRefPtr stage, const omni::physx::usdparser::InfiniteVoxelMapDesc& desc)
    :
    mScene(scene), mStage(stage), mRoot(desc.rootPrim)
{
    // Chunk size
    mChunkSize = Int3 { 32, 32, 32 };

    // Add empty geom
    mGeoms.push_back(::physx::PxGeometryHolder());
    *reinterpret_cast<PxGeometryType::Enum*>(&mGeoms[0]) = PxGeometryType::eINVALID; // Need 'empty' geometry?

    // Add box geom
    mGeoms.push_back(::physx::PxBoxGeometry(PxVec3(0.5f)));

    // Add air voxel type
    addVoxelType(Int2{ 0, 0 }, "Air", "Air", eNoCollision);

    auto physics = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getPhysics();
    mDefaultMaterial = physics->createMaterial(0.6f, 0.6f, 0);

    //

    auto root = mStage->GetPrimAtPath(mRoot);
    if (root.IsValid())
    {
        root.GetAttribute(pxr::TfToken("chunkSize")).Get(&mUsdChunkSize);

        auto blocks = root.GetPrimAtPath(pxr::SdfPath("BlockLib/Blocks"));
        if (blocks.IsValid())
        {
            std::vector<PxVec3> vertices;
            std::vector<PxU32> indices;

            for (auto child : blocks.GetChildren())
            {
                int blockType = 0;
                child.GetAttribute(pxr::TfToken("blockType")).Get(&blockType);

                if (blockType == 0) continue; // ??? The first child looks like some garbage. ???

                int blockSubType;
                child.GetAttribute(pxr::TfToken("blockSubType")).Get(&blockSubType);
                std::string typeName;
                child.GetAttribute(pxr::TfToken("typeName")).Get(&typeName);
                std::string subTypeName;
                child.GetAttribute(pxr::TfToken("subTypeName")).Get(&subTypeName);
                int blockFlags;
                child.GetAttribute(pxr::TfToken("blockFlags")).Get(&blockFlags);

                auto collision = eNoCollision;
                vertices.clear();
                indices.clear();

                if (blockFlags & BLF_WHOLE)
                    collision = eBoxCollision;
                else if (blockFlags & BLF_ALMOST_WHOLE)
                {
                    collision = eConvexMeshCollision;
                    collectChildMeshes(child, vertices, indices);
                }
                else if (blockFlags & BLF_HALF)
                {
                    collision = eConvexMeshCollision;
                    collectChildMeshes(child, vertices, indices);
                }
                else if (blockFlags & BLF_STAIRS)
                {
                    collision = eTriangleMeshCollision;
                    collectChildMeshes(child, vertices, indices);
                }

                addVoxelType(Int2{ blockType, blockSubType }, typeName, subTypeName, collision, vertices, indices);
            }
        }

        for (auto child : root.GetChildren())
        {
            if (!child.IsA<pxr::UsdGeomPointInstancer>())
                continue;

            const pxr::UsdGeomPointInstancer& instancer = (const pxr::UsdGeomPointInstancer&)child;

            pxr::VtArray<int> indices;
            instancer.GetProtoIndicesAttr().Get(&indices);
            pxr::VtArray<pxr::GfVec3f> positions;
            instancer.GetPositionsAttr().Get(&positions);
            pxr::SdfPathVector prototypes;
            instancer.GetPrototypesRel().GetTargets(&prototypes);
            std::vector<carb::Int2> types;
            for (const auto& proto : prototypes)
            {
                int type, subType;
                sscanf_s(proto.GetName().c_str(), "Block_%d_%d", &type, &subType);
                types.push_back(carb::Int2{ type, subType });
            }

            for (int i = 0; i < (int)indices.size(); ++i)
            {
                auto pos = positions[i];
                auto type = types[indices[i]];
                setVoxel(Int3{ (int)pos[0], (int)pos[1], (int)pos[2] }, type);
            }
        }
    }
}

InfiniteVoxelMap::~InfiniteVoxelMap()
{
    for (auto& type : mLib)
        for (auto subType : type)
            if (subType)
                delete subType;

    mLib.clear();

    for (auto it : mChunks)
        delete it.second;

    mChunks.clear();

    for (auto m : mMeshes)
        m->release();

    mMeshes.clear();

    mDefaultMaterial->release();
}

void InfiniteVoxelMap::addVoxelType(const Int2& type, const std::string& name, const std::string& subName, VoxelCollision collision, const std::vector<PxVec3>& vertices, const std::vector<PxU32>& indices)
{
    if (type.x >= mLib.size())
        mLib.resize(type.x + 1, std::vector<VoxelType*>());

    if (type.y >= mLib[type.x].size())
        mLib[type.x].resize(type.y + 1, nullptr);

    if (mLib[type.x][type.y] == nullptr)
        mLib[type.x][type.y] = new VoxelType();

    auto& v = *mLib[type.x][type.y];

    v.type = type.x;
    v.subType = type.y;
    v.name = name;
    v.subName = subName;

    switch (collision)
    {
    case eNoCollision:
    {
        v.geomIndex = 0;
        break;
    }
    case eBoxCollision:
    {
        v.geomIndex = 1;
        break;
    }
    case eConvexMeshCollision:
    {
        v.geomIndex = addMeshGeometry(vertices.data(), (PxU32)vertices.size(), 1.0f);
        break;
    }
    case eTriangleMeshCollision:
    {
        v.geomIndex = addMeshGeometry(vertices.data(), (PxU32)vertices.size(), indices.data(), (PxU32)indices.size(), 1.0f);
        break;
    }
    }
}

const InfiniteVoxelMap::VoxelType& InfiniteVoxelMap::getVoxelType(const Int2& type) const
{
    if (type.x >= mLib.size() || type.y >= mLib[type.x].size() || mLib[type.x][type.y] == nullptr)
        return *mLib[0][0];

    return *mLib[type.x][type.y];
}

bool InfiniteVoxelMap::setVoxelRegion(const carb::Int3& regionMin, const carb::Int3& regionMax, const carb::Int2& type, const std::function<void(const carb::Int3&)>& callback)
{
    Int3 cMi{ (int)floorf((float)regionMin.x / mChunkSize.x),
              (int)floorf((float)regionMin.y / mChunkSize.y),
              (int)floorf((float)regionMin.z / mChunkSize.z) };
    Int3 cMa{ (int)floorf((float)regionMax.x / mChunkSize.x),
              (int)floorf((float)regionMax.y / mChunkSize.y),
              (int)floorf((float)regionMax.z / mChunkSize.z) };

    bool changed = false;

    for (int cx = cMi.x; cx <= cMa.x; ++cx)
        for (int cy = cMi.y; cy <= cMa.y; ++cy)
            for (int cz = cMi.z; cz <= cMa.z; ++cz)
            {
                Int3 chunkCoords{ cx, cy, cz };
                VoxelChunk* chunk = nullptr;

                auto chunkIt = mChunks.find(chunkCoords);
                if (chunkIt == mChunks.end())
                {
                    if (type.x == 0) continue;

                    chunk = new VoxelChunk(*this, chunkCoords);
                    mChunks[chunkCoords] = chunk;
                }
                else 
                    chunk = chunkIt->second;

                Int3 bMi{ std::max(regionMin.x, cx * mChunkSize.x),
                          std::max(regionMin.y, cy * mChunkSize.y),
                          std::max(regionMin.z, cz * mChunkSize.z) };
                Int3 bMa{ std::min(regionMax.x, (cx + 1) * mChunkSize.x - 1),
                          std::min(regionMax.y, (cy + 1) * mChunkSize.y - 1),
                          std::min(regionMax.z, (cz + 1) * mChunkSize.z - 1) };

                for (int bx = bMi.x; bx <= bMa.x; ++bx)
                    for (int by = bMi.y; by <= bMa.y; ++by)
                        for (int bz = bMi.z; bz <= bMa.z; ++bz)
                        {
                            if (chunk->setVoxel(Int3{ bx - cx * mChunkSize.x, by - cy * mChunkSize.y, bz - cz * mChunkSize.z }, type))
                            {
                                changed = true;

                                if (callback)
                                    callback(Int3{ bx, by, bz });
                            }
                        }

                if (chunk->empty())
                {
                    delete chunk;
                    mChunks.erase(chunkCoords);
                }
            }

    return changed;
}

bool InfiniteVoxelMap::setVoxel(const Int3& coords, const Int2& type)
{
    return setVoxelRegion(coords, coords, type);
}

Int2 InfiniteVoxelMap::getVoxel(const Int3& coords) const
{
    Int3 chunkCoords { (int)floorf((float)coords.x / mChunkSize.x),
                       (int)floorf((float)coords.y / mChunkSize.y),
                       (int)floorf((float)coords.z / mChunkSize.z) };

    VoxelChunk* chunk = nullptr;
    auto chunkIt = mChunks.find(chunkCoords);
    if (chunkIt == mChunks.end())
        return Int2{ 0, 0 };
    else
        chunk = chunkIt->second;

    Int3 localCoords { coords.x - chunkCoords.x * mChunkSize.x,
                       coords.y - chunkCoords.y * mChunkSize.y,
                       coords.z - chunkCoords.z * mChunkSize.z };

    return chunk->voxel(localCoords);
}

void InfiniteVoxelMap::setVoxels_USD(const Int3& regionMin, const Int3& regionMax, const Int2& type, bool update)
{
    setVoxelRegion(regionMin, regionMax, type,
        [this](const Int3& c)
        {
            Int3 cC{ (int)floorf((float)c.x / mUsdChunkSize[0]),
                     (int)floorf((float)c.y / mUsdChunkSize[1]),
                     (int)floorf((float)c.z / mUsdChunkSize[2]) };

            mDirtyUsdChunks.insert(cC);
        }
    );

    if (update && !mDirtyUsdChunks.empty())
    {
        auto root = mStage->GetPrimAtPath(mRoot);
        if (root.IsValid())
        {
            pxr::VtArray<int> indices;
            pxr::VtArray<pxr::GfVec3f> positions;
            pxr::SdfPathVector prototypes;

            struct UpdateItem
            {
                pxr::SdfPath chunkPath;
                pxr::VtArray<int> indices;
                pxr::VtArray<pxr::GfVec3f> positions;
                pxr::SdfPathVector prototypes;
            };
            std::list<UpdateItem> updateList;

            if (!mDirtyUsdChunks.empty())
            {
                pxr::SdfChangeBlock changeBlock;

                for (Int3 cC : mDirtyUsdChunks)
                {
                    carb::Int3 s{ cC.x * mUsdChunkSize[0],
                                  cC.y * mUsdChunkSize[1],
                                  cC.z * mUsdChunkSize[2], };
                    carb::Int3 e{ s.x + mUsdChunkSize[0] - 1,
                                  s.y + mUsdChunkSize[1] - 1,
                                  s.z + mUsdChunkSize[2] - 1, };

                    queryBlockRegion(s, e, indices, positions, prototypes);

                    bool chunkHasBlocks = !prototypes.empty();

                    char chunkName[32];
                    sprintf_s(chunkName, "Chunk_%s%d%s%d%s%d", (cC.x < 0 ? "N" : "P"), (cC.x < 0 ? -cC.x : cC.x),
                                                               (cC.y < 0 ? "N" : "P"), (cC.y < 0 ? -cC.y : cC.y),
                                                               (cC.z < 0 ? "N" : "P"), (cC.z < 0 ? -cC.z : cC.z));

                    auto chunkPath = root.GetPath().AppendChild(pxr::TfToken(chunkName));

                    auto chunk = mStage->GetPrimAtPath(chunkPath);

                    if (!chunk.IsValid() && chunkHasBlocks)
                        chunk = mStage->DefinePrim(chunkPath, pxr::TfToken("PointInstancer"));

                    // @@@ If we edit a layer on top of original map
                    // @@@ we shouldn't remove empty chunks.
                    //if (chunk.IsValid() && !chunkHasBlocks)
                    //    mStage->RemovePrim(chunkPath);

                    // @@@ We should update empty chunks
                    // @@@ if we don't remove them.
                    //if (chunkHasBlocks)
                    {
                        updateList.push_back(UpdateItem());
                        UpdateItem& item = updateList.back();
                        item.chunkPath = chunkPath;
                        item.indices.swap(indices);
                        item.positions.swap(positions);
                        item.prototypes.swap(prototypes);
                    }
                }
            }

            if (!updateList.empty())
            {
                pxr::SdfChangeBlock changeBlock;

                for (auto& item : updateList)
                {
                    auto chunk = mStage->GetPrimAtPath(item.chunkPath);
                    pxr::UsdGeomPointInstancer& instancer = (pxr::UsdGeomPointInstancer&)chunk;
                    instancer.GetProtoIndicesAttr().Set(item.indices);
                    instancer.GetPositionsAttr().Set(item.positions);
                    instancer.GetPrototypesRel().SetTargets(item.prototypes);
                }
            }
        }

        mDirtyUsdChunks.clear();
    }
}

void InfiniteVoxelMap::setUpdateVoxel(int sx, int sy, int sz, int ex, int ey, int ez, int type, int subType, int update)
{
    setVoxels_USD(Int3{ sx, sy, sz }, Int3{ ex, ey, ez }, Int2{ type, subType }, update != 0);


    //setVoxel(Int3{ x, y, z }, Int2{ type, subType });

    ////
    //auto root = mStage->GetPrimAtPath(mRoot);
    //if (root.IsValid())
    //{
    //    pxr::GfVec3i chunkSize;
    //    root.GetAttribute(pxr::TfToken("chunkSize")).Get(&chunkSize);

    //    Int3 cC { (int)floorf((float)x / chunkSize[0]),
    //              (int)floorf((float)y / chunkSize[1]),
    //              (int)floorf((float)z / chunkSize[2]) };

    //    pxr::VtArray<int> indices;
    //    pxr::VtArray<pxr::GfVec3f> positions;
    //    pxr::SdfPathVector prototypes;

    //    carb::Int3 s{ cC.x * chunkSize[0],
    //                  cC.y * chunkSize[1],
    //                  cC.z * chunkSize[2], };
    //    carb::Int3 e{ s.x + chunkSize[0] - 1,
    //                  s.y + chunkSize[1] - 1,
    //                  s.z + chunkSize[2] - 1, };

    //    queryBlockRegion(s, e, indices, positions, prototypes);

    //    bool chunkHasBlocks = !prototypes.empty();

    //    char chunkName[32];
    //    sprintf_s(chunkName, "Chunk_%s%d%s%d%s%d", (cC.x < 0 ? "N" : "P"), (cC.x < 0 ? -cC.x : cC.x),
    //                                               (cC.y < 0 ? "N" : "P"), (cC.y < 0 ? -cC.y : cC.y),
    //                                               (cC.z < 0 ? "N" : "P"), (cC.z < 0 ? -cC.z : cC.z));

    //    auto chunkPath = root.GetPath().AppendChild(pxr::TfToken(chunkName));

    //    {
    //        auto chunk = mStage->GetPrimAtPath(chunkPath);

    //        if (!chunk.IsValid() && chunkHasBlocks)
    //            chunk = mStage->DefinePrim(chunkPath, pxr::TfToken("PointInstancer"));

    //        if (chunk.IsValid() && !chunkHasBlocks)
    //            mStage->RemovePrim(chunkPath);

    //        if (chunkHasBlocks)
    //        {
    //            pxr::UsdGeomPointInstancer& instancer = (pxr::UsdGeomPointInstancer&)chunk;

    //            pxr::SdfChangeBlock changeBlock;

    //            instancer.GetProtoIndicesAttr().Set(indices);
    //            instancer.GetPositionsAttr().Set(positions);
    //            instancer.GetPrototypesRel().SetTargets(prototypes);
    //        }
    //    }
    //}
}

void InfiniteVoxelMap::collectChildMeshes(const pxr::UsdPrim& prim, std::vector<PxVec3>& vertices, std::vector<PxU32>& indices)
{
    for (const auto& child : prim.GetChildren())
    {
        if (child.IsA<pxr::UsdGeomMesh>())
        {
            const auto& mesh = (const pxr::UsdGeomMesh&)child;

            PxU32 base = (PxU32)vertices.size();

            pxr::VtArray<pxr::GfVec3f> points;
            mesh.GetPointsAttr().Get(&points);
            for (const auto& p : points)
                vertices.push_back(PxVec3(p[0] - 0.5f, p[1] - 0.5f, p[2] - 0.5f));

            pxr::VtArray<int> faceVertexCounts;
            mesh.GetFaceVertexCountsAttr().Get(&faceVertexCounts);
            pxr::VtArray<int> faceVertexIndices;
            mesh.GetFaceVertexIndicesAttr().Get(&faceVertexIndices);
            int start = 0;
            for (auto count : faceVertexCounts)
            {
                for (int i = 1; i < count - 1; ++i)
                {
                    indices.push_back(base + faceVertexIndices[start]);
                    indices.push_back(base + faceVertexIndices[start + i]);
                    indices.push_back(base + faceVertexIndices[start + i + 1]);
                }
                start += count;
            }
        }
    }
}

void InfiniteVoxelMap::queryBlockRegion(const carb::Int3& s, const carb::Int3& e, pxr::VtArray<int>& indices, pxr::VtArray<pxr::GfVec3f>& positions, pxr::SdfPathVector& prototypes) const
{
    Int3 cMi { (int)floorf((float)s.x / mChunkSize.x),
               (int)floorf((float)s.y / mChunkSize.y),
               (int)floorf((float)s.z / mChunkSize.z) };
    Int3 cMa { (int)floorf((float)e.x / mChunkSize.x),
               (int)floorf((float)e.y / mChunkSize.y),
               (int)floorf((float)e.z / mChunkSize.z) };

    indices.clear();
    positions.clear();
    std::vector<PxU32> types;
    for (int cx = cMi.x; cx <= cMa.x; ++cx)
        for (int cy = cMi.y; cy <= cMa.y; ++cy)
            for (int cz = cMi.z; cz <= cMa.z; ++cz)
            {
                auto chunkIt = mChunks.find(Int3{ cx, cy, cz });
                if (chunkIt == mChunks.end()) continue;

                const auto& chunk = chunkIt->second;

                Int3 bMi { std::max(s.x, cx * mChunkSize.x),
                           std::max(s.y, cy * mChunkSize.y),
                           std::max(s.z, cz * mChunkSize.z) };
                Int3 bMa { std::min(e.x, (cx + 1) * mChunkSize.x - 1),
                           std::min(e.y, (cy + 1) * mChunkSize.y - 1),
                           std::min(e.z, (cz + 1) * mChunkSize.z - 1) };

                for (int bx = bMi.x; bx <= bMa.x; ++bx)
                    for (int by = bMi.y; by <= bMa.y; ++by)
                        for (int bz = bMi.z; bz <= bMa.z; ++bz)
                        {
                            auto voxel = chunk->voxel(Int3{ bx - cx * mChunkSize.x, by - cy * mChunkSize.y, bz - cz * mChunkSize.z });
                            if (voxel.x == 0) continue;

                            PxU32 type = (voxel.x << 16) | (voxel.y & 0xff);

                            auto typeIt = std::find(types.begin(), types.end(), type);
                            if (typeIt == types.end())
                            {
                                indices.push_back((int)types.size());
                                types.push_back(type);
                            }
                            else
                            {
                                indices.push_back((int)(typeIt - types.begin()));
                            }
                            positions.push_back(pxr::GfVec3f((float)bx, (float)by, (float)bz));
                        }
            }

    prototypes.clear();
    for (PxU32 t : types)
    {
        int type = (t >> 16);
        int subType = (t & 0xff);
        char blockPath[64];
        sprintf_s(blockPath, "../BlockLib/Blocks/Block_%d_%d", type, subType);
        prototypes.push_back(pxr::SdfPath(blockPath));
    }
}

int InfiniteVoxelMap::addMeshGeometry(const PxVec3* verts, PxU32 vertCount, const PxU32* tris, PxU32 triCount, float scale)
{
    auto physics = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getPhysics();

    PxTriangleMeshDesc desc;
    desc.points.count = vertCount;
    desc.points.stride = sizeof(PxVec3);
    desc.points.data = verts;
    desc.triangles.count = triCount;
    desc.triangles.stride = sizeof(PxU32) * 3;
    desc.triangles.data = tris;

    auto mesh = PxCreateTriangleMesh(omni::physx::OmniPhysX::getInstance().getPhysXSetup().getDefaultCookingParams(), desc, physics->getPhysicsInsertionCallback());
    if (mesh)
    {
        mMeshes.push_back(mesh);
        mGeoms.push_back(PxTriangleMeshGeometry(mesh, PxMeshScale(PxVec3(scale))));
        return PxU32(mGeoms.size() - 1);
    }

    return 0;
}

int InfiniteVoxelMap::addMeshGeometry(const PxVec3* verts, PxU32 vertCount, float scale)
{
    auto physics = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getPhysics();

    PxConvexMeshDesc desc;
    desc.points.count = vertCount;
    desc.points.stride = sizeof(PxVec3);
    desc.points.data = verts;
    desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

    auto mesh = PxCreateConvexMesh(omni::physx::OmniPhysX::getInstance().getPhysXSetup().getDefaultCookingParams(), desc, physics->getPhysicsInsertionCallback());
    if (mesh)
    {
        mMeshes.push_back(mesh);
        mGeoms.push_back(PxConvexMeshGeometry(mesh, PxMeshScale(PxVec3(scale))));
        return PxU32(mGeoms.size() - 1);
    }

    return 0;
}

//

InfiniteVoxelMap::VoxelChunk::VoxelChunk(InfiniteVoxelMap& voxelMap, const carb::Int3& coords)
    :
    voxelMap(voxelMap), mCounter(0)
{
    mVoxels.resize(dimX() * dimY() * dimZ(), 0);

    auto physics = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getPhysics();

    mRigidStatic = physics->createRigidStatic(PxTransform(PxVec3(float(coords.x * voxelMap.mChunkSize.x), float(coords.y * voxelMap.mChunkSize.y), float(coords.z * voxelMap.mChunkSize.z))));
    PxRigidActorExt::createExclusiveShape(*mRigidStatic, PxCustomGeometry(*this), *voxelMap.mDefaultMaterial);

    voxelMap.mScene->addActor(*mRigidStatic);
}

InfiniteVoxelMap::VoxelChunk::~VoxelChunk()
{
    if (mRigidStatic)
        mRigidStatic->release();
}

PxBounds3 InfiniteVoxelMap::VoxelChunk::getLocalBounds(const PxGeometry& /*geometry*/) const {
    return PxBounds3::centerExtents(extents() * 0.5f, extents());
};

bool InfiniteVoxelMap::VoxelChunk::generateContacts(const PxGeometry& geom0, const PxGeometry& geom1, const PxTransform& pose0, const PxTransform& pose1,
    const PxReal contactDistance, const PxReal meshContactMargin, const PxReal toleranceLength,
    PxContactBuffer& contactBuffer) const
{
    const PxGeometry* pGeom1 = &geom1;
    PxTransform pose1in0 = pose0.transformInv(pose1);
    PxBounds3 bounds1; PxGeometryQuery::computeGeomBounds(bounds1, geom1, pose1in0, contactDistance, 1.0f, PxGeometryQueryFlags(0));

    struct ContactRecorder : immediate::PxContactRecorder
    {
        PxContactBuffer* contactBuffer;
        ContactRecorder(PxContactBuffer& _contactBuffer) : contactBuffer(&_contactBuffer) {}
        virtual bool recordContacts(const PxContactPoint* contactPoints, const PxU32 nbContacts, const PxU32 index)
        {
            for (PxU32 i = 0; i < nbContacts; ++i)
                contactBuffer->contact(contactPoints[i]);
            return true;
        }
    }
    contactRecorder(contactBuffer);

    PxCache contactCache;

    struct ContactCacheAllocator : PxCacheAllocator
    {
        PxU8 buffer[1024 * 2];
        ContactCacheAllocator() { clear(); }
        void clear() { memset(buffer, 0, sizeof(buffer)); }
        virtual PxU8* allocateCacheData(const PxU32 byteSize) { return (PxU8*)(size_t(buffer + 0xf) & ~0xf); }
    }
    contactCacheAllocator;

    int sx, sy, sz, ex, ey, ez;
    getVoxelRegion(bounds1, sx, sy, sz, ex, ey, ez);
    for (int x = sx; x <= ex; ++x)
        for (int y = sy; y <= ey; ++y)
            for (int z = sz; z <= ez; ++z)
                if (voxelHasCollision(x, y, z))
                {
                    const PxGeometry& voxelGeom = voxelGeometry(x, y, z);
                    const PxGeometry* pGeom0 = &voxelGeom;
                    PxTransform p0 = pose0.transform(PxTransform(voxelPos(x, y, z)));
                    contactCache.reset(); // It crashes without this sometimes
                    immediate::PxGenerateContacts(&pGeom0, &pGeom1, &p0, &pose1, &contactCache, 1, contactRecorder,
                        contactDistance, meshContactMargin, toleranceLength, contactCacheAllocator);
                }

    return true;
};

PxU32 InfiniteVoxelMap::VoxelChunk::raycast(const PxVec3& origin, const PxVec3& unitDir, const PxGeometry& geom, const PxTransform& pose,
    PxReal maxDist, PxHitFlags hitFlags, PxU32 maxHits, PxGeomRaycastHit* rayHits, PxU32 stride, PxRaycastThreadContext* /*threadContext*/) const
{
    PxVec3 p = pose.transformInv(origin);
    PxVec3 n = pose.rotateInv(unitDir);
    PxVec3 s = voxelSize() * 0.5f;
    int x, y, z; pointCoords(p, x, y, z);
    int hitCount = 0;
    PxU8* hitBuffer = (PxU8*)rayHits;
    float currDist = 0;
    PxVec3 hitN(0);

    while (currDist < maxDist)
    {
        PxVec3 v = voxelPos(x, y, z);
        if (voxelHasCollision(x, y, z))
        {
            if (voxelIsCube(x, y, z))
            {
                PxGeomRaycastHit& h = *((PxGeomRaycastHit*)(hitBuffer + hitCount * stride));
                h.distance = currDist;
                h.position = origin + unitDir * currDist;
                h.normal = hitN;
                h.faceIndex = (x) | (y << 10) | (z << 20);
                hitCount += 1;
            }
            else
            {
                const PxGeometry& voxelGeom = voxelGeometry(x, y, z);
                PxTransform p0 = pose.transform(PxTransform(voxelPos(x, y, z)));
                PxGeomRaycastHit hits[100]; // Assume this be enough
                int newHits = PxGeometryQuery::raycast(origin, unitDir, voxelGeom, p0, maxDist, hitFlags, PxMin((int)maxHits - hitCount, 100), hits);
                for (int i = 0; i < newHits; ++i)
                {
                    PxGeomRaycastHit& h = hits[i];
                    h.faceIndex = (x) | (y << 10) | (z << 20);
                    *((PxGeomRaycastHit*)(hitBuffer + hitCount++ * stride)) = h;
                }
            }
        }
        if (hitCount == (int)maxHits)
            break;
        float step = FLT_MAX;
        int dx = 0, dy = 0, dz = 0;
        if (n.x > FLT_EPSILON)
        {
            float d = (v.x + s.x - p.x) / n.x;
            if (d < step) { step = d; dx = 1; dy = 0; dz = 0; }
        }
        if (n.x < -FLT_EPSILON)
        {
            float d = (v.x - s.x - p.x) / n.x;
            if (d < step) { step = d; dx = -1; dy = 0; dz = 0; }
        }
        if (n.y > FLT_EPSILON)
        {
            float d = (v.y + s.y - p.y) / n.y;
            if (d < step) { step = d; dx = 0; dy = 1; dz = 0; }
        }
        if (n.y < -FLT_EPSILON)
        {
            float d = (v.y - s.y - p.y) / n.y;
            if (d < step) { step = d; dx = 0; dy = -1; dz = 0; }
        }
        if (n.z > FLT_EPSILON)
        {
            float d = (v.z + s.z - p.z) / n.z;
            if (d < step) { step = d; dx = 0; dy = 0; dz = 1; }
        }
        if (n.z < -FLT_EPSILON)
        {
            float d = (v.z - s.z - p.z) / n.z;
            if (d < step) { step = d; dx = 0; dy = 0; dz = -1; }
        }
        x += dx; y += dy; z += dz;
        hitN = PxVec3(static_cast<float>(-dx), static_cast<float>(-dy), static_cast<float>(-dz));
        currDist = step;
    }

    return hitCount;
};

bool InfiniteVoxelMap::VoxelChunk::overlap(const PxGeometry& geom0, const PxTransform& pose0, const PxGeometry& geom1, const PxTransform& pose1, PxRaycastThreadContext* /*threadContext*/) const
{
    PxTransform pose1in0 = pose0.transformInv(pose1);
    PxBounds3 bounds1; PxGeometryQuery::computeGeomBounds(bounds1, geom1, pose1in0, 0, 1.0f, PxGeometryQueryFlags(0));

    int sx, sy, sz, ex, ey, ez;
    getVoxelRegion(bounds1, sx, sy, sz, ex, ey, ez);
    for (int x = sx; x <= ex; ++x)
        for (int y = sy; y <= ey; ++y)
            for (int z = sz; z <= ez; ++z)
                if (voxelHasCollision(x, y, z))
                {
                    const PxGeometry& voxelGeom = voxelGeometry(x, y, z);
                    PxTransform p0 = pose0.transform(PxTransform(voxelPos(x, y, z)));
                    if (PxGeometryQuery::overlap(voxelGeom, p0, geom1, pose1, PxGeometryQueryFlags(0)))
                        return true;
                }

    return false;
};

bool InfiniteVoxelMap::VoxelChunk::sweep(const PxVec3& unitDir, const PxReal maxDist,
    const PxGeometry& geom0, const PxTransform& pose0, const PxGeometry& geom1, const PxTransform& pose1,
    PxGeomSweepHit& sweepHit, PxHitFlags hitFlags, const PxReal inflation, PxRaycastThreadContext* /*threadContext*/) const
{
    PxTransform pose1in0 = pose0.transformInv(pose1);
    PxBounds3 b; PxGeometryQuery::computeGeomBounds(b, geom1, pose1in0, 0, 1.0f, PxGeometryQueryFlags(0));
    PxVec3 n = pose0.rotateInv(unitDir);
    PxVec3 s = voxelSize();

    int sx, sy, sz, ex, ey, ez;
    getVoxelRegion(b, sx, sy, sz, ex, ey, ez);
    int sx1, sy1, sz1, ex1, ey1, ez1;
    sx1 = sy1 = sz1 = -1; ex1 = ey1 = ez1 = 0;
    float currDist = 0;
    sweepHit.distance = FLT_MAX;
    while (currDist <= maxDist && currDist < sweepHit.distance)
    {
        for (int x = sx; x <= ex; ++x)
            for (int y = sy; y <= ey; ++y)
                for (int z = sz; z <= ez; ++z)
                    if (voxelHasCollision(x, y, z))
                    {
                        if (x >= sx1 && x <= ex1 && y >= sy1 && y <= ey1 && z >= sz1 && z <= ez1)
                            continue;

                        const PxGeometry& voxelGeom = voxelGeometry(x, y, z);

                        PxGeomSweepHit hit;
                        PxTransform p0 = pose0.transform(PxTransform(voxelPos(x, y, z)));
                        if (PxGeometryQuery::sweep(unitDir, maxDist, geom1, pose1, voxelGeom, p0, hit, hitFlags, inflation, PxGeometryQueryFlags(0)))
                            if (hit.distance < sweepHit.distance)
                                sweepHit = hit;
                    }

        PxVec3 mi = b.minimum, ma = b.maximum;
        PxVec3 bs = voxelPos(sx, sy, sz) - s, be = voxelPos(ex, ey, ez) + s;
        float dist = FLT_MAX;
        if (n.x > FLT_EPSILON)
        {
            float d = (be.x - ma.x) / n.x;
            if (d < dist) dist = d;
        }
        if (n.x < -FLT_EPSILON)
        {
            float d = (bs.x - mi.x) / n.x;
            if (d < dist) dist = d;
        }
        if (n.y > FLT_EPSILON)
        {
            float d = (be.y - ma.y) / n.y;
            if (d < dist) dist = d;
        }
        if (n.y < -FLT_EPSILON)
        {
            float d = (bs.y - mi.y) / n.y;
            if (d < dist) dist = d;
        }
        if (n.z > FLT_EPSILON)
        {
            float d = (be.z - ma.z) / n.z;
            if (d < dist) dist = d;
        }
        if (n.z < -FLT_EPSILON)
        {
            float d = (bs.z - mi.z) / n.z;
            if (d < dist) dist = d;
        }
        if (0.0f == currDist && dist > maxDist)
        {
            // ensure that maxDist is tested in case first step exceeds maxDist:
            dist = maxDist;
        }
        sx1 = sx; sy1 = sy; sz1 = sz; ex1 = ex; ey1 = ey; ez1 = ez;
        auto b1 = b; b1.minimum += n * dist; b1.maximum += n * dist;
        getVoxelRegion(b1, sx, sy, sz, ex, ey, ez);
        currDist = dist;
    }

    return sweepHit.distance < FLT_MAX;
}

void InfiniteVoxelMap::VoxelChunk::visualize(const PxGeometry& geometry, PxRenderOutput& out, const PxTransform& absPose, const PxBounds3& cullbox) const
{
    const PxVec3 X(1, 0, 0);
    const PxVec3 Y(0, 1, 0);
    const PxVec3 Z(0, 0, 1);
    const float s = 0.5f;
    out << 0xff00ff;
    PxVec3 halfExtents(voxelMap.mChunkSize.x * voxelSizeX() * 0.5f,
                       voxelMap.mChunkSize.y * voxelSizeY() * 0.5f,
                       voxelMap.mChunkSize.z * voxelSizeZ() * 0.5f);
    out << absPose;
    out << PxDebugBox(halfExtents, halfExtents);
    out << PxTransform(PxIdentity);
    for (int x = 0; x < (int)dimX(); ++x)
        for (int y = 0; y < (int)dimY(); ++y)
            for (int z = 0; z < (int)dimZ(); ++z)
            {
                if (voxelHasCollision(x, y, z))
                {
                    PxVec3 p = voxelPos(x, y, z);
                    PxVec3 p0 = absPose.transform(p + (X + Y + Z) * s);
                    PxVec3 p1 = absPose.transform(p + (-X + Y + Z) * s);
                    PxVec3 p2 = absPose.transform(p + (-X + Y + -Z) * s);
                    PxVec3 p3 = absPose.transform(p + (X + Y + -Z) * s);
                    PxVec3 p4 = absPose.transform(p + (X + -Y + Z) * s);
                    PxVec3 p5 = absPose.transform(p + (-X + -Y + Z) * s);
                    PxVec3 p6 = absPose.transform(p + (-X + -Y + -Z) * s);
                    PxVec3 p7 = absPose.transform(p + (X + -Y + -Z) * s);

                    PxBounds3 bounds = PxBounds3::empty();
                    bounds.include(p0);
                    bounds.include(p1);
                    bounds.include(p2);
                    bounds.include(p3);
                    bounds.include(p4);
                    bounds.include(p5);
                    bounds.include(p6);
                    bounds.include(p7);

                    if (bounds.intersects(cullbox))
                    {
                        if (voxelIsCube(x, y, z))
                        {
                            out.outputSegment(p0, p1);
                            out.outputSegment(p1, p2);
                            out.outputSegment(p2, p3);
                            out.outputSegment(p3, p0);
                            out.outputSegment(p4, p5);
                            out.outputSegment(p5, p6);
                            out.outputSegment(p6, p7);
                            out.outputSegment(p7, p4);
                            out.outputSegment(p0, p4);
                            out.outputSegment(p1, p5);
                            out.outputSegment(p2, p6);
                            out.outputSegment(p3, p7);
                        }
                        else
                        {
                            const PxGeometry& geom = voxelGeometry(x, y, z);
                            if (geom.getType() == PxGeometryType::eCONVEXMESH)
                            {
                                const PxConvexMeshGeometry& convexMeshGeom = static_cast<const PxConvexMeshGeometry&>(geom);
                                const PxVec3* verts = convexMeshGeom.convexMesh->getVertices();
                                const PxU8* indxs = convexMeshGeom.convexMesh->getIndexBuffer();
                                float scale = convexMeshGeom.scale.scale.x; // Uniform, no rotation
                                int polyCount = convexMeshGeom.convexMesh->getNbPolygons();
                                for (int i = 0; i < polyCount; ++i)
                                {
                                    PxHullPolygon data;
                                    convexMeshGeom.convexMesh->getPolygonData(i, data);
                                    for (int j0 = 0; j0 < data.mNbVerts; ++j0)
                                    {
                                        int j1 = (j0 + 1) % data.mNbVerts;
                                        PxVec3 v0 = absPose.transform(p + verts[indxs[data.mIndexBase + j0]] * scale);
                                        PxVec3 v1 = absPose.transform(p + verts[indxs[data.mIndexBase + j1]] * scale);
                                        out.outputSegment(v0, v1);
                                    }
                                }
                            }
                            else if (geom.getType() == PxGeometryType::eTRIANGLEMESH)
                            {
                                const PxTriangleMeshGeometry& triangleMeshGeom = static_cast<const PxTriangleMeshGeometry&>(geom);
                                const PxVec3* verts = triangleMeshGeom.triangleMesh->getVertices();
                                const PxU16* tris = (const PxU16*)triangleMeshGeom.triangleMesh->getTriangles(); // Assume it's always 16 bits
                                float scale = triangleMeshGeom.scale.scale.x; // Uniform, no rotation
                                int triCount = triangleMeshGeom.triangleMesh->getNbTriangles();
                                for (int i = 0; i < triCount; ++i)
                                {
                                    PxVec3 v0 = absPose.transform(p + verts[tris[i * 3 + 0]] * scale);
                                    PxVec3 v1 = absPose.transform(p + verts[tris[i * 3 + 1]] * scale);
                                    PxVec3 v2 = absPose.transform(p + verts[tris[i * 3 + 2]] * scale);
                                    out.outputSegment(v0, v1);
                                    out.outputSegment(v1, v2);
                                    out.outputSegment(v2, v0);
                                }
                            }
                        }
                    }
                }
            }
}

PxVec3 InfiniteVoxelMap::VoxelChunk::voxelPos(int x, int y, int z) const
{
    return PxVec3((x + 0.5f) * voxelSizeX(), (y + 0.5f) * voxelSizeY(), (z + 0.5f) * voxelSizeZ());
}

void InfiniteVoxelMap::VoxelChunk::pointCoords(const PxVec3& p, int& x, int& y, int& z) const
{
    PxVec3 l = p, s = voxelSize();
    x = (int)floorf(l.x / s.x);
    y = (int)floorf(l.y / s.y);
    z = (int)floorf(l.z / s.z);
}

void InfiniteVoxelMap::VoxelChunk::getVoxelRegion(const PxBounds3& b, int& sx, int& sy, int& sz, int& ex, int& ey, int& ez) const
{
    pointCoords(b.minimum, sx, sy, sz);
    pointCoords(b.maximum, ex, ey, ez);
}

bool InfiniteVoxelMap::VoxelChunk::voxelHasCollision(int x, int y, int z) const
{
    Int2 type = voxel(Int3{ x, y, z });
    return voxelMap.getVoxelType(type).geomIndex > 0;
}

bool InfiniteVoxelMap::VoxelChunk::voxelIsCube(int x, int y, int z) const
{
    Int2 type = voxel(Int3{ x, y, z });
    return voxelMap.getVoxelType(type).geomIndex == 1;
}

const PxGeometry& InfiniteVoxelMap::VoxelChunk::voxelGeometry(int x, int y, int z) const
{
    Int2 type = voxel(Int3{ x, y, z });
    int geomIndex = voxelMap.getVoxelType(type).geomIndex;
    return voxelMap.mGeoms[geomIndex].any();
}

bool InfiniteVoxelMap::VoxelChunk::setVoxel(const Int3& coords, const Int2& type)
{
    if (coords.x < 0 || coords.x >= (int)dimX() ||
        coords.y < 0 || coords.y >= (int)dimY() ||
        coords.z < 0 || coords.z >= (int)dimZ())
        return false;

    auto oldType = voxel(coords);
    if (oldType.x == 0 && type.x != 0) ++mCounter;
    if (oldType.x != 0 && type.x == 0) --mCounter;

    PxU32 data = (type.x << 16) | (type.y & 0xffff);
    mVoxels[coords.x + coords.y * dimX() + coords.z * dimX() * dimY()] = data;

    return oldType.x != type.x || oldType.y != type.y;
}

Int2 InfiniteVoxelMap::VoxelChunk::voxel(const Int3& coords) const
{
    if (coords.x < 0 || coords.x >= (int)dimX() ||
        coords.y < 0 || coords.y >= (int)dimY() ||
        coords.z < 0 || coords.z >= (int)dimZ())
        return Int2 { 0, 0 };

    PxU32 data = mVoxels[coords.x + coords.y * dimX() + coords.z * dimX() * dimY()];
    return Int2 { (data >> 16), (data & 0xffff) };
}
