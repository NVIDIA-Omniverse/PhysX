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
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.


#include "NvBlastAssert.h"

#include "NvBlastTkFrameworkImpl.h"
#include "NvBlastTkAssetImpl.h"
#include "NvBlastTkFamilyImpl.h"
#include "NvBlastTkGroupImpl.h"
#include "NvBlastTkActorImpl.h"
#include "NvBlastTkJointImpl.h"
#include "NvBlastTkTypeImpl.h"

#include "NvBlastGlobals.h"

#include <algorithm>


using namespace nvidia;
using namespace nvidia::shdfnd;


NV_INLINE bool operator < (const NvBlastID& id1, const NvBlastID& id2)
{
    return memcmp(&id1, &id2, sizeof(NvBlastID)) < 0;
}


namespace Nv
{
namespace Blast
{

//////// Local definitions ////////

// Map type ID to static type data
#define NVBLASTTK_REGISTER_TYPE(_name)                                      \
    if (!Tk##_name##Impl::s_type.indexIsValid())                            \
    {                                                                       \
        Tk##_name##Impl::s_type.setIndex(TkTypeIndex::_name);               \
    }                                                                       \
    m_types[TkTypeIndex::_name] = &Tk##_name##Impl::s_type;                 \
    m_typeIDToIndex[Tk##_name##Impl::s_type.getID()] = TkTypeIndex::_name


#define NVBLASTTK_RELEASE_TYPE(_name)                                       \
    {                                                                       \
        TkTypeImpl& type = Tk##_name##Impl::s_type;                         \
        auto& toRelease = m_objects[type.getIndex()];                       \
        for (TkObject* obj : toRelease)                                     \
        {                                                                   \
            obj->release();                                                 \
        }                                                                   \
    }


//////// TkFrameworkImpl static variables ////////

TkFrameworkImpl* TkFrameworkImpl::s_framework = nullptr;


//////// TkFrameworkImpl static function ////////

TkFrameworkImpl* TkFrameworkImpl::get()
{
    return s_framework;
}


bool TkFrameworkImpl::set(TkFrameworkImpl* framework)
{
    if (s_framework != nullptr)
    {
        if (framework != nullptr)
        {
            NVBLAST_LOG_ERROR("TkFrameworkImpl::set: framework already set.  Pass NULL to this function to destroy framework.");
            return false;
        }

        NVBLAST_DELETE(s_framework, TkFrameworkImpl);
    }

    s_framework = framework;

    return true;
}


//////// TkFrameworkImpl methods ////////

TkFrameworkImpl::TkFrameworkImpl()
    : TkFramework()
{
    // Register types
    m_types.resize(TkTypeIndex::TypeCount);
    m_objects.resize(TkTypeIndex::TypeCount);
    NVBLASTTK_REGISTER_TYPE(Asset);
    NVBLASTTK_REGISTER_TYPE(Family);
    NVBLASTTK_REGISTER_TYPE(Group);
}


TkFrameworkImpl::~TkFrameworkImpl()
{
}


void TkFrameworkImpl::release()
{
    // Special release of joints, which are not TkIdentifiable:
    Array<TkJointImpl*>::type joints;   // Since the EraseIterator is not exposed
    joints.reserve(m_joints.size());
    for (auto j = m_joints.getIterator(); !j.done(); ++j)
    {
        joints.pushBack(*j);
    }
    for (uint32_t i = 0; i < joints.size(); ++i)
    {
        joints[i]->release();
    }
    NVBLAST_ASSERT(m_joints.size() == 0);
    joints.reset(); // Since we will be deleting the allocator

    NVBLASTTK_RELEASE_TYPE(Group);
    NVBLASTTK_RELEASE_TYPE(Asset);
    set(nullptr);
}


const TkType* TkFrameworkImpl::getType(TkTypeIndex::Enum typeIndex) const
{
    if (typeIndex < 0 || typeIndex >= TkTypeIndex::TypeCount)
    {
        NVBLAST_LOG_WARNING("TkFrameworkImpl::getType: invalid typeIndex.");
        return nullptr;
    }

    return m_types[typeIndex];
}


TkIdentifiable* TkFrameworkImpl::findObjectByID(const NvBlastID& id) const
{
    TkIdentifiable* object = findObjectByIDInternal(id);

    if (object == nullptr)
    {
        NVBLAST_LOG_WARNING("TkFrameworkImpl::findObjectByID: object not found.");
    }

    return object;
}


uint32_t TkFrameworkImpl::getObjectCount(const TkType& type) const
{
    const uint32_t index = static_cast<const TkTypeImpl&>(type).getIndex();

    if (index >= m_objects.size())
    {
        NVBLAST_LOG_ERROR("TkFrameworkImpl::getObjectCount: BlastTk object type unrecognized.");
        return 0;

    }

    return m_objects[index].size();
}


uint32_t TkFrameworkImpl::getObjects(TkIdentifiable** buffer, uint32_t bufferSize, const TkType& type, uint32_t indexStart /* = 0 */) const
{
    const uint32_t index = static_cast<const TkTypeImpl&>(type).getIndex();

    if (index >= m_objects.size())
    {
        NVBLAST_LOG_ERROR("TkFrameworkImpl::getObjectCount: BlastTk object type unrecognized.");
        return 0;
    }

    const auto& objectArray = m_objects[index];

    uint32_t objectCount = objectArray.size();
    if (objectCount <= indexStart)
    {
        NVBLAST_LOG_WARNING("TkFrameworkImpl::getObjects: indexStart beyond end of object list.");
        return 0;
    }

    objectCount -= indexStart;
    if (objectCount > bufferSize)
    {
        objectCount = bufferSize;
    }

    memcpy(buffer, objectArray.begin() + indexStart, objectCount * sizeof(TkObject*));

    return objectCount;
}


bool TkFrameworkImpl::reorderAssetDescChunks(NvBlastChunkDesc* chunkDescs, uint32_t chunkCount, NvBlastBondDesc* bondDescs, uint32_t bondCount, uint32_t* chunkReorderMap /*= nullptr*/, bool keepBondNormalChunkOrder /*= false*/) const
{
    uint32_t* map = chunkReorderMap != nullptr ? chunkReorderMap : static_cast<uint32_t*>(NVBLAST_ALLOC_NAMED(chunkCount * sizeof(uint32_t), "reorderAssetDescChunks:chunkReorderMap"));
    void* scratch = NVBLAST_ALLOC_NAMED(chunkCount * sizeof(NvBlastChunkDesc), "reorderAssetDescChunks:scratch");
    const bool result = NvBlastReorderAssetDescChunks(chunkDescs, chunkCount, bondDescs, bondCount, map, keepBondNormalChunkOrder, scratch, logLL);
    NVBLAST_FREE(scratch);
    if (chunkReorderMap == nullptr)
    {
        NVBLAST_FREE(map);
    }
    return result;
}


bool TkFrameworkImpl::ensureAssetExactSupportCoverage(NvBlastChunkDesc* chunkDescs, uint32_t chunkCount) const
{
    void* scratch = NVBLAST_ALLOC_NAMED(chunkCount, "ensureAssetExactSupportCoverage:scratch");
    const bool result = NvBlastEnsureAssetExactSupportCoverage(chunkDescs, chunkCount, scratch, logLL);
    NVBLAST_FREE(scratch);
    return result;
}


TkAsset* TkFrameworkImpl::createAsset(const TkAssetDesc& desc)
{
    TkAssetImpl* asset = TkAssetImpl::create(desc);
    if (asset == nullptr)
    {
        NVBLAST_LOG_ERROR("TkFrameworkImpl::createAsset: failed to create asset.");
    }

    return asset;
}


TkAsset* TkFrameworkImpl::createAsset(const NvBlastAsset* assetLL, Nv::Blast::TkAssetJointDesc* jointDescs, uint32_t jointDescCount, bool ownsAsset)
{
    TkAssetImpl* asset = TkAssetImpl::create(assetLL, jointDescs, jointDescCount, ownsAsset);
    if (asset == nullptr)
    {
        NVBLAST_LOG_ERROR("TkFrameworkImpl::createAsset: failed to create asset.");
    }

    return asset;
}


TkGroup* TkFrameworkImpl::createGroup(const TkGroupDesc& desc)
{
    TkGroupImpl* group = TkGroupImpl::create(desc);
    if (group == nullptr)
    {
        NVBLAST_LOG_ERROR("TkFrameworkImpl::createGroup: failed to create group.");
    }

    return group;
}


TkActor* TkFrameworkImpl::createActor(const TkActorDesc& desc)
{
    TkActor* actor = TkActorImpl::create(desc);
    if (actor == nullptr)
    {
        NVBLAST_LOG_ERROR("TkFrameworkImpl::createActor: failed to create actor.");
    }

    return actor;
}


TkJoint* TkFrameworkImpl::createJoint(const TkJointDesc& desc)
{
    TkJointImpl** handle0 = nullptr;
    TkJointImpl** handle1 = nullptr;

    TkFamilyImpl* family0 = static_cast<TkFamilyImpl*>(desc.families[0]);
    TkFamilyImpl* family1 = static_cast<TkFamilyImpl*>(desc.families[1]);

    NVBLAST_CHECK_ERROR(family0 != nullptr || family1 != nullptr, "TkFrameworkImpl::createJoint: at least one family in the TkJointDesc must be valid.", return nullptr);

    NVBLAST_CHECK_ERROR(family0 == nullptr || desc.chunkIndices[0] < family0->getAssetImpl()->getChunkCount(), "TkFrameworkImpl::createJoint: desc.chunkIndices[0] is invalid.", return nullptr);
    NVBLAST_CHECK_ERROR(family1 == nullptr || desc.chunkIndices[1] < family1->getAssetImpl()->getChunkCount(), "TkFrameworkImpl::createJoint: desc.chunkIndices[1] is invalid.", return nullptr);

    const bool actorsAreTheSame = family0 == family1 && family0->getActorByChunk(desc.chunkIndices[0]) == family1->getActorByChunk(desc.chunkIndices[1]);
    NVBLAST_CHECK_ERROR(!actorsAreTheSame, "TkFrameworkImpl::createJoint: the chunks listed in the TkJointDesc must be in different actors.", return nullptr);

    if (family0 != nullptr)
    {
        const bool isSupportChunk = !isInvalidIndex(NvBlastAssetGetChunkToGraphNodeMap(family0->getAssetImpl()->getAssetLLInternal(), logLL)[desc.chunkIndices[0]]);
        NVBLAST_CHECK_ERROR(isSupportChunk, "TkFrameworkImpl::createJoint: desc.chunkIndices[0] is not a support chunk in the asset for desc.families[0].  Joint not created.", return nullptr);
        handle0 = family0->createExternalJointHandle(getFamilyID(family1), desc.chunkIndices[0], desc.chunkIndices[1]);
        NVBLAST_CHECK_ERROR(handle0 != nullptr, "TkFrameworkImpl::createJoint: could not create joint handle in family[0].  Joint not created.", return nullptr);
    }

    if (family1 != nullptr)
    {
        const bool isSupportChunk = !isInvalidIndex(NvBlastAssetGetChunkToGraphNodeMap(family1->getAssetImpl()->getAssetLLInternal(), logLL)[desc.chunkIndices[1]]);
        NVBLAST_CHECK_ERROR(isSupportChunk, "TkFrameworkImpl::createJoint: desc.chunkIndices[1] is not a support chunk in the asset for desc.families[1].  Joint not created.", return nullptr);
        if (family1 != family0)
        {
            handle1 = family1->createExternalJointHandle(getFamilyID(family0), desc.chunkIndices[1], desc.chunkIndices[0]);
            NVBLAST_CHECK_ERROR(handle1 != nullptr, "TkFrameworkImpl::createJoint: could not create joint handle in family[1].  Joint not created.", return nullptr);
        }
    }

    TkJointImpl* joint = NVBLAST_NEW(TkJointImpl)(desc, nullptr);
    NVBLAST_CHECK_ERROR(joint != nullptr, "TkFrameworkImpl::createJoint: failed to create joint.", return nullptr);

    const TkJointData& jointData = joint->getDataInternal();

    if (handle0 != nullptr)
    {
        *handle0 = joint;
        static_cast<TkActorImpl*>(jointData.actors[0])->addJoint(joint->m_links[0]);
    }

    if (handle1 != nullptr)
    {
        *handle1 = joint;
        if (jointData.actors[0] != jointData.actors[1])
        {
            static_cast<TkActorImpl*>(jointData.actors[1])->addJoint(joint->m_links[1]);
        }
    }

    return joint;
}


void TkFrameworkImpl::onCreate(TkIdentifiable& object)
{
    const TkTypeImpl& type = static_cast<const TkTypeImpl&>(object.getType());

    const uint32_t index = type.getIndex();

    if (index >= m_objects.size())
    {
        if (!isInvalidIndex(index))
        {
            NVBLAST_LOG_ERROR("TkFrameworkImpl::addObject: object type unrecognized.");
        }
        return;
    }

    auto& objectArray = m_objects[index];
    NVBLAST_ASSERT(objectArray.find(&object) == objectArray.end());
    objectArray.pushBack(&object);
}


void TkFrameworkImpl::onDestroy(TkIdentifiable& object)
{
    // remove from id map if present
    const auto id = object.getID();
    if (!TkGUIDIsZero(&id))
    {
        m_IDToObject.erase(id);
    }

    // remove from object list
    const TkTypeImpl& type = static_cast<const TkTypeImpl&>(object.getType());

    const uint32_t index = type.getIndex();

    if (index >= m_objects.size())
    {
        if (!isInvalidIndex(index))
        {
            NVBLAST_LOG_ERROR("TkFrameworkImpl::removeObject: object type unrecognized.");
        }
        return;
    }

    auto& objectArray = m_objects[index];
    objectArray.findAndReplaceWithLast(&object);
}


void TkFrameworkImpl::onCreate(TkJointImpl& joint)
{
    NVBLAST_CHECK_ERROR(m_joints.insert(&joint), "TkFrameworkImpl::onCreate: Joint already tracked.", return);
}


void TkFrameworkImpl::onDestroy(TkJointImpl& joint)
{
    NVBLAST_CHECK_ERROR(m_joints.erase(&joint), "TkFrameworkImpl::onDestroy: Joint not tracked.", return);
}


void TkFrameworkImpl::onIDChange(TkIdentifiable& object, const NvBlastID& IDPrev, const NvBlastID& IDCurr)
{
    if (!TkGUIDIsZero(&IDPrev))
    {
        if (!m_IDToObject.erase(IDPrev))
        {
            NVBLAST_LOG_ERROR("TkFrameworkImpl::reportIDChanged: object with previous ID doesn't exist.");
        }
    }

    if (!TkGUIDIsZero(&IDCurr))
    {
        auto& value = m_IDToObject[IDCurr];
        if (value != nullptr)
        {
            NVBLAST_LOG_ERROR("TkFrameworkImpl::reportIDChanged: object with new ID already exists.");
            return;
        }
        value = &object;
    }
}

} // namespace Blast
} // namespace Nv


//////// Global API implementation ////////

Nv::Blast::TkFramework* NvBlastTkFrameworkCreate()
{
    if (Nv::Blast::TkFrameworkImpl::get() != nullptr)
    {
        NVBLAST_LOG_ERROR("TkFramework::create: framework already created.  Use TkFramework::get() to access.");
        return nullptr;
    }

    Nv::Blast::TkFrameworkImpl* framework = NVBLAST_NEW(Nv::Blast::TkFrameworkImpl) ();
    Nv::Blast::TkFrameworkImpl::set(framework);

    return Nv::Blast::TkFrameworkImpl::get();
}


Nv::Blast::TkFramework* NvBlastTkFrameworkGet()
{
    return Nv::Blast::TkFrameworkImpl::get();
}
