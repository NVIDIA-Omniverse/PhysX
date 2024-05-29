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


#ifndef NVBLASTTKFRAMEWORKIMPL_H
#define NVBLASTTKFRAMEWORKIMPL_H

#include "NvBlastTkFramework.h"
#include "NvBlastInternalProfiler.h"

#include "NvBlastTkCommon.h"

#include "NvBlastArray.h"
#include "NvBlastHashMap.h"
#include "NvBlastHashSet.h"


namespace Nv
{
namespace Blast
{

// Forward declarations
class TkTypeImpl;
class TkJointImpl;

/**
Implementation of TkFramework
*/
class TkFrameworkImpl : public TkFramework
{
public:
                                        TkFrameworkImpl();
                                        ~TkFrameworkImpl();

    // Begin TkFramework
    virtual void                        release() override;

    virtual const TkType*               getType(TkTypeIndex::Enum typeIndex) const override;

    virtual TkIdentifiable*             findObjectByID(const NvBlastID& id) const override;

    virtual uint32_t                    getObjectCount(const TkType& type) const override;

    virtual uint32_t                    getObjects(TkIdentifiable** buffer, uint32_t bufferSize, const TkType& type, uint32_t indexStart = 0) const override;

    virtual bool                        reorderAssetDescChunks(NvBlastChunkDesc* chunkDescs, uint32_t chunkCount, NvBlastBondDesc* bondDescs, uint32_t bondCount, uint32_t* chunkReorderMap = nullptr, bool keepBondNormalChunkOrder = false) const override;

    virtual bool                        ensureAssetExactSupportCoverage(NvBlastChunkDesc* chunkDescs, uint32_t chunkCount) const override;

    virtual TkAsset*                    createAsset(const TkAssetDesc& desc) override;

    virtual TkAsset*                    createAsset(const NvBlastAsset* assetLL, Nv::Blast::TkAssetJointDesc* jointDescs = nullptr, uint32_t jointDescCount = 0, bool ownsAsset = false) override;

    virtual TkGroup*                    createGroup(const TkGroupDesc& desc) override;

    virtual TkActor*                    createActor(const TkActorDesc& desc) override;

    virtual TkJoint*                    createJoint(const TkJointDesc& desc) override;
    // End TkFramework

    // Public methods
    /**
    To be called by any TkIdentifiable object when it is created, so the framework can track it.
    */
    void                                onCreate(TkIdentifiable& object);

    /**
    To be called by any TkIdentifiable object when it is deleted, so the framework can stop tracking it.
    */
    void                                onDestroy(TkIdentifiable& object);

    /**
    Special onCreate method for joints, since they are not TkIdentifiable.
    */
    void                                onCreate(TkJointImpl& joint);

    /**
    Special onDestroy method for joints, since they are not TkIdentifiable.
    */
    void                                onDestroy(TkJointImpl& joint);

    /**
    Must be called whenever a TkIdentifiable object's ID is changed, so that the framework can associate the new ID with it.
    */
    void                                onIDChange(TkIdentifiable& object, const NvBlastID& IDPrev, const NvBlastID& IDCurr);

    /**
    Internal (non-virtual) method to find a TkIdentifiable object based upon its NvBlastID.
    */
    TkIdentifiable*                     findObjectByIDInternal(const NvBlastID& id) const;

    // Access to singleton

    /** Retrieve the global singleton. */
    static TkFrameworkImpl*             get();

    /** Set the global singleton, if it's not already set, or set it to NULL.  Returns true iff successful. */
    static bool                         set(TkFrameworkImpl* framework);

private:
    // Enums
    enum { ClassID = NVBLAST_FOURCC('T', 'K', 'F', 'W') };  //!< TkFramework identifier token, used in serialization

    // Static data
    static TkFrameworkImpl*                                                     s_framework;            //!< Global (singleton) object pointer

    // Types
    InlineArray<const TkTypeImpl*, TkTypeIndex::TypeCount>::type                m_types;                //!< TkIdentifiable static type data
    HashMap<uint32_t, uint32_t>::type                                           m_typeIDToIndex;        //!< Map to type data keyed by ClassID

    // Objects and object names
    HashMap<NvBlastID, TkIdentifiable*>::type                                   m_IDToObject;           //!< Map to all TkIdentifiable objects, keyed by NvBlastID
    InlineArray<Array<TkIdentifiable*>::type, TkTypeIndex::TypeCount>::type     m_objects;              //!< Catalog of all TkIdentifiable objects, grouped by type.  (Revisit implementation.)

    // Track external joints (to do: make this a pool)
    HashSet<TkJointImpl*>::type                                                 m_joints;               //!< All internal joints
};


//////// TkFrameworkImpl inline methods ////////

NV_INLINE TkIdentifiable* TkFrameworkImpl::findObjectByIDInternal(const NvBlastID& id) const
{
    const auto entry = m_IDToObject.find(id);
    if (entry == nullptr)
    {
        return nullptr;
    }

    return entry->second;
}

} // namespace Blast
} // namespace Nv


#endif // ifndef NVBLASTTKFRAMEWORKIMPL_H
