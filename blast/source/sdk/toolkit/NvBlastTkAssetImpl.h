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


#ifndef NVBLASTTKASSETIMPL_H
#define NVBLASTTKASSETIMPL_H


#include "NvBlastTkCommon.h"
#include "NvBlastTkJoint.h"
#include "NvBlastTkAsset.h"
#include "NvBlastTkTypeImpl.h"
#include "NvBlastArray.h"


// Forward declarations
struct NvBlastAsset;


namespace Nv
{
namespace Blast
{

/**
Implementation of TkAsset
*/
NVBLASTTK_IMPL_DECLARE(Asset)
{
public:
    TkAssetImpl();
    TkAssetImpl(const NvBlastID& id);
    ~TkAssetImpl();

    NVBLASTTK_IMPL_DEFINE_IDENTIFIABLE('A', 'S', 'S', 'T');

    // Public methods

    /**
    Factory create method.  This method creates a low-level asset and stores a reference to it.

    \param[in]  desc    Asset descriptor set by the user.

    \return a pointer to a new TkAssetImpl object if successful, NULL otherwise.
    */
    static TkAssetImpl*                 create(const TkAssetDesc& desc);
    
    /**
    Static method to create an asset from an existing low-level asset.

    \param[in]  assetLL         A valid low-level asset passed in by the user.
    \param[in]  jointDescs      Optional joint descriptors to add to the new asset.
    \param[in]  jointDescCount  The number of joint descriptors in the jointDescs array.  If non-zero, jointDescs cannot be NULL.
    \param[in]  ownsAsset       Whether or not to let this TkAssetImpl object release the low-level NvBlastAsset memory upon its own release.

    \return a pointer to a new TkAssetImpl object if successful, NULL otherwise.
    */
    static TkAssetImpl*                 create(const NvBlastAsset* assetLL, Nv::Blast::TkAssetJointDesc* jointDescs = nullptr, uint32_t jointDescCount = 0, bool ownsAsset = false);

    /**
    \return a pointer to the underlying low-level NvBlastAsset associated with this asset.
    */
    const NvBlastAsset*                 getAssetLLInternal() const;

    /**
    \return the number of internal joint descriptors stored with this asset.
    */
    uint32_t                            getJointDescCountInternal() const;

    /**
    \return the array of internal joint descriptors stored with this asset, with size given by getJointDescCountInternal().
    */
    const TkAssetJointDesc*             getJointDescsInternal() const;

    // Begin TkAsset
    virtual const NvBlastAsset*         getAssetLL() const override;

    virtual uint32_t                    getChunkCount() const override;

    virtual uint32_t                    getLeafChunkCount() const override;

    virtual uint32_t                    getBondCount() const override;

    virtual const NvBlastChunk*         getChunks() const override;

    virtual const NvBlastBond*          getBonds() const override;

    virtual const NvBlastSupportGraph   getGraph() const override;

    virtual uint32_t                    getDataSize() const override;

    virtual uint32_t                    getJointDescCount() const override;

    virtual const TkAssetJointDesc*     getJointDescs() const override;
    // End TkAsset

private:
    /**
    Utility to add a joint descriptor between the indexed chunks.  The two chunks
    must be support chunks, and there must exist a bond between them.  The joint's
    attachment positions will be the bond centroid.

    \param[in]  chunkIndex0 The first chunk index.
    \param[in]  chunkIndex1 The second chunk index.

    \return true iff successful.
    */
    bool                                addJointDesc(uint32_t chunkIndex0, uint32_t chunkIndex1);

    NvBlastAsset*                   m_assetLL;      //!< The underlying low-level asset.
    Array<TkAssetJointDesc>::type   m_jointDescs;   //!< The array of internal joint descriptors.
    bool                            m_ownsAsset;    //!< Whether or not this asset should release its low-level asset upon its own release.
};


//////// TkAssetImpl inline methods ////////

NV_INLINE const NvBlastAsset* TkAssetImpl::getAssetLLInternal() const
{
    return m_assetLL;
}


NV_INLINE uint32_t TkAssetImpl::getJointDescCountInternal() const
{
    return m_jointDescs.size();
}


NV_INLINE const TkAssetJointDesc* TkAssetImpl::getJointDescsInternal() const
{
    return m_jointDescs.begin();
}

} // namespace Blast
} // namespace Nv


#endif // ifndef NVBLASTTKASSETIMPL_H
