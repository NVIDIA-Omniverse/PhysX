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


#ifndef NVBLASTTKTYPEIMPL_H
#define NVBLASTTKTYPEIMPL_H


#include "NvPreprocessor.h"

#include "NvBlastTkType.h"


namespace Nv
{
namespace Blast
{

/**
Implementation of TkType, storing class information for TkIdentifiable-derived classes.
*/
class TkTypeImpl : public TkType
{
public:
    TkTypeImpl(const char* typeName, uint32_t typeID, uint32_t version);

    // Begin TkType
    virtual const char* getName() const override { return getNameInternal(); }

    virtual uint32_t    getVersion() const override { return getVersionInternal(); }
    // End TkType

    // Public methods

    /**
    Access to the class name.

    \return a C string pointer to the class name.
    */
    const char*         getNameInternal() const;

    /**
    Access to the data format version for the class.

    \return the data format version.
    */
    uint32_t            getVersionInternal() const;

    /**
    Access to a unique identifier for the class (set using the NVBLASTTK_IMPL_DEFINE_IDENTIFIABLE macro).

    \return the class's unique identifier.
    */
    uint32_t            getID() const;

    /**
    Access to a runtime-unique small index for the class.

    \return the index for the class.
    */
    uint32_t            getIndex() const;

    /**
    \return whether or not the index has been set (see setIndex) to a valid value.
    */
    bool                indexIsValid() const;

private:
    enum { InvalidIndex = 0xFFFFFFFF };

    /**
    Sets the type index.

    \param[in]  index   The index to set.
    */
    void                setIndex(uint32_t index);

    const char*     m_name;             //!<    The name of the class, set by the constructor.
    uint32_t        m_ID;               //!<    The unique identifier for the class, set by the constructor. 
    uint32_t        m_version;          //!<    The data format version for the class, set by the constructor.
    uint32_t        m_index;            //!<    The index set for this class, set using setIndex().

    friend class TkFrameworkImpl;
};


//////// TkTypeImpl inline methods ////////

NV_INLINE TkTypeImpl::TkTypeImpl(const char* typeName, uint32_t typeID, uint32_t version)
    : m_name(typeName)
    , m_ID(typeID)
    , m_version(version)
    , m_index((uint32_t)InvalidIndex)
{
}


NV_INLINE const char* TkTypeImpl::getNameInternal() const
{
    return m_name;
}


NV_INLINE uint32_t TkTypeImpl::getVersionInternal() const
{
    return m_version;
}


NV_INLINE uint32_t TkTypeImpl::getID() const
{
    return m_ID;
}


NV_INLINE uint32_t TkTypeImpl::getIndex() const
{
    return m_index;
}


NV_INLINE bool TkTypeImpl::indexIsValid() const
{
    return m_index != (uint32_t)InvalidIndex;
}


NV_INLINE void TkTypeImpl::setIndex(uint32_t index)
{
    m_index = index;
}

} // namespace Blast
} // namespace Nv


#endif // ifndef NVBLASTTKTYPEIMPL_H
