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


#ifndef NVBLASTTKCOMMON_H
#define NVBLASTTKCOMMON_H


#include "NvBlastGlobals.h"
#include "NvBlastTkGUID.h"


// Macro to define standard object classes.  An intermediate class is defined which holds common implementations.
#define NVBLASTTK_IMPL_DECLARE(_name)                                               \
class Tk##_name##Type : public Tk##_name                                            \
{                                                                                   \
public:                                                                             \
    /* Blank constructor generates a new NvBlastID and informs framework  */        \
    Tk##_name##Type()                                                               \
    {                                                                               \
        memset(&m_ID, 0, sizeof(NvBlastID));                                        \
        setID(TkGenerateGUID(this));                                                \
        TkFrameworkImpl::get()->onCreate(*this);                                    \
    }                                                                               \
                                                                                    \
    /* This constructor takes an existing NvBlastID and informs framework  */       \
    Tk##_name##Type(const NvBlastID& id)                                            \
    {                                                                               \
        memset(&m_ID, 0, sizeof(NvBlastID));                                        \
        setID(id);                                                                  \
        TkFrameworkImpl::get()->onCreate(*this);                                    \
    }                                                                               \
                                                                                    \
    /* Destructor informs framework  */                                             \
    ~Tk##_name##Type()  { TkFrameworkImpl::get()->onDestroy(*this); }               \
                                                                                    \
    /* Begin TkIdentifiable */                                                      \
    virtual void                setID(const NvBlastID& id) override                 \
    {                                                                               \
        /* Inform framework of ID change */                                         \
        TkFrameworkImpl::get()->onIDChange(*this, m_ID, id);                        \
        m_ID = id;                                                                  \
    }                                                                               \
    virtual const NvBlastID&    getID() const override { return getIDInternal(); }  \
    virtual const TkType&       getType() const override { return s_type; }         \
    /* End TkIdentifiable */                                                        \
                                                                                    \
    /* Begin public API */                                                          \
                                                                                    \
    /* Inline method for internal access to NvBlastID */                            \
    const  NvBlastID&           getIDInternal() const { return m_ID; }              \
                                                                                    \
    /* End public API */                                                            \
                                                                                    \
    /* Static type information */                                                   \
    static TkTypeImpl   s_type;                                                     \
                                                                                    \
private:                                                                            \
    NvBlastID           m_ID;   /* NvBlastID for a TkIdentifiable object */         \
};                                                                                  \
                                                                                    \
/* Derive object implementation from common implementation class above */           \
class Tk##_name##Impl final : public Tk##_name##Type


// Macro to declare standard object interfaces, enums, etc.
#define NVBLASTTK_IMPL_DEFINE_IDENTIFIABLE(_id0, _id1, _id2, _id3)      \
    /* Begin TkObject */                                                \
    virtual void        release() override;                             \
    /* End TkObject */                                                  \
                                                                        \
    /* Enums */                                                         \
                                                                        \
    /* Generate a ClassID enum used to identify this TkIdentifiable. */ \
    enum { ClassID = NVBLAST_FOURCC(_id0, _id1, _id2, _id3) }


// Macro to define class type data
#define NVBLASTTK_DEFINE_TYPE_IDENTIFIABLE(_name) \
    TkTypeImpl Tk##_name##Type::s_type("Tk" #_name, Tk##_name##Impl::ClassID, 0)


#endif // ifndef NVBLASTTKCOMMON_H
