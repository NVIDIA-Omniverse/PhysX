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

#pragma once

#include "NvBlastExtDamageShaders.h"
#include "NvBounds3.h"


namespace Nv
{
namespace Blast
{


class ExtDamageAcceleratorInternal : public NvBlastExtDamageAccelerator
{
public:
    struct QueryBondData
    {
        uint32_t bond;
        uint32_t node0;
        uint32_t node1;
    };

    class ResultCallback
    {
    public:
        ResultCallback(QueryBondData* buffer, uint32_t count) :
            m_bondBuffer(buffer), m_bondMaxCount(count), m_bondCount(0) {}

        virtual void processResults(const QueryBondData* bondBuffer, uint32_t count) = 0;

        void push(uint32_t bond, uint32_t node0, uint32_t node1)
        {
            m_bondBuffer[m_bondCount].bond = bond;
            m_bondBuffer[m_bondCount].node0 = node0;
            m_bondBuffer[m_bondCount].node1 = node1;
            m_bondCount++;
            if (m_bondCount == m_bondMaxCount)
            {
                dispatch();
            }
        }

        void dispatch()
        {
            if (m_bondCount)
            {
                processResults(m_bondBuffer, m_bondCount);
                m_bondCount = 0;
            }
        }
        
    private:
        QueryBondData* m_bondBuffer;
        uint32_t       m_bondMaxCount;

        uint32_t       m_bondCount;
    };

    virtual void findBondCentroidsInBounds(const nvidia::NvBounds3& bounds, ResultCallback& resultCallback) const = 0;
    virtual void findBondSegmentsInBounds(const nvidia::NvBounds3& bounds, ResultCallback& resultCallback) const = 0;
    virtual void findBondSegmentsPlaneIntersected(const nvidia::NvPlane& plane, ResultCallback& resultCallback) const = 0;

    // Non-thread safe! Multiple calls return the same memory.
    virtual void* getImmediateScratch(size_t size) = 0;
};


} // namespace Blast
} // namespace Nv
