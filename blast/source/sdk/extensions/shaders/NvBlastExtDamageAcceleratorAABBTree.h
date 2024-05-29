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

#include "NvBlastExtDamageAcceleratorInternal.h"
#include "NvBlast.h"
#include "NvBlastArray.h"


namespace Nv
{
namespace Blast
{

class ExtDamageAcceleratorAABBTree final : public ExtDamageAcceleratorInternal
{
public:
    //////// ctor ////////

    ExtDamageAcceleratorAABBTree() :
         m_root(nullptr)
    {
    }

    virtual ~ExtDamageAcceleratorAABBTree()
    {
    }

    static ExtDamageAcceleratorAABBTree* create(const NvBlastAsset* asset);


    //////// interface ////////

    virtual void release() override;

    virtual void findBondCentroidsInBounds(const nvidia::NvBounds3& bounds, ResultCallback& resultCallback) const override
    {
        const_cast<ExtDamageAcceleratorAABBTree*>(this)->findInBounds(bounds, resultCallback, false);
    }

    virtual void findBondSegmentsInBounds(const nvidia::NvBounds3& bounds, ResultCallback& resultCallback) const override
    {
        const_cast<ExtDamageAcceleratorAABBTree*>(this)->findInBounds(bounds, resultCallback, true);

    }

    virtual void findBondSegmentsPlaneIntersected(const nvidia::NvPlane& plane, ResultCallback& resultCallback) const override;

    virtual Nv::Blast::DebugBuffer fillDebugRender(int depth, bool segments) override;

    virtual void* getImmediateScratch(size_t size) override
    {
        m_scratch.resizeUninitialized(size);
        return m_scratch.begin();
    }


private:
    // no copy/assignment
    ExtDamageAcceleratorAABBTree(ExtDamageAcceleratorAABBTree&);
    ExtDamageAcceleratorAABBTree& operator=(const ExtDamageAcceleratorAABBTree& tree);

    // Tree node 
    struct Node
    {
        int child[2];
        uint32_t first;
        uint32_t last;
        nvidia::NvBounds3 pointsBound;
        nvidia::NvBounds3 segmentsBound;
    };


    void build(const NvBlastAsset* asset);

    int createNode(uint32_t startIdx, uint32_t endIdx, uint32_t depth);

    void pushResult(ResultCallback& callback, uint32_t pointIndex) const
    {
        callback.push(pointIndex, m_bonds[pointIndex].node0, m_bonds[pointIndex].node1);
    }

    void findInBounds(const nvidia::NvBounds3& bounds, ResultCallback& callback, bool segments) const;

    void findPointsInBounds(const Node& node, ResultCallback& callback, const nvidia::NvBounds3& bounds) const;

    void findSegmentsInBounds(const Node& node, ResultCallback& callback, const nvidia::NvBounds3& bounds) const;

    void findSegmentsPlaneIntersected(const Node& node, ResultCallback& callback, const nvidia::NvPlane& plane) const;

    void fillDebugBuffer(const Node& node, int currentDepth, int depth, bool segments);


    //////// data ////////

    Node*                                 m_root;
    Array<Node>::type                     m_nodes;
    Array<uint32_t>::type                 m_indices;

    Array<nvidia::NvVec3>::type            m_points;

    struct Segment
    {
        nvidia::NvVec3   p0;
        nvidia::NvVec3   p1;
    };
    Array<Segment>::type                  m_segments;

    struct BondData
    {
        uint32_t        node0;
        uint32_t        node1;
    };
    Array<BondData>::type                 m_bonds;

    Array<Nv::Blast::DebugLine>::type     m_debugLineBuffer;

    Array<char>::type                     m_scratch;
};


} // namespace Blast
} // namespace Nv
