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

#include "NvBlastExtDamageAcceleratorAABBTree.h"
#include "NvBlastIndexFns.h"
#include "NvBlastAssert.h"
#include "NvVec4.h"
#include <algorithm>

using namespace nvidia;


namespace Nv
{
namespace Blast
{

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  Creation
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ExtDamageAcceleratorAABBTree* ExtDamageAcceleratorAABBTree::create(const NvBlastAsset* asset)
{
    ExtDamageAcceleratorAABBTree* tree = NVBLAST_NEW(Nv::Blast::ExtDamageAcceleratorAABBTree) ();
    tree->build(asset);
    return tree;
}


void ExtDamageAcceleratorAABBTree::release()
{
    NVBLAST_DELETE(this, ExtDamageAcceleratorAABBTree);
}


void ExtDamageAcceleratorAABBTree::build(const NvBlastAsset* asset)
{
    NVBLAST_ASSERT(m_root == nullptr);

    const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(asset, logLL);
    const NvBlastBond* bonds = NvBlastAssetGetBonds(asset, logLL);
    const NvBlastChunk* chunks = NvBlastAssetGetChunks(asset, logLL);
    const uint32_t N = NvBlastAssetGetBondCount(asset, logLL);

    m_indices.resizeUninitialized(N);
    m_points.resizeUninitialized(N);
    m_segments.resizeUninitialized(N);
    m_bonds.resizeUninitialized(N);
    m_nodes.reserve(2 * N);

    for (uint32_t node0 = 0; node0 < graph.nodeCount; ++node0)
    {
        for (uint32_t j = graph.adjacencyPartition[node0]; j < graph.adjacencyPartition[node0 + 1]; ++j)
        {
            uint32_t bondIndex = graph.adjacentBondIndices[j];
            uint32_t node1 = graph.adjacentNodeIndices[j];
            if (node0 < node1)
            {
                const NvBlastBond& bond = bonds[bondIndex];
                const NvVec3& p = (reinterpret_cast<const NvVec3&>(bond.centroid));
                m_points[bondIndex] = p;
                m_indices[bondIndex] = bondIndex;
                m_bonds[bondIndex].node0 = node0;
                m_bonds[bondIndex].node1 = node1;

                // filling bond segments as a connection of 2 chunk centroids
                const uint32_t chunk0 = graph.chunkIndices[node0];
                const uint32_t chunk1 = graph.chunkIndices[node1];
                if (isInvalidIndex(chunk1))
                {
                    // for world node we don't have it's centroid, so approximate with projection on bond normal 
                    m_segments[bondIndex].p0 = (reinterpret_cast<const NvVec3&>(chunks[chunk0].centroid));
                    const NvVec3 normal = (reinterpret_cast<const NvVec3&>(bond.normal));
                    m_segments[bondIndex].p1 = m_segments[bondIndex].p0 + normal * (p - m_segments[bondIndex].p0).dot(normal) * 2;

                }
                else
                {
                    m_segments[bondIndex].p0 = (reinterpret_cast<const NvVec3&>(chunks[chunk0].centroid));
                    m_segments[bondIndex].p1 = (reinterpret_cast<const NvVec3&>(chunks[chunk1].centroid));
                }
            }
        }

    }

    int rootIndex = N > 0 ? createNode(0, N - 1, 0) : -1;
    m_root = rootIndex >= 0 ? &m_nodes[rootIndex] : nullptr;
}

int ExtDamageAcceleratorAABBTree::createNode(uint32_t startIdx, uint32_t endIdx, uint32_t depth)
{
    if (startIdx > endIdx)
        return -1;

    Node node;
    node.first = startIdx;
    node.last = endIdx;

    // calc node bounds
    node.pointsBound = NvBounds3::empty();
    node.segmentsBound = NvBounds3::empty();
    for (uint32_t i = node.first; i <= node.last; i++)
    {
        const uint32_t idx = m_indices[i];
        node.pointsBound.include(m_points[idx]);
        node.segmentsBound.include(m_segments[idx].p0);
        node.segmentsBound.include(m_segments[idx].p1);
    }

    // select axis of biggest extent
    const NvVec3 ext = node.pointsBound.getExtents();
    uint32_t axis = 0;
    for (uint32_t k = 1; k < 3; k++)
    {
        if (ext[k] > ext[axis])
        {
            axis = k;
        }
    }

    // split on selected axis and partially sort around the middle
    const uint32_t mid = startIdx + (endIdx - startIdx) / 2;
    std::nth_element(m_indices.begin() + startIdx, m_indices.begin() + mid, m_indices.begin() + endIdx + 1, [&](uint32_t lhs, uint32_t rhs)
    {
        return m_points[lhs][axis] < m_points[rhs][axis];
    });

    const uint32_t BUCKET = 32;
    if (endIdx - startIdx > BUCKET && mid > startIdx && mid < endIdx)
    {
        node.child[0] = createNode(startIdx, mid, depth + 1);
        node.child[1] = createNode(mid + 1, endIdx, depth + 1);
    }
    else
    {
        node.child[0] = -1;
        node.child[1] = -1;
    }

    m_nodes.pushBack(node);

    return m_nodes.size() - 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                      Queries
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ExtDamageAcceleratorAABBTree::findInBounds(const nvidia::NvBounds3& bounds, ResultCallback& callback, bool segments) const
{
    if (m_root)
    {
        if (segments)
            findSegmentsInBounds(*m_root, callback, bounds);
        else
            findPointsInBounds(*m_root, callback, bounds);
        callback.dispatch();
    }
}

void ExtDamageAcceleratorAABBTree::findPointsInBounds(const Node& node, ResultCallback& callback, const nvidia::NvBounds3& bounds) const
{
    if (!bounds.intersects(node.pointsBound))
    {
        return;
    }

    // if search bound contains node bound, simply add all point indexes.
    if (node.pointsBound.isInside(bounds))
    {
        for (uint32_t i = node.first; i <= node.last; i++)
            pushResult(callback, m_indices[i]);
        return;  // early pruning.
    }

    if (node.child[0] < 0)
    {
        for (uint32_t i = node.first; i <= node.last; i++)
        {
            const uint32_t idx = m_indices[i];
            if (bounds.contains(m_points[idx]))
                pushResult(callback, idx);
        }

        return;
    }

    // check whether child nodes are in range.
    for (uint32_t c = 0; c < 2; ++c)
    {
        findPointsInBounds(m_nodes[node.child[c]], callback, bounds);
    }
}

void ExtDamageAcceleratorAABBTree::findSegmentsInBounds(const Node& node, ResultCallback& callback, const nvidia::NvBounds3& bounds) const
{
    if (!bounds.intersects(node.segmentsBound))
    {
        return;
    }

    // if search bound contains node bound, simply add all point indexes.
    if (node.segmentsBound.isInside(bounds))
    {
        for (uint32_t i = node.first; i <= node.last; i++)
            pushResult(callback, m_indices[i]);
        return;  // early pruning.
    }

    if (node.child[0] < 0)
    {
        for (uint32_t i = node.first; i <= node.last; i++)
        {
            const uint32_t idx = m_indices[i];
            if (bounds.contains(m_segments[idx].p0) || bounds.contains(m_segments[idx].p1))
                pushResult(callback, idx);
        }

        return;
    }

    // check whether child nodes are in range.
    for (uint32_t c = 0; c < 2; ++c)
    {
        findSegmentsInBounds(m_nodes[node.child[c]], callback, bounds);
    }
}

bool intersectSegmentPlane(const NvVec3& v1, const NvVec3& v2, const NvPlane& p)
{
    const bool s1 = p.distance(v1) > 0.f;
    const bool s2 = p.distance(v2) > 0.f;
    return (s1 && !s2) || (s2 && !s1);
}

bool intersectBoundsPlane(const NvBounds3& b, const NvPlane& p)
{
    const NvVec3 extents = b.getExtents();
    const NvVec3 center = b.getCenter();

    float r =  extents.x * NvAbs(p.n.x) + extents.y * NvAbs(p.n.y) + extents.z * NvAbs(p.n.z);
    float s = p.n.dot(center) + p.d;

    return NvAbs(s) <= r;
}

void ExtDamageAcceleratorAABBTree::findBondSegmentsPlaneIntersected(const nvidia::NvPlane& plane, ResultCallback& resultCallback) const
{
    if (m_root)
    {
        findSegmentsPlaneIntersected(*m_root, resultCallback, plane);
        resultCallback.dispatch();
    }
}

void ExtDamageAcceleratorAABBTree::findSegmentsPlaneIntersected(const Node& node, ResultCallback& callback, const nvidia::NvPlane& plane) const
{
    if (!intersectBoundsPlane(node.segmentsBound, plane))
    {
        return;
    }

    if (node.child[0] < 0)
    {
        for (uint32_t i = node.first; i <= node.last; i++)
        {
            const uint32_t idx = m_indices[i];
            if (intersectSegmentPlane(m_segments[idx].p0, m_segments[idx].p1, plane))
                pushResult(callback, idx);
        }

        return;
    }

    // check whether child nodes are in range.
    for (uint32_t c = 0; c < 2; ++c)
    {
        findSegmentsPlaneIntersected(m_nodes[node.child[c]], callback, plane);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  Debug Render
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline uint32_t NvVec4ToU32Color(const NvVec4& color)
{
    uint32_t c = 0;
    c |= (int)(color.w * 255); c <<= 8;
    c |= (int)(color.z * 255); c <<= 8;
    c |= (int)(color.y * 255); c <<= 8;
    c |= (int)(color.x * 255);
    return c;
}

Nv::Blast::DebugBuffer ExtDamageAcceleratorAABBTree::fillDebugRender(int depth, bool segments)
{
    Nv::Blast::DebugBuffer debugBuffer = { nullptr, 0 };

    m_debugLineBuffer.clear();

    if (m_root)
    {
        fillDebugBuffer(*m_root, 0, depth, segments);
    }

    debugBuffer.lines = m_debugLineBuffer.begin();
    debugBuffer.lineCount = m_debugLineBuffer.size();

    return debugBuffer;
}

void ExtDamageAcceleratorAABBTree::fillDebugBuffer(const Node& node, int currentDepth, int depth, bool segments)
{
    if (depth < 0 || currentDepth == depth)
    {
        const NvVec4 LEAF_COLOR(1.0f, 1.0f, 1.0f, 1.0f);
        const NvVec4 NON_LEAF_COLOR(0.3f, 0.3f, 0.3f, 1.0f);

        // draw box
        const NvBounds3 bounds = segments ? node.segmentsBound : node.pointsBound;
        const NvVec3 center = bounds.getCenter();
        const NvVec3 extents = bounds.getExtents();

        const int vs[] = { 0,3,5,6 };
        for (int i = 0; i < 4; i++)
        {
            int v = vs[i];
            for (int d = 1; d < 8; d <<= 1)
            {
                auto flip = [](int x, int k) { return ((x >> k) & 1) * 2.f - 1.f; };
                const float s = std::pow(0.99f, currentDepth);
                NvVec3 p0 = center + s * extents.multiply(NvVec3(flip(v, 0), flip(v, 1), flip(v, 2)));
                NvVec3 p1 = center + s * extents.multiply(NvVec3(flip(v^d, 0), flip(v^d, 1), flip(v^d, 2)));
                m_debugLineBuffer.pushBack(Nv::Blast::DebugLine(
                    reinterpret_cast<NvcVec3&>(p0), 
                    reinterpret_cast<NvcVec3&>(p1), 
                    NvVec4ToU32Color(LEAF_COLOR * (1.f - (currentDepth + 1) * 0.1f)))
                );
            }
        }
    }

    for (uint32_t i = 0; i < 2; ++i)
    {
        if (node.child[i] >= 0)
        {
            fillDebugBuffer(m_nodes[node.child[i]], currentDepth + 1, depth, segments);
        }
    }
}


} // namespace Blast
} // namespace Nv
