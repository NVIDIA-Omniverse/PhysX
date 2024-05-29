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


#include "NvBlastExtAuthoringAcceleratorImpl.h"
#include "NvBlastExtAuthoringMesh.h"
#include "NvBlastExtAuthoringInternalCommon.h"
#include "NvBlastGlobals.h"
#include "NvBlastNvSharedHelpers.h"
#include "NvCMath.h"

namespace Nv
{
namespace Blast
{

DummyAccelerator::DummyAccelerator(int32_t count) : m_count(count)
{
    m_current = 0;
}

void DummyAccelerator::release()
{
    NVBLAST_DELETE(this, DummyAccelerator);
}

void DummyAccelerator::setState(const Vertex* pos, const Edge* ed, const Facet& fc)
{
    m_current = 0;
    NV_UNUSED(pos);
    NV_UNUSED(ed);
    NV_UNUSED(fc);
}

void DummyAccelerator::setState(const NvcBounds3* bound) {
    m_current = 0;
    NV_UNUSED(bound);
}

void DummyAccelerator::setState(const NvcVec3& point) {
    m_current = 0;
    NV_UNUSED(point);
}

int32_t DummyAccelerator::getNextFacet()
{
    if (m_current < m_count)
    {
        ++m_current;
        return m_current - 1;
    }
    else
        return -1;
}


Grid::Grid(int32_t resolution) : m_resolution(resolution)
{
    /**
        Set up 3d grid
    */
    m_r3 = resolution * resolution * resolution;
    m_spatialMap.resize(resolution * resolution * resolution);
}

void Grid::release()
{
    NVBLAST_DELETE(this, Grid);
}

void Grid::setMesh(const Mesh* m)
{
    nvidia::NvBounds3 bd = toNvShared(m->getBoundingBox());
    m_mappedFacetCount  = m->getFacetCount();
    bd.fattenFast(0.001f);
    m_spos = fromNvShared(bd.minimum);
    m_deltas = { m_resolution / bd.getDimensions().x, m_resolution / bd.getDimensions().y,
                 m_resolution / bd.getDimensions().z };

    for (int32_t i = 0; i < m_r3; ++i)
        m_spatialMap[i].clear();

    const float ofs = 0.001f;

    for (uint32_t fc = 0; fc < m->getFacetCount(); ++fc)
    {
        NvcBounds3 cfc = *m->getFacetBound(fc);

        int32_t is = (int32_t)std::max(0.f, (cfc.minimum.x - m_spos.x - ofs) * m_deltas.x);
        int32_t ie = (int32_t)std::max(0.f, (cfc.maximum.x - m_spos.x + ofs) * m_deltas.x);

        int32_t js = (int32_t)std::max(0.f, (cfc.minimum.y - m_spos.y - ofs) * m_deltas.y);
        int32_t je = (int32_t)std::max(0.f, (cfc.maximum.y - m_spos.y + ofs) * m_deltas.y);

        int32_t ks = (int32_t)std::max(0.f, (cfc.minimum.z - m_spos.z - ofs) * m_deltas.z);
        int32_t ke = (int32_t)std::max(0.f, (cfc.maximum.z - m_spos.z + ofs) * m_deltas.z);

        for (int32_t i = is; i < m_resolution && i <= ie; ++i)
        {
            for (int32_t j = js; j < m_resolution && j <= je; ++j)
            {
                for (int32_t k = ks; k < m_resolution && k <= ke; ++k)
                {
                    m_spatialMap[(i * m_resolution + j) * m_resolution + k].push_back(fc);
                }
            }
        }
    }
}


GridAccelerator::GridAccelerator(Grid* grd)
{
    m_grid = grd;
    m_alreadyGotValue = 0;
    m_alreadyGotFlag.resize(1 << 12);
    m_cellList.resize(1 << 12);
    m_pointCmdDir = 0;
}

void GridAccelerator::release()
{
    NVBLAST_DELETE(this, GridAccelerator);
}

void GridAccelerator::setState(const Vertex* pos, const Edge* ed, const Facet& fc)
{
    
    nvidia::NvBounds3 cfc(nvidia::NvBounds3::empty());

    for (uint32_t v = 0; v < fc.edgesCount; ++v)
    {
        cfc.include(toNvShared(pos[ed[fc.firstEdgeNumber + v].s].p));
        cfc.include(toNvShared(pos[ed[fc.firstEdgeNumber + v].e].p));
    }
    setState(&fromNvShared(cfc));
}

void GridAccelerator::setState(const NvcBounds3* facetBounding)
{
    m_alreadyGotValue++;
    m_iteratorCell = -1;
    m_iteratorFacet = -1;
    m_gotCells = 0;

    NvcBounds3 cfc = *facetBounding;

    int32_t is = (int32_t)std::max(0.f, (cfc.minimum.x - m_grid->m_spos.x - 0.001f) * m_grid->m_deltas.x);
    int32_t ie = (int32_t)std::max(0.f, (cfc.maximum.x - m_grid->m_spos.x + 0.001f) * m_grid->m_deltas.x);

    int32_t js = (int32_t)std::max(0.f, (cfc.minimum.y - m_grid->m_spos.y - 0.001f) * m_grid->m_deltas.y);
    int32_t je = (int32_t)std::max(0.f, (cfc.maximum.y - m_grid->m_spos.y + 0.001f) * m_grid->m_deltas.y);

    int32_t ks = (int32_t)std::max(0.f, (cfc.minimum.z - m_grid->m_spos.z - 0.001f) * m_grid->m_deltas.z);
    int32_t ke = (int32_t)std::max(0.f, (cfc.maximum.z - m_grid->m_spos.z + 0.001f) * m_grid->m_deltas.z);

    for (int32_t i = is; i < m_grid->m_resolution && i <= ie; ++i)
    {
        for (int32_t j = js; j < m_grid->m_resolution && j <= je; ++j)
        {
            for (int32_t k = ks; k < m_grid->m_resolution && k <= ke; ++k)
            {
                int32_t id = (i * m_grid->m_resolution + j) * m_grid->m_resolution + k;
                if (!m_grid->m_spatialMap[id].empty())
                {
                    m_cellList[m_gotCells++] = id;
                }

            }
        }
    }
    if (m_gotCells != 0)
    {
        m_iteratorFacet = 0;
        m_iteratorCell = m_cellList[m_gotCells - 1];
        m_gotCells--;
    }
}

void GridAccelerator::setPointCmpDirection(int32_t d)
{
    m_pointCmdDir = d;
}

void GridAccelerator::setState(const NvcVec3& point)
{
    m_alreadyGotValue++;
    m_iteratorCell = -1;
    m_iteratorFacet = -1;
    m_gotCells = 0;
    
    int32_t is = (int32_t)std::max(0.f, (point.x - m_grid->m_spos.x - 0.001f) * m_grid->m_deltas.x);
    int32_t ie = (int32_t)std::max(0.f, (point.x - m_grid->m_spos.x + 0.001f) * m_grid->m_deltas.x);

    int32_t js = (int32_t)std::max(0.f, (point.y - m_grid->m_spos.y - 0.001f) * m_grid->m_deltas.y);
    int32_t je = (int32_t)std::max(0.f, (point.y - m_grid->m_spos.y + 0.001f) * m_grid->m_deltas.y);

    int32_t ks = 0;
    int32_t ke = m_grid->m_resolution;
    switch (m_pointCmdDir)
    {
    case 1:
        ks = (int32_t)std::max(0.f, (point.z - m_grid->m_spos.z - 0.001f) * m_grid->m_deltas.z);
        break;
    case -1:
        ke = (int32_t)std::max(0.f, (point.z - m_grid->m_spos.z + 0.001f) * m_grid->m_deltas.z);
    }

    for (int32_t i = is; i < m_grid->m_resolution && i <= ie; ++i)
    {
        for (int32_t j = js; j < m_grid->m_resolution && j <= je; ++j)
        {
            for (int32_t k = ks; k <= ke && k < m_grid->m_resolution; ++k)
            {
                int32_t id = (i * m_grid->m_resolution + j) * m_grid->m_resolution + k;
                if (!m_grid->m_spatialMap[id].empty())
                {
                    m_cellList[m_gotCells++] = id;
                }
            }
        }
    }

    if (m_gotCells != 0)
    {
        m_iteratorFacet = 0;
        m_iteratorCell = m_cellList[m_gotCells - 1];
        m_gotCells--;
    }
}

int32_t GridAccelerator::getNextFacet()
{
    int32_t facetId = -1;

    while (m_iteratorCell != -1)
    {
        if (m_iteratorFacet >= (int32_t)m_grid->m_spatialMap[m_iteratorCell].size())
        {
            if (m_gotCells != 0)
            {
                m_iteratorCell = m_cellList[m_gotCells - 1];
                m_gotCells--;
                m_iteratorFacet = 0;
            }
            else
            {
                m_iteratorCell = -1;
                break;
            }
        }
        if (m_alreadyGotFlag[m_grid->m_spatialMap[m_iteratorCell][m_iteratorFacet]] != m_alreadyGotValue)
        {
            facetId = m_grid->m_spatialMap[m_iteratorCell][m_iteratorFacet];
            m_iteratorFacet++;
            break;
        }
        else
        {
            m_iteratorFacet++;
        }
    }
    if (facetId != -1)
    {
        m_alreadyGotFlag[facetId] = m_alreadyGotValue;
    }
    return facetId;
}


BBoxBasedAccelerator::BBoxBasedAccelerator(const Mesh* mesh, int32_t resolution) : m_resolution(resolution), m_alreadyGotValue(1)
{
    m_bounds = mesh->getBoundingBox();
    m_spatialMap.resize(resolution * resolution * resolution);
    m_cells.resize(resolution * resolution * resolution);
    int32_t currentCell = 0;
    NvcVec3 incr = (m_bounds.maximum - m_bounds.minimum) * (1.0f / m_resolution);
    for (int32_t z = 0; z < resolution; ++z)
    {
        for (int32_t y = 0; y < resolution; ++y)
        {
            for (int32_t x = 0; x < resolution; ++x)
            {
                m_cells[currentCell].minimum.x = m_bounds.minimum.x + x * incr.x;
                m_cells[currentCell].minimum.y = m_bounds.minimum.y + y * incr.y;
                m_cells[currentCell].minimum.z = m_bounds.minimum.z + z * incr.z;

                m_cells[currentCell].maximum.x = m_bounds.minimum.x + (x + 1) * incr.x;
                m_cells[currentCell].maximum.y = m_bounds.minimum.y + (y + 1) * incr.y;
                m_cells[currentCell].maximum.z = m_bounds.minimum.z + (z + 1) * incr.z;

                ++currentCell;
            }
        }
    }
    m_cellList.resize(1 << 16);
    m_gotCells = 0;
    buildAccelStructure(mesh->getVertices(), mesh->getEdges(), mesh->getFacetsBuffer(), mesh->getFacetCount());
}

void BBoxBasedAccelerator::release()
{
    NVBLAST_DELETE(this, BBoxBasedAccelerator);
}

BBoxBasedAccelerator::~BBoxBasedAccelerator()
{
    m_resolution = 0;
    toNvShared(m_bounds).setEmpty();
    m_spatialMap.clear();
    m_cells.clear();
    m_cellList.clear();
}

int32_t BBoxBasedAccelerator::getNextFacet()
{
    int32_t facetId = -1;

    while (m_iteratorCell != -1)
    {
        if (m_iteratorFacet >= (int32_t)m_spatialMap[m_iteratorCell].size())
        {
            if (m_gotCells != 0)
            {
                m_iteratorCell = m_cellList[m_gotCells - 1];
                m_gotCells--;
                m_iteratorFacet = 0;
            }
            else
            {
                m_iteratorCell = -1;
                break;
            }
        }
        if (m_alreadyGotFlag[m_spatialMap[m_iteratorCell][m_iteratorFacet]] != m_alreadyGotValue)
        {
            facetId = m_spatialMap[m_iteratorCell][m_iteratorFacet];
            m_iteratorFacet++;
            break;
        }
        else
        {
            m_iteratorFacet++;
        }
    }
    if (facetId != -1)
    {
        m_alreadyGotFlag[facetId] = m_alreadyGotValue;
    }
    return facetId;
}

void BBoxBasedAccelerator::setState(const Vertex* pos, const Edge* ed, const Facet& fc)
{

    nvidia::NvBounds3 cfc(nvidia::NvBounds3::empty());

    for (uint32_t v = 0; v < fc.edgesCount; ++v)
    {
        cfc.include(toNvShared(pos[ed[fc.firstEdgeNumber + v].s].p));
        cfc.include(toNvShared(pos[ed[fc.firstEdgeNumber + v].e].p));
    }
    setState(&fromNvShared(cfc));
}

void BBoxBasedAccelerator::setState(const NvcBounds3* facetBox)
{
    m_alreadyGotValue++;
    m_iteratorCell = -1;
    m_iteratorFacet = -1;
    m_gotCells = 0;
    
    for (uint32_t i = 0; i < m_cells.size(); ++i)
    {
        if (weakBoundingBoxIntersection(toNvShared(m_cells[i]), *toNvShared(facetBox)))
        {
            if (!m_spatialMap[i].empty())
                m_cellList[m_gotCells++] = i;
        }
    }
    if (m_gotCells != 0)
    {
        m_iteratorFacet = 0;
        m_iteratorCell = m_cellList[m_gotCells - 1];
        m_gotCells--;
    }
}

void BBoxBasedAccelerator::setState(const NvcVec3& p)
{
    m_alreadyGotValue++;
    m_iteratorCell = -1;
    m_iteratorFacet = -1;
    m_gotCells = 0;
    int32_t perSlice = m_resolution * m_resolution;
    for (uint32_t i = 0; i < m_cells.size(); ++i)
    {
        if (toNvShared(m_cells[i]).contains(toNvShared(p)))
        {
            int32_t xyCellId = i % perSlice;
            for (int32_t zCell = 0; zCell < m_resolution; ++zCell)
            {
                int32_t cell = zCell * perSlice + xyCellId;
                if (!m_spatialMap[cell].empty())
                    m_cellList[m_gotCells++] = cell;
            }
        }
    }
    if (m_gotCells != 0)
    {
        m_iteratorFacet = 0;
        m_iteratorCell = m_cellList[m_gotCells - 1];
        m_gotCells--;
    }
}

void BBoxBasedAccelerator::buildAccelStructure(const Vertex* pos, const Edge* edges, const Facet* fc, int32_t facetCount)
{
    for (int32_t facet = 0; facet < facetCount; ++facet)
    {
        nvidia::NvBounds3 bBox;
        bBox.setEmpty();
        const Edge* edge = &edges[0] + fc->firstEdgeNumber;
        int32_t count = fc->edgesCount;
        for (int32_t ec = 0; ec < count; ++ec)
        {
            bBox.include(toNvShared(pos[edge->s].p));
            bBox.include(toNvShared(pos[edge->e].p));
            edge++;
        }

        for (uint32_t i = 0; i < m_cells.size(); ++i)
        {
            if (weakBoundingBoxIntersection(toNvShared(m_cells[i]), bBox))
            {
                m_spatialMap[i].push_back(facet);
            }
        }
        fc++;
    }
    m_alreadyGotFlag.resize(facetCount, 0);
}


#define SWEEP_RESOLUTION 2048

void buildIndex(std::vector<SegmentToIndex>& segm, float offset, float mlt, std::vector<std::vector<uint32_t>>& blocks)
{
    std::set<uint32_t> currentEnabled;
    uint32_t lastBlock = 0;
    for (uint32_t i = 0; i < segm.size(); ++i)
    {
        uint32_t currentBlock = (uint32_t)((segm[i].coord - offset) * mlt);
        if (currentBlock >= SWEEP_RESOLUTION) break;
        if (currentBlock != lastBlock)
        {
            for (uint32_t j = lastBlock + 1; j <= currentBlock; ++j)
            {
                for (auto id : currentEnabled)
                    blocks[j].push_back(id);
            }
            lastBlock = currentBlock;
        }
        if (segm[i].end == false)
        {
            blocks[lastBlock].push_back(segm[i].index);
            currentEnabled.insert(segm[i].index);
        }
        else
        {
            currentEnabled.erase(segm[i].index);
        }
    }
    
}

SweepingAccelerator::SweepingAccelerator(const Nv::Blast::Mesh* in)
{
    nvidia::NvBounds3 bnd;

    const Vertex* verts = in->getVertices();
    const Edge* edges = in->getEdges();

    m_facetCount = in->getFacetCount();

    m_foundx.resize(m_facetCount, 0);
    m_foundy.resize(m_facetCount, 0);


    std::vector<SegmentToIndex> xevs;
    std::vector<SegmentToIndex> yevs;
    std::vector<SegmentToIndex> zevs;


    for (uint32_t i = 0; i < in->getFacetCount(); ++i)
    {
        const Facet* fc = in->getFacet(i);
        bnd.setEmpty();
        for (uint32_t v = 0; v < fc->edgesCount; ++v)
        {
            bnd.include(toNvShared(verts[edges[v + fc->firstEdgeNumber].s].p));
        }
        bnd.scaleFast(1.1f);
        xevs.push_back(SegmentToIndex(bnd.minimum.x, i, false));
        xevs.push_back(SegmentToIndex(bnd.maximum.x, i, true));

        yevs.push_back(SegmentToIndex(bnd.minimum.y, i, false));
        yevs.push_back(SegmentToIndex(bnd.maximum.y, i, true));

        zevs.push_back(SegmentToIndex(bnd.minimum.z, i, false));
        zevs.push_back(SegmentToIndex(bnd.maximum.z, i, true));

    }

    std::sort(xevs.begin(), xevs.end());
    std::sort(yevs.begin(), yevs.end());
    std::sort(zevs.begin(), zevs.end());

    
    m_minimal.x = xevs[0].coord;
    m_minimal.y = yevs[0].coord;
    m_minimal.z = zevs[0].coord;

    
    m_maximal.x = xevs.back().coord;
    m_maximal.y = yevs.back().coord;
    m_maximal.z = zevs.back().coord;

        
    m_rescale = (m_maximal - m_minimal) * 1.01f;
    m_rescale.x = 1.0f / m_rescale.x * SWEEP_RESOLUTION;
    m_rescale.y = 1.0f / m_rescale.y * SWEEP_RESOLUTION;
    m_rescale.z = 1.0f / m_rescale.z * SWEEP_RESOLUTION;

    m_xSegm.resize(SWEEP_RESOLUTION);
    m_ySegm.resize(SWEEP_RESOLUTION);
    m_zSegm.resize(SWEEP_RESOLUTION);


    buildIndex(xevs, m_minimal.x, m_rescale.x, m_xSegm);
    buildIndex(yevs, m_minimal.y, m_rescale.y, m_ySegm);
    buildIndex(zevs, m_minimal.z, m_rescale.z, m_zSegm);

    
    m_iterId  = 1;
    m_current = 0;
}

void SweepingAccelerator::release()
{
    NVBLAST_DELETE(this, SweepingAccelerator);
}

void SweepingAccelerator::setState(const NvcBounds3* facetBounds)
{
    m_current = 0;
    m_indices.clear();
    
    nvidia::NvBounds3 bnd = *toNvShared(facetBounds);

    bnd.scaleFast(1.1f);
    uint32_t start = (uint32_t)((std::max(0.0f, bnd.minimum.x - m_minimal.x)) * m_rescale.x);
    uint32_t end   = (uint32_t)((std::max(0.0f, bnd.maximum.x - m_minimal.x)) * m_rescale.x);
    for (uint32_t i = start; i <= end && i < SWEEP_RESOLUTION; ++i)
    {
        for (auto id : m_xSegm[i])
        {
            m_foundx[id] = m_iterId;
        }
    }
    start = (uint32_t)((std::max(0.0f, bnd.minimum.y - m_minimal.y)) * m_rescale.y);
    end   = (uint32_t)((std::max(0.0f, bnd.maximum.y - m_minimal.y)) * m_rescale.y);
    for (uint32_t i = start; i <= end && i < SWEEP_RESOLUTION; ++i)
    {
        for (auto id : m_ySegm[i])
        {
            m_foundy[id] = m_iterId;
        }
    }
    start = (uint32_t)((std::max(0.0f, bnd.minimum.z - m_minimal.z)) * m_rescale.z);
    end   = (uint32_t)((std::max(0.0f, bnd.maximum.z - m_minimal.z)) * m_rescale.z);
    for (uint32_t i = start; i <= end && i < SWEEP_RESOLUTION; ++i)
    {
        for (auto id : m_zSegm[i])
        {
            if (m_foundy[id] == m_iterId && m_foundx[id] == m_iterId)
            {
                m_foundx[id] = m_iterId + 1;
                m_foundy[id] = m_iterId + 1;
                m_indices.push_back(id);
            }
        }
    }

    m_iterId += 2;
}

void SweepingAccelerator::setState(const Vertex* pos, const Edge* ed, const Facet& fc)
{

    nvidia::NvBounds3 cfc(nvidia::NvBounds3::empty());

    for (uint32_t v = 0; v < fc.edgesCount; ++v)
    {
        cfc.include(toNvShared(pos[ed[fc.firstEdgeNumber + v].s].p));
        cfc.include(toNvShared(pos[ed[fc.firstEdgeNumber + v].e].p));
    }
    setState(&fromNvShared(cfc));
}

void SweepingAccelerator::setState(const NvcVec3& point) {
    
    m_indices.clear();

    /*for (uint32_t i = 0; i < facetCount; ++i)
    {
        indices.push_back(i);
    }*/

    uint32_t yIndex = (uint32_t)((point.y - m_minimal.y) * m_rescale.y);
    uint32_t xIndex = (uint32_t)((point.x - m_minimal.x) * m_rescale.x);

    for (uint32_t i = 0; i < m_xSegm[xIndex].size(); ++i)
    {
        m_foundx[m_xSegm[xIndex][i]] = m_iterId;
    }
    for (uint32_t i = 0; i < m_ySegm[yIndex].size(); ++i)
    {
        if (m_foundx[m_ySegm[yIndex][i]] == m_iterId)
        {
            m_indices.push_back(m_ySegm[yIndex][i]);
        }
    }
    m_iterId++;
    m_current = 0;
    NV_UNUSED(point);
}

int32_t SweepingAccelerator::getNextFacet()
{
    if (static_cast<uint32_t>(m_current) < m_indices.size())
    {
        ++m_current;
        return m_indices[m_current - 1];
    }
    else
        return -1;
}

} // namespace Blast
} // namespace Nv
