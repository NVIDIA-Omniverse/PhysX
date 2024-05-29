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


#ifndef NVBLASTEXTAUTHORINGBOOLEANTOOLIMPL_H
#define NVBLASTEXTAUTHORINGBOOLEANTOOLIMPL_H

#include "NvBlastExtAuthoringTypes.h"
#include "NvBlastExtAuthoringInternalCommon.h"
#include "NvBlastExtAuthoringBooleanTool.h"
#include <vector>
#include "NvBlastTypes.h"


namespace Nv
{
namespace Blast
{

class Mesh;

/**
    Boolean tool config, used to perform different operations: UNION, INTERSECTION, DIFFERENCE
*/
struct BooleanConf
{
    int32_t ca, cb, ci;
    BooleanConf(int32_t a, int32_t b, int32_t c) : ca(a), cb(b), ci(c)
    {
    }
};


namespace BooleanConfigurations
{
/**
    Creates boolean tool configuration to perform intersection of meshes A and B.
*/
inline BooleanConf BOOLEAN_INTERSECTION()
{
    return BooleanConf(0, 0, 1);
}

/**
    Creates boolean tool configuration to perform union of meshes A and B.
*/
inline BooleanConf BOOLEAN_UNION()
{
    return BooleanConf(1, 1, -1);
}
/**
    Creates boolean tool configuration to perform difference of meshes(A - B).
*/
inline BooleanConf BOOLEAN_DIFFERENCE()
{
    return BooleanConf(1, 0, -1);
}
}

/**
    Structure which holds information about intersection facet with edge.
*/
struct EdgeFacetIntersectionData
{
    int32_t edId;
    int32_t intersectionType;
    Vertex  intersectionPoint;
    EdgeFacetIntersectionData(int32_t edId, int32_t intersType, Vertex& inters) : edId(edId), intersectionType(intersType), intersectionPoint(inters)
    {   }
    EdgeFacetIntersectionData(int32_t edId) : edId(edId)
    {   }
    bool operator<(const EdgeFacetIntersectionData& b) const
    {
        return edId < b.edId;
    }
};


class SpatialAccelerator;

/**
    Tool for performing boolean operations on polygonal meshes.
    Tool supports only closed meshes. Performing boolean on meshes with holes can lead to unexpected behavior, e.g. holes in result geometry.
*/
class BooleanEvaluator
{

public:
    BooleanEvaluator();
    ~BooleanEvaluator();

    /**
        Perform boolean operation on two polygonal meshes (A and B).
        \param[in] meshA    Mesh A 
        \param[in] meshB    Mesh B
        \param[in] spAccelA Acceleration structure for mesh A
        \param[in] spAccelB Acceleration structure for mesh B
        \param[in] mode     Boolean operation type
    */
    void    performBoolean(const Mesh* meshA, const Mesh* meshB, SpatialAccelerator* spAccelA, SpatialAccelerator* spAccelB, const BooleanConf& mode);

    /**
        Perform boolean operation on two polygonal meshes (A and B).
        \param[in] meshA    Mesh A
        \param[in] meshB    Mesh B
        \param[in] mode     Boolean operation type
    */
    void    performBoolean(const Mesh* meshA, const Mesh* meshB, const BooleanConf& mode);

    /**
        Perform cutting of mesh with some large box, which represents cutting plane. This method skips part of intersetion computations, so
        should be used ONLY with cutting box, received from getBigBox(...) method from NvBlastExtAuthoringMesh.h. For cutting use only BOOLEAN_INTERSECTION or BOOLEAN_DIFFERENCE mode.
        \param[in] meshA    Mesh A
        \param[in] meshB    Cutting box
        \param[in] spAccelA Acceleration structure for mesh A
        \param[in] spAccelB Acceleration structure for cutting box
        \param[in] mode     Boolean operation type
    */
    void    performFastCutting(const Mesh* meshA, const Mesh* meshB, SpatialAccelerator* spAccelA, SpatialAccelerator* spAccelB, const BooleanConf& mode);

    /**
        Perform cutting of mesh with some large box, which represents cutting plane. This method skips part of intersetion computations, so
        should be used ONLY with cutting box, received from getBigBox(...) method from NvBlastExtAuthoringMesh.h. For cutting use only BOOLEAN_INTERSECTION or BOOLEAN_DIFFERENCE mode.
        \param[in] meshA    Mesh A
        \param[in] meshB    Cutting box
        \param[in] mode     Boolean operation type
    */
    void    performFastCutting(const Mesh* meshA, const Mesh* meshB, const BooleanConf& mode);

    /**
        Test whether point contained in mesh.
        \param[in] mesh     Mesh geometry
        \param[in] point    Point which should be tested
        \return not 0 if point is inside of mesh
    */
    int32_t isPointContainedInMesh(const Mesh* mesh, const NvcVec3& point);
    /**
        Test whether point contained in mesh.
        \param[in] mesh     Mesh geometry
        \param[in] spAccel  Acceleration structure for mesh
        \param[in] point    Point which should be tested
        \return not 0 if point is inside of mesh
    */
    int32_t isPointContainedInMesh(const Mesh* mesh, SpatialAccelerator* spAccel, const NvcVec3& point);


    /**
        Generates result polygon mesh after performing boolean operation.
        \return If not nullptr - result mesh geometry.
    */
    Mesh*   createNewMesh();

    /**
        Reset tool state.
    */
    void    reset();

private:

    void    buildFaceFaceIntersections(const BooleanConf& mode);
    void    buildFastFaceFaceIntersection(const BooleanConf& mode);
    void    collectRetainedPartsFromA(const BooleanConf& mode);
    void    collectRetainedPartsFromB(const BooleanConf& mode);

    int32_t addIfNotExist(const Vertex& p);
    void    addEdgeIfValid(const EdgeWithParent& ed);
private:

    int32_t vertexMeshStatus03(const NvcVec3& p, const Mesh* mesh);
    int32_t vertexMeshStatus30(const NvcVec3& p, const Mesh* mesh);

    const Mesh*                                             mMeshA;
    const Mesh*                                             mMeshB;

    SpatialAccelerator*                                     mAcceleratorA;
    SpatialAccelerator*                                     mAcceleratorB;

    std::vector<EdgeWithParent>                             mEdgeAggregate;
    std::vector<Vertex>                                     mVerticesAggregate;

    std::vector<std::vector<EdgeFacetIntersectionData> >    mEdgeFacetIntersectionData12;
    std::vector<std::vector<EdgeFacetIntersectionData> >    mEdgeFacetIntersectionData21;
};


/// BooleanTool

class BooleanToolImpl : public BooleanTool
{
public:
    /**
     *  Release BooleanTool memory
     */
    virtual void    release() override;

    virtual Mesh*   performBoolean(const Mesh* meshA, SpatialAccelerator* accelA, const Mesh* meshB, SpatialAccelerator* accelB, BooleanTool::Op op) override;

    virtual bool    pointInMesh(const Mesh* mesh, SpatialAccelerator* accel, const NvcVec3& point) override;

private:
    BooleanEvaluator m_evaluator;
};

} // namespace Blast
} // namespace Nv


#endif // ifndef NVBLASTEXTAUTHORINGBOOLEANTOOLIMPL_H
