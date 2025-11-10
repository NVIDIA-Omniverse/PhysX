// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "QuickHull.h"
#include "Quantizer.h"
#include <stdio.h>
#include "QhPreprocessor.h"
#include "Qh.h"
#include "QhVec3.h"
#include "QhBounds3.h"

#include <vector>

#ifdef _MSC_VER
#pragma warning(disable:4100)
#endif

namespace quickhull
{

static const double DISTANCE_EPSILON = 0.000001f;	// close enough to consider two floating point numbers to be 'the same'.
static const double RESIZE_VALUE = 0.01f;			// if the provided points AABB is very thin resize it to this size
static const double PLANE_TOLERANCE = 0.01f;

/**
\brief Polygon data

Plane format: (mPlane[0],mPlane[1],mPlane[2]).dot(x) + mPlane[3] = 0
With the normal outward-facing from the hull.
*/
struct QhHullPolygon
{
    double			mPlane[4];		//!< Plane equation for this polygon
    uint16_t			mNbVerts;		//!< Number of vertices/edges in the polygon
    uint16_t			mIndexBase;		//!< Offset in index buffer
};

// Output after triangulation has occurred.
struct HullResult
{
    uint32_t    mVertexCount{0};
    QhVec3      *mVertices{nullptr};
    uint32_t    mIndexCount{0};
    uint32_t    *mIndices{nullptr};         // Polygon indices (not triangle indices)
    uint32_t    mPolygonCount{0};           // number of polygon
    QhHullPolygon *mPolygons{nullptr};
};

    //////////////////////////////////////////////////////////////////////////
    // checks if points form a valid AABB cube, if not construct a default CUBE
static bool checkPointsAABBValidity(uint32_t numPoints, const QhVec3* points, uint32_t stride, double distanceEpsilon,
    double resizeValue, uint32_t& vcount, QhVec3* vertices, bool fCheck = false)
{
    const char* vtx = reinterpret_cast<const char *> (points);
    QhBounds3 bounds;
    bounds.setEmpty();

    // get the bounding box		
    for (uint32_t i = 0; i < numPoints; i++)
    {
        const QhVec3& p = *reinterpret_cast<const QhVec3 *> (vtx);
        vtx += stride;

        bounds.include(p);

        vertices[i] = p;
    }

    QhVec3 dim = bounds.getDimensions();
    QhVec3 center = bounds.getCenter();

    // special case, the AABB is very thin or user provided us with only input 2 points
    // we construct an AABB cube and return it
    if (dim.x < distanceEpsilon || dim.y < distanceEpsilon || dim.z < distanceEpsilon || numPoints < 3)
    {
        double len = FLT_MAX;

        // pick the shortest size bigger than the distance epsilon
        if (dim.x > distanceEpsilon && dim.x < len)
            len = dim.x;
        if (dim.y > distanceEpsilon && dim.y < len)
            len = dim.y;
        if (dim.z > distanceEpsilon && dim.z < len)
            len = dim.z;

        // if the AABB is small in all dimensions, resize it
        if (len == FLT_MAX)
        {
            dim = QhVec3(resizeValue);
        }
        // if one edge is small, set to 1/5th the shortest non-zero edge.
        else
        {
            if (dim.x < distanceEpsilon)
                dim.x = QhMin(len * 0.05f, resizeValue);
            else
                dim.x *= 0.5f;
            if (dim.y < distanceEpsilon)
                dim.y = QhMin(len * 0.05f, resizeValue);
            else
                dim.y *= 0.5f;
            if (dim.z < distanceEpsilon)
                dim.z = QhMin(len * 0.05f, resizeValue);
            else
                dim.z *= 0.5f;
        }

        // construct the AABB
        const QhVec3 extPos = center + dim;
        const QhVec3 extNeg = center - dim;

        if (fCheck)
            vcount = 0;

        vertices[vcount++] = extNeg;
        vertices[vcount++] = QhVec3(extPos.x, extNeg.y, extNeg.z);
        vertices[vcount++] = QhVec3(extPos.x, extPos.y, extNeg.z);
        vertices[vcount++] = QhVec3(extNeg.x, extPos.y, extNeg.z);
        vertices[vcount++] = QhVec3(extNeg.x, extNeg.y, extPos.z);
        vertices[vcount++] = QhVec3(extPos.x, extNeg.y, extPos.z);
        vertices[vcount++] = extPos;
        vertices[vcount++] = QhVec3(extNeg.x, extPos.y, extPos.z);
        return true; // return cube
    }

    vcount = numPoints;
    return false;
}

struct QuickHullFace;

#define QH_EPS_F32 FLT_EPSILON // maximum relative error of double rounding
#define QH_EPS_F64 DBL_EPSILON // maximum relative error of double rounding


//////////////////////////////////////////////////////////////////////////
static const double MIN_ADJACENT_ANGLE = 3.0f;  // in degrees  - result wont have two adjacent facets within this angle of each other.
static const double PLANE_THICKNES = 3.0f * QH_EPS_F32;  // points within this distance are considered on a plane	
static const double MAXDOT_MINANG = cos(QhDegToRad(MIN_ADJACENT_ANGLE)); // adjacent angle for dot product tests


//////////////////////////////////////////////////////////////////////////
template<typename T, bool useIndexing>
class MemBlock
{
public:
    MemBlock(uint32_t preallocateSize)
        : mPreallocateSize(preallocateSize), mCurrentBlock(0), mCurrentIndex(0)
    {
        assert(preallocateSize);
        T* block = reinterpret_cast<T*>(malloc(sizeof(T)*preallocateSize));
        mBlocks.push_back(block);
    }

    MemBlock()
        : mPreallocateSize(0), mCurrentBlock(0), mCurrentIndex(0)
    {
    }

    void init(uint32_t preallocateSize)
    {
        assert(preallocateSize);
        mPreallocateSize = preallocateSize;
        T* block = reinterpret_cast<T*>(malloc(sizeof(T)*preallocateSize));
        if (useIndexing)
        {
            for (uint32_t i = 0; i < mPreallocateSize; i++)
            {
                // placement new to index data
                QH_PLACEMENT_NEW(&block[i], T)(i);
            }
        }
        mBlocks.push_back(block);
    }

    ~MemBlock()
    {
        for (uint32_t i = 0; i < mBlocks.size(); i++)
        {
            free(mBlocks[i]);
        }
        mBlocks.clear();
    }

    void reset()
    {
        for (uint32_t i = 0; i < mBlocks.size(); i++)
        {
            free(mBlocks[i]);
        }
        mBlocks.clear();

        mCurrentBlock = 0;
        mCurrentIndex = 0;

        init(mPreallocateSize);
    }

    T* getItem(uint32_t index)
    {
        const uint32_t block = index / mPreallocateSize;
        const uint32_t itemIndex = index % mPreallocateSize;
        assert(block <= mCurrentBlock);
        assert(itemIndex < mPreallocateSize);
        return &(mBlocks[block])[itemIndex];
    }

    T* getFreeItem()
    {
        assert(mPreallocateSize);
        // check if we have enough space in block, otherwise allocate new block
        if (mCurrentIndex < mPreallocateSize)
        {
            return &(mBlocks[mCurrentBlock])[mCurrentIndex++];
        }
        else
        {
            T* block = reinterpret_cast<T*>(malloc(sizeof(T)*mPreallocateSize));
            mCurrentBlock++;
            if (useIndexing)
            {
                for (uint32_t i = 0; i < mPreallocateSize; i++)
                {
                    // placement new to index data
                    QH_PLACEMENT_NEW(&block[i], T)(mCurrentBlock*mPreallocateSize + i);
                }
            }
            mBlocks.push_back(block);
            mCurrentIndex = 0;
            return &(mBlocks[mCurrentBlock])[mCurrentIndex++];
        }
    }

private:
    uint32_t			mPreallocateSize;
    uint32_t			mCurrentBlock;
    uint32_t			mCurrentIndex;
    std::vector<T*>	mBlocks;
};

//////////////////////////////////////////////////////////////////////////
struct QuickHullResult
{
    enum Enum
    {
        eSUCCESS, // ok
        eZERO_AREA_TEST_FAILED, // area test failed for simplex
        eVERTEX_LIMIT_REACHED, // vertex limit reached need to expand hull
        ePOLYGONS_LIMIT_REACHED, // polygons hard limit reached
        eFAILURE // general failure
    };
};

//////////////////////////////////////////////////////////////////////////
// representation of quick hull vertex
struct QuickHullVertex
{
    QhVec3					point;		// point vector
    uint32_t				index;		// point index for compare
    double					dist;		// distance from plane if necessary

    QuickHullVertex*		next;		// link to next vertex, linked list used for conflict list

    QuickHullVertex& operator=(const QuickHullVertex& other)
    {
        point = other.point;
        index = other.index;
        dist = other.dist;
        next = other.next;

        return *this;
    }

    QH_FORCE_INLINE bool operator==(const QuickHullVertex& vertex) const
    {
        return index == vertex.index ? true : false;
    }

    QH_FORCE_INLINE bool operator <(const QuickHullVertex& vertex) const
    {
        return dist < vertex.dist ? true : false;
    }
};

//////////////////////////////////////////////////////////////////////////
// representation of quick hull half edge
struct QuickHullHalfEdge
{
    QuickHullHalfEdge() : prev(NULL), next(NULL), twin(NULL), face(NULL), edgeIndex(0xFFFFFFFF)
    {
    }

    QuickHullHalfEdge(uint32_t)
        : prev(NULL), next(NULL), twin(NULL), face(NULL), edgeIndex(0xFFFFFFFF)
    {
    }

    QuickHullVertex			tail;  // tail vertex, head vertex is the tail of the twin

    QuickHullHalfEdge*		prev;  // previous edge
    QuickHullHalfEdge*		next;  // next edge
    QuickHullHalfEdge*		twin;  // twin/opposite edge

    QuickHullFace*			face;  // face where the edge belong

    uint32_t					edgeIndex; // edge index used for edge creation

    QH_FORCE_INLINE const QuickHullVertex& getTail() const
    {
        return tail;
    }

    QH_FORCE_INLINE const QuickHullVertex& getHead() const
    {
        assert(twin);
        return twin->tail;
    }

    QH_FORCE_INLINE void setTwin(QuickHullHalfEdge* edge)
    {
        twin = edge;
        edge->twin = this;
    }

    QH_FORCE_INLINE QuickHullFace* getOppositeFace() const
    {
        return twin->face;
    }

    double getOppositeFaceDistance() const;
};



//////////////////////////////////////////////////////////////////////////

typedef std::vector<QuickHullVertex*>		QuickHullVertexArray;
typedef std::vector<QuickHullHalfEdge*>	QuickHullHalfEdgeArray;
typedef std::vector<QuickHullFace*>		QuickHullFaceArray;

//////////////////////////////////////////////////////////////////////////
// representation of quick hull face
struct QuickHullFace
{
    enum FaceState
    {
        eVISIBLE,
        eDELETED,
        eNON_CONVEX
    };

    QuickHullHalfEdge*		edge;			// starting edge
    uint16_t					numEdges;		// num edges on the face
    QuickHullVertex*		conflictList;	// conflict list, used to determine unclaimed vertices

    QhVec3					normal;			// Newell plane normal
    double					area;			// face area
    QhVec3					centroid;		// face centroid

    double					planeOffset;	// Newell plane offset
    double					expandOffset;	// used for plane expansion if vertex limit reached

    FaceState				state;			// face validity state

    QuickHullFace*			nextFace;		// used to indicate next free face in faceList
    uint32_t					index;			// face index for compare identification
    uint8_t					outIndex;		// face index used for output descriptor

public:
    QuickHullFace()
        : edge(NULL), numEdges(0), conflictList(NULL), area(0.0f), planeOffset(0.0f), expandOffset(-FLT_MAX),
        state(eVISIBLE), nextFace(NULL), outIndex(0)
    {
    }

    QuickHullFace(uint32_t ind)
        : edge(NULL), numEdges(0), conflictList(NULL), area(0.0f), planeOffset(0.0f), expandOffset(-FLT_MAX),
        state(eVISIBLE), nextFace(NULL), index(ind), outIndex(0)
    {
    }

    ~QuickHullFace()
    {
    }

    // get edge on index
    QH_FORCE_INLINE QuickHullHalfEdge* getEdge(uint32_t i) const
    {
        QuickHullHalfEdge* he = edge;
        while (i > 0)
        {
            he = he->next;
            i--;
        }
        return he;
    }

    // distance from a plane to provided point
    QH_FORCE_INLINE double distanceToPlane(const QhVec3 p) const
    {
        return normal.dot(p) - planeOffset;
    }

    // compute face normal and centroid
    QH_FORCE_INLINE void	computeNormalAndCentroid()
    {
        assert(edge);
        normal = QhVec3(QhZero);
        numEdges = 1;

        QuickHullHalfEdge* testEdge = edge;
        QuickHullHalfEdge* startEdge = NULL;
        double maxDist = 0.0f;
        for (uint32_t i = 0; i < 3; i++)
        {
            const double d = (testEdge->tail.point - testEdge->next->tail.point).magnitudeSquared();
            if (d > maxDist)
            {
                maxDist = d;
                startEdge = testEdge;
            }
            testEdge = testEdge->next;
        }
        assert(startEdge);

        QuickHullHalfEdge* he = startEdge->next;
        const QhVec3& p0 = startEdge->tail.point;
        const QhVec3 d = he->tail.point - p0;
        centroid = startEdge->tail.point;

        do
        {
            numEdges++;
            centroid += he->tail.point;

            normal += d.cross(he->next->tail.point - p0);

            he = he->next;
        } while (he != startEdge);

        area = normal.normalize();
        centroid *= (1.0f / double(numEdges));

        planeOffset = normal.dot(centroid);
    }

    // merge adjacent face
    bool	mergeAdjacentFace(QuickHullHalfEdge* halfEdge, QuickHullFaceArray& discardedFaces);

    // check face consistency
    bool	checkFaceConsistency();

private:
    // connect halfedges
    QuickHullFace* connectHalfEdges(QuickHullHalfEdge* hedgePrev, QuickHullHalfEdge* hedge);

    // check if the face does have only 3 vertices
    QH_FORCE_INLINE bool	isTriangle() const
    {
        return numEdges == 3 ? true : false;
    }

};

//////////////////////////////////////////////////////////////////////////
// Quickhull base class holding the hull during construction
class QuickHullImpl : public QuickHull
{
    QH_NOCOPY(QuickHullImpl)
public:

    QuickHullImpl(void);

    ~QuickHullImpl();

    virtual void release(void) final
    {
        delete this;
    }

    // preallocate the edges, faces, vertices
    void preallocate(uint32_t numVertices);

    // parse the input verts, store them into internal format
    void parseInputVertices(const QhVec3* verts, uint32_t numVerts);

    // sets the precomputed min/max data
    void setPrecomputedMinMax(const QuickHullVertex* minVertex, const QuickHullVertex* maxVertex, const double tolerance, const double planeTolerance);

    // main entry function to build the hull from provided points
    QuickHullResult::Enum buildHull();

    uint32_t maxNumVertsPerFace() const;

protected:
    // compute min max verts
    void computeMinMaxVerts();

    // find the initial simplex
    bool findSimplex();

    // add the initial simplex
    void addSimplex(QuickHullVertex* simplex, bool flipTriangle);

    // finds next point to add
    QuickHullVertex* nextPointToAdd(QuickHullFace*& eyeFace);

    // adds point to the hull
    bool addPointToHull(const QuickHullVertex* vertex, QuickHullFace& face, bool& addFailed);

    // creates new face from given triangles 
    QuickHullFace* createTriangle(const QuickHullVertex& v0, const QuickHullVertex& v1, const QuickHullVertex& v2);

    // adds point to the face conflict list
    void addPointToFace(QuickHullFace& face, QuickHullVertex* vertex, double dist);

    // removes eye point from the face conflict list
    void removeEyePointFromFace(QuickHullFace& face, const QuickHullVertex* vertex);

    // calculate the horizon fro the eyePoint against a given face
    void calculateHorizon(const QhVec3& eyePoint, QuickHullHalfEdge* edge, QuickHullFace& face, QuickHullHalfEdgeArray& horizon, QuickHullFaceArray& removedFaces);

    // adds new faces from given horizon and eyePoint
    void addNewFacesFromHorizon(const QuickHullVertex* eyePoint, const QuickHullHalfEdgeArray& horizon, QuickHullFaceArray& newFaces);

    // merge adjacent face
    bool doAdjacentMerge(QuickHullFace& face, bool mergeWrtLargeFace, bool& mergeFailed);

    // merge adjacent face doing normal test
    bool doPostAdjacentMerge(QuickHullFace& face, const double minAngle);

    // delete face points
    void deleteFacePoints(QuickHullFace& faceToDelete, QuickHullFace* absorbingFace);

    // resolve unclaimed points
    void resolveUnclaimedPoints(const QuickHullFaceArray& newFaces);

    // merges polygons with similar normals
    void postMergeHull();

    // check if 2 faces can be merged
    bool canMergeFaces(const QuickHullHalfEdge& he);

    // get next free face
    QH_FORCE_INLINE QuickHullFace* getFreeHullFace()
    {
        return mFreeFaces.getFreeItem();
    }

    // get next free half edge
    QH_FORCE_INLINE QuickHullHalfEdge* getFreeHullHalfEdge()
    {
        return mFreeHalfEdges.getFreeItem();
    }

    QH_FORCE_INLINE uint32_t getNbHullVerts() { return mOutputNumVertices; }

    //////////////////////////////////////////////////////////////////////////
// fixup the input vertices to be not colinear or coplanar for the initial simplex find
    bool cleanupForSimplex(QhVec3* vertices, uint32_t vertexCount, QuickHullVertex* minimumVertex,
        QuickHullVertex* maximumVertex, double& tolerance, double& planeTolerance)
    {
        bool retVal = true;

        for (uint32_t i = 0; i < 3; i++)
        {
            minimumVertex[i].point = vertices[0];
            minimumVertex[i].index = 0;
            maximumVertex[i].point = vertices[0];
            maximumVertex[i].index = 0;

        }

        QhVec3 max = vertices[0];
        QhVec3 min = vertices[0];

        // get the max min vertices along the x,y,z
        for (uint32_t i = 1; i < vertexCount; i++)
        {
            const QhVec3& testPoint = vertices[i];
            if (testPoint.x > max.x)
            {
                max.x = testPoint.x;
                maximumVertex[0].point = testPoint;
                maximumVertex[0].index = i;
            }
            else if (testPoint.x < min.x)
            {
                min.x = testPoint.x;
                minimumVertex[0].point = testPoint;
                minimumVertex[0].index = i;
            }

            if (testPoint.y > max.y)
            {
                max.y = testPoint.y;
                maximumVertex[1].point = testPoint;
                maximumVertex[1].index = i;
            }
            else if (testPoint.y < min.y)
            {
                min.y = testPoint.y;
                minimumVertex[1].point = testPoint;
                minimumVertex[1].index = i;
            }

            if (testPoint.z > max.z)
            {
                max.z = testPoint.z;
                maximumVertex[2].point = testPoint;
                maximumVertex[2].index = i;
            }
            else if (testPoint.z < min.z)
            {
                min.z = testPoint.z;
                minimumVertex[2].point = testPoint;
                minimumVertex[2].index = i;
            }
        }

        const double sizeTol = (max.x - min.x + max.y - min.y + max.z - min.z)*0.5f;
        tolerance = QhMax(PLANE_THICKNES * sizeTol, PLANE_THICKNES);
        planeTolerance = QhMax(PLANE_TOLERANCE *sizeTol, PLANE_TOLERANCE);

        double fmax = 0;
        uint32_t imax = 0;

        for (uint32_t i = 0; i < 3; i++)
        {
            double diff = (maximumVertex[i].point)[i] - (minimumVertex[i].point)[i];
            if (diff > fmax)
            {
                fmax = diff;
                imax = i;
            }
        }

        QhVec3 simplex[4];

        // set first two vertices to be those with the greatest
        // one dimensional separation
        simplex[0] = maximumVertex[imax].point;
        simplex[1] = minimumVertex[imax].point;

        // This is just to silence warning of using a variable that 
        // has noot been initialized, as compiler can't prove
        // that we will execute at least once the if(lengSqr > maxDist)
        // that will write simplex[2] and simplex[3]
        simplex[2] = simplex[0];
        simplex[3] = simplex[1];

        // set third vertex to be the vertex farthest from
        // the line between simplex[0] and simplex[1]
        QhVec3 normal {0, 0, 1};
        double maxDist = 0;
        imax = 0;
        QhVec3 u01 = (simplex[1] - simplex[0]);
        u01.normalize();

        for (uint32_t i = 0; i < vertexCount; i++)
        {
            const QhVec3& testPoint = vertices[i];
            const QhVec3 diff = testPoint - simplex[0];
            const QhVec3 xprod = u01.cross(diff);
            const double lenSqr = xprod.magnitudeSquared();
            if (lenSqr > maxDist)
            {
                maxDist = lenSqr;
                simplex[2] = testPoint;
                normal = xprod;
                imax = i;
            }
        }

        if (QhSqrt(maxDist) < planeTolerance)
        {
            // points are collinear, we have to move the point further
            QhVec3 u02 = simplex[2] - simplex[0];
            double fT = u02.dot(u01);
            const double sqrLen = u01.magnitudeSquared();
            fT /= sqrLen;
            QhVec3 n = u02 - fT * u01;
            n.normalize();
            const QhVec3 mP = simplex[2] + n * planeTolerance;
            simplex[2] = mP;
            vertices[imax] = mP;
            retVal = false;
        }
        normal.normalize();

        // set the forth vertex in the normal direction	
        double d0 = simplex[2].dot(normal);
        maxDist = 0.0f;
        imax = 0;
        for (uint32_t i = 0; i < vertexCount; i++)
        {
            const QhVec3& testPoint = vertices[i];
            double dist = QhAbs(testPoint.dot(normal) - d0);
            if (dist > maxDist)
            {
                maxDist = dist;
                simplex[3] = testPoint;
                imax = i;
            }
        }

        if (QhAbs(maxDist) < planeTolerance)
        {
            const double dist = (vertices[imax].dot(normal) - d0);
            if (dist > 0)
                vertices[imax] = vertices[imax] + normal * planeTolerance;
            else
                vertices[imax] = vertices[imax] - normal * planeTolerance;
            retVal = false;
        }

        return retVal;
    }

    uint32_t buildHull(const double *points,uint32_t pcount)
    {
        uint32_t ret = 0;

        mMaxHullVerts = pcount;

        releaseResults();

        uint32_t outvcount = pcount;
        if (outvcount < 8)
        {
            outvcount = 8;
        }
        QhVec3 *outvsource = new QhVec3[outvcount];
        if (!shiftAndcleanupVertices(pcount, (const QhVec3 *)points, sizeof(QhVec3), outvcount, outvsource))
        {
            delete[]outvsource;
            return 0;
        }

        QuickHullVertex minimumVertex[3];
        QuickHullVertex maximumVertex[3];
        double tolerance;
        double planeTolerance;
        bool canReuse = cleanupForSimplex(outvsource, outvcount, &minimumVertex[0], &maximumVertex[0], tolerance, planeTolerance);

        preallocate(outvcount); //pre-allocate vertices

        if (canReuse)
        {
            setPrecomputedMinMax(minimumVertex, maximumVertex, tolerance, planeTolerance);
        }

        mNumVertices = outvcount;
        for (uint32_t i = 0; i < outvcount; i++)
        {
            mVerticesList[i].point = outvsource[i];
            mVerticesList[i].index = i;
        }

        QuickHullResult::Enum qhRes = buildHull();

        if ( qhRes == QuickHullResult::eSUCCESS )
        {
            postMergeHull();
            triangulateResults();
        }
        else
        {
            releaseResults();
        }
        delete[]outvsource;

        ret = (uint32_t)mIndices.size() / 3; // return number of triangles in output

        return ret;

    }

    virtual uint32_t computeConvexHull(const HullPoints &hp) final
    {
        uint32_t ret = 0;

        mMaxQuantizeVerts = hp.mMaxQuantizeVertexCount;
        mMaxHullVerts = hp.mVertexCount;

        ret = buildHull(hp.mVertices,hp.mVertexCount);

        if ( mResult.mVertexCount > hp.mMaxHullVertices )
        {
            WuQuantizer *q = WuQuantizer::create();
            uint32_t outputCount;
            const double *points = q->wuQuantize3D(mResult.mVertexCount,(const double *)mResult.mVertices,true,hp.mMaxHullVertices,outputCount);
            if ( points )
            {
                buildHull(points,outputCount);
            }

            q->release();
        }

        return ret;
    }

    void triangulateResults(void)
    {
        // get the number of indices needed
        uint32_t numIndices = 0;
        uint32_t numFaces = (uint32_t)mHullFaces.size();
        uint32_t numFacesOut = 0;
        uint32_t largestFace = 0;			// remember the largest face, we store it as the first face, required for GRB test (max 32 vers per face supported)
        for (uint32_t i = 0; i < numFaces; i++)
        {
            const QuickHullFace& face = *mHullFaces[i];
            if (face.state == QuickHullFace::eVISIBLE)
            {
                numFacesOut++;
                numIndices += face.numEdges;
                if (face.numEdges > mHullFaces[largestFace]->numEdges)
                    largestFace = i;
            }
        }

        // allocate out buffers
        const uint32_t indicesBufferSize = sizeof(uint32_t)*numIndices;
        const uint32_t verticesBufferSize = sizeof(QhVec3)*(mNumVertices + 1);
        const uint32_t facesBufferSize = sizeof(QhHullPolygon)*numFacesOut;
        const uint32_t faceTranslationTableSize = sizeof(uint16_t)*numFacesOut;
        const uint32_t translationTableSize = sizeof(uint32_t)*mNumVertices;
        const uint32_t bufferMemorySize = indicesBufferSize + verticesBufferSize + facesBufferSize + faceTranslationTableSize + translationTableSize;
        mOutMemoryBuffer = reinterpret_cast<uint8_t*>(malloc(bufferMemorySize));

        uint32_t* indices = reinterpret_cast<uint32_t*> (mOutMemoryBuffer);
        QhVec3* vertices = reinterpret_cast<QhVec3*> (mOutMemoryBuffer + indicesBufferSize);
        QhHullPolygon* polygons = reinterpret_cast<QhHullPolygon*> (mOutMemoryBuffer + indicesBufferSize + verticesBufferSize);
        mFaceTranslateTable = reinterpret_cast<uint16_t*> (mOutMemoryBuffer + indicesBufferSize + verticesBufferSize + facesBufferSize);
        int32_t* translateTable = reinterpret_cast<int32_t*> (mOutMemoryBuffer + indicesBufferSize + verticesBufferSize + facesBufferSize + faceTranslationTableSize);
        memset(translateTable, -1, mNumVertices * sizeof(uint32_t));

        // go over the hullPolygons and mark valid vertices, create translateTable
        uint32_t numVertices = 0;
        for (uint32_t i = 0; i < numFaces; i++)
        {
            const QuickHullFace& face = *mHullFaces[i];
            if (face.state == QuickHullFace::eVISIBLE)
            {
                QuickHullHalfEdge* he = face.edge;
                if (translateTable[he->tail.index] == -1)
                {
                    vertices[numVertices] = he->tail.point;
                    translateTable[he->tail.index] = int32_t(numVertices);
                    numVertices++;
                }
                he = he->next;
                while (he != face.edge)
                {
                    if (translateTable[he->tail.index] == -1)
                    {
                        vertices[numVertices] = he->tail.point;
                        translateTable[he->tail.index] = int32_t(numVertices);
                        numVertices++;
                    }
                    he = he->next;
                }
            }
        }
#if 1
        for (uint32_t i=0; i<numVertices; i++)
        {
            vertices[i]+=mOriginShift; // restore vertice origin
        }
#endif
        mResult.mVertexCount = numVertices;
        mResult.mVertices = vertices;

        mResult.mIndexCount = numIndices;
        mResult.mIndices = indices;

        mResult.mPolygonCount = numFacesOut;
        mResult.mPolygons = polygons;

        uint16_t indexOffset = 0;
        numFacesOut = 0;
        for (uint32_t i = 0; i < numFaces; i++)
        {
            // faceIndex - store the largest face first then the rest
            uint32_t faceIndex;
            if (i == 0)
            {
                faceIndex = largestFace;
            }
            else
            {
                faceIndex = (i == largestFace) ? 0 : i;
            }

            QuickHullFace& face = *mHullFaces[faceIndex];
            if (face.state == QuickHullFace::eVISIBLE)
            {
                //create index data
                QuickHullHalfEdge* he = face.edge;
                uint32_t index = 0;
                he->edgeIndex = 0xFFFFFFFF;
                indices[index + indexOffset] = uint32_t(translateTable[he->tail.index]);
                index++;
                he = he->next;
                while (he != face.edge)
                {
                    indices[index + indexOffset] = uint32_t(translateTable[he->tail.index]);
                    index++;
                    he->edgeIndex = 0xFFFFFFFF;
                    he = he->next;
                }

                // create polygon
                QhHullPolygon polygon;
                polygon.mPlane[0] = face.normal[0];
                polygon.mPlane[1] = face.normal[1];
                polygon.mPlane[2] = face.normal[2];
                polygon.mPlane[3] = -face.planeOffset;

                polygon.mIndexBase = indexOffset;
                polygon.mNbVerts = face.numEdges;
                indexOffset += face.numEdges;
                polygons[numFacesOut] = polygon;
                mFaceTranslateTable[numFacesOut] = uint16_t(faceIndex);
                face.outIndex = uint8_t(numFacesOut);
                numFacesOut++;
            }
        }

        assert(mNumHullFaces == numFacesOut);

        mIndices.clear();
        for (uint32_t i=0; i<numFacesOut; i++)
        {
            const QhHullPolygon &p = polygons[i];
            uint32_t indexBase = p.mIndexBase;

            uint32_t i1 = indices[indexBase+0];
            uint32_t i2 = indices[indexBase+1];
            uint32_t i3 = indices[indexBase+2];

            assert(i1 < numVertices );
            assert(i2 < numVertices );
            assert(i3 < numVertices );

            mIndices.push_back(i1);
            mIndices.push_back(i2);
            mIndices.push_back(i3);

            for (uint32_t j=3; j<p.mNbVerts; j++)
            {
                i2 = i3;
                i3 = indices[indexBase+j];

                assert(i3 < numVertices );

                mIndices.push_back(i1);
                mIndices.push_back(i2);
                mIndices.push_back(i3);
            }
        }
    }

    virtual uint32_t getPolygonCount(void) const // report number of polygons
    {
        return mResult.mPolygonCount;
    }

    double computePlane(const double* A, const double* B, const double* C, double* n) const // returns D
    {
        double vx = (B[0] - C[0]);
        double vy = (B[1] - C[1]);
        double vz = (B[2] - C[2]);

        double wx = (A[0] - B[0]);
        double wy = (A[1] - B[1]);
        double wz = (A[2] - B[2]);

        double vw_x = vy * wz - vz * wy;
        double vw_y = vz * wx - vx * wz;
        double vw_z = vx * wy - vy * wx;

        double mag = (double)sqrt((vw_x * vw_x) + (vw_y * vw_y) + (vw_z * vw_z));

        if (mag < 0.000001f)
        {
            mag = 0;
        }
        else
        {
            mag = 1.0f / mag;
        }

        double x = vw_x * mag;
        double y = vw_y * mag;
        double z = vw_z * mag;


        double D = 0.0f - ((x * A[0]) + (y * A[1]) + (z * A[2]));

        n[0] = x;
        n[1] = y;
        n[2] = z;

        return D;
    }



    virtual const uint32_t *getPolygon(uint32_t index,
                                       uint32_t &numPoints,
                                       uint32_t &indexBase,
                                       double plane[4]) const final
    {
        const uint32_t *ret = nullptr;
        numPoints = 0;

        if ( index < mResult.mPolygonCount )
        {
            const QhHullPolygon &p = mResult.mPolygons[index];
            numPoints = p.mNbVerts;
            indexBase = p.mIndexBase;
            uint32_t i1 = mResult.mIndices[indexBase+0];
            uint32_t i2 = mResult.mIndices[indexBase+1];
            uint32_t i3 = mResult.mIndices[indexBase+2];
            const double *p1 = &mResult.mVertices[i1].x;
            const double *p2 = &mResult.mVertices[i2].x;
            const double *p3 = &mResult.mVertices[i3].x;
            plane[3] = computePlane(p3,p2,p1,plane);
            ret = &mResult.mIndices[p.mIndexBase];
        }

        return ret;
    }


    virtual const double *getVertices(uint32_t &vcount) const final
    {
        const double *ret = nullptr;
        vcount = 0;

        if ( mResult.mVertexCount )
        {
            vcount = mResult.mVertexCount;
            ret = (const double *) mResult.mVertices;
        }

        return ret;
    }

    virtual const uint32_t *getIndices(uint32_t &tcount) const final
    {
        const uint32_t *ret = nullptr;
        tcount = 0;

        if ( !mIndices.empty() )
        {
            tcount = (uint32_t)mIndices.size()/3;
            ret = &mIndices[0];
        }

        return ret;
    }

    //////////////////////////////////////////////////////////////////////////
// normalize point cloud, remove duplicates!
    bool cleanupVertices(uint32_t svcount, const QhVec3* svertices, uint32_t stride,uint32_t& vcount, QhVec3* vertices)
    {
        if (svcount == 0)
            return false;

        const QhVec3* verticesToClean = svertices;
        uint32_t numVerticesToClean = svcount;
        WuQuantizer* quantizer = NULL;

        // if quantization is enabled, parse the input vertices and produce new quantized vertices, 
        // that will be then cleaned the same way
        if ( svcount > mMaxQuantizeVerts )
        {
            quantizer = WuQuantizer::create();
            uint32_t vertsOutCount;
            const QhVec3* vertsOut = (const QhVec3*) quantizer->wuQuantize3D(svcount, (const double *)svertices, true,mMaxQuantizeVerts, vertsOutCount);
            if (vertsOut)
            {
                numVerticesToClean = vertsOutCount;
                verticesToClean = vertsOut;
            }
        }

        const double distanceEpsilon = DISTANCE_EPSILON * 1;
        const double resizeValue = RESIZE_VALUE * 1;

        vcount = 0;
        // check for the AABB from points, if its very tiny return a resized CUBE
        if (checkPointsAABBValidity(numVerticesToClean, verticesToClean, stride, distanceEpsilon, resizeValue, vcount, vertices, false))
        {
            if (quantizer)
                quantizer->release();
            return true;
        }

        if (vcount < 4)
        {
            if (quantizer)
                quantizer->release();
            printf("ConvexHullLib::cleanupVertices: Less than four valid vertices were found. Provide at least four valid (e.g. each at a different position) vertices.\n");
            return false;
        }

        if (quantizer)
            quantizer->release();
        return true;
    }

    //////////////////////////////////////////////////////////////////////////
    // shift vertices around origin and normalize point cloud, remove duplicates!
    bool shiftAndcleanupVertices(uint32_t svcount, const QhVec3* svertices, uint32_t stride,
        uint32_t& vcount, QhVec3* vertices)
    {
        delete []mShiftedVerts;
        mShiftedVerts = new QhVec3[svcount];
        const char* vtx = reinterpret_cast<const char *> (svertices);
        QhBounds3 bounds;
        bounds.setEmpty();

        // get the bounding box		
        for (uint32_t i = 0; i < svcount; i++)
        {
            const QhVec3& p = *reinterpret_cast<const QhVec3 *> (vtx);
            vtx += stride;
            bounds.include(p);
        }
        mOriginShift = bounds.getCenter();
        vtx = reinterpret_cast<const char *> (svertices);
        for (uint32_t i = 0; i < svcount; i++)
        {
            const QhVec3& p = *reinterpret_cast<const QhVec3 *> (vtx);
            vtx += stride;
            mShiftedVerts[i] = p - mOriginShift;
        }
        return cleanupVertices(svcount, mShiftedVerts, sizeof(QhVec3), vcount, vertices);
    }

    void releaseResults(void)
    {
        free(mOutMemoryBuffer);
        mOutMemoryBuffer = nullptr;
        mFaceTranslateTable = nullptr;
        mIndices.clear();
        mVertices.clear();
        free(mVerticesList);
        mVerticesList = nullptr;
        mHullFaces.clear();
        mNumHullFaces = 0;
        mUnclaimedPoints.clear();
        mHorizon.clear();
        mRemovedFaces.clear();
        mDiscardedFaces.clear();

        mResult = HullResult();
    }


    // Inflate the convex hull by this distance
    virtual void inflate(double distance) final
    {
        QhVec3 sum(0, 0, 0);
        for (uint32_t i = 0; i < mResult.mVertexCount; i++)
        {
            sum += mResult.mVertices[i];
        }

        double recip = 1.0f / double(mResult.mVertexCount);

        sum.x *= recip;
        sum.y *= recip;
        sum.z *= recip;

        for (uint32_t i = 0; i < mResult.mVertexCount; i++)
        {
            QhVec3 &p = mResult.mVertices[i];

            QhVec3 dir = p - sum;
            dir.normalize();
            p += distance * dir;
        }
    }

protected:
    std::vector< QhVec3 >           mVertices;
    std::vector< uint32_t >         mIndices;

    QhVec3							mOriginShift;   // origin shift point
    QhVec3*							mShiftedVerts{nullptr};  // shfited vertices

    QhVec3					mInteriorPoint;		// interior point for int/ext tests
    uint32_t                    mMaxQuantizeVerts;
    uint32_t                    mMaxHullVerts;      // maximum number of vertices in the output convex hull
    uint32_t					mMaxVertices;		// maximum number of vertices (can be different as we may add vertices during the cleanup
    uint32_t					mNumVertices;		// actual number of input vertices
    uint32_t					mOutputNumVertices;	// num vertices of the computed hull
    uint32_t					mTerminalVertex;	// in case we failed to generate hull in a regular run we set the terminal vertex and rerun

    QuickHullVertex*		mVerticesList;		// vertices list preallocated
    MemBlock<QuickHullHalfEdge, false>	mFreeHalfEdges;	// free half edges
    MemBlock<QuickHullFace, true>	mFreeFaces;			// free faces

    QuickHullFaceArray		mHullFaces;			// actual hull faces, contains also invalid and not used faces
    uint32_t					mNumHullFaces;		// actual number of hull faces

    bool					mPrecomputedMinMax; // if we got the precomputed min/max values
    QuickHullVertex			mMinVertex[3];		// min vertex
    QuickHullVertex			mMaxVertex[3];		// max vertex
    double					mTolerance;			// hull tolerance, used for plane thickness and merge strategy
    double					mPlaneTolerance;	// used for post merge stage

    QuickHullVertexArray	mUnclaimedPoints;	// holds temp unclaimed points

    QuickHullHalfEdgeArray	mHorizon;			// array for horizon computation
    QuickHullFaceArray		mNewFaces;			// new faces created during horizon computation
    QuickHullFaceArray		mRemovedFaces;		// removd faces during horizon computation
    QuickHullFaceArray      mDiscardedFaces;	// discarded faces during face merging

    uint8_t*					mOutMemoryBuffer{nullptr};   // memory buffer used for output data
    uint16_t*					mFaceTranslateTable{nullptr}; // translation table mapping output faces to internal quick hull table

    HullResult              mResult;

};


//////////////////////////////////////////////////////////////////////////

QuickHullImpl::QuickHullImpl(void)
    : mOutputNumVertices(0), mTerminalVertex(0xFFFFFFFF), mVerticesList(NULL), mNumHullFaces(0), mPrecomputedMinMax(false),
    mTolerance(-1.0f), mPlaneTolerance(-1.0f)
{
}

//////////////////////////////////////////////////////////////////////////

QuickHullImpl::~QuickHullImpl()
{
    releaseResults();
    delete []mShiftedVerts;
}

//////////////////////////////////////////////////////////////////////////
// sets the precomputed min/max values
void QuickHullImpl::setPrecomputedMinMax(const QuickHullVertex* minVertex, const QuickHullVertex* maxVertex, const double tolerance, const double planeTolerance)
{
    for (uint32_t i = 0; i < 3; i++)
    {
        mMinVertex[i] = minVertex[i];
        mMaxVertex[i] = maxVertex[i];
    }

    mTolerance = tolerance;
    mPlaneTolerance = planeTolerance;

    mPrecomputedMinMax = true;
}

//////////////////////////////////////////////////////////////////////////
// preallocate internal buffers
void QuickHullImpl::preallocate(uint32_t numVertices)
{
    assert(numVertices > 0);

    // max num vertices = numVertices
    mMaxVertices = QhMax(uint32_t(8), numVertices); // 8 is min, since we can expand to AABB during the clean vertices phase
    mVerticesList = reinterpret_cast<QuickHullVertex*> (malloc(sizeof(QuickHullVertex)*mMaxVertices));

    // estimate the max half edges
    uint32_t maxHalfEdges = (3 * mMaxVertices - 6) * 3;
    mFreeHalfEdges.init(maxHalfEdges);

    // estimate the max faces
    uint32_t maxFaces = (2 * mMaxVertices - 4);
    mFreeFaces.init(maxFaces * 2);

    mHullFaces.reserve(maxFaces);
    mUnclaimedPoints.reserve(numVertices);

    mNewFaces.reserve(32);
    mRemovedFaces.reserve(32);
    mDiscardedFaces.reserve(32);
    mHorizon.reserve(QhMin(numVertices, uint32_t(128)));
}

//////////////////////////////////////////////////////////////////////////
// returns the maximum number of vertices on a face
uint32_t QuickHullImpl::maxNumVertsPerFace() const
{
    uint32_t numFaces = (uint32_t)mHullFaces.size();
    uint32_t maxVerts = 0;
    for (uint32_t i = 0; i < numFaces; i++)
    {
        const QuickHullFace& face = *mHullFaces[i];
        if (face.state == QuickHullFace::eVISIBLE)
        {
            if (face.numEdges > maxVerts)
                maxVerts = face.numEdges;
        }
    }
    return maxVerts;
}

//////////////////////////////////////////////////////////////////////////
// parse the input vertices and store them in the hull
void QuickHullImpl::parseInputVertices(const QhVec3* verts, uint32_t numVerts)
{
    assert(verts);
    assert(numVerts <= mMaxVertices);

    mNumVertices = numVerts;
    for (uint32_t i = 0; i < numVerts; i++)
    {
        mVerticesList[i].point = verts[i];
        mVerticesList[i].index = i;
    }
}

//////////////////////////////////////////////////////////////////////////
// compute min max verts
void QuickHullImpl::computeMinMaxVerts()
{
    for (uint32_t i = 0; i < 3; i++)
    {
        mMinVertex[i] = mVerticesList[0];
        mMaxVertex[i] = mVerticesList[0];
    }

    QhVec3 max = mVerticesList[0].point;
    QhVec3 min = mVerticesList[0].point;

    // get the max min vertices along the x,y,z
    for (uint32_t i = 1; i < mNumVertices; i++)
    {
        const QuickHullVertex& testVertex = mVerticesList[i];
        const QhVec3& testPoint = testVertex.point;
        if (testPoint.x > max.x)
        {
            max.x = testPoint.x;
            mMaxVertex[0] = testVertex;
        }
        else if (testPoint.x < min.x)
        {
            min.x = testPoint.x;
            mMinVertex[0] = testVertex;
        }

        if (testPoint.y > max.y)
        {
            max.y = testPoint.y;
            mMaxVertex[1] = testVertex;
        }
        else if (testPoint.y < min.y)
        {
            min.y = testPoint.y;
            mMinVertex[1] = testVertex;
        }

        if (testPoint.z > max.z)
        {
            max.z = testPoint.z;
            mMaxVertex[2] = testVertex;
        }
        else if (testPoint.z < min.z)
        {
            min.z = testPoint.z;
            mMinVertex[2] = testVertex;
        }
    }

    const double sizeTol = (max.x - min.x + max.y - min.y + max.z - min.z)*0.5f;
    mTolerance = QhMax(PLANE_THICKNES * sizeTol, PLANE_THICKNES);
//TODO:TODO::TODO    mPlaneTolerance = QhMax(mCookingParams.planeTolerance * sizeTol, mCookingParams.planeTolerance);
}

//////////////////////////////////////////////////////////////////////////
// find the initial simplex
// 1. search in max axis from compute min,max
// 2. 3rd point is the furthest vertex from the initial line
// 3. 4th vertex is along the line, 3rd vertex normal
bool QuickHullImpl::findSimplex()
{
    double max = 0;
    uint32_t imax = 0;

    for (uint32_t i = 0; i < 3; i++)
    {
        double diff = mMaxVertex[i].point[i] - mMinVertex[i].point[i];
        if (diff > max)
        {
            max = diff;
            imax = i;
        }
    }

    if (max <= mTolerance)
    {
        // should not happen as we clear the vertices before and expand them if they are really close to each other
        printf("QuickHullConvexHullLib::findSimplex: Simplex input points appers to be almost at the same place\n");
        return false;
    }

    QuickHullVertex simplex[4];

    // set first two vertices to be those with the greatest
    // one dimensional separation
    simplex[0] = mMaxVertex[imax];
    simplex[1] = mMinVertex[imax];

    // set third vertex to be the vertex farthest from
    // the line between simplex[0] and simplex[1]
    QhVec3 normal(0.0f, 0.0f, 1.0f);
    double maxDist = 0;
    QhVec3 u01 = (simplex[1].point - simplex[0].point);
    u01.normalize();

    for (uint32_t i = 0; i < mNumVertices; i++)
    {
        const QuickHullVertex& testVert = mVerticesList[i];
        const QhVec3& testPoint = testVert.point;
        const QhVec3 diff = testPoint - simplex[0].point;
        const QhVec3 xprod = u01.cross(diff);
        const double lenSqr = xprod.magnitudeSquared();
        if (lenSqr > maxDist && testVert.index != simplex[0].index && testVert.index != simplex[1].index)
        {
            maxDist = lenSqr;
            simplex[2] = testVert;
            normal = xprod;
        }
    }

    if (QhSqrt(maxDist) <= mTolerance)
    {
        printf("QuickHullConvexHullLib::findSimplex: Simplex input points appers to be colinear.\n");
        return false;
    }
    normal.normalize();

    // set the forth vertex in the normal direction	
    const double d0 = simplex[2].point.dot(normal);
    maxDist = 0.0f;
    for (uint32_t i = 0; i < mNumVertices; i++)
    {
        const QuickHullVertex& testVert = mVerticesList[i];
        const QhVec3& testPoint = testVert.point;
        const double dist = QhAbs(testPoint.dot(normal) - d0);
        if (dist > maxDist && testVert.index != simplex[0].index &&
            testVert.index != simplex[1].index && testVert.index != simplex[2].index)
        {
            maxDist = dist;
            simplex[3] = testVert;
        }
    }

    if (QhAbs(maxDist) <= mTolerance)
    {
        printf("QuickHullConvexHullLib::findSimplex: Simplex input points appers to be coplanar.\n");
        return false;
    }

    // now create faces from those triangles
    addSimplex(&simplex[0], simplex[3].point.dot(normal) - d0 < 0);

    return true;
}

//////////////////////////////////////////////////////////////////////////
// create triangle from given vertices, produce new face and connect the half edges
QuickHullFace* QuickHullImpl::createTriangle(const QuickHullVertex& v0, const QuickHullVertex& v1, const QuickHullVertex& v2)
{
    QuickHullFace* face = getFreeHullFace();

    QuickHullHalfEdge* he0 = getFreeHullHalfEdge();
    he0->face = face;
    he0->tail = v0;

    QuickHullHalfEdge* he1 = getFreeHullHalfEdge();
    he1->face = face;
    he1->tail = v1;

    QuickHullHalfEdge* he2 = getFreeHullHalfEdge();
    he2->face = face;
    he2->tail = v2;

    he0->prev = he2;
    he0->next = he1;
    he1->prev = he0;
    he1->next = he2;
    he2->prev = he1;
    he2->next = he0;

    face->edge = he0;
    face->nextFace = NULL;

    // compute the normal and offset
    face->computeNormalAndCentroid();
    return face;
}


//////////////////////////////////////////////////////////////////////////
// add initial simplex to the quickhull
// construct triangles from the simplex points and connect them with half edges
void QuickHullImpl::addSimplex(QuickHullVertex* simplex, bool flipTriangle)
{
    assert(simplex);

    // get interior point
    QhVec3 vectorSum = simplex[0].point;
    for (uint32_t i = 1; i < 4; i++)
    {
        vectorSum += simplex[i].point;
    }
    mInteriorPoint = vectorSum / 4.0f;

    QuickHullFace* tris[4];
    // create the triangles from the initial simplex
    if (flipTriangle)
    {
        tris[0] = createTriangle(simplex[0], simplex[1], simplex[2]);
        tris[1] = createTriangle(simplex[3], simplex[1], simplex[0]);
        tris[2] = createTriangle(simplex[3], simplex[2], simplex[1]);
        tris[3] = createTriangle(simplex[3], simplex[0], simplex[2]);

        for (uint32_t i = 0; i < 3; i++)
        {
            uint32_t k = (i + 1) % 3;
            tris[i + 1]->getEdge(1)->setTwin(tris[k + 1]->getEdge(0));
            tris[i + 1]->getEdge(2)->setTwin(tris[0]->getEdge(k));
        }
    }
    else
    {
        tris[0] = createTriangle(simplex[0], simplex[2], simplex[1]);
        tris[1] = createTriangle(simplex[3], simplex[0], simplex[1]);
        tris[2] = createTriangle(simplex[3], simplex[1], simplex[2]);
        tris[3] = createTriangle(simplex[3], simplex[2], simplex[0]);

        for (uint32_t i = 0; i < 3; i++)
        {
            uint32_t k = (i + 1) % 3;
            tris[i + 1]->getEdge(0)->setTwin(tris[k + 1]->getEdge(1));
            tris[i + 1]->getEdge(2)->setTwin(tris[0]->getEdge((3 - i) % 3));
        }
    }

    // push back the first 4 faces created from the simplex
    for (uint32_t i = 0; i < 4; i++)
    {
        mHullFaces.push_back(tris[i]);
    }
    mNumHullFaces = 4;

    // go through points and add point to faces if they are on the plane
    for (uint32_t i = 0; i < mNumVertices; i++)
    {
        const QuickHullVertex& v = mVerticesList[i];

        if (v == simplex[0] || v == simplex[1] || v == simplex[2] || v == simplex[3])
        {
            continue;
        }

        double maxDist = mTolerance;
        QuickHullFace* maxFace = NULL;
        for (uint32_t k = 0; k < 4; k++)
        {
            const double dist = tris[k]->distanceToPlane(v.point);
            if (dist > maxDist)
            {
                maxFace = tris[k];
                maxDist = dist;
            }
        }

        if (maxFace != NULL)
        {
            addPointToFace(*maxFace, &mVerticesList[i], maxDist);
        }
    }
}

//////////////////////////////////////////////////////////////////////////
// adds a point to the conflict list
// the trick here is to store the most furthest point as the last, thats the only one we care about
// the rest is not important, we just need to store them and claim to new faces later, if the 
// faces most furthest point is the current global maximum
void QuickHullImpl::addPointToFace(QuickHullFace& face, QuickHullVertex* vertex, double dist)
{
    // if we dont have a conflict list, store the vertex as the first one in the conflict list
    vertex->dist = dist;
    if (!face.conflictList)
    {
        face.conflictList = vertex;
        vertex->dist = dist;
        vertex->next = NULL;
        return;
    }

    assert(face.conflictList);

    // this is not the furthest vertex, store it as next in the linked list
    if (face.conflictList->dist > dist)
    {
        vertex->next = face.conflictList->next;
        face.conflictList->next = vertex;
    }
    else
    {
        // this is the furthest vertex, store it as first in the linked list
        vertex->next = face.conflictList;
        face.conflictList = vertex;
    }
}

//////////////////////////////////////////////////////////////////////////
// removes eye point from a conflict list
// we know that the vertex must the last, as we store it at the back, so just popback()
void QuickHullImpl::removeEyePointFromFace(QuickHullFace& face, const QuickHullVertex* vertex)
{
    QH_UNUSED(vertex);
    // the picked vertex should always be the first in the linked list
    assert(face.conflictList == vertex);

    face.conflictList = face.conflictList->next;
}

//////////////////////////////////////////////////////////////////////////
// merge polygons with similar normals
void QuickHullImpl::postMergeHull()
{
    // merge faces with similar normals 
    for (uint32_t i = 0; i < mHullFaces.size(); i++)
    {
        QuickHullFace& face = *mHullFaces[i];

        if (face.state == QuickHullFace::eVISIBLE)
        {
            assert(face.checkFaceConsistency());
            while (doPostAdjacentMerge(face, MAXDOT_MINANG));
        }
    }
}

//////////////////////////////////////////////////////////////////////////
// builds the hull
// 1. find the initial simplex
// 2. check if simplex has a valid area
// 3. add vertices to the hull. We add vertex most furthest from the hull
// 4. terminate if hull limit reached or we have added all vertices
QuickHullResult::Enum QuickHullImpl::buildHull()
{
    QuickHullVertex* eyeVtx = NULL;
    QuickHullFace*	eyeFace;

    // compute the vertex min max along x,y,z
    if (!mPrecomputedMinMax)
        computeMinMaxVerts();

    // find the initial simplex of the hull
    if (!findSimplex())
    {
        return QuickHullResult::eFAILURE;
    }

    // simplex area test
    const bool useAreaTest = false; // TODO: mConvexDesc.flags & QhConvexFlag::eCHECK_ZERO_AREA_TRIANGLES ? true : false;
    const double areaEpsilon = 0.06f * 2.0f;
    if (useAreaTest)
    {
        for (uint32_t i = 0; i < mHullFaces.size(); i++)
        {
            if (mHullFaces[i]->area < areaEpsilon)
            {
                return QuickHullResult::eZERO_AREA_TEST_FAILED;
            }
        }
    }

    // add points to the hull
    uint32_t numVerts = 4; // initial vertex count - simplex vertices		
    while ((eyeVtx = nextPointToAdd(eyeFace)) != NULL && eyeVtx->index != mTerminalVertex)
    {
        // if plane shifting vertex limit, we need the reduced hull
// POSSIBLY NEED TO ADD THIS BACK IN AGAIN: TODO TODO TODO
//        if ((mConvexDesc.flags & QhConvexFlag::ePLANE_SHIFTING) && (numVerts >= mConvexDesc.vertexLimit))
//            break;

        bool addFailed = false;
        assert(eyeFace);
        if (!addPointToHull(eyeVtx, *eyeFace, addFailed))
        {
            mOutputNumVertices = numVerts;
            // we hit the polygons hard limit
            return QuickHullResult::ePOLYGONS_LIMIT_REACHED;
        }
        // We failed to add the vertex, store the vertex as terminal vertex and re run the hull generator
        if (addFailed)
        {
            // set the terminal vertex
            mTerminalVertex = eyeVtx->index;

            // reset the edges/faces memory
            mFreeHalfEdges.reset();
            mFreeFaces.reset();

            // reset the hull state
            mHullFaces.clear();
            mNumHullFaces = 0;
            mUnclaimedPoints.clear();
            mHorizon.clear();
            mNewFaces.clear();
            mRemovedFaces.clear();
            mDiscardedFaces.clear();

            // rerun the hull generator
            return buildHull();
        }
        numVerts++;
    }
    mOutputNumVertices = numVerts;

    // vertex limit has been reached. We did not stopped the iteration, since we
    // will use the produced hull to compute OBB from it and use the planes
    // to slice the initial OBB
    if (numVerts > mMaxHullVerts)
    {
        return QuickHullResult::eVERTEX_LIMIT_REACHED;
    }

    return QuickHullResult::eSUCCESS;
}

//////////////////////////////////////////////////////////////////////////
// finds the best point to add to the hull
// go through the faces conflict list and pick the global maximum
QuickHullVertex* QuickHullImpl::nextPointToAdd(QuickHullFace*& eyeFace)
{
    QuickHullVertex* eyeVtx = NULL;
    QuickHullFace* eyeF = NULL;
    double maxDist = mPlaneTolerance;
    for (uint32_t i = 0; i < mHullFaces.size(); i++)
    {
        if (mHullFaces[i]->state == QuickHullFace::eVISIBLE && mHullFaces[i]->conflictList)
        {
            const double dist = mHullFaces[i]->conflictList->dist;
            if (maxDist < dist)
            {
                maxDist = dist;
                eyeVtx = mHullFaces[i]->conflictList;
                eyeF = mHullFaces[i];
            }
        }
    }

    eyeFace = eyeF;
    return eyeVtx;
}

//////////////////////////////////////////////////////////////////////////
// adds vertex to the hull
// sets addFailed to true if we failed to add a point because the merging failed
// this can happen as the face plane equation changes and some faces might become concave
// returns false if the new faces count would hit the hull face hard limit (255)
bool QuickHullImpl::addPointToHull(const QuickHullVertex* eyeVtx, QuickHullFace& eyeFace, bool& addFailed)
{
    addFailed = false;

    // removes the eyePoint from the conflict list
    removeEyePointFromFace(eyeFace, eyeVtx);

    // calculates the horizon from the eyePoint
    calculateHorizon(eyeVtx->point, NULL, eyeFace, mHorizon, mRemovedFaces);
#if 0
    // check if we dont hit the polygons hard limit
    if (mNumHullFaces + mHorizon.size() > 255)
    {
        // make the faces visible again and quit 
        for (uint32_t i = 0; i < mRemovedFaces.size(); i++)
        {
            mRemovedFaces[i]->state = QuickHullFace::eVISIBLE;
        }
        mNumHullFaces += (uint32_t)mRemovedFaces.size();
        return false;
    }
#endif
    // adds new faces from given horizon and eyePoint
    addNewFacesFromHorizon(eyeVtx, mHorizon, mNewFaces);

    bool mergeFailed = false;
    // first merge pass ... merge faces which are non-convex
    // as determined by the larger face
    for (uint32_t i = 0; i < (uint32_t)mNewFaces.size(); i++)
    {
        QuickHullFace& face = *mNewFaces[i];

        if (face.state == QuickHullFace::eVISIBLE)
        {
            assert(face.checkFaceConsistency());
            while (doAdjacentMerge(face, true, mergeFailed));
        }
    }
    if (mergeFailed)
    {
        addFailed = true;
        return true;
    }

    // second merge pass ... merge faces which are non-convex
    // wrt either face	     
    for (uint32_t i = 0; i < mNewFaces.size(); i++)
    {
        QuickHullFace& face = *mNewFaces[i];
        if (face.state == QuickHullFace::eNON_CONVEX)
        {
            face.state = QuickHullFace::eVISIBLE;
            while (doAdjacentMerge(face, false, mergeFailed));
        }
    }
    if (mergeFailed)
    {
        addFailed = true;
        return true;
    }

    resolveUnclaimedPoints(mNewFaces);

    mHorizon.clear();
    mNewFaces.clear();
    mRemovedFaces.clear();

    return true;
}

//////////////////////////////////////////////////////////////////////////
// merge adjacent faces
// We merge 2 adjacent faces if they lie on the same thick plane defined by the mTolerance
// we do this in 2 steps to ensure we dont leave non-convex faces
bool QuickHullImpl::doAdjacentMerge(QuickHullFace& face, bool mergeWrtLargeFace, bool& mergeFailed)
{
    QuickHullHalfEdge* hedge = face.edge;
    mergeFailed = false;

    bool convex = true;
    do
    {
        const QuickHullFace& oppFace = *hedge->getOppositeFace();
        bool merge = false;

        if (mergeWrtLargeFace)
        {
            // merge faces if they are parallel or non-convex
            // wrt to the larger face; otherwise, just mark
            // the face non-convex for the second pass.
            if (face.area > oppFace.area)
            {
                if (hedge->getOppositeFaceDistance() > -mTolerance)
                {
                    merge = true;
                }
                else if (hedge->twin->getOppositeFaceDistance() > -mTolerance)
                {
                    convex = false;
                }
            }
            else
            {
                if (hedge->twin->getOppositeFaceDistance() > -mTolerance)
                {
                    merge = true;
                }
                else if (hedge->getOppositeFaceDistance() > -mTolerance)
                {
                    convex = false;
                }
            }
        }
        else
        {
            // then merge faces if they are definitively non-convex
            if (hedge->getOppositeFaceDistance() > -mTolerance ||
                hedge->twin->getOppositeFaceDistance() > -mTolerance)
            {
                merge = true;
            }
        }

        if (merge)
        {
            mDiscardedFaces.clear();
            if (!face.mergeAdjacentFace(hedge, mDiscardedFaces))
            {
                mergeFailed = true;
                return false;
            }
            mNumHullFaces -= (uint32_t)mDiscardedFaces.size();
            for (uint32_t i = 0; i < mDiscardedFaces.size(); i++)
            {
                deleteFacePoints(*mDiscardedFaces[i], &face);
            }
            assert(face.checkFaceConsistency());
            return true;
        }
        hedge = hedge->next;
    } while (hedge != face.edge);

    if (!convex)
    {
        face.state = QuickHullFace::eNON_CONVEX;
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////
// return the distance from opposite face
double QuickHullHalfEdge::getOppositeFaceDistance() const
{
    assert(face);
    assert(twin);
    return face->distanceToPlane(twin->face->centroid);
}

//////////////////////////////////////////////////////////////////////////
// merge adjacent face from provided half edge. 
// 1. set new half edges
// 2. connect the new half edges - check we did not produced redundant triangles, discard them
// 3. recompute the plane and check consistency
// Returns false if merge failed
bool QuickHullFace::mergeAdjacentFace(QuickHullHalfEdge* hedgeAdj, QuickHullFaceArray& discardedFaces)
{
    QuickHullFace* oppFace = hedgeAdj->getOppositeFace();

    discardedFaces.push_back(oppFace);
    oppFace->state = QuickHullFace::eDELETED;

    QuickHullHalfEdge* hedgeOpp = hedgeAdj->twin;

    QuickHullHalfEdge* hedgeAdjPrev = hedgeAdj->prev;
    QuickHullHalfEdge* hedgeAdjNext = hedgeAdj->next;
    QuickHullHalfEdge* hedgeOppPrev = hedgeOpp->prev;
    QuickHullHalfEdge* hedgeOppNext = hedgeOpp->next;

    // check if we are lining up with the face in adjPrev dir
    QuickHullHalfEdge* breakEdge = hedgeAdjPrev;
    while (hedgeAdjPrev->getOppositeFace() == oppFace)
    {
        hedgeAdjPrev = hedgeAdjPrev->prev;
        hedgeOppNext = hedgeOppNext->next;

        // Edge case merge face is degenerated and we need to abort merging
        if (hedgeAdjPrev == breakEdge)
        {
            return false;
        }
    }

    // check if we are lining up with the face in adjNext dir
    breakEdge = hedgeAdjNext;
    while (hedgeAdjNext->getOppositeFace() == oppFace)
    {
        hedgeOppPrev = hedgeOppPrev->prev;
        hedgeAdjNext = hedgeAdjNext->next;

        // Edge case merge face is degenerated and we need to abort merging
        if (hedgeAdjNext == breakEdge)
        {
            return false;
        }
    }

    QuickHullHalfEdge* hedge;

    // set new face owner for the line up edges
    for (hedge = hedgeOppNext; hedge != hedgeOppPrev->next; hedge = hedge->next)
    {
        hedge->face = this;
    }

    // if we are about to delete the shared edge, check if its not the starting edge of the face
    if (hedgeAdj == edge)
    {
        edge = hedgeAdjNext;
    }

    // handle the half edges at the head
    QuickHullFace* discardedFace;
    discardedFace = connectHalfEdges(hedgeOppPrev, hedgeAdjNext);
    if (discardedFace != NULL)
    {
        discardedFaces.push_back(discardedFace);
    }

    // handle the half edges at the tail
    discardedFace = connectHalfEdges(hedgeAdjPrev, hedgeOppNext);
    if (discardedFace != NULL)
    {
        discardedFaces.push_back(discardedFace);
    }

    computeNormalAndCentroid();
    assert(checkFaceConsistency());

    return true;
}

//////////////////////////////////////////////////////////////////////////
// connect half edges of 2 adjacent faces
// if we find redundancy - edges are in a line, we drop the addional face if it is just a skinny triangle
QuickHullFace* QuickHullFace::connectHalfEdges(QuickHullHalfEdge* hedgePrev, QuickHullHalfEdge* hedge)
{
    QuickHullFace* discardedFace = NULL;

    // redundant edge - can be in a line
    if (hedgePrev->getOppositeFace() == hedge->getOppositeFace())
    {
        // then there is a redundant edge that we can get rid off
        QuickHullFace* oppFace = hedge->getOppositeFace();
        QuickHullHalfEdge* hedgeOpp;

        if (hedgePrev == edge)
        {
            edge = hedge;
        }

        // check if its not a skinny face with just 3 vertices - 3 edges
        if (oppFace->isTriangle())
        {
            // then we can get rid of the opposite face altogether
            hedgeOpp = hedge->twin->prev->twin;

            oppFace->state = QuickHullFace::eDELETED;
            discardedFace = oppFace;
        }
        else
        {
            // if not triangle, merge the 2 opposite halfedges into one
            hedgeOpp = hedge->twin->next;

            if (oppFace->edge == hedgeOpp->prev)
            {
                oppFace->edge = hedgeOpp;
            }
            hedgeOpp->prev = hedgeOpp->prev->prev;
            hedgeOpp->prev->next = hedgeOpp;
        }

        hedge->prev = hedgePrev->prev;
        hedge->prev->next = hedge;

        hedge->twin = hedgeOpp;
        hedgeOpp->twin = hedge;

        // oppFace was modified, so need to recompute
        oppFace->computeNormalAndCentroid();
    }
    else
    {
        // just merge the halfedges
        hedgePrev->next = hedge;
        hedge->prev = hedgePrev;
    }
    return discardedFace;
}

//////////////////////////////////////////////////////////////////////////
// check face consistency
bool QuickHullFace::checkFaceConsistency()
{
    // do a sanity check on the face
    QuickHullHalfEdge* hedge = edge;
    uint32_t numv = 0;

    // check degenerate face
    do
    {
        numv++;
        hedge = hedge->next;
    } while (hedge != edge);

    // degenerate face found
    assert(numv > 2);

    numv = 0;
    hedge = edge;
    do
    {
        QuickHullHalfEdge* hedgeOpp = hedge->twin;

        // check if we have twin set
        assert(hedgeOpp != NULL);

        // twin for the twin must be the original edge
        assert(hedgeOpp->twin == hedge);

        QuickHullFace* oppFace = hedgeOpp->face;

        QH_UNUSED(oppFace);

        // opposite edge face must be set and valid
        assert(oppFace != NULL);
        assert(oppFace->state != QuickHullFace::eDELETED);

        // edges face must be this one
        assert(hedge->face == this);

        hedge = hedge->next;
    } while (hedge != edge);

    return true;
}



//////////////////////////////////////////////////////////////////////////
// merge adjacent faces doing normal test
// we try to merge more aggressively 2 faces with the same normal. 	
bool QuickHullImpl::doPostAdjacentMerge(QuickHullFace& face, const double maxdot_minang)
{
    QuickHullHalfEdge* hedge = face.edge;

    do
    {
        const QuickHullFace& oppFace = *hedge->getOppositeFace();
        bool merge = false;
        const QhVec3& ni = face.normal;
        const QhVec3& nj = oppFace.normal;
        const double dotP = ni.dot(nj);

        if (dotP > maxdot_minang)
        {
            if (face.area >= oppFace.area)
            {
                // check if we can merge the 2 faces
                merge = canMergeFaces(*hedge);
            }
        }

        if (merge)
        {
            QuickHullFaceArray discardedFaces;
            face.mergeAdjacentFace(hedge, discardedFaces);
            mNumHullFaces -= (uint32_t)discardedFaces.size();
            for (uint32_t i = 0; i < (uint32_t)discardedFaces.size(); i++)
            {
                deleteFacePoints(*discardedFaces[i], &face);
            }
            assert(face.checkFaceConsistency());
            return true;
        }
        hedge = hedge->next;
    } while (hedge != face.edge);

    return false;
}

//////////////////////////////////////////////////////////////////////////
// checks if 2 adjacent faces can be merged
// 1. creates a face with merged vertices
// 2. computes new normal and centroid
// 3. checks that all verts are not too far away from the plane
// 4. checks that the new polygon is still convex
// 5. checks if we are about to merge only 2 neighbor faces, we dont 
// want to merge additional faces, that might corrupt the convexity
bool QuickHullImpl::canMergeFaces(const QuickHullHalfEdge& he)
{
    const QuickHullFace& face1 = *he.face;
    const QuickHullFace& face2 = *he.twin->face;

    // construct the merged face
    QuickHullHalfEdge *edges = (QuickHullHalfEdge *)malloc(sizeof(QuickHullHalfEdge)*(face1.numEdges + face2.numEdges));
    memset(edges, 0, (face1.numEdges + face2.numEdges) * sizeof(QuickHullHalfEdge));
    QuickHullFace mergedFace;
    mergedFace.edge = &edges[0];

    // copy the first face edges
    uint32_t currentEdge = 0;
    const QuickHullHalfEdge* heTwin = NULL;
    const QuickHullHalfEdge* heCopy = NULL;
    const QuickHullHalfEdge* startEdge = (face1.edge != &he) ? face1.edge : face1.edge->next;
    const QuickHullHalfEdge* copyHe = startEdge;
    do
    {
        edges[currentEdge].face = &mergedFace;
        edges[currentEdge].tail = copyHe->tail;
        if (copyHe == &he)
        {
            heTwin = copyHe->twin;
            heCopy = &edges[currentEdge];
        }
        const uint32_t nextIndex = (copyHe->next == startEdge) ? 0 : currentEdge + 1;
        const uint32_t prevIndex = (currentEdge == 0) ? face1.numEdges - 1 : currentEdge - 1;
        edges[currentEdge].next = &edges[nextIndex];
        edges[currentEdge].prev = &edges[prevIndex];

        currentEdge++;
        copyHe = copyHe->next;
    } while (copyHe != startEdge);

    // copy the second face edges
    copyHe = face2.edge;
    do
    {
        edges[currentEdge].face = &mergedFace;
        edges[currentEdge].tail = copyHe->tail;
        if (heTwin == copyHe)
            heTwin = &edges[currentEdge];
        const uint32_t nextIndex = (copyHe->next == face2.edge) ? face1.numEdges : currentEdge + 1;
        const uint32_t prevIndex = (currentEdge == face1.numEdges) ? face1.numEdges + face2.numEdges - 1 : currentEdge - 1;
        edges[currentEdge].next = &edges[nextIndex];
        edges[currentEdge].prev = &edges[prevIndex];

        currentEdge++;
        copyHe = copyHe->next;
    } while (copyHe != face2.edge);

    assert(heTwin);
    if (!heCopy || !heTwin)
    {
        return false;
    }

    QuickHullHalfEdge* hedgeAdjPrev = heCopy->prev;
    QuickHullHalfEdge* hedgeAdjNext = heCopy->next;
    QuickHullHalfEdge* hedgeOppPrev = heTwin->prev;
    QuickHullHalfEdge* hedgeOppNext = heTwin->next;

    hedgeOppPrev->next = hedgeAdjNext;
    hedgeAdjNext->prev = hedgeOppPrev;

    hedgeAdjPrev->next = hedgeOppNext;
    hedgeOppNext->prev = hedgeAdjPrev;

    // compute normal and centroid
    mergedFace.computeNormalAndCentroid();

    // test the vertex distance
    const double maxDist = mPlaneTolerance;
    for (uint32_t iVerts = 0; iVerts < mNumVertices; iVerts++)
    {
        const QuickHullVertex& vertex = mVerticesList[iVerts];
        const double dist = mergedFace.distanceToPlane(vertex.point);
        if (dist > maxDist)
        {
            free(edges);
            return false;
        }
    }

    // check the convexity
    QuickHullHalfEdge* qhe = mergedFace.edge;
    do
    {
        const QuickHullVertex& vertex = qhe->tail;
        const QuickHullVertex& nextVertex = qhe->next->tail;

        QhVec3 edgeVector = nextVertex.point - vertex.point;
        edgeVector.normalize();
        const QhVec3 outVector = -mergedFace.normal.cross(edgeVector);

        QuickHullHalfEdge* testHe = qhe->next;
        do
        {
            const QuickHullVertex& testVertex = testHe->tail;
            const double dist = (testVertex.point - vertex.point).dot(outVector);

            if (dist > mTolerance)
            {
                free(edges);
                return false;
            }

            testHe = testHe->next;
        } while (testHe != qhe->next);

        qhe = qhe->next;
    } while (qhe != mergedFace.edge);


    const QuickHullFace* oppFace = he.getOppositeFace();

    QuickHullHalfEdge* hedgeOpp = he.twin;

    hedgeAdjPrev = he.prev;
    hedgeAdjNext = he.next;
    hedgeOppPrev = hedgeOpp->prev;
    hedgeOppNext = hedgeOpp->next;

    // check if we are lining up with the face in adjPrev dir
    while (hedgeAdjPrev->getOppositeFace() == oppFace)
    {
        hedgeAdjPrev = hedgeAdjPrev->prev;
        hedgeOppNext = hedgeOppNext->next;
    }

    // check if we are lining up with the face in adjNext dir
    while (hedgeAdjNext->getOppositeFace() == oppFace)
    {
        hedgeOppPrev = hedgeOppPrev->prev;
        hedgeAdjNext = hedgeAdjNext->next;
    }

    // no redundant merges, just clean merge of 2 neighbour faces
    if (hedgeOppPrev->getOppositeFace() == hedgeAdjNext->getOppositeFace())
    {
        free(edges);
        return false;
    }

    if (hedgeAdjPrev->getOppositeFace() == hedgeOppNext->getOppositeFace())
    {
        free(edges);
        return false;
    }
    free(edges);
    return true;
}

//////////////////////////////////////////////////////////////////////////
// delete face points and store them as unclaimed, so we can add them back to new faces later
void QuickHullImpl::deleteFacePoints(QuickHullFace& face, QuickHullFace* absorbingFace)
{
    // no conflict list for this face
    if (!face.conflictList)
        return;

    QuickHullVertex* unclaimedVertex = face.conflictList;
    QuickHullVertex* vertexToClaim = NULL;
    while (unclaimedVertex)
    {
        vertexToClaim = unclaimedVertex;
        unclaimedVertex = unclaimedVertex->next;
        vertexToClaim->next = NULL;
        if (!absorbingFace)
        {
            mUnclaimedPoints.push_back(vertexToClaim);
        }
        else
        {
            const double dist = absorbingFace->distanceToPlane(vertexToClaim->point);
            if (dist > mTolerance)
            {
                addPointToFace(*absorbingFace, vertexToClaim, dist);
            }
            else
            {
                mUnclaimedPoints.push_back(vertexToClaim);
            }
        }
    }

    face.conflictList = NULL;
}

//////////////////////////////////////////////////////////////////////////
// calculate the horizon from the eyePoint against a given face
void QuickHullImpl::calculateHorizon(const QhVec3& eyePoint, QuickHullHalfEdge* edge0, QuickHullFace& face, QuickHullHalfEdgeArray& horizon, QuickHullFaceArray& removedFaces)
{
    deleteFacePoints(face, NULL);
    face.state = QuickHullFace::eDELETED;
    removedFaces.push_back(&face);
    mNumHullFaces--;
    QuickHullHalfEdge* edge;
    if (edge0 == NULL)
    {
        edge0 = face.getEdge(0);
        edge = edge0;
    }
    else
    {
        edge = edge0->next;
    }

    do
    {
        QuickHullFace* oppFace = edge->getOppositeFace();
        if (oppFace->state == QuickHullFace::eVISIBLE)
        {
            const double dist = oppFace->distanceToPlane(eyePoint);
            if (dist > mTolerance)
            {
                calculateHorizon(eyePoint, edge->twin, *oppFace, horizon, removedFaces);
            }
            else
            {
                horizon.push_back(edge);
            }
        }
        edge = edge->next;
    } while (edge != edge0);
}

//////////////////////////////////////////////////////////////////////////
// adds new faces from given horizon and eyePoint
void QuickHullImpl::addNewFacesFromHorizon(const QuickHullVertex* eyePoint, const QuickHullHalfEdgeArray& horizon, QuickHullFaceArray& newFaces)
{
    QuickHullHalfEdge* hedgeSidePrev = NULL;
    QuickHullHalfEdge* hedgeSideBegin = NULL;

    for (uint32_t i = 0; i < horizon.size(); i++)
    {
        const QuickHullHalfEdge& horizonHe = *horizon[i];

        QuickHullFace* face = createTriangle(*eyePoint, horizonHe.getHead(), horizonHe.getTail());
        mHullFaces.push_back(face);
        mNumHullFaces++;
        face->getEdge(2)->setTwin(horizonHe.twin);

        QuickHullHalfEdge* hedgeSide = face->edge;
        if (hedgeSidePrev != NULL)
        {
            hedgeSide->next->setTwin(hedgeSidePrev);
        }
        else
        {
            hedgeSideBegin = hedgeSide;
        }
        newFaces.push_back(face);
        hedgeSidePrev = hedgeSide;
    }
    if (hedgeSideBegin && hedgeSideBegin->next)
        hedgeSideBegin->next->setTwin(hedgeSidePrev);
}

//////////////////////////////////////////////////////////////////////////
// resolve unclaimed points
void QuickHullImpl::resolveUnclaimedPoints(const QuickHullFaceArray& newFaces)
{
    for (uint32_t i = 0; i < mUnclaimedPoints.size(); i++)
    {
        QuickHullVertex* vtx = mUnclaimedPoints[i];

        double maxDist = mTolerance;
        QuickHullFace* maxFace = NULL;
        for (uint32_t j = 0; j < newFaces.size(); j++)
        {
            const QuickHullFace& newFace = *newFaces[j];
            if (newFace.state == QuickHullFace::eVISIBLE)
            {
                const double dist = newFace.distanceToPlane(vtx->point);
                if (dist > maxDist)
                {
                    maxDist = dist;
                    maxFace = newFaces[j];
                }
            }
        }
        if (maxFace != NULL)
        {
            addPointToFace(*maxFace, vtx, maxDist);
        }
    }

    mUnclaimedPoints.clear();
}

QuickHull *QuickHull::create(void)
{
    auto ret = new QuickHullImpl;
    return static_cast< QuickHull *>(ret);
}

} // end of quickhull namespace
