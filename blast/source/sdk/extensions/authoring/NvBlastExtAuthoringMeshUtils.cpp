#include "NvBlastExtAuthoringMeshUtils.h"
#include "NvBlastExtAuthoringMeshImpl.h"
#include "NvBlastExtAuthoringPerlinNoise.h"
#include "NvBlastExtAuthoringFractureTool.h"
#include <NvBlastNvSharedHelpers.h>
#include <NvCMath.h>
#include <algorithm>


using namespace nvidia;

#define UV_SCALE 1.f

#define CYLINDER_UV_SCALE (UV_SCALE * 1.732f)


namespace Nv
{
namespace Blast
{

void getTangents(const NvVec3& normal, NvVec3& t1, NvVec3& t2)
{

    if (std::abs(normal.z) < 0.9)
    {
        t1 = normal.cross(NvVec3(0, 0, 1));
    }
    else
    {
        t1 = normal.cross(NvVec3(1, 0, 0));
    }
    t2 = t1.cross(normal);
}

Mesh* getCuttingBox(const NvVec3& point, const NvVec3& normal, float size, int64_t id, int32_t interiorMaterialId)
{
    NvVec3 lNormal = normal.getNormalized();
    NvVec3 t1, t2;
    getTangents(lNormal, t1, t2);

    std::vector<Vertex> positions(8);
    toNvShared(positions[0].p) = point + (t1 + t2) * size;
    toNvShared(positions[1].p) = point + (t2 - t1) * size;

    toNvShared(positions[2].p) = point + (-t1 - t2) * size;
    toNvShared(positions[3].p) = point + (t1 - t2) * size;


    toNvShared(positions[4].p) = point + (t1 + t2 + lNormal) * size;
    toNvShared(positions[5].p) = point + (t2 - t1 + lNormal) * size;

    toNvShared(positions[6].p) = point + (-t1 - t2 + lNormal) * size;
    toNvShared(positions[7].p) = point + (t1 - t2 + lNormal) * size;

    toNvShared(positions[0].n) = -lNormal;
    toNvShared(positions[1].n) = -lNormal;

    toNvShared(positions[2].n) = -lNormal;
    toNvShared(positions[3].n) = -lNormal;


    toNvShared(positions[4].n) = -lNormal;
    toNvShared(positions[5].n) = -lNormal;

    toNvShared(positions[6].n) = -lNormal;
    toNvShared(positions[7].n) = -lNormal;

    positions[0].uv[0] = { 0, 0 };
    positions[1].uv[0] = {UV_SCALE, 0};

    positions[2].uv[0] = {UV_SCALE, UV_SCALE};
    positions[3].uv[0] = {0, UV_SCALE};


    positions[4].uv[0] = {0, 0};
    positions[5].uv[0] = {UV_SCALE, 0};

    positions[6].uv[0] = {UV_SCALE, UV_SCALE};
    positions[7].uv[0] = {0, UV_SCALE};


    std::vector<Edge> edges;
    std::vector<Facet> facets;

    edges.push_back({0, 1});
    edges.push_back({1, 2});
    edges.push_back({2, 3});
    edges.push_back({3, 0});
    facets.push_back({0, 4, id, interiorMaterialId, -1});


    edges.push_back({0, 3});
    edges.push_back({3, 7});
    edges.push_back({7, 4});
    edges.push_back({4, 0});
    facets.push_back({4, 4, id, interiorMaterialId, -1});

    edges.push_back({3, 2});
    edges.push_back({2, 6});
    edges.push_back({6, 7});
    edges.push_back({7, 3});
    facets.push_back({8, 4, id, interiorMaterialId, -1});

    edges.push_back({5, 6});
    edges.push_back({6, 2});
    edges.push_back({2, 1});
    edges.push_back({1, 5});
    facets.push_back({12, 4, id, interiorMaterialId, -1});

    edges.push_back({4, 5});
    edges.push_back({5, 1});
    edges.push_back({1, 0});
    edges.push_back({0, 4});
    facets.push_back({16, 4, id, interiorMaterialId, -1});

    edges.push_back({4, 7});
    edges.push_back({7, 6});
    edges.push_back({6, 5});
    edges.push_back({5, 4});
    facets.push_back({20, 4, id, interiorMaterialId, -1});
    return new MeshImpl(positions.data(), edges.data(), facets.data(), static_cast<uint32_t>(positions.size()),
                        static_cast<uint32_t>(edges.size()), static_cast<uint32_t>(facets.size()));
}

void inverseNormalAndIndices(Mesh* mesh)
{
    for (uint32_t i = 0; i < mesh->getVerticesCount(); ++i)
    {
        toNvShared(mesh->getVerticesWritable()[i].n) *= -1.0f;
    }
    for (uint32_t i = 0; i < mesh->getFacetCount(); ++i)
    {
        mesh->getFacetWritable(i)->userData = -mesh->getFacet(i)->userData;
    }
}

void setCuttingBox(const NvVec3& point, const NvVec3& normal, Mesh* mesh, float size, int64_t id)
{
    NvVec3 t1, t2;
    NvVec3 lNormal = normal.getNormalized();
    getTangents(lNormal, t1, t2);

    Vertex* positions = mesh->getVerticesWritable();
    toNvShared(positions[0].p) = point + (t1 + t2) * size;
    toNvShared(positions[1].p) = point + (t2 - t1) * size;

    toNvShared(positions[2].p) = point + (-t1 - t2) * size;
    toNvShared(positions[3].p) = point + (t1 - t2) * size;


    toNvShared(positions[4].p) = point + (t1 + t2 + lNormal) * size;
    toNvShared(positions[5].p) = point + (t2 - t1 + lNormal) * size;

    toNvShared(positions[6].p) = point + (-t1 - t2 + lNormal) * size;
    toNvShared(positions[7].p) = point + (t1 - t2 + lNormal) * size;

    toNvShared(positions[0].n) = -lNormal;
    toNvShared(positions[1].n) = -lNormal;

    toNvShared(positions[2].n) = -lNormal;
    toNvShared(positions[3].n) = -lNormal;


    toNvShared(positions[4].n) = -lNormal;
    toNvShared(positions[5].n) = -lNormal;

    toNvShared(positions[6].n) = -lNormal;
    toNvShared(positions[7].n) = -lNormal;

    for (uint32_t i = 0; i < mesh->getFacetCount(); ++i)
    {
        mesh->getFacetWritable(i)->userData = id;
    }
    mesh->recalculateBoundingBox();
}

struct Stepper
{
    virtual nvidia::NvVec3 getStep1(uint32_t w, uint32_t h) const  = 0;
    virtual nvidia::NvVec3 getStep2(uint32_t w) const              = 0;
    virtual nvidia::NvVec3 getStart() const                        = 0;
    virtual nvidia::NvVec3 getNormal(uint32_t w, uint32_t h) const = 0;
    virtual bool isStep2ClosedLoop() const
    {
        return false;
    }
    virtual bool isStep2FreeBoundary() const
    {
        return false;
    }
};

struct PlaneStepper : public Stepper
{
    PlaneStepper(const nvidia::NvVec3& normal, const nvidia::NvVec3& point, float sizeX, float sizeY,
                 uint32_t resolutionX, uint32_t resolutionY, bool swapTangents = false)
    {
        NvVec3 t1, t2;
        lNormal = normal.getNormalized();
        getTangents(lNormal, t1, t2);
        if (swapTangents)
        {
            std::swap(t1, t2);
        }
        t11d = -t1 * 2.0f * sizeX / resolutionX;
        t12d = -t2 * 2.0f * sizeY / resolutionY;
        t21d = t11d;
        t22d = t12d;
        cPos = point + (t1 * sizeX + t2 * sizeY);
        resY = resolutionY;
    }
    // Define face by 4 corner points, points should lay in plane
    PlaneStepper(const nvidia::NvVec3& p11, const nvidia::NvVec3& p12, const nvidia::NvVec3& p21, const nvidia::NvVec3& p22,
                 uint32_t resolutionX, uint32_t resolutionY)
    {
        lNormal = -(p21 - p11).cross(p12 - p11).getNormalized();
        if (lNormal.magnitude() < 1e-5)
        {
            lNormal = (p21 - p22).cross(p12 - p22).getNormalized();
        }
        t11d = (p11 - p21) / resolutionX;
        t12d = (p12 - p11) / resolutionY;
        t21d = (p12 - p22) / resolutionX;
        t22d = (p22 - p21) / resolutionY;
        cPos = p21;
        resY = resolutionY;
    }
    nvidia::NvVec3 getStep1(uint32_t y, uint32_t) const
    {
        return (t11d * (resY - y) + t21d * y) / resY;
    }
    nvidia::NvVec3 getStep2(uint32_t) const
    {
        return t22d;
    }
    nvidia::NvVec3 getStart() const
    {
        return cPos;
    }
    nvidia::NvVec3 getNormal(uint32_t, uint32_t) const
    {
        return lNormal;
    }

    NvVec3 t11d, t12d, t21d, t22d, cPos, lNormal;
    uint32_t resY;
};

void fillEdgesAndFaces(std::vector<Edge>& edges, std::vector<Facet>& facets, uint32_t h, uint32_t w,
                       uint32_t firstVertex, uint32_t verticesCount, int64_t id, int32_t interiorMaterialId,
                       int32_t smoothingGroup = -1, bool reflected = false)
{
    for (uint32_t i = 0; i < w; ++i)
    {
        for (uint32_t j = 0; j < h; ++j)
        {
            int32_t start = edges.size();
            uint32_t idx00 = i * (h + 1) + j + firstVertex;
            uint32_t idx01 = idx00 + 1;
            uint32_t idx10 = (idx00 + h + 1) % verticesCount;
            uint32_t idx11 = (idx01 + h + 1) % verticesCount;
            if (reflected)
            {
                edges.push_back({idx01, idx11});
                edges.push_back({idx11, idx10});
                edges.push_back({idx10, idx01});
                facets.push_back({start, 3, id, interiorMaterialId, smoothingGroup});

                start = edges.size();
                edges.push_back({idx01, idx10});
                edges.push_back({idx10, idx00});
                edges.push_back({idx00, idx01});
                facets.push_back({start, 3, id, interiorMaterialId, smoothingGroup});
            }
            else
            {
                edges.push_back({idx00, idx01});
                edges.push_back({idx01, idx11});
                edges.push_back({idx11, idx00});
                facets.push_back({start, 3, id, interiorMaterialId, smoothingGroup});

                start = edges.size();
                edges.push_back({idx00, idx11});
                edges.push_back({idx11, idx10});
                edges.push_back({idx10, idx00});
                facets.push_back({start, 3, id, interiorMaterialId, smoothingGroup});
            }
        }
    }
}

void getNoisyFace(std::vector<Vertex>& vertices, std::vector<Edge>& edges, std::vector<Facet>& facets, uint32_t h,
                  uint32_t w, const nvidia::NvVec2& uvOffset, const nvidia::NvVec2& uvScale, const Stepper& stepper,
                  SimplexNoise& nEval, int64_t id, int32_t interiorMaterialId, bool randomizeLast = false)
{
    uint32_t randIdx     = randomizeLast ? 1 : 0;
    NvVec3 cPosit        = stepper.getStart();
    uint32_t firstVertex = vertices.size();
    for (uint32_t i = 0; i < w + 1; ++i)
    {
        NvVec3 lcPosit = cPosit;
        for (uint32_t j = 0; j < h + 1; ++j)
        {
            vertices.push_back(Vertex());
            toNvShared(vertices.back().p) = lcPosit;
            toNvShared(vertices.back().uv[0]) = uvOffset + uvScale.multiply(nvidia::NvVec2(j, i));
            lcPosit += stepper.getStep1(i, j);
        }
        cPosit += stepper.getStep2(i);
    }

    for (uint32_t i = 1 - randIdx; i < w + randIdx; ++i)
    {
        for (uint32_t j = 1; j < h; ++j)
        {
            // TODO limit max displacement for cylinder
            NvVec3& pnt = toNvShared(vertices[i * (h + 1) + j + firstVertex].p);
            pnt += stepper.getNormal(i, j) * nEval.sample(pnt);
        }
    }

    fillEdgesAndFaces(edges, facets, h, w, firstVertex, vertices.size(), id, interiorMaterialId);
}

 uint32_t unsignedMod(int32_t n, uint32_t modulus)
{
    const int32_t d = n / (int32_t)modulus;
    const int32_t m = n - d * (int32_t)modulus;
    return m >= 0 ? (uint32_t)m : (uint32_t)m + modulus;
}

void calculateNormals(std::vector<Vertex>& vertices, uint32_t h, uint32_t w, bool inverseNormals = false)
{
    for (uint32_t i = 1; i < w; ++i)
    {
        for (uint32_t j = 1; j < h; ++j)
        {
            int32_t idx = i * (h + 1) + j;
            NvVec3 v1   = toNvShared(vertices[idx + h + 1].p - vertices[idx].p);
            NvVec3 v2   = toNvShared(vertices[idx + 1].p - vertices[idx].p);
            NvVec3 v3   = toNvShared(vertices[idx - (h + 1)].p - vertices[idx].p);
            NvVec3 v4   = toNvShared(vertices[idx - 1].p - vertices[idx].p);

            NvVec3& n = toNvShared(vertices[idx].n);
            n = v1.cross(v2) + v2.cross(v3) + v3.cross(v4) + v4.cross(v1);
            if (inverseNormals)
            {
                n = -n;
            }
            n.normalize();
        }
    }
}

Mesh* getNoisyCuttingBoxPair(const nvidia::NvVec3& point, const nvidia::NvVec3& normal, float size, float jaggedPlaneSize,
                             nvidia::NvVec3 resolution, int64_t id, float amplitude, float frequency, int32_t octaves,
                             int32_t seed, int32_t interiorMaterialId)
{
    NvVec3 t1, t2;
    NvVec3 lNormal = normal.getNormalized();
    getTangents(lNormal, t1, t2);
    float sz = 2.f * jaggedPlaneSize;
    uint32_t resolutionX =
        std::max(1u, (uint32_t)std::roundf(sz * std::abs(t1.x) * resolution.x + sz * std::abs(t1.y) * resolution.y +
                                           sz * std::abs(t1.z) * resolution.z));
    uint32_t resolutionY =
        std::max(1u, (uint32_t)std::roundf(sz * std::abs(t2.x) * resolution.x + sz * std::abs(t2.y) * resolution.y +
                                           sz * std::abs(t2.z) * resolution.z));

    PlaneStepper stepper(normal, point, jaggedPlaneSize, jaggedPlaneSize, resolutionX, resolutionY);
    SimplexNoise nEval(amplitude, frequency, octaves, seed);

    std::vector<Vertex> vertices;
    vertices.reserve((resolutionX + 1) * (resolutionY + 1) + 12);
    std::vector<Edge> edges;
    std::vector<Facet> facets;
    getNoisyFace(vertices, edges, facets, resolutionX, resolutionY, nvidia::NvVec2(0.f),
                 nvidia::NvVec2(UV_SCALE / resolutionX, UV_SCALE / resolutionY), stepper, nEval, id, interiorMaterialId);
    calculateNormals(vertices, resolutionX, resolutionY);

    uint32_t offset = (resolutionX + 1) * (resolutionY + 1);
    vertices.resize(offset + 12);

    toNvShared(vertices[0 + offset].p) = point + (t1 + t2) * size;
    toNvShared(vertices[1 + offset].p) = point + (t2 - t1) * size;

    toNvShared(vertices[2 + offset].p) = point + (-t1 - t2) * size;
    toNvShared(vertices[3 + offset].p) = point + (t1 - t2) * size;

    toNvShared(vertices[8 + offset].p) = point + (t1 + t2) * jaggedPlaneSize;
    toNvShared(vertices[9 + offset].p) = point + (t2 - t1) * jaggedPlaneSize;

    toNvShared(vertices[10 + offset].p) = point + (-t1 - t2) * jaggedPlaneSize;
    toNvShared(vertices[11 + offset].p) = point + (t1 - t2) * jaggedPlaneSize;

    toNvShared(vertices[4 + offset].p) = point + (t1 + t2 + lNormal) * size;
    toNvShared(vertices[5 + offset].p) = point + (t2 - t1 + lNormal) * size;

    toNvShared(vertices[6 + offset].p) = point + (-t1 - t2 + lNormal) * size;
    toNvShared(vertices[7 + offset].p) = point + (t1 - t2 + lNormal) * size;

    int32_t edgeOffset = edges.size();
    edges.push_back({0 + offset, 1 + offset});
    edges.push_back({ 1 + offset, 2 + offset });
    edges.push_back({ 2 + offset, 3 + offset });
    edges.push_back({3 + offset, 0 + offset});

    edges.push_back({ 11 + offset, 10 + offset });
    edges.push_back({ 10 + offset, 9 + offset });
    edges.push_back({ 9 + offset, 8 + offset });
    edges.push_back({ 8 + offset, 11 + offset });

    facets.push_back({ edgeOffset, 8, id, interiorMaterialId, -1 });

    edges.push_back({ 0 + offset, 3 + offset });
    edges.push_back({ 3 + offset, 7 + offset });
    edges.push_back({ 7 + offset, 4 + offset });
    edges.push_back({ 4 + offset, 0 + offset });
    facets.push_back({ 8 + edgeOffset, 4, id, interiorMaterialId, -1 });

    edges.push_back({ 3 + offset, 2 + offset });
    edges.push_back({ 2 + offset, 6 + offset });
    edges.push_back({ 6 + offset, 7 + offset });
    edges.push_back({ 7 + offset, 3 + offset });
    facets.push_back({ 12 + edgeOffset, 4, id, interiorMaterialId, -1 });

    edges.push_back({ 5 + offset, 6 + offset });
    edges.push_back({ 6 + offset, 2 + offset });
    edges.push_back({ 2 + offset, 1 + offset });
    edges.push_back({ 1 + offset, 5 + offset });
    facets.push_back({ 16 + edgeOffset, 4, id, interiorMaterialId, -1 });

    edges.push_back({ 4 + offset, 5 + offset });
    edges.push_back({ 5 + offset, 1 + offset });
    edges.push_back({ 1 + offset, 0 + offset });
    edges.push_back({ 0 + offset, 4 + offset });
    facets.push_back({ 20 + edgeOffset, 4, id, interiorMaterialId, -1 });

    edges.push_back({ 4 + offset, 7 + offset });
    edges.push_back({ 7 + offset, 6 + offset });
    edges.push_back({ 6 + offset, 5 + offset });
    edges.push_back({ 5 + offset, 4 + offset });
    facets.push_back({ 24 + edgeOffset, 4, id, interiorMaterialId, -1 });

    //
    return new MeshImpl(vertices.data(), edges.data(), facets.data(), vertices.size(), edges.size(), facets.size());
}

Mesh* getBigBox(const NvVec3& point, float size, int32_t interiorMaterialId)
{
    NvVec3 normal(0, 0, 1);
    normal.normalize();
    NvVec3 t1, t2;
    getTangents(normal, t1, t2);

    std::vector<Vertex> positions(8);
    toNvShared(positions[0].p) = point + (t1 + t2 - normal) * size;
    toNvShared(positions[1].p) = point + (t2 - t1 - normal) * size;

    toNvShared(positions[2].p) = point + (-t1 - t2 - normal) * size;
    toNvShared(positions[3].p) = point + (t1 - t2 - normal) * size;

    toNvShared(positions[4].p) = point + (t1 + t2 + normal) * size;
    toNvShared(positions[5].p) = point + (t2 - t1 + normal) * size;

    toNvShared(positions[6].p) = point + (-t1 - t2 + normal) * size;
    toNvShared(positions[7].p) = point + (t1 - t2 + normal) * size;

    positions[0].uv[0] = {0, 0};
    positions[1].uv[0] = {UV_SCALE, 0};

    positions[2].uv[0] = {UV_SCALE, UV_SCALE};
    positions[3].uv[0] = {0, UV_SCALE};


    positions[4].uv[0] = {0, 0};
    positions[5].uv[0] = {UV_SCALE, 0};

    positions[6].uv[0] = {UV_SCALE, UV_SCALE};
    positions[7].uv[0] = {0, UV_SCALE};


    std::vector<Edge> edges;
    std::vector<Facet> facets;

    edges.push_back({0, 1});
    edges.push_back({1, 2});
    edges.push_back({2, 3});
    edges.push_back({3, 0});
    facets.push_back({0, 4, 0, interiorMaterialId, -1});


    edges.push_back({0, 3});
    edges.push_back({3, 7});
    edges.push_back({7, 4});
    edges.push_back({4, 0});
    facets.push_back({4, 4, 0, interiorMaterialId, -1});

    edges.push_back({3, 2});
    edges.push_back({2, 6});
    edges.push_back({6, 7});
    edges.push_back({7, 3});
    facets.push_back({8, 4, 0, interiorMaterialId, -1});

    edges.push_back({5, 6});
    edges.push_back({6, 2});
    edges.push_back({2, 1});
    edges.push_back({1, 5});
    facets.push_back({12, 4, 0, interiorMaterialId, -1});

    edges.push_back({4, 5});
    edges.push_back({5, 1});
    edges.push_back({1, 0});
    edges.push_back({0, 4});
    facets.push_back({16, 4, 0, interiorMaterialId, -1});

    edges.push_back({4, 7});
    edges.push_back({7, 6});
    edges.push_back({6, 5});
    edges.push_back({5, 4});
    facets.push_back({20, 4, 0, interiorMaterialId, -1});
    for (int i = 0; i < 8; ++i)
        positions[i].n = {0, 0, 0};
    return new MeshImpl(positions.data(), edges.data(), facets.data(), static_cast<uint32_t>(positions.size()),
                        static_cast<uint32_t>(edges.size()), static_cast<uint32_t>(facets.size()));
}

bool CmpSharedFace::
operator()(const std::pair<nvidia::NvVec3, nvidia::NvVec3>& pv1, const std::pair<nvidia::NvVec3, nvidia::NvVec3>& pv2) const
{
    CmpVec vc;
    if ((pv1.first - pv2.first).magnitude() < 1e-5)
    {
        return vc(pv1.second, pv2.second);
    }
    return vc(pv1.first, pv2.first);
}

#define INDEXER_OFFSET (1ll << 32)

void buildCuttingConeFaces(const CutoutConfiguration& conf, const std::vector<std::vector<nvidia::NvVec3> >& cutoutPoints,
                           float heightBot, float heightTop, float conicityBot, float conicityTop, int64_t& id,
                           int32_t seed, int32_t interiorMaterialId, SharedFacesMap& sharedFacesMap)
{
    if (conf.noise.amplitude <= FLT_EPSILON)
    {
        return;
    }
    std::map<nvidia::NvVec3, std::pair<uint32_t, std::vector<nvidia::NvVec3> >, CmpVec> newCutoutPoints;
    uint32_t resH = std::max((uint32_t)std::roundf((heightBot + heightTop) / conf.noise.samplingInterval.z), 1u);

    // generate noisy faces
    SimplexNoise nEval(conf.noise.amplitude, conf.noise.frequency, conf.noise.octaveNumber, seed);

    for (uint32_t i = 0; i < cutoutPoints.size(); i++)
    {
        auto& points        = cutoutPoints[i];
        uint32_t pointCount = points.size();
        float finalP = 0, currentP = 0;
        for (uint32_t j = 0; j < pointCount; j++)
        {
            finalP += (points[(j + 1) % pointCount] - points[j]).magnitude();
        }

        for (uint32_t p = 0; p < pointCount; p++)
        {
            auto p0 = points[p];
            auto p1 = points[(p + 1) % pointCount];

            auto cp0 = newCutoutPoints.find(p0);
            if (cp0 == newCutoutPoints.end())
            {
                newCutoutPoints[p0] = std::make_pair(0u, std::vector<nvidia::NvVec3>(resH + 1, nvidia::NvVec3(0.f)));
                cp0                 = newCutoutPoints.find(p0);
            }
            auto cp1 = newCutoutPoints.find(p1);
            if (cp1 == newCutoutPoints.end())
            {
                newCutoutPoints[p1] = std::make_pair(0u, std::vector<nvidia::NvVec3>(resH + 1, nvidia::NvVec3(0.f)));
                cp1                 = newCutoutPoints.find(p1);
            }


            auto vec        = p1 - p0;
            auto cPos       = (p0 + p1) * 0.5f;
            uint32_t numPts = (uint32_t)(std::abs(vec.x) / conf.noise.samplingInterval.x +
                                         std::abs(vec.y) / conf.noise.samplingInterval.y) +
                              1;
            auto normal = vec.cross(nvidia::NvVec3(0, 0, 1));
            normal      = normal;

            auto p00 = p0 * conicityBot;
            p00.z    = -heightBot;
            auto p01 = p1 * conicityBot;
            p01.z    = -heightBot;
            auto p10 = p0 * conicityTop;
            p10.z    = heightTop;
            auto p11 = p1 * conicityTop;
            p11.z    = heightTop;
            PlaneStepper stepper(p00, p01, p10, p11, resH, numPts);

            PlaneStepper stepper1(normal, cPos, heightTop, vec.magnitude() * 0.5f, resH, numPts, true);
            stepper1.getNormal(0, 0);

            auto t    = std::make_pair(p0, p1);
            auto sfIt = sharedFacesMap.find(t);
            if (sfIt == sharedFacesMap.end() && sharedFacesMap.find(std::make_pair(p1, p0)) == sharedFacesMap.end())
            {
                sharedFacesMap[t] = SharedFace(numPts, resH, -(id + INDEXER_OFFSET), interiorMaterialId);
                sfIt              = sharedFacesMap.find(t);
                auto& SF          = sfIt->second;
                getNoisyFace(SF.vertices, SF.edges, SF.facets, resH, numPts,
                             nvidia::NvVec2(0, CYLINDER_UV_SCALE * currentP / (heightBot + heightTop)),
                             nvidia::NvVec2(CYLINDER_UV_SCALE / resH,
                                           CYLINDER_UV_SCALE * vec.magnitude() / (heightBot + heightTop) / numPts),
                             stepper, nEval, id++ + INDEXER_OFFSET, interiorMaterialId, true);

                currentP += vec.magnitude();
                cp0->second.first++;
                cp1->second.first++;
                for (uint32_t k = 0; k <= resH; k++)
                {
                    cp0->second.second[k] += toNvShared(SF.vertices[k].p);
                    cp1->second.second[k] += toNvShared(SF.vertices[SF.vertices.size() - resH - 1 + k].p);
                }
            }
        }
    }

    // limit faces displacement iteratively
    for (uint32_t i = 0; i < cutoutPoints.size(); i++)
    {
        auto& points        = cutoutPoints[i];
        uint32_t pointCount = points.size();
        for (uint32_t p = 0; p < pointCount; p++)
        {
            auto p0   = points[p];
            auto p1   = points[(p + 1) % pointCount];
            auto p2   = points[(p + 2) % pointCount];
            auto& cp1 = newCutoutPoints.find(p1)->second;
            float d   = nvidia::NvClamp((p1 - p0).getNormalized().dot((p2 - p1).getNormalized()), 0.f, 1.f);

            for (uint32_t h = 0; h <= resH; h++)
            {
                float z         = cp1.second[h].z;
                float conicity  = (conicityBot * h + conicityTop * (resH - h)) / resH;
                cp1.second[h]   = cp1.second[h] * d + p1 * cp1.first * conicity * (1.f - d);
                cp1.second[h].z = z;
            }
        }
    }

    // relax nearby points for too big faces displacement limitations
    for (uint32_t i = 0; i < cutoutPoints.size(); i++)
    {
        auto& points        = cutoutPoints[i];
        uint32_t pointCount = points.size();
        for (uint32_t p = 0; p < pointCount; p++)
        {
            auto p0   = points[p];
            auto p1   = points[(p + 1) % pointCount];
            auto& cp0 = newCutoutPoints.find(p0)->second;
            auto& cp1 = newCutoutPoints.find(p1)->second;

            auto SFIt = sharedFacesMap.find(std::make_pair(p0, p1));

            uint32_t idx0 = 0, idx1;
            if (SFIt == sharedFacesMap.end())
            {
                SFIt = sharedFacesMap.find(std::make_pair(p1, p0));
                idx1 = 0;
                idx0 = SFIt->second.w * (SFIt->second.h + 1);
            }
            else
            {
                idx1 = SFIt->second.w * (SFIt->second.h + 1);
            }

            for (uint32_t h = 0; h <= resH; h++)
            {
                float z        = cp1.second[h].z;
                float R0       = (cp0.second[h] / cp0.first - toNvShared(SFIt->second.vertices[idx0 + h].p)).magnitude();
                float R1       = (cp1.second[h] / cp1.first - toNvShared(SFIt->second.vertices[idx1 + h].p)).magnitude();
                float R        = R0 - R1;
                float r        = 0.25f * (cp1.second[h] / cp1.first - cp0.second[h] / cp0.first).magnitude();
                float conicity = (conicityBot * h + conicityTop * (resH - h)) / resH;
                if (R > r)
                {
                    float w         = std::min(1.f, r / R);
                    cp1.second[h]   = cp1.second[h] * w + p1 * cp1.first * conicity * (1.f - w);
                    cp1.second[h].z = z;
                }
            }
        }

        for (int32_t p = pointCount - 1; p >= 0; p--)
        {
            auto p0   = points[p];
            auto p1   = points[unsignedMod(p - 1, pointCount)];
            auto& cp0 = newCutoutPoints.find(p0)->second;
            auto& cp1 = newCutoutPoints.find(p1)->second;

            auto SFIt     = sharedFacesMap.find(std::make_pair(p0, p1));
            uint32_t idx0 = 0, idx1;
            if (SFIt == sharedFacesMap.end())
            {
                SFIt = sharedFacesMap.find(std::make_pair(p1, p0));
                idx1 = 0;
                idx0 = SFIt->second.w * (SFIt->second.h + 1);
            }
            else
            {
                idx1 = SFIt->second.w * (SFIt->second.h + 1);
            }

            for (uint32_t h = 0; h <= resH; h++)
            {
                float z        = cp1.second[h].z;
                float R0       = (cp0.second[h] / cp0.first - toNvShared(SFIt->second.vertices[idx0 + h].p)).magnitude();
                float R1       = (cp1.second[h] / cp1.first - toNvShared(SFIt->second.vertices[idx1 + h].p)).magnitude();
                float R        = R0 - R1;
                float r        = 0.25f * (cp1.second[h] / cp1.first - cp0.second[h] / cp0.first).magnitude();
                float conicity = (conicityBot * h + conicityTop * (resH - h)) / resH;
                if (R > r)
                {
                    float w         = std::min(1.f, r / R);
                    cp1.second[h]   = cp1.second[h] * w + p1 * cp1.first * conicity * (1.f - w);
                    cp1.second[h].z = z;
                }
            }
        }
    }

    // glue faces
    for (auto& SF : sharedFacesMap)
    {
        auto& cp0  = newCutoutPoints.find(SF.first.first)->second;
        auto& cp1  = newCutoutPoints.find(SF.first.second)->second;
        auto& v    = SF.second.vertices;
        float invW = 1.f / SF.second.w;

        for (uint32_t w = 0; w <= SF.second.w; w++)
        {
            for (uint32_t h = 0; h <= SF.second.h; h++)
            {
                toNvShared(v[w * (SF.second.h + 1) + h].p) +=
                    ((cp0.second[h] / cp0.first - toNvShared(v[h].p)) * (SF.second.w - w) +
                     (cp1.second[h] / cp1.first - toNvShared(v[SF.second.w * (SF.second.h + 1) + h].p)) * w) *
                    invW;
            }
        }
    }
}

Mesh* getNoisyCuttingCone(const std::vector<nvidia::NvVec3>& points, const std::set<int32_t>& smoothingGroups,
                          const nvidia::NvTransform& transform, bool useSmoothing, float heightBot, float heightTop,
                          float conicityMultiplierBot, float conicityMultiplierTop, nvidia::NvVec3 samplingInterval,
                          int32_t interiorMaterialId, const SharedFacesMap& sharedFacesMap, bool inverseNormals)
{
    NV_UNUSED(conicityMultiplierTop);
    NV_UNUSED(conicityMultiplierBot);
    
    uint32_t pointCount = points.size();
    uint32_t resP       = pointCount;
    for (uint32_t i = 0; i < pointCount; i++)
    {
        auto vec = (points[(i + 1) % pointCount] - points[i]);
        resP += (uint32_t)(std::abs(vec.x) / samplingInterval.x + std::abs(vec.y) / samplingInterval.y);
    }
    uint32_t resH = std::max((uint32_t)std::roundf((heightBot + heightTop) / samplingInterval.z), 1u);

    std::vector<Vertex> positions;
    positions.reserve((resH + 1) * (resP + 1));
    std::vector<Edge> edges;
    edges.reserve(resH * resP * 6 + (resP + 1) * 2);
    std::vector<Facet> facets;
    facets.reserve(resH * resP * 2 + 2);

    uint32_t pCount = 0;
    int sg          = useSmoothing ? 1 : -1;
    for (uint32_t p = 0; p < pointCount; p++)
    {
        if (useSmoothing && smoothingGroups.find(p) != smoothingGroups.end())
        {
            sg = sg ^ 3;
        }
        auto p0 = points[p];
        auto p1 = points[(p + 1) % pointCount];

        uint32_t firstVertexIndex = positions.size();
        uint32_t firstEdgeIndex   = edges.size();

        auto sfIt      = sharedFacesMap.find(std::make_pair(p0, p1));
        int32_t vBegin = 0, vEnd = -1, vIncr = 1;
        if (sfIt == sharedFacesMap.end())
        {
            sfIt = sharedFacesMap.find(std::make_pair(p1, p0));
            ;
            vBegin = sfIt->second.w;
            vIncr  = -1;
        }
        else
        {
            vEnd = sfIt->second.w + 1;
        }

        auto& SF = sfIt->second;
        positions.resize(firstVertexIndex + (SF.w + 1) * (SF.h + 1));
        if (vBegin < vEnd)
        {
            for (auto& e : SF.edges)
            {
                edges.push_back({e.s + firstVertexIndex, e.e + firstVertexIndex});
            }
            for (auto& f : SF.facets)
            {
                facets.push_back(f);
                facets.back().firstEdgeNumber += firstEdgeIndex;
                facets.back().smoothingGroup = sg;
            }
        }
        else
        {
            fillEdgesAndFaces(edges, facets, SF.h, SF.w, firstVertexIndex, positions.size(), SF.f.userData,
                              SF.f.materialId, sg, true);
        }
        for (int32_t v = vBegin; v != vEnd; v += vIncr)
        {
            std::copy(SF.vertices.begin() + v * (resH + 1), SF.vertices.begin() + (v + 1) * (SF.h + 1),
                      positions.begin() + firstVertexIndex);
            firstVertexIndex += SF.h + 1;
        }
        pCount += SF.vertices.size() / (resH + 1) - 1;
    }

    if (inverseNormals)
    {
        for (uint32_t e = 0; e < edges.size(); e += 3)
        {
            std::swap(edges[e + 0].s, edges[e + 0].e);
            std::swap(edges[e + 1].s, edges[e + 1].e);
            std::swap(edges[e + 2].s, edges[e + 2].e);
            std::swap(edges[e + 0], edges[e + 2]);
        }
    }

    uint32_t totalCount = pCount + pointCount;
    calculateNormals(positions, resH, totalCount - 1, inverseNormals);

    std::vector<float> xPos, yPos;
    int32_t ii = 0;
    for (auto& p : positions)
    {
        if ((ii++) % (resH + 1) == 1)
        {
            xPos.push_back(p.p.x);
            yPos.push_back(p.p.y);
        }
        toNvShared(p.p) = transform.transform(toNvShared(p.p));
        toNvShared(p.n) = transform.rotate(toNvShared(p.n));
    }
    totalCount /= 2;

    for (uint32_t i = 0; i < totalCount; i++)
    {
        uint32_t idx = 2 * i * (resH + 1);
        edges.push_back({idx, (idx + 2 * (resH + 1)) % (uint32_t)positions.size()});
    }
    for (int32_t i = totalCount; i > 0; i--)
    {
        uint32_t idx = (2 * i + 1) * (resH + 1) - 1;
        edges.push_back({ idx % (uint32_t)positions.size(), idx - 2 * (resH + 1)});
    }

    if (smoothingGroups.find(0) != smoothingGroups.end() || smoothingGroups.find(pointCount - 1) != smoothingGroups.end())
    {
        if (facets[0].smoothingGroup == facets[facets.size() - 1].smoothingGroup)
        {
            for (uint32_t i = 0; i < resH; i++)
            {
                facets[i].smoothingGroup = 4;
            }
        }
    }

    facets.push_back({ (int32_t)(resH * pCount * 6), totalCount, 0, interiorMaterialId, -1 });
    facets.push_back({ (int32_t)(resH * pCount * 6 + totalCount), totalCount, 0, interiorMaterialId, -1 });
    return new MeshImpl(positions.data(), edges.data(), facets.data(), static_cast<uint32_t>(positions.size()),
                        static_cast<uint32_t>(edges.size()), static_cast<uint32_t>(facets.size()));
}

Mesh* getCuttingCone(const CutoutConfiguration& conf, const std::vector<nvidia::NvVec3>& points,
                     const std::set<int32_t>& smoothingGroups, float heightBot, float heightTop, float conicityBot,
                     float conicityTop, int64_t& id, int32_t seed, int32_t interiorMaterialId,
                     const SharedFacesMap& sharedFacesMap, bool inverseNormals)
{
    NV_UNUSED(seed);

    uint32_t pointCount = points.size();
    if (conf.noise.amplitude > FLT_EPSILON)
    {
        return getNoisyCuttingCone(points, smoothingGroups, toNvShared(conf.transform), conf.useSmoothing, heightBot, heightTop,
                                   conicityBot, conicityTop, toNvShared(conf.noise.samplingInterval), interiorMaterialId,
                                   sharedFacesMap, inverseNormals);
    }

    float currentP = 0;
    std::vector<Vertex> positions((pointCount + 1) * 2);
    std::vector<Edge> edges(pointCount * 6 + 2);
    std::vector<Facet> facets(pointCount + 2);

    int sg = conf.useSmoothing ? 1 : -1;
    for (uint32_t i = 0; i < pointCount + 1; i++)
    {
        if (conf.useSmoothing && smoothingGroups.find(i) != smoothingGroups.end())
        {
            sg = sg ^ 3;
        }
        uint32_t i1 = i + pointCount + 1;
        uint32_t i3 = i + 1;
        uint32_t i2 = i3 + pointCount + 1;

        auto& p0 = positions[i];
        auto& p1 = positions[i1];
        p0.n = p1.n = {0.f, 0.f, 0.f};
        toNvShared(p0.p)        = points[i % pointCount] * conicityBot;
        p0.p.z      = -heightBot;
        toNvShared(p1.p)        = points[i % pointCount] * conicityTop;
        p1.p.z      = heightTop;
        toNvShared(p0.p)        = toNvShared(conf.transform).transform(toNvShared(p0.p));
        toNvShared(p1.p)        = toNvShared(conf.transform).transform(toNvShared(p1.p));
        p0.uv[0]    = {0.f, CYLINDER_UV_SCALE * currentP / (heightBot + heightTop)};
        p1.uv[0]    = {CYLINDER_UV_SCALE, CYLINDER_UV_SCALE * currentP / (heightBot + heightTop)};
        if (i == pointCount)
        {
            break;
        }
        currentP += (points[(i + 1) % pointCount] - points[i]).magnitude();

        int32_t edgeIdx = 4 * i;
        if (inverseNormals)
        {
            edges[edgeIdx + 1] = {i1, i2};
            edges[edgeIdx + 2] = {i2, i3};
            edges[edgeIdx + 3] = {i3, i};
            edges[edgeIdx + 0] = {i, i1};
        }
        else
        {
            edges[edgeIdx + 0] = {i, i3};
            edges[edgeIdx + 1] = {i3, i2};
            edges[edgeIdx + 2] = {i2, i1};
            edges[edgeIdx + 3] = {i1, i};
        }
        facets[i] = {edgeIdx, 4, id++, interiorMaterialId, sg};

        edges[5 * pointCount + i + 1] = {i1, i2};
        edges[5 * pointCount - i - 1] = {i3, i};
    }
    edges[5 * pointCount]     = {0, pointCount};
    edges[6 * pointCount + 1] = {2 * pointCount + 1, pointCount + 1};

    if (smoothingGroups.find(0) != smoothingGroups.end() || smoothingGroups.find(pointCount - 1) != smoothingGroups.end())
    {
        if (facets[0].smoothingGroup == facets[pointCount - 1].smoothingGroup)
        {
            facets[0].smoothingGroup = 4;
        }
    }

    facets[pointCount + 0] = { 4 * (int32_t)pointCount, pointCount + 1, 0, interiorMaterialId, -1 };
    facets[pointCount + 1] = { 5 * (int32_t)pointCount + 1, pointCount + 1, interiorMaterialId, 0, -1 };
    return new MeshImpl(positions.data(), edges.data(), facets.data(), static_cast<uint32_t>(positions.size()),
                        static_cast<uint32_t>(edges.size()), static_cast<uint32_t>(facets.size()));
}

}  // namespace Blast
}  // namespace Nv