#include <cinttypes>
#include <map>
#include <set>
#include <vector>
#include "NvBlastExtAuthoringTypes.h"

namespace nvidia
{
    class NvVec3;
};

namespace Nv
{
namespace Blast
{
    
    class Mesh;

/**
    Helper functions
*/

/**
Set cutting box at some particular position.
\param[in] point    Cutting face center
\param[in] normal   Cutting face normal
\param[in] mesh     Cutting box mesh
\param[in] size     Cutting box size
\param[in] id   Cutting box ID
*/
void    setCuttingBox(const nvidia::NvVec3& point, const nvidia::NvVec3& normal, Mesh* mesh, float size, int64_t id);
/**
Create cutting box at some particular position.
\param[in] point    Cutting face center
\param[in] normal   Cutting face normal
\param[in] size     Cutting box size
\param[in] id   Cutting box ID
*/
Mesh*   getCuttingBox(const nvidia::NvVec3& point, const nvidia::NvVec3& normal, float size, int64_t id, int32_t interiorMaterialId);

/**
Create box at some particular position.
\param[in] point    Cutting face center
\param[in] size     Cutting box size
*/
Mesh*   getBigBox(const nvidia::NvVec3& point, float size, int32_t interiorMaterialId);

/**
Create slicing box with noisy cutting surface.
\param[in] point            Cutting face center
\param[in] normal           Cutting face normal
\param[in] size             Cutting box size
\param[in] jaggedPlaneSize  Noisy surface size
\param[in] resolution       Noisy surface resolution
\param[in] id               Cutting box ID
\param[in] amplitude        Noise amplitude
\param[in] frequency        Noise frequency
\param[in] octaves          Noise octaves
\param[in] seed             Random generator seed, used for noise generation.
*/
Mesh* getNoisyCuttingBoxPair(const nvidia::NvVec3& point, const nvidia::NvVec3& normal, float size, float jaggedPlaneSize, nvidia::NvVec3 resolution, int64_t id, float amplitude, float frequency, int32_t octaves, int32_t seed, int32_t interiorMaterialId);


/**
Inverses normals of cutting box and sets indices.
\param[in] mesh     Cutting box mesh
*/
void inverseNormalAndIndices(Mesh* mesh);

struct CmpVec
{
    bool operator()(const nvidia::NvVec3& v1, const nvidia::NvVec3& v2) const;
};

typedef std::map<nvidia::NvVec3, std::map<uint32_t, uint32_t>, CmpVec> PointMap;

struct SharedFace
{
    SharedFace() {}
    SharedFace(uint32_t inW, uint32_t inH, int64_t inUD, int32_t inMatId) : w(inW), h(inH), f(Facet( 0, 3, inUD, inMatId ))
    {
        vertices.reserve((w + 1) * (h + 1));
    }
    uint32_t w, h;
    Facet f;
    std::vector<Nv::Blast::Vertex> vertices;
    std::vector<Nv::Blast::Edge> edges;
    std::vector<Nv::Blast::Facet> facets;
};

struct CmpSharedFace
{
    bool operator()(const std::pair<nvidia::NvVec3, nvidia::NvVec3>& pv1, const std::pair<nvidia::NvVec3, nvidia::NvVec3>& pv2) const;
};

typedef std::map<std::pair<nvidia::NvVec3, nvidia::NvVec3>, SharedFace, CmpSharedFace> SharedFacesMap;

struct CutoutConfiguration;

void buildCuttingConeFaces(const CutoutConfiguration& conf, const std::vector<std::vector<nvidia::NvVec3>>& points,
    float heightBot, float heightTop, float conicityBot, float conicityTop,
    int64_t& id, int32_t seed, int32_t interiorMaterialId, SharedFacesMap& sharedFacesMap);

/**
Create cutting cone at some particular position.
\param[in] conf         Cutout configuration parameters and data
\param[in] meshId       Cutout index
\param[in] points       Array of points for loop
\param[in] smoothingGroups  Array of point indices at which smoothing group should be toggled
\param[in] heightBot    Cutting cone bottom height (below z = 0)
\param[in] heightTop    Cutting cone top height (below z = 0)
\param[in] conicityBot  Cutting cone bottom points multiplier
\param[in] conicityTop  Cutting cone top points multiplier
\param[in] id           Cutting cylinder ID
\param[in] seed         Seed for RNG
\param[in] interiorMaterialId Interior material index
\param[in] sharedFacesMap Shared faces for noisy fracture
*/
Mesh*   getCuttingCone(const CutoutConfiguration& conf,
    const std::vector<nvidia::NvVec3>& points, const std::set<int32_t>& smoothingGroups,
    float heightBot, float heightTop, float conicityBot, float conicityTop,
    int64_t& id, int32_t seed, int32_t interiorMaterialId, const SharedFacesMap& sharedFacesMap, bool inverseNormals = false);

};
};