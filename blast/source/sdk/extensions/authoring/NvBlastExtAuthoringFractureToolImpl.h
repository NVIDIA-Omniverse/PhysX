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

#ifndef NVBLASTAUTHORINGFRACTURETOOLIMPL_H
#define NVBLASTAUTHORINGFRACTURETOOLIMPL_H

#include "NvBlastExtAuthoringFractureTool.h"
#include "NvBlastExtAuthoringMesh.h"
#include <vector>
#include <set>

namespace Nv
{
namespace Blast
{

class SpatialAccelerator;
class Triangulator;


/**
    Class for voronoi sites generation inside supplied mesh.
*/
class VoronoiSitesGeneratorImpl : public VoronoiSitesGenerator
{
public:
    
    /** 
        Voronoi sites should not be generated outside of the fractured mesh, so VoronoiSitesGenerator
        should be supplied with fracture mesh.
        \param[in] mesh         Fracture mesh
        \param[in] rnd          User supplied random value generator.
        \return
    */
    VoronoiSitesGeneratorImpl(const Mesh* mesh, RandomGeneratorBase* rnd);
    ~VoronoiSitesGeneratorImpl();

    void                        release() override;

    /**
        Set base fracture mesh
    */
    void                        setBaseMesh(const Mesh* m) override;

    /**
        Access to generated voronoi sites.
        \note User should call NVBLAST_FREE for hulls and hullsOffset when it not needed anymore
        \param[out]             Pointer to generated voronoi sites
        \return                 Count of generated voronoi sites.
    */
     uint32_t                   getVoronoiSites(const NvcVec3*& sites) override;
    
    /**
        Add site in particular point
        \param[in] site     Site coordinates
    */
    void                        addSite(const NvcVec3& site) override;
    /**
        Uniformly generate sites inside the mesh
        \param[in] numberOfSites    Number of generated sites
    */
    void                        uniformlyGenerateSitesInMesh(uint32_t numberOfSites) override;

    /**
        Generate sites in clustered fashion
        \param[in] numberOfClusters Number of generated clusters
        \param[in] sitesPerCluster  Number of sites in each cluster
        \param[in] clusterRadius    Voronoi cells cluster radius
    */
    void                        clusteredSitesGeneration(uint32_t numberOfClusters, uint32_t sitesPerCluster, float clusterRadius) override;

    /**
        Radial pattern of sites generation
        \param[in] center       Center of generated pattern
        \param[in] normal       Normal to plane in which sites are generated
        \param[in] radius       Pattern radius
        \param[in] angularSteps Number of angular steps
        \param[in] radialSteps  Number of radial steps
        \param[in] angleOffset  Angle offset at each radial step
        \param[in] variability  Randomness of sites distribution 
    */
    void                        radialPattern(const NvcVec3& center, const NvcVec3& normal, float radius, int32_t angularSteps, int32_t radialSteps, float angleOffset = 0.0f, float variability = 0.0f) override;

    /**
        Generate sites inside sphere
        \param[in] count        Count of generated sites
        \param[in] radius       Radius of sphere
        \param[in] center       Center of sphere
    */
    void                        generateInSphere(const uint32_t count, const float radius, const NvcVec3& center) override;
    /**
        Set stencil mesh. With stencil mesh sites are generated only inside both of fracture and stencil meshes. 
        \param[in] stencil      Stencil mesh.
    */
    void                        setStencil(const Mesh* stencil) override;
    /**
        Removes stencil mesh
    */
    void                        clearStencil() override;

    /** 
        Deletes sites inside supplied sphere
        \param[in] radius               Radius of sphere
        \param[in] center               Center of sphere
        \param[in] eraserProbability    Probability of removing some particular site
    */
    void                        deleteInSphere(const float radius, const NvcVec3& center, const float eraserProbability = 1) override;

private:
    std::vector <NvcVec3>   mGeneratedSites;
    const Mesh*                 mMesh;
    const Mesh*                 mStencil;
    RandomGeneratorBase*        mRnd;
    SpatialAccelerator*         mAccelerator;
};



/**
    FractureTool class provides methods to fracture provided mesh and generate Blast asset data
*/
class FractureToolImpl : public FractureTool
{

public:

    /**
        FractureTool can log asset creation info if logCallback is provided.
    */
    FractureToolImpl() : mRemoveIslands(false)
    {
        reset();
    }

    ~FractureToolImpl()
    {
        reset();
    }

    void                                    release() override;

    /**
        Reset FractureTool state.
    */
    void                                    reset() override;
    
    /**
    Set the material id to use for new interior faces. Defaults to kMaterialInteriorId
    */
    void                                    setInteriorMaterialId(int32_t materialId) override;

    /**
    Gets the material id to use for new interior faces
    */
    int32_t                                 getInteriorMaterialId() const override;
    
    /**
    Replaces an material id on faces with a new one
    */
    void                                    replaceMaterialId(int32_t oldMaterialId, int32_t newMaterialId) override;

    /**
        Set input meshes which will be fractured, FractureTool will be reset.
        If ids != nullptr, it must point to an array of length meshSizes.
        Each mesh will be assigned to a chunk with ID given by the corresponding element in ids.
        If the corresponding element is negative, or ids is NULL, then the chunk will be assigned
        an arbitrary (but currently unused) ID.
        Returns true iff all meshes were assigned chunks with valid IDs.
    */
    bool                                    setSourceMeshes(Mesh const * const * meshes, uint32_t meshesSize, const int32_t* ids = nullptr) override;

    /**
        Set chunk mesh, parentId should be valid, return ID of new chunk.
        if chunkId >= 0 and currently unused, then that ID will be used (and returned).
        Otherwise an arbitrary (but currently unused) ID will be used and returned.
    */
    int32_t                                 setChunkMesh(const Mesh* mesh, int32_t parentId, int32_t chunkId = -1) override;

    /**
        Get chunk mesh in polygonal representation
    */
    Mesh*                                   createChunkMesh(int32_t chunkInfoIndex, bool splitUVs = true) override;

    /**
        Fractures specified chunk with voronoi method.
        \param[in] chunkId              Chunk to fracture
        \param[in] cellPoints           Array of voronoi sites
        \param[in] replaceChunk         if 'true', newly generated chunks will replace source chunk, if 'false', newly generated chunks will be at next depth level, source chunk will be parent for them.
                                        Case replaceChunk == true && chunkId == 0 considered as wrong input parameters
        \return   If 0, fracturing is successful.
    */
    int32_t                                 voronoiFracturing(uint32_t chunkId, uint32_t cellCount, const NvcVec3* cellPoints, bool replaceChunk) override;

    /**
        Fractures specified chunk with voronoi method. Cells can be scaled along x,y,z axes.
        \param[in] chunkId              Chunk to fracture
        \param[in] cellPoints           Array of voronoi sites
        \param[in] cellPoints           Array of voronoi sites
        \param[in] scale                Voronoi cells scaling factor
        \param[in] rotation             Voronoi cells rotation. Has no effect without cells scale factor
        \param[in] replaceChunk         if 'true', newly generated chunks will replace source chunk, if 'false', newly generated chunks will be at next depth level, source chunk will be parent for them.
                                        Case replaceChunk == true && chunkId == 0 considered as wrong input parameters
        \return   If 0, fracturing is successful.
    */
    int32_t                                 voronoiFracturing(uint32_t chunkId, uint32_t cellCount, const NvcVec3* cellPoints, const NvcVec3& scale, const NvcQuat& rotation, bool replaceChunk) override;


    /**
        Fractures specified chunk with slicing method.
        \param[in] chunkId              Chunk to fracture
        \param[in] conf                 Slicing parameters, see SlicingConfiguration.
        \param[in] replaceChunk         if 'true', newly generated chunks will replace source chunk, if 'false', newly generated chunks will be at next depth level, source chunk will be parent for them.
                                        Case replaceChunk == true && chunkId == 0 considered as wrong input parameters
        \param[in] rnd                  User supplied random number generator

        \return   If 0, fracturing is successful.
    */
    int32_t                                 slicing(uint32_t chunkId, const SlicingConfiguration& conf, bool replaceChunk, RandomGeneratorBase* rnd) override;


    /**
    Cut chunk with plane.
    \param[in] chunkId              Chunk to fracture
    \param[in] normal               Plane normal
    \param[in] position             Point on plane
    \param[in] noise                Noise configuration for plane-chunk intersection, see NoiseConfiguration.
    \param[in] replaceChunk         if 'true', newly generated chunks will replace source chunk, if 'false', newly generated chunks will be at next depth level, source chunk will be parent for them.
    Case replaceChunk == true && chunkId == 0 considered as wrong input parameters
    \param[in] rnd                  User supplied random number generator

    \return   If 0, fracturing is successful.
    */
    int32_t                                 cut(uint32_t chunkId, const NvcVec3& normal, const NvcVec3& position, const NoiseConfiguration& noise, bool replaceChunk, RandomGeneratorBase* rnd) override;

    /**
    Cutout fracture for specified chunk.
    \param[in] chunkId              Chunk to fracture
    \param[in] conf                 Cutout parameters, see CutoutConfiguration.
    \param[in] replaceChunk         if 'true', newly generated chunks will replace source chunk, if 'false', newly generated chunks will be at next depth level, source chunk will be parent for them.
    Case replaceChunk == true && chunkId == 0 considered as wrong input parameters
    \param[in] rnd                  User supplied random number generator

    \return   If 0, fracturing is successful.
    */
    int32_t                                 cutout(uint32_t chunkId, CutoutConfiguration conf, bool replaceChunk, RandomGeneratorBase* rnd) override;


    /**
        Creates resulting fractured mesh geometry from intermediate format
    */
    void                                    finalizeFracturing() override;
    
    uint32_t                                getChunkCount() const override;

    /**
        Get chunk information
    */
    const ChunkInfo&                        getChunkInfo(int32_t chunkInfoIndex) override;

    /**
        Get percentage of mesh overlap.
        percentage computed as volume(intersection(meshA , meshB)) / volume (meshA) 
        \param[in] meshA Mesh A
        \param[in] meshB Mesh B
        \return mesh overlap percentage
    */
    float                                   getMeshOverlap(const Mesh& meshA, const Mesh& meshB) override;

    /**
        Get chunk base mesh
        \note User should call NVBLAST_FREE for output when it not needed anymore
        \param[in] chunkIndex Chunk index
        \param[out] output Array of triangles to be filled
        \return number of triangles in base mesh
    */
    uint32_t                                getBaseMesh(int32_t chunkIndex, Triangle*& output) override;

    /**
        Update chunk base mesh
        \note Doesn't allocates output array, Triangle* output should be preallocated by user
        \param[in] chunkIndex Chunk index
        \param[out] output Array of triangles to be filled
        \return number of triangles in base mesh
    */
    uint32_t                                updateBaseMesh(int32_t chunkIndex, Triangle* output) override;

    /**
        Return info index of chunk with specified chunkId
        \param[in] chunkId Chunk ID
        \return Chunk index in internal buffer, if not exist -1 is returned.
    */
    int32_t                                 getChunkInfoIndex(int32_t chunkId) const override;

    /**
        Return id of chunk with specified index.
        \param[in] chunkInfoIndex Chunk info index
        \return Chunk id or -1 if there is no such chunk.
    */
    int32_t                                 getChunkId(int32_t chunkInfoIndex) const override;

    /**
        Return depth level of the given chunk
        \param[in] chunkId Chunk ID
        \return Chunk depth or -1 if there is no such chunk.
    */
    int32_t                                 getChunkDepth(int32_t chunkId) const override;

    /**
        Return array of chunks IDs with given depth.
        \note User should call NVBLAST_FREE for chunkIds when it not needed anymore
        \param[in]  depth Chunk depth
        \param[out] Pointer to array of chunk IDs
        \return Number of chunks in array
    */
    uint32_t                                getChunksIdAtDepth(uint32_t depth, int32_t*& chunkIds) const override;


    /**
        Get result geometry without noise as vertex and index buffers, where index buffers contain series of triplets
        which represent triangles.
        \note User should call NVBLAST_FREE for vertexBuffer, indexBuffer and indexBufferOffsets when it not needed anymore
        \param[out] vertexBuffer Array of vertices to be filled
        \param[out] indexBuffer Array of indices to be filled
        \param[out] indexBufferOffsets Array of offsets in indexBuffer for each base mesh. 
                    Contains getChunkCount() + 1 elements. Last one is indexBuffer size
        \return Number of vertices in vertexBuffer
    */
    uint32_t                                getBufferedBaseMeshes(Vertex*& vertexBuffer, uint32_t*& indexBuffer, uint32_t*& indexBufferOffsets) override;

    /**
        Set automatic islands removing. May cause instabilities.
        \param[in] isRemoveIslands Flag whether remove or not islands.
    */
    void                                    setRemoveIslands(bool isRemoveIslands) override;

    /**
        Try find islands and remove them on some specifical chunk. If chunk has childs, island removing can lead to wrong results! Apply it before further chunk splitting.
        \param[in] chunkId Chunk ID which should be checked for islands
        \return Number of found islands is returned
    */
    int32_t                                 islandDetectionAndRemoving(int32_t chunkId, bool createAtNewDepth = false) override;

    /**
        Check if input mesh contains open edges. Open edges can lead to wrong fracturing results.
        \return true if mesh contains open edges
    */
    bool                                    isMeshContainOpenEdges(const Mesh* input) override;

    bool                                    deleteChunkSubhierarchy(int32_t chunkId, bool deleteRoot = false) override;

    void                                    uniteChunks(uint32_t threshold, uint32_t targetClusterSize,
                                                        const uint32_t* chunksToMerge, uint32_t mergeChunkCount,
                                                        const NvcVec2i* adjChunks, uint32_t adjChunksSize,
                                                        bool removeOriginalChunks = false) override;
    
    bool                                    setApproximateBonding(uint32_t chunkId, bool useApproximateBonding) override;

    /**
        Rescale interior uv coordinates of given chunk to fit square of given size. 
        \param[in] side Size of square side
        \param[in] chunkId Chunk ID for which UVs should be scaled.
    */
    void                                    fitUvToRect(float side, uint32_t chunkId) override;

    /**
        Rescale interior uv coordinates of all existing chunks to fit square of given size, relative sizes will be preserved.
        \param[in] side Size of square side
    */
    void                                    fitAllUvToRect(float side) override;



private:    
    bool                                    isAncestorForChunk(int32_t ancestorId, int32_t chunkId);
    int32_t                                 slicingNoisy(uint32_t chunkId, const SlicingConfiguration& conf, bool replaceChunk, RandomGeneratorBase* rnd);
    uint32_t                                stretchGroup(const std::vector<uint32_t>& group, std::vector<std::vector<uint32_t>>& graph);
    void                                    rebuildAdjGraph(const std::vector<uint32_t>& chunksToRebuild, const NvcVec2i* adjChunks, uint32_t adjChunksSize,
                                                            std::vector<std::vector<uint32_t> >& chunkGraph);
    void                                    fitAllUvToRect(float side, std::set<uint32_t>& mask);
    void                                    markLeaves();

    /*
     * Meshes are transformed to fit a unit cube, for algorithmic stability.  This transform is stored
     * in the ChunkInfo.  Some meshes are created from already-transformed chunks.  If so, set
     * fromTransformed = true, so that the transform-to-world can be concatenated with the source mesh's.
     * 
     * chunkInfo.parentChunkId must be valid if fromTransformed == true.
     * 
     * Returns true iff successful.
     */
    bool                                    setChunkInfoMesh(ChunkInfo& chunkInfo, Mesh* mesh, bool fromTransformed = true);

    /**
        Returns newly created chunk index in mChunkData.
    */
    uint32_t                                createNewChunk(uint32_t parentChunkId);

    /**
     * Returns a previously unused ID.
     */
    int32_t                                 createId();
    /**
     * Mark the given ID as being used.  Returns false if that ID was already marked as in use, true otherwise
     */
    bool                                    reserveId(int32_t id);

protected:
    /* Chunk mesh wrappers */
    std::vector<Triangulator*>          mChunkPostprocessors;

    int64_t                             mPlaneIndexerOffset;
    int32_t                             mNextChunkId;
    std::set<int32_t>                   mChunkIdsUsed;
    std::vector<ChunkInfo>              mChunkData;

    bool                                mRemoveIslands;
    int32_t                             mInteriorMaterialId;
};

int32_t findCellBasePlanes(const std::vector<NvcVec3>& sites, std::vector<std::vector<std::pair<int32_t, int32_t>>>& neighbors);
Mesh* getCellMesh(class BooleanEvaluator& eval, int32_t planeIndexerOffset, int32_t cellId, const std::vector<NvcVec3>& sites, const std::vector<std::vector<std::pair<int32_t, int32_t>>>& neighbors, int32_t interiorMaterialId, NvcVec3 origin);

} // namespace Blast
} // namespace Nv


#endif // ifndef NVBLASTAUTHORINGFRACTURETOOLIMPL_H
