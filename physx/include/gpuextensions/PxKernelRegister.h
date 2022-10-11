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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


KERNEL_DEF(scanPerBlockKernel, "scanPerBlockKernel")
KERNEL_DEF(scanPerBlockKernel4x4, "scanPerBlockKernel4x4")
KERNEL_DEF(addBlockSumsKernel, "addBlockSumsKernel")
KERNEL_DEF(addBlockSumsKernel4x4, "addBlockSumsKernel4x4")
KERNEL_DEF(radixFourBitCountPerBlockKernel, "radixFourBitCountPerBlockKernel")
KERNEL_DEF(radixFourBitReorderKernel, "radixFourBitReorderKernel")
KERNEL_DEF(reorderKernel, "reorderKernel")

KERNEL_DEF(smoothPositionsLaunch, "smoothPositionsLaunch")
KERNEL_DEF(calculateAnisotropyLaunch, "calculateAnisotropyLaunch")
KERNEL_DEF(anisotropyKernel, "anisotropyKernel")
KERNEL_DEF(smoothPositionsKernel, "smoothPositionsKernel")

KERNEL_DEF(iso_ComputeParticleDensityUsingSDF, "iso_ComputeParticleDensityUsingSDF")
KERNEL_DEF(iso_ComputeParticleDensityUsingSDFSparse, "iso_ComputeParticleDensityUsingSDFSparse")
KERNEL_DEF(iso_ComputeParticleDensity, "iso_ComputeParticleDensity")
KERNEL_DEF(iso_ComputeParticleDensitySparse, "iso_ComputeParticleDensitySparse")
KERNEL_DEF(iso_CountCellVerts, "iso_CountCellVerts")
KERNEL_DEF(iso_CountCellVertsSparse, "iso_CountCellVertsSparse")
KERNEL_DEF(iso_CountCellVertsDC, "iso_CountCellVertsDC")
KERNEL_DEF(iso_CountCellVertsDCSparse, "iso_CountCellVertsDCSparse")
KERNEL_DEF(iso_CreateVerts, "iso_CreateVerts")
KERNEL_DEF(iso_CreateVertsSparse, "iso_CreateVertsSparse")
KERNEL_DEF(iso_CreateVertsDC, "iso_CreateVertsDC")
KERNEL_DEF(iso_CreateVertsDCSparse, "iso_CreateVertsDCSparse")
KERNEL_DEF(iso_CountTriIds, "iso_CountTriIds")
KERNEL_DEF(iso_CountTriIdsSparse, "iso_CountTriIdsSparse")
KERNEL_DEF(iso_CountTriIdsDC, "iso_CountTriIdsDC")
KERNEL_DEF(iso_CountTriIdsDCSparse, "iso_CountTriIdsDCSparse")
KERNEL_DEF(iso_CreateTriIds, "iso_CreateTriIds")
KERNEL_DEF(iso_CreateTriIdsSparse, "iso_CreateTriIdsSparse")
KERNEL_DEF(iso_CreateTriIdsDC, "iso_CreateTriIdsDC")
KERNEL_DEF(iso_CreateTriIdsDCSparse, "iso_CreateTriIdsDCSparse")
KERNEL_DEF(iso_SmoothVerts, "iso_SmoothVerts")
KERNEL_DEF(iso_AverageVerts, "iso_AverageVerts")
KERNEL_DEF(iso_SmoothNormals, "iso_SmoothNormals")
KERNEL_DEF(iso_SmoothNormalsSparse, "iso_SmoothNormalsSparse")
KERNEL_DEF(iso_SmoothNormalsNormalize, "iso_SmoothNormalsNormalize")
KERNEL_DEF(iso_SmoothNormalsNormalizeSparse, "iso_SmoothNormalsNormalizeSparse")
KERNEL_DEF(iso_ComputeNormals, "iso_ComputeNormals")
KERNEL_DEF(iso_ComputeNormalsSparse, "iso_ComputeNormalsSparse")
KERNEL_DEF(iso_NormalizeNormals, "iso_NormalizeNormals")
KERNEL_DEF(iso_NormalizeNormalsSparse, "iso_NormalizeNormalsSparse")
KERNEL_DEF(iso_GridFilterGauss, "iso_GridFilterGauss")
KERNEL_DEF(iso_GridFilterGaussSparse, "iso_GridFilterGaussSparse")
KERNEL_DEF(iso_GridFilterDilateErode, "iso_GridFilterDilateErode")
KERNEL_DEF(iso_GridFilterDilateErodeSparse, "iso_GridFilterDilateErodeSparse")

KERNEL_DEF(computeLennardJonedForcesKernel, "computeLennardJonedForcesKernel")
KERNEL_DEF(explicitIntegrationKernel, "explicitIntegrationKernel")
KERNEL_DEF(copySortedToUnsortedBufferKernel, "copySortedToUnsortedBufferKernel")

KERNEL_DEF(computeEmbeddedPoints, "computeEmbeddedPoints")

KERNEL_DEF(sg_SparseGridCalcSubgridHashes, "sg_SparseGridCalcSubgridHashes")
KERNEL_DEF(sg_SparseGridMarkRequiredNeighbors, "sg_SparseGridMarkRequiredNeighbors")
KERNEL_DEF(sg_SparseGridSortedArrayToDelta, "sg_SparseGridSortedArrayToDelta")
KERNEL_DEF(sg_SparseGridGetUniqueValues, "sg_SparseGridGetUniqueValues")
KERNEL_DEF(sg_SparseGridClearDensity, "sg_SparseGridClearDensity")
KERNEL_DEF(sg_SparseGridBuildSubgridNeighbors, "sg_SparseGridBuildSubgridNeighbors")
KERNEL_DEF(sg_MarkSubgridEndIndices, "sg_MarkSubgridEndIndices")
KERNEL_DEF(sg_ReuseSubgrids, "sg_ReuseSubgrids")
KERNEL_DEF(sg_AddReleasedSubgridsToUnusedStack, "sg_AddReleasedSubgridsToUnusedStack")
KERNEL_DEF(sg_AllocateNewSubgrids, "sg_AllocateNewSubgrids")

KERNEL_DEF(extractLinesFromStrands, "extractLinesFromStrands")
KERNEL_DEF(interleaveBuffers, "interleaveBuffers")
