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

#ifndef PX_SOFTBODY_GPU_H
#define PX_SOFTBODY_GPU_H
/** \addtogroup extensions
  @{
*/

#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxArray.h"

#if PX_SUPPORT_GPU_PHYSX

#if !PX_DOXYGEN
namespace physx
{
#endif

	class PxScene;
	class PxSoftBody;

	/**
	\brief Allows to embed points into a softbody
	*/
	class PxSoftBodyEmbedding
	{
		PX_NOCOPY(PxSoftBodyEmbedding)
	public:

		/**
		*/
		static const PxU32 BAD_ID = PxU32(-1);

		static PxSoftBodyEmbedding* create();
		void release();

		/**
		\brief Create the embedding for a set of points and a soft body
		\param[in] softBody The soft body
		\param[in] localPoints The points in the soft body's collision mesh local space
		\param[in] pointCount The total number of points

		\return The embedding ID
		*/
		PxU32 embedPoints(PxSoftBody* softBody, const PxVec3* localPoints, PxU32 pointCount);

		/**
		\brief Destroys the embedding
		\param[in] embeddingID The embedding ID
		*/
		void removeEmbedding(PxU32 embeddingID);

		/**
		\brief Returns the pointer to the embedded points
		\param[in] embeddingID The embedding ID
		*/
		const PxVec3* getPoints(PxU32 embeddingID);

		/**
		\brief Returns the number of the embedded points
		\param[in] embeddingID The embedding ID
		*/
		PxU32 getPointCount(PxU32 embeddingID);

		/**
		\brief Returns the soft body related to the embedding
		\param[in] embeddingID The embedding ID
		*/
		PxSoftBody* getSoftBody(PxU32 embeddingID);

		/**
		\brief Updates all embeddings
		*/
		void update();

		/**
		\brief Destroys all embeddings
		*/
		void clear();

	private:

		PxSoftBodyEmbedding();
		~PxSoftBodyEmbedding();

		struct SceneInfo
		{
			PxArray<PxSoftBody*> softBodies;
			PxScene* scene = NULL;
			PxVec4** vertexData = NULL;
			PxU32* vertexDataSizes = NULL;
			PxU32** tetrahedras = NULL;
			PxU32* softBodyIndices = NULL;
			PxU32 maxVertexDataSize;
			PxU32 embeddingCount;
			PxU32 maxPointCount;
			PxU32* embeddingSoftBody = NULL;
			PxU32* pointCounts = NULL;
			PxVec3** points = NULL;
			PxU32** tetrahedraIndices = NULL;
			PxVec4** barycentricCoords = NULL;
			bool dirty = true;
		};
		struct SoftBodyInfo
		{
			PxArray<PxU32> embeddings;
			PxSoftBody* softBody = NULL;
			PxScene* scene = NULL;
			PxVec4* vertexData = NULL;
			PxU32* tetrahedras = NULL;
		};
		struct EmbeddingInfo
		{
			PxSoftBody* softBody = NULL;
			PxU32 pointCount;
			PxVec3* points = NULL;
			PxVec3* pointsHost = NULL;
			PxU32* tetrahedraIndices = NULL;
			PxVec4* barycentricCoords = NULL;
			bool copy;
		};

		SoftBodyInfo& getSoftBodyInfo(PxSoftBody* softBody);
		void removeSoftBodyInfo(PxSoftBody* softBody);
		SceneInfo& getSceneInfo(PxScene* scene);
		void removeSceneInfo(PxScene* scene);

		PxU32 mNextID = 0;
		PxHashMap<PxU32, EmbeddingInfo> mEmbeddingInfo;
		PxHashMap<PxSoftBody*, SoftBodyInfo> mSoftBodyInfo;
		PxHashMap<PxScene*, SceneInfo> mSceneInfo;
	};

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif

/** @} */
#endif
