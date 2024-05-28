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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PX_SOFT_BODY_EXT_H
#define PX_SOFT_BODY_EXT_H

#include "foundation/PxTransform.h"
#include "foundation/PxUserAllocated.h"
#include "PxSoftBody.h"
#include "PxSoftBodyFlag.h"
#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaTypes.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

struct PxCookingParams;
class PxSimpleTriangleMesh;
class PxInsertionCallback;
class PxSoftBodyMesh;

/**
\brief Utility functions for use with PxSoftBody and subclasses
*/
class PxSoftBodyExt
{
public:
	/** 
	\brief Computes the SoftBody's vertex masses from the provided density and the volume of the tetrahedra

	The buffers affected by this operation can be obtained from the SoftBody using the methods getSimPositionInvMassBufferD() and getSimVelocityBufferD()

	The inverse mass is stored in the 4th component (the first three components are x, y, z coordinates) of the simulation mesh's position buffer. 

	\param[in] softBody The soft body which will get its mass updated
	\param[in] density The density to used to calculate the mass from the body's volume
	\param[in] maxInvMassRatio Maximum allowed ratio defined as max(vertexMasses) / min(vertexMasses) where vertexMasses is a list of float values with a mass for every vertex in the simulation mesh
	\param[in] simPositionsPinned A pointer to a pinned host memory buffer containing positions and inverse masses for each vertex of the simulation mesh.

	\see PxSoftBody PxSoftBody::getSimPositionInvMassBufferD()
	*/
	static void updateMass(PxSoftBody& softBody, const PxReal density, const PxReal maxInvMassRatio, PxVec4* simPositionsPinned);

	/**
	\brief Computes the SoftBody's vertex masses such that the sum of all masses is equal to the provided mass

	The buffers affected by this operation can be obtained from the SoftBody using the methods getSimPositionInvMassBufferD()) and getSimVelocityBufferD()

	The inverse mass is stored in the 4th component (the first three components are x, y, z coordinates) of the simulation mesh's position buffer.

	\param[in] softBody The soft body which will get its mass updated
	\param[in] mass The SoftBody's mass
	\param[in] maxInvMassRatio Maximum allowed ratio defined as max(vertexMasses) / min(vertexMasses) where vertexMasses is a list of float values with a mass for every vertex in the simulation mesh
	\param[in] simPositionsPinned A pointer to a pinned host memory buffer containing positions and inverse masses for each vertex of the simulation mesh.

	\see PxSoftBody PxSoftBody::getSimPositionInvMassBufferD()
	*/
	static void setMass(PxSoftBody& softBody, const PxReal mass, const PxReal maxInvMassRatio, PxVec4* simPositionsPinned);

	/**
	\brief Transforms a SoftBody

	The buffers affected by this operation can be obtained from the SoftBody using the methods getSimPositionInvMassBufferD() and getSimVelocityBufferD()

	Applies a transformation to the simulation mesh's positions an velocities. Velocities only get rotated and scaled (translation is not applicable to direction vectors).
	It does not modify the body's mass. 
	If the method is called multiple times, the transformation will compound with the ones previously applied.

	\param[in] softBody The soft body which is transformed
	\param[in] transform The transform to apply
	\param[in] scale A scaling factor
	\param[in] simPositionsPinned A pointer to a pinned host memory buffer containing positions and inverse masses for each vertex of the simulation mesh.
	\param[in] simVelocitiesPinned A pointer to a pinned host memory buffer containing velocities for each vertex of the simulation mesh.
	\param[in] collPositionsPinned A pointer to a pinned host memory buffer containing positions and inverse masses for each vertex of the collision mesh.
	\param[in] restPositionsPinned A pointer to a pinned host memory buffer containing rest positions of the collision mesh.
	
	\see PxSoftBody
	*/
	static void transform(PxSoftBody& softBody, const PxTransform& transform, const PxReal scale, PxVec4* simPositionsPinned, PxVec4* simVelocitiesPinned, PxVec4* collPositionsPinned, PxVec4* restPositionsPinned);

	/**
	\brief Updates the collision mesh's vertex positions to match the simulation mesh's transformation and scale.
	
	The buffer affected by this operation can be obtained from the SoftBody using the method getPositionInvMassBufferD()

	\param[in] softBody The soft body which will get its collision mesh vertices updated
	\param[in] simPositionsPinned A pointer to a pinned host memory buffer containing positions and inverse masses for each vertex of the simulation mesh.
	\param[in] collPositionsPinned A pointer to a pinned host memory buffer containing positions and inverse masses for each vertex of the collision mesh.

	\see PxSoftBody
	*/
	static void updateEmbeddedCollisionMesh(PxSoftBody& softBody, PxVec4* simPositionsPinned, PxVec4* collPositionsPinned);

	/**
	\brief Uploads prepared SoftBody data to the GPU. It ensures that the embedded collision mesh matches the simulation mesh's transformation and scale.

	\param[in] softBody The soft body which will perform the data upload
	\param[in] flags Specifies which buffers the data transfer should include
	\param[in] simPositionsPinned A pointer to a pinned host memory buffer containing positions and inverse masses for each vertex of the simulation mesh.
	\param[in] simVelocitiesPinned A pointer to a pinned host memory buffer containing velocities for each vertex of the simulation mesh.
	\param[in] collPositionsPinned A pointer to a pinned host memory buffer containing positions and inverse masses for each vertex of the collision mesh.
	\param[in] restPositionsPinned A pointer to a pinned host memory buffer containing rest positions of the collision mesh.
	\param[in] stream A cuda stream to perform the copies.
	
	\see PxSoftBody
	\deprecated Use copyToDevice() instead.
	*/
	PX_DEPRECATED static void commit(PxSoftBody& softBody, PxSoftBodyDataFlags flags, PxVec4* simPositionsPinned, PxVec4* simVelocitiesPinned, PxVec4* collPositionsPinned, PxVec4* restPositionsPinned, CUstream stream = CUstream(0));

	/**
	\brief Uploads prepared SoftBody data to the GPU. It ensures that the embedded collision mesh matches the simulation mesh's transformation and scale.

	\param[in] softBody The soft body which will perform the data upload
	\param[in] flags Specifies which buffers the data transfer should include
	\param[in] simPositionsPinned A pointer to a pinned host memory buffer containing positions and inverse masses for each vertex of the simulation mesh.
	\param[in] simVelocitiesPinned A pointer to a pinned host memory buffer containing velocities for each vertex of the simulation mesh.
	\param[in] collPositionsPinned A pointer to a pinned host memory buffer containing positions and inverse masses for each vertex of the collision mesh.
	\param[in] restPositionsPinned A pointer to a pinned host memory buffer containing rest positions of the collision mesh.
	\param[in] stream A cuda stream to perform the copies.
	
	\see PxSoftBody
	*/
	static void copyToDevice(PxSoftBody& softBody, PxSoftBodyDataFlags flags, PxVec4* simPositionsPinned, PxVec4* simVelocitiesPinned, PxVec4* collPositionsPinned, PxVec4* restPositionsPinned, CUstream stream = CUstream(0));

	/**
	\brief Creates a full SoftBody mesh matching the shape given as input. Uses a voxel mesh for FEM simulation and a surface-matching mesh for collision detection. 

	\param[in] params Cooking params instance required for mesh processing
	\param[in] surfaceMesh Input triangle mesh that represents the surface of the SoftBody
	\param[in] numVoxelsAlongLongestAABBAxis The number of voxels along the longest bounding box axis
	\param[in] insertionCallback The insertion interface from PxPhysics
	\param[in] validate If set to true the input triangle mesh will get analyzed to find possible deficiencies
	\return SoftBody mesh if cooking was successful, NULL otherwise
	\see PxSoftBodyMesh
	*/
	static PxSoftBodyMesh* createSoftBodyMesh(const PxCookingParams& params, const PxSimpleTriangleMesh& surfaceMesh, PxU32 numVoxelsAlongLongestAABBAxis, PxInsertionCallback& insertionCallback, const bool validate = true);

	/**
	\brief Creates a full SoftBody mesh matching the shape given as input. Uses the same surface-matching mesh for collision detection and FEM simulation.

	\param[in] params Cooking params instance required for mesh processing
	\param[in] surfaceMesh Input triangle mesh that represents the surface of the SoftBody
	\param[in] insertionCallback The insertion interface from PxPhysics
	\param[in] maxWeightRatioInTet Upper limit for the ratio of node weights that are adjacent to the same tetrahedron. The closer to one (while remaining larger than one), the more stable the simulation. 
	\param[in] validate If set to true the input triangle mesh will get analyzed to find possible deficiencies
	\return SoftBody mesh if cooking was successful, NULL otherwise
	\see PxSoftBodyMesh
	*/
	static PxSoftBodyMesh* createSoftBodyMeshNoVoxels(const PxCookingParams& params, const PxSimpleTriangleMesh& surfaceMesh, PxInsertionCallback& insertionCallback, PxReal maxWeightRatioInTet = 1.5f, const bool validate = true);


	/**
	\brief Creates a SoftBody instance from a SoftBody mesh

	\param[in] softBodyMesh The SoftBody mesh
	\param[in] transform The transform that defines initial position and orientation of the SoftBody
	\param[in] material The material
	\param[in] cudaContextManager A cuda context manager
	\param[in] density The density used to compute the mass properties
	\param[in] solverIterationCount The number of iterations the solver should apply during simulation
	\param[in] femParams Additional parameters to specify e. g. damping
	\param[in] scale The scaling of the SoftBody
	\return SoftBody instance
	\see PxSoftBodyMesh, PxSoftBody
	*/
	static PxSoftBody* createSoftBodyFromMesh(PxSoftBodyMesh* softBodyMesh, const PxTransform& transform, 
		const PxFEMSoftBodyMaterial& material, PxCudaContextManager& cudaContextManager, PxReal density = 100.0f, PxU32 solverIterationCount = 30,
		const PxFEMParameters& femParams = PxFEMParameters(), PxReal scale = 1.0f);


	/**
	\brief Creates a SoftBody instance with a box shape

	\param[in] transform The transform that defines initial position and orientation of the SoftBody
	\param[in] boxDimensions The dimensions (side lengths) of the box shape
	\param[in] material The material
	\param[in] cudaContextManager A cuda context manager
	\param[in] maxEdgeLength The maximal length of a triangle edge. Subdivision will get applied until the edge length criteria is matched. -1 means no subdivision is applied.
	\param[in] density The density used to compute the mass properties
	\param[in] solverIterationCount The number of iterations the solver should apply during simulation
	\param[in] femParams Additional parameters to specify e. g. damping
	\param[in] numVoxelsAlongLongestAABBAxis The number of voxels to use for the simulation mesh along the longest bounding box dimension
	\param[in] scale The scaling of the SoftBody
	\return SoftBody instance
	\see PxSoftBodyMesh, PxSoftBody
	*/
	static PxSoftBody* createSoftBodyBox(const PxTransform& transform, const PxVec3& boxDimensions, const PxFEMSoftBodyMaterial& material,
		PxCudaContextManager& cudaContextManager, PxReal maxEdgeLength = -1.0f, PxReal density = 100.0f, PxU32 solverIterationCount = 30, 
		const PxFEMParameters& femParams = PxFEMParameters(), PxU32 numVoxelsAlongLongestAABBAxis = 10, PxReal scale = 1.0f);

	/**
	\brief allocates and initializes pinned host memory buffers from an actor with shape.

	\param[in] softBody A PxSoftBody that has a valid shape attached to it.
	\param[in] cudaContextManager The PxCudaContextManager of the scene this soft body will be simulated in
    \param[in] simPositionInvMassPinned A reference to a pointer for the return value of the simPositionInvMassPinned buffer, will be set by this function.
    \param[in] simVelocityPinned A reference to a pointer for the return value of the simVelocityPinned buffer, will be set by this function.
    \param[in] collPositionInvMassPinned A reference to a pointer for the return value of the collPositionInvMassPinned buffer, will be set by this function.
    \param[in] restPositionPinned A reference to a pointer for the return value of the restPositionPinned buffer, will be set by this function.

	\see PxSoftBody
	 */
	static void allocateAndInitializeHostMirror(PxSoftBody& softBody, PxCudaContextManager* cudaContextManager, PxVec4*& simPositionInvMassPinned, PxVec4*& simVelocityPinned, PxVec4*& collPositionInvMassPinned, PxVec4*& restPositionPinned);
	
	/**
	\brief Given a set of points and a set of tetrahedra, it finds the equilibrium state of the softbody. Every input point is either fixed or can move freely.

	\param[in] verticesOriginal Mesh vertex positions in undeformed original state.
	\param[in] verticesDeformed Mesh vertex positions in new deformed state. Only fixed vertices must have their final location, all other locations will get updated by the method.
	\param[in] nbVertices The number of vertices.
	\param[in] tetrahedra The tetrahedra.
	\param[in] nbTetraheda The number of tetrahedra.
	\param[in] vertexIsFixed Optional input that specifies which vertex is fixed and which one can move to relax the tension. If not provided, vertices from verticesOriginal which have a .w value of 0 will be considered fixed.
	\param[in] numIterations The number of stress relaxation iterations to run.
	*/
	static void relaxSoftBodyMesh(const PxVec4* verticesOriginal, PxVec4* verticesDeformed, PxU32 nbVertices, const PxU32* tetrahedra, PxU32 nbTetraheda, const bool* vertexIsFixed = NULL, PxU32 numIterations = 200);
};

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
