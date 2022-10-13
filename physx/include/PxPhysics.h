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

#ifndef PX_PHYSICS_H
#define PX_PHYSICS_H

/** \addtogroup physics
@{
*/

#include "PxPhysXConfig.h"
#include "PxDeletionListener.h"
#include "foundation/PxTransform.h"
#include "PxShape.h"
#include "PxAggregate.h"
#include "PxBuffer.h"
#include "PxParticleSystem.h"
#include "foundation/PxPreprocessor.h"
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
#include "PxFEMCloth.h"
#include "PxHairSystem.h"
#endif


#if !PX_DOXYGEN
namespace physx
{
#endif

class PxPvd;
class PxOmniPvd;
class PxInsertionCallback;

class PxRigidActor;
class PxConstraintConnector;
struct PxConstraintShaderTable;

class PxGeometry;
class PxFoundation;
class PxSerializationRegistry;

class PxPruningStructure;
class PxBVH;
typedef PX_DEPRECATED PxBVH PxBVHStructure;

class PxGpuDispatcher;

class PxParticleClothBuffer;
class PxParticleRigidBuffer;

/**
\brief Abstract singleton factory class used for instancing objects in the Physics SDK.

In addition you can use PxPhysics to set global parameters which will effect all scenes and create
objects that can be shared across multiple scenes.

You can get an instance of this class by calling PxCreateBasePhysics() or PxCreatePhysics() with pre-registered modules.

@see PxCreatePhysics() PxCreateBasePhysics() PxScene PxVisualizationParameter
*/
class PxPhysics
{
public:

	/** @name Basics
	*/
	//@{

	virtual ~PxPhysics() {}

	/**
	\brief Destroys the instance it is called on.

	Use this release method to destroy an instance of this class. Be sure
	to not keep a reference to this object after calling release.
	Avoid release calls while a scene is simulating (in between simulate() and fetchResults() calls).

	Note that this must be called once for each prior call to PxCreatePhysics, as
	there is a reference counter. Also note that you mustn't destroy the allocator or the error callback (if available) until after the
	reference count reaches 0 and the SDK is actually removed.

	Releasing an SDK will also release any scenes, triangle meshes, convex meshes, heightfields and shapes
	created through it, provided the user hasn't already done so.

	\note This function is required to be called to release foundation usage.

	@see PxCreatePhysics()
	*/
	virtual	void release() = 0;

	/**
	\brief Retrieves the Foundation instance.
	\return A reference to the Foundation object.
	*/
	virtual PxFoundation&		getFoundation() = 0;

	/**
	\brief Retrieves the PxOmniPvd instance if there is one registered with PxPhysics
	\return A pointer to a PxOmniPvd object.
	*/
	virtual PxOmniPvd* getOmniPvd() = 0;

	/**
	\brief Creates an aggregate with the specified maximum size and filtering hint.

	The previous API used "bool enableSelfCollision" which should now silently evaluates
	to a PxAggregateType::eGENERIC aggregate with its self-collision bit.

	Use PxAggregateType::eSTATIC or PxAggregateType::eKINEMATIC for aggregates that will
	only contain static or kinematic actors. This provides faster filtering when used in
	combination with PxPairFilteringMode.

	\param	[in] maxActor		The maximum number of actors that may be placed in the aggregate.
	\param	[in] maxShape		The maximum number of shapes that may be placed in the aggregate.
	\param	[in] filterHint		The aggregate's filtering hint.
	\return The new aggregate.

	@see PxAggregate PxAggregateFilterHint PxAggregateType PxPairFilteringMode
	*/
	virtual	PxAggregate*		createAggregate(const PxU32 maxActor, const PxU32 maxShape, PxAggregateFilterHint filterHint) = 0;

	/**
	\brief Creates an aggregate with the specified maximum size and filtering hint.

	The previous API used "bool enableSelfCollision" which should now silently evaluates
	to a PxAggregateType::eGENERIC aggregate with its self-collision bit.

	Use PxAggregateType::eSTATIC or PxAggregateType::eKINEMATIC for aggregates that will
	only contain static or kinematic actors. This provides faster filtering when used in
	combination with PxPairFilteringMode.

	\note This variation of the method is not compatible with GPU rigid bodies.

	\param	[in] maxActor		The maximum number of actors that may be placed in the aggregate.
	\param	[in] filterHint		The aggregate's filtering hint.
	\return The new aggregate.

	@see PxAggregate PxAggregateFilterHint PxAggregateType PxPairFilteringMode
	*/
	PX_FORCE_INLINE PX_DEPRECATED PxAggregate* createAggregate(const PxU32 maxActor, PxAggregateFilterHint filterHint)
	{
		return createAggregate(maxActor, PX_MAX_U32, filterHint);
	}

	/**
	\brief Returns the simulation tolerance parameters.
	\return The current simulation tolerance parameters.
	*/
	virtual const PxTolerancesScale&		getTolerancesScale() const = 0;

	//@}
	/** @name Meshes
	*/
	//@{

	/**
	\brief Creates a triangle mesh object.

	This can then be instanced into #PxShape objects.

	\param	[in] stream	The triangle mesh stream.
	\return The new triangle mesh.

	@see PxTriangleMesh PxMeshPreprocessingFlag PxTriangleMesh.release() PxInputStream PxTriangleMeshFlag
	*/
	virtual PxTriangleMesh*    createTriangleMesh(PxInputStream& stream) = 0;

	/**
	\brief Return the number of triangle meshes that currently exist.

	\return Number of triangle meshes.

	@see getTriangleMeshes()
	*/
	virtual PxU32				getNbTriangleMeshes() const = 0;

	/**
	\brief Writes the array of triangle mesh pointers to a user buffer.

	Returns the number of pointers written.

	The ordering of the triangle meshes in the array is not specified.

	\param	[out] userBuffer	The buffer to receive triangle mesh pointers.
	\param	[in]  bufferSize	The number of triangle mesh pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first mesh pointer to be retrieved
	\return The number of triangle mesh pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbTriangleMeshes() PxTriangleMesh
	*/
	virtual	PxU32				getTriangleMeshes(PxTriangleMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const = 0;


	//@}
	/** @name Tetrahedron Meshes
	*/
	//@{

	/**
	\brief Creates a tetrahedron mesh object.

	This can then be instanced into #PxShape objects.

	\param[in] stream The tetrahedron mesh stream.
	\return The new tetrahedron mesh.

	@see PxTriangleMesh PxMeshPreprocessingFlag PxTriangleMesh.release() PxInputStream PxTriangleMeshFlag
	*/
	virtual PxTetrahedronMesh*    createTetrahedronMesh(PxInputStream& stream) = 0;

	/**
	\brief Creates a softbody mesh object.

	\param[in] stream The softbody mesh stream.
	\return The new softbody mesh.

	@see createTetrahedronMesh
	*/
	virtual PxSoftBodyMesh*		  createSoftBodyMesh(PxInputStream& stream) = 0;

	/**
	\brief Return the number of tetrahedron meshes that currently exist.

	\return Number of tetrahedron meshes.

	@see getTetrahedronMeshes()
	*/
	virtual PxU32				getNbTetrahedronMeshes() const = 0;

	/**
	\brief Writes the array of tetrahedron mesh pointers to a user buffer.

	Returns the number of pointers written.

	The ordering of the triangle meshes in the array is not specified.

	\param[out] userBuffer The buffer to receive tetrahedron mesh pointers.
	\param[in] bufferSize The number of tetrahedron mesh pointers which can be stored in the buffer.
	\param[in] startIndex Index of first mesh pointer to be retrieved
	\return The number of tetrahedron mesh pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbTetrahedronMeshes() PxTetrahedronMesh
	*/
	virtual	PxU32				getTetrahedronMeshes(PxTetrahedronMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const = 0;

	/**
	\brief Creates a heightfield object from previously cooked stream.

	This can then be instanced into #PxShape objects.

	\param	[in] stream	The heightfield mesh stream.
	\return The new heightfield.

	@see PxHeightField PxHeightField.release() PxInputStream PxRegisterHeightFields
	*/
	virtual PxHeightField*		createHeightField(PxInputStream& stream) = 0;

	/**
	\brief Return the number of heightfields that currently exist.

	\return Number of heightfields.

	@see getHeightFields()
	*/
	virtual PxU32				getNbHeightFields() const = 0;

	/**
	\brief Writes the array of heightfield pointers to a user buffer.

	Returns the number of pointers written.

	The ordering of the heightfields in the array is not specified.

	\param	[out] userBuffer	The buffer to receive heightfield pointers.
	\param	[in]  bufferSize	The number of heightfield pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first heightfield pointer to be retrieved
	\return The number of heightfield pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbHeightFields() PxHeightField
	*/
	virtual	PxU32				getHeightFields(PxHeightField** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const = 0;

	/**
	\brief Creates a convex mesh object.

	This can then be instanced into #PxShape objects.

	\param	[in] stream	The stream to load the convex mesh from.
	\return The new convex mesh.

	@see PxConvexMesh PxConvexMesh.release() PxInputStream createTriangleMesh() PxConvexMeshGeometry PxShape
	*/
	virtual PxConvexMesh*		createConvexMesh(PxInputStream& stream) = 0;

	/**
	\brief Return the number of convex meshes that currently exist.

	\return Number of convex meshes.

	@see getConvexMeshes()
	*/
	virtual PxU32				getNbConvexMeshes() const = 0;

	/**
	\brief Writes the array of convex mesh pointers to a user buffer.

	Returns the number of pointers written.

	The ordering of the convex meshes in the array is not specified.

	\param	[out] userBuffer	The buffer to receive convex mesh pointers.
	\param	[in]  bufferSize	The number of convex mesh pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first convex mesh pointer to be retrieved
	\return The number of convex mesh pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbConvexMeshes() PxConvexMesh
	*/
	virtual	PxU32				getConvexMeshes(PxConvexMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const = 0;

	/**
	\brief Creates a bounding volume hierarchy.

	\param	[in] stream	The stream to load the BVH from.
	\return The new BVH.

	@see PxBVH PxInputStream
	*/
	virtual PxBVH*		createBVH(PxInputStream& stream) = 0;

	PX_DEPRECATED PX_FORCE_INLINE PxBVH*	createBVHStructure(PxInputStream& stream)
											{
												return createBVH(stream);
											}

	/**
	\brief Return the number of bounding volume hierarchies that currently exist.

	\return Number of bounding volume hierarchies.

	@see PxBVH getBVHs()
	*/
	virtual PxU32				getNbBVHs() const = 0;

	PX_DEPRECATED PX_FORCE_INLINE PxU32		getNbBVHStructures() const
											{
												return getNbBVHs();
											}

	/**
	\brief Writes the array of bounding volume hierarchies pointers to a user buffer.

	Returns the number of pointers written.

	The ordering of the BVHs in the array is not specified.

	\param	[out] userBuffer	The buffer to receive BVH pointers.
	\param	[in]  bufferSize	The number of BVH pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first BVH pointer to be retrieved
	\return The number of BVH pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbBVHs() PxBVH
	*/
	virtual	PxU32				getBVHs(PxBVH** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const = 0;

	PX_DEPRECATED PX_FORCE_INLINE PxU32	getBVHStructures(PxBVHStructure** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const
										{
											return getBVHs(userBuffer, bufferSize, startIndex);
										}

	//@}
	/** @name Scenes
	*/
	//@{

	/**
	\brief Creates a scene.

	\note Every scene uses a Thread Local Storage slot. This imposes a platform specific limit on the
	number of scenes that can be created.

	\param	[in] sceneDesc	Scene descriptor. See #PxSceneDesc
	\return The new scene object.

	@see PxScene PxScene.release() PxSceneDesc
	*/
	virtual PxScene*			createScene(const PxSceneDesc& sceneDesc) = 0;

	/**
	\brief Gets number of created scenes.

	\return The number of scenes created.

	@see getScene()
	*/
	virtual PxU32				getNbScenes()			const = 0;

	/**
	\brief Writes the array of scene pointers to a user buffer.

	Returns the number of pointers written.

	The ordering of the scene pointers in the array is not specified.

	\param	[out] userBuffer	The buffer to receive scene pointers.
	\param	[in]  bufferSize	The number of scene pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first scene pointer to be retrieved
	\return The number of scene pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbScenes() PxScene
	*/
	virtual	PxU32				getScenes(PxScene** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const = 0;

	//@}
	/** @name Actors
	*/
	//@{

	/**
	\brief Creates a static rigid actor with the specified pose and all other fields initialized
	to their default values.

	\param	[in] pose	The initial pose of the actor. Must be a valid transform

	@see PxRigidStatic
	*/
	virtual PxRigidStatic*      createRigidStatic(const PxTransform& pose) = 0;

	/**
	\brief Creates a dynamic rigid actor with the specified pose and all other fields initialized
	to their default values.

	\param	[in] pose	The initial pose of the actor. Must be a valid transform

	@see PxRigidDynamic
	*/
	virtual PxRigidDynamic*      createRigidDynamic(const PxTransform& pose) = 0;

	/**
	\brief Creates a pruning structure from actors.

	\note Every provided actor needs at least one shape with the eSCENE_QUERY_SHAPE flag set.
	\note Both static and dynamic actors can be provided.
	\note It is not allowed to pass in actors which are already part of a scene.
	\note Articulation links cannot be provided.

	\param	[in] actors		Array of actors to add to the pruning structure. Must be non NULL.
	\param	[in] nbActors	Number of actors in the array. Must be >0.
	\return Pruning structure created from given actors, or NULL if any of the actors did not comply with the above requirements.
	@see PxActor PxPruningStructure
	*/
	virtual PxPruningStructure*	createPruningStructure(PxRigidActor*const* actors, PxU32 nbActors) = 0;

	//@}
	/** @name Shapes
	*/
	//@{

	/**
	\brief Creates a shape which may be attached to multiple actors

	The shape will be created with a reference count of 1.

	\param	[in] geometry		The geometry for the shape
	\param	[in] material		The material for the shape
	\param	[in] isExclusive	Whether this shape is exclusive to a single actor or maybe be shared
	\param	[in] shapeFlags		The PxShapeFlags to be set
	\return The shape

	\note Shared shapes are not mutable when they are attached to an actor

	@see PxShape
	*/
	PX_FORCE_INLINE	PxShape*	createShape(	const PxGeometry& geometry,
												const PxMaterial& material,
												bool isExclusive = false,
												PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE)
	{
		PxMaterial* materialPtr = const_cast<PxMaterial*>(&material);
		return createShape(geometry, &materialPtr, 1, isExclusive, shapeFlags);
	}

	PX_FORCE_INLINE	PxShape*	createShape(	const PxGeometry& geometry,
												const PxFEMSoftBodyMaterial& material,
												bool isExclusive = false,
												PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE)
	{
		PxFEMSoftBodyMaterial* materialPtr = const_cast<PxFEMSoftBodyMaterial*>(&material);
		return createShape(geometry, &materialPtr, 1, isExclusive, shapeFlags);
	}

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	PX_FORCE_INLINE	PxShape*	createShape(	const PxGeometry& geometry,
												const PxFEMClothMaterial& material,
												bool isExclusive = false,
												PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE)
	{
		PxFEMClothMaterial* materialPtr = const_cast<PxFEMClothMaterial*>(&material);
		return createShape(geometry, &materialPtr, 1, isExclusive, shapeFlags);
	}
#endif

	/**
	\brief Creates a shape which may be attached to multiple actors

	The shape will be created with a reference count of 1.

	\param	[in] geometry		The geometry for the shape
	\param	[in] materials		The materials for the shape
	\param	[in] materialCount	The number of materials
	\param	[in] isExclusive	Whether this shape is exclusive to a single actor or may be shared
	\param	[in] shapeFlags		The PxShapeFlags to be set
	\return The shape

	\note Shared shapes are not mutable when they are attached to an actor
	\note Shapes created from *SDF* triangle-mesh geometries do not support more than one material.

	@see PxShape
	*/
	virtual PxShape*			createShape(	const PxGeometry& geometry,
												PxMaterial*const * materials,
												PxU16 materialCount,
												bool isExclusive = false,
												PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE) = 0;

	virtual PxShape*			createShape(	const PxGeometry& geometry,
												PxFEMSoftBodyMaterial*const * materials,
												PxU16 materialCount,
												bool isExclusive = false,
												PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE) = 0;

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	virtual PxShape*			createShape(	const PxGeometry& geometry,
												PxFEMClothMaterial*const * materials,
												PxU16 materialCount,
												bool isExclusive = false,
												PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE) = 0;
#endif

	/**
	\brief Return the number of shapes that currently exist.

	\return Number of shapes.

	@see getShapes()
	*/
	virtual PxU32				getNbShapes() const = 0;

	/**
	\brief Writes the array of shape pointers to a user buffer.

	Returns the number of pointers written.

	The ordering of the shapes in the array is not specified.

	\param	[out] userBuffer	The buffer to receive shape pointers.
	\param	[in]  bufferSize	The number of shape pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first shape pointer to be retrieved
	\return The number of shape pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbShapes() PxShape
	*/
	virtual	PxU32				getShapes(PxShape** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const = 0;

	//@}
	/** @name Constraints and Articulations
	*/
	//@{

	/**
	\brief Creates a constraint shader.

	\note A constraint shader will get added automatically to the scene the two linked actors belong to. Either, but not both, of actor0 and actor1 may
	be NULL to denote attachment to the world.

	\param	[in] actor0		The first actor
	\param	[in] actor1		The second actor
	\param	[in] connector	The connector object, which the SDK uses to communicate with the infrastructure for the constraint
	\param	[in] shaders	The shader functions for the constraint
	\param	[in] dataSize	The size of the data block for the shader

	\return The new shader.

	@see PxConstraint
	*/
	virtual PxConstraint*		createConstraint(PxRigidActor* actor0, PxRigidActor* actor1, PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize) = 0;

	/**
	\brief Creates a reduced-coordinate articulation with all fields initialized to their default values.

	\return the new articulation

	@see PxArticulationReducedCoordinate, PxRegisterArticulationsReducedCoordinate
	*/
	virtual PxArticulationReducedCoordinate*	createArticulationReducedCoordinate() = 0;


#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	/**
	\brief Creates a FEM-based cloth with all fields initialized to their default values.

	\param[in] cudaContextManager The PxCudaContextManager this instance is tied to.
	\return the new FEM-cloth

	@see PxFEMCloth
	*/
	virtual PxFEMCloth*	createFEMCloth(PxCudaContextManager& cudaContextManager) = 0;
#endif

	/**
	\brief Creates a FEM-based soft body with all fields initialized to their default values.

	\param[in] cudaContextManager The PxCudaContextManager this instance is tied to.
	\return the new soft body

	@see PxSoftBody
	*/
	virtual PxSoftBody*	createSoftBody(PxCudaContextManager& cudaContextManager) = 0;

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	/**
	\brief Creates a hair system with all fields initialized to their default values.

	\param[in] cudaContextManager The PxCudaContextManager this instance is tied to.
	\return the new hair system

	@see PxHairSystem
	*/
	virtual PxHairSystem*	createHairSystem(PxCudaContextManager& cudaContextManager) = 0;
#endif


	/**
	\brief Creates a particle system with all fields initialized to their default values.

	\return the new particle system

	@see PxPBDParticleSystem
	*/
	virtual PxPBDParticleSystem*	createPBDParticleSystem(PxCudaContextManager& cudaContextManager, PxU32 maxNeighborhood = 96) = 0;

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	/**
	\brief Creates a particle system with all fields initialized to their default values.

	\return the new particle system

	@see PxFLIPParticleSystem
	*/
	virtual PxFLIPParticleSystem*	createFLIPParticleSystem(PxCudaContextManager& cudaContextManager) = 0;

	/**
	\brief Creates a particle system with all fields initialized to their default values.

	\return the new particle system

	@see PxFLIPParticleSystem
	*/
	virtual PxMPMParticleSystem*	createMPMParticleSystem(PxU32 maxParticles, 
		PxCudaContextManager& cudaContextManager) = 0;

	/**
	\brief Creates a particle system with all fields initialized to their default values.

	\return the new particle system

	@see PxFLIPParticleSystem
	*/
	virtual PxCustomParticleSystem*	createCustomParticleSystem(PxU32 maxParticles,
		PxCudaContextManager& cudaContextManager, PxU32 maxNeighborhood) = 0;
#endif
	/**
	\brief Create a buffer for reading and writing data across host and device memory spaces.

	\param[in] byteSize The size of the buffer in bytes.
	\param[in] bufferType The memory space of the buffer.
	\param[in] cudaContextManager The PxCudaContextManager this buffer is tied to.
	\return PxBuffer instance.

	@see PxBuffer
	*/
	virtual PxBuffer*			createBuffer(PxU64 byteSize, PxBufferType::Enum bufferType, PxCudaContextManager* cudaContextManager) = 0;

	/**
	\brief Create particle buffer for fluid/granular material
	\param[in] maxParticles The max number of particles
	\param[in] maxVolumes The max number of volumes
	\param[in] cudaContextManager The PxCudaContextManager this buffer is tied to.
	\return PxParticleBuffer instance

	@see PxParticleBuffer
	*/
	virtual		PxParticleBuffer*			createParticleBuffer(const PxU32 maxParticles, const PxU32 maxVolumes, PxCudaContextManager* cudaContextManager) = 0;

	/**
	\brief Create diffuse buffer for fluid dynamic with diffuse particles
	\param[in] maxParticles The max number of particles
	\param[in] maxVolumes The max number of volumes
	\param[in] maxDiffuseParticles The max number of diffuse particles
	\param[in] cudaContextManager The PxCudaContextManager this buffer is tied to.
	\return PxParticleAndDiffuseBuffer instance

	@see PxParticleAndDiffuseBuffer
	*/
	virtual		PxParticleAndDiffuseBuffer*	createParticleAndDiffuseBuffer(const PxU32 maxParticles, const PxU32 maxVolumes, const PxU32 maxDiffuseParticles, PxCudaContextManager* cudaContextManager) = 0;

	/**
	\brief Create cloth buffer
	\param[in] maxParticles The max number of particles
	\param[in] maxNumVolumes The max number of volumes
	\param[in] maxNumCloths The max number of cloths
	\param[in] maxNumTriangles The max number of triangles for aerodynamics
	\param[in] maxNumSprings The max number of springs to connect particles
	\param[in] cudaContextManager The PxCudaContextManager this buffer is tied to.
	\return PxParticleClothBuffer instance

	@see PxParticleClothBuffer
	*/
	virtual		PxParticleClothBuffer*		createParticleClothBuffer(const PxU32 maxParticles, const PxU32 maxNumVolumes, const PxU32 maxNumCloths, const PxU32 maxNumTriangles, const PxU32 maxNumSprings, PxCudaContextManager* cudaContextManager) = 0;
	
	/**
	\brief Create rigid buffer for shape matching rigid bodies
	\param[in] maxParticles The max number of particles
	\param[in] maxNumVolumes The max number of volumes
	\param[in] maxNumRigids The max number of shape matching rigid bodies
	\param[in] cudaContextManager The PxCudaContextManager this buffer is tied to.
	\return PxParticleRigidBuffer instance

	@see PxParticleRigidBuffer
	*/
	virtual		PxParticleRigidBuffer*		createParticleRigidBuffer(const PxU32 maxParticles, const PxU32 maxNumVolumes, const PxU32 maxNumRigids, PxCudaContextManager* cudaContextManager) = 0;

	//@}
	/** @name Materials
	*/
	//@{

	/**
	\brief Creates a new material with default properties.

	\return The new material.

	\param	[in] staticFriction		The coefficient of static friction
	\param	[in] dynamicFriction	The coefficient of dynamic friction
	\param	[in] restitution		The coefficient of restitution

	@see PxMaterial
	*/
	virtual PxMaterial*        createMaterial(PxReal staticFriction, PxReal dynamicFriction, PxReal restitution) = 0;


	/**
	\brief Return the number of materials that currently exist.

	\return Number of materials.

	@see getMaterials()
	*/
	virtual PxU32				getNbMaterials() const = 0;

	/**
	\brief Writes the array of material pointers to a user buffer.

	Returns the number of pointers written.

	The ordering of the materials in the array is not specified.

	\param	[out] userBuffer	The buffer to receive material pointers.
	\param	[in]  bufferSize	The number of material pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first material pointer to be retrieved
	\return The number of material pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbMaterials() PxMaterial
	*/
	virtual	PxU32				getMaterials(PxMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const = 0;


	//@}
	/** @name Materials
	*/
	//@{

	/**
	\brief Creates a new fem material with default properties.

	\return The new fem material.

	\param	[in] youngs					The young's modulus
	\param	[in] poissons				The poissons's ratio
	\param	[in] dynamicFriction		The dynamic friction coefficient

	@see PxFEMSoftBodyMaterial
	*/
	virtual PxFEMSoftBodyMaterial*        createFEMSoftBodyMaterial(PxReal youngs, PxReal poissons, PxReal dynamicFriction) = 0;


	/**
	\brief Return the number of fem materials that currently exist.

	\return Number of fem materials.

	@see getFEMSoftBodyMaterials()
	*/
	virtual PxU32						getNbFEMSoftBodyMaterials() const = 0;

	/**
	\brief Writes the array of material pointers to a user buffer.

	Returns the number of pointers written.

	The ordering of the materials in the array is not specified.

	\param	[out] userBuffer	The buffer to receive material pointers.
	\param	[in]  bufferSize	The number of material pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first material pointer to be retrieved
	\return The number of material pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbFEMSoftBodyMaterials() PxFEMSoftBodyMaterial
	*/
	virtual	PxU32						getFEMSoftBodyMaterials(PxFEMSoftBodyMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const = 0;

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	/**
	\brief Creates a new fem cloth material with default properties.

	\return The new fem material.

	\param	[in] youngs					The young's modulus
	\param	[in] poissons				The poissons's ratio
	\param	[in] dynamicFriction		The dynamic friction coefficient

	@see PxFEMClothMaterial
	*/
	virtual		PxFEMClothMaterial*			createFEMClothMaterial(PxReal youngs, PxReal poissons, PxReal dynamicFriction) = 0;
#endif


	/**
	\brief Return the number of fem cloth materials that currently exist.

	\return Number of fem cloth materials.

	@see getFEMClothMaterials()
	*/
	virtual		PxU32						getNbFEMClothMaterials() const = 0;

	/**
	\brief Writes the array of material pointers to a user buffer.

	Returns the number of pointers written.

	The ordering of the materials in the array is not specified.

	\param	[out] userBuffer	The buffer to receive material pointers.
	\param	[in]  bufferSize	The number of material pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first material pointer to be retrieved
	\return The number of material pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbFEMClothMaterials() PxFEMClothMaterial
	*/
	virtual		PxU32						getFEMClothMaterials(PxFEMClothMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const = 0;

	/**
	\brief Creates a new PBD material with default properties.

	\param	[in]  friction				The friction parameter
	\param	[in]  damping				The velocity damping parameter
	\param	[in]  adhesion				The adhesion parameter
	\param	[in]  viscosity				The viscosity parameter
	\param	[in]  vorticityConfinement	The vorticity confinement coefficient
	\param	[in]  surfaceTension		The surface tension coefficient
	\param	[in]  cohesion				The cohesion parameter
	\param	[in]  lift					The lift parameter
	\param	[in]  drag					The drag parameter
	\param	[in]  cflCoefficient		The cfl coefficient
	\param	[in]  gravityScale			The gravity scale
	\return The new PBD material.

	@see PxPBDMaterial
	*/
	virtual		PxPBDMaterial*				createPBDMaterial(PxReal friction, PxReal damping, PxReal adhesion, PxReal viscosity, PxReal vorticityConfinement, PxReal surfaceTension, PxReal cohesion, PxReal lift, PxReal drag, PxReal cflCoefficient = 1.f, PxReal gravityScale = 1.f) = 0;

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	virtual		PxCustomMaterial*			createCustomMaterial(void* gpuBuffer) = 0;
#endif

	/**
	\brief Return the number of PBD materials that currently exist.

	\return Number of PBD materials.

	@see getPBDMaterials()
	*/
	virtual		PxU32						getNbPBDMaterials() const = 0;

	/**
	\brief Writes the array of material pointers to a user buffer.

	Returns the number of pointers written.

	The ordering of the materials in the array is not specified.

	\param	[out] userBuffer	The buffer to receive material pointers.
	\param	[in]  bufferSize	The number of material pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first material pointer to be retrieved
	\return The number of material pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbPBDMaterials() PxPBDMaterial
	*/
	virtual		PxU32						getPBDMaterials(PxPBDMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const = 0;
	
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	/**
	\brief Creates a new FLIP material with default properties.

	\param	[in]  friction				The friction parameter
	\param	[in]  damping				The velocity damping parameter
	\param	[in]  adhesion				The maximum velocity magnitude of particles
	\param	[in]  viscosity				The viscosity parameter
	\param	[in]  gravityScale			The gravity scale
	\return The new FLIP material.

	@see PxFLIPMaterial
	*/
	virtual		PxFLIPMaterial*			createFLIPMaterial(PxReal friction, PxReal damping, PxReal adhesion, PxReal viscosity, PxReal gravityScale = 1.f) = 0;


	/**
	\brief Return the number of FLIP materials that currently exist.

	\return Number of FLIP materials.

	@see getFLIPMaterials()
	*/
	virtual		PxU32						getNbFLIPMaterials() const = 0;

	/**
	\brief Writes the array of material pointers to a user buffer.

	Returns the number of pointers written.

	The ordering of the materials in the array is not specified.

	\param	[out] userBuffer	The buffer to receive material pointers.
	\param	[in]  bufferSize	The number of material pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first material pointer to be retrieved
	\return The number of material pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbFLIPMaterials() PxFLIPMaterial
	*/
	virtual		PxU32						getFLIPMaterials(PxFLIPMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const = 0;
	
	/**
	\brief Creates a new MPM material with default properties.
	
	\param	[in]  friction						The friction parameter
	\param	[in]  damping						The velocity damping parameter
	\param	[in]  adhesion						The maximum velocity magnitude of particles
	\param	[in]  isPlastic						True if plastic
	\param	[in]  youngsModulus					The Young's modulus
	\param	[in]  poissons						The Poissons's ratio
	\param	[in]  hardening						The hardening parameter
	\param	[in]  criticalCompression			The critical compression parameter
	\param	[in]  criticalStretch				The critical stretch parameter
	\param	[in]  tensileDamageSensitivity		The tensile damage sensitivity parameter
	\param	[in]  compressiveDamageSensitivity	The compressive damage sensitivity parameter
	\param	[in]  attractiveForceResidual		The attractive force residual parameter
	\param	[in]  gravityScale					The gravity scale
	\return The new MPM material.

	@see PxMPMMaterial
	*/
	virtual		PxMPMMaterial*				createMPMMaterial(PxReal friction, PxReal damping, PxReal adhesion, bool isPlastic, PxReal youngsModulus, PxReal poissons, PxReal hardening, PxReal criticalCompression, PxReal criticalStretch, PxReal tensileDamageSensitivity, PxReal compressiveDamageSensitivity, PxReal attractiveForceResidual, PxReal gravityScale = 1.f) = 0;


	/**
	\brief Return the number of MPM materials that currently exist.

	\return Number of MPM materials.

	@see getMPMMaterials()
	*/
	virtual		PxU32						getNbMPMMaterials() const = 0;

	/**
	\brief Writes the array of material pointers to a user buffer.

	Returns the number of pointers written.

	The ordering of the materials in the array is not specified.

	\param	[out] userBuffer	The buffer to receive material pointers.
	\param	[in]  bufferSize	The number of material pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first material pointer to be retrieved
	\return The number of material pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbMPMMaterials() PxMPMMaterial
	*/
	virtual		PxU32						getMPMMaterials(PxMPMMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const = 0;
#endif
	//@}
	/** @name Deletion Listeners
	*/
	//@{

	/**
	\brief Register a deletion listener. Listeners will be called whenever an object is deleted.

	It is illegal to register or unregister a deletion listener while deletions are being processed.

	\note By default a registered listener will receive events from all objects. Set the restrictedObjectSet parameter to true on registration and use #registerDeletionListenerObjects to restrict the received events to specific objects.

	\note The deletion events are only supported on core PhysX objects. In general, objects in extension modules do not provide this functionality, however, in the case of PxJoint objects, the underlying PxConstraint will send the events.

	\param	[in] observer				Observer object to send notifications to.
	\param	[in] deletionEvents			The deletion event types to get notified of.
	\param	[in] restrictedObjectSet	If false, the deletion listener will get events from all objects, else the objects to receive events from have to be specified explicitly through #registerDeletionListenerObjects.

	@see PxDeletionListener unregisterDeletionListener
	*/
	virtual void registerDeletionListener(PxDeletionListener& observer, const PxDeletionEventFlags& deletionEvents, bool restrictedObjectSet = false) = 0;

	/**
	\brief Unregister a deletion listener.

	It is illegal to register or unregister a deletion listener while deletions are being processed.

	\param	[in] observer	Observer object to send notifications to

	@see PxDeletionListener registerDeletionListener
	*/
	virtual void unregisterDeletionListener(PxDeletionListener& observer) = 0;

	/**
	\brief Register specific objects for deletion events.

	This method allows for a deletion listener to limit deletion events to specific objects only.

	\note It is illegal to register or unregister objects while deletions are being processed.

	\note The deletion listener has to be registered through #registerDeletionListener() and configured to support restricted objects sets prior to this method being used.

	\param	[in] observer			Observer object to send notifications to.
	\param	[in] observables		List of objects for which to receive deletion events. Only PhysX core objects are supported. In the case of PxJoint objects, the underlying PxConstraint can be used to get the events.
	\param	[in] observableCount	Size of the observables list.

	@see PxDeletionListener unregisterDeletionListenerObjects
	*/
	virtual void registerDeletionListenerObjects(PxDeletionListener& observer, const PxBase* const* observables, PxU32 observableCount) = 0;

	/**
	\brief Unregister specific objects for deletion events.

	This method allows to clear previously registered objects for a deletion listener (see #registerDeletionListenerObjects()).

	\note It is illegal to register or unregister objects while deletions are being processed.

	\note The deletion listener has to be registered through #registerDeletionListener() and configured to support restricted objects sets prior to this method being used.

	\param	[in] observer			Observer object to stop sending notifications to.
	\param	[in] observables		List of objects for which to not receive deletion events anymore.
	\param	[in] observableCount	Size of the observables list.

	@see PxDeletionListener registerDeletionListenerObjects
	*/
	virtual void unregisterDeletionListenerObjects(PxDeletionListener& observer, const PxBase* const* observables, PxU32 observableCount) = 0;

	/**
	\brief Gets PxPhysics object insertion interface.

	The insertion interface is needed ie. for PxCooking::createTriangleMesh, this allows runtime mesh creation.

	@see PxCooking::createTriangleMesh PxCooking::createHeightfield PxCooking::createTetrahedronMesh PxCooking::createBVH
	*/
	virtual PxInsertionCallback& getPhysicsInsertionCallback() = 0;

	//@}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/**
\brief Enables the usage of the reduced coordinate articulations feature.  This function is called automatically inside PxCreatePhysics().
On resource constrained platforms, it is possible to call PxCreateBasePhysics() and then NOT call this function
to save on code memory if your application does not use reduced coordinate articulations.  In this case the linker should strip out
the relevant implementation code from the library.  If you need to use reduced coordinate articulations but not some other optional
component, you shoud call PxCreateBasePhysics() followed by this call.
*/
PX_C_EXPORT PX_PHYSX_CORE_API void PX_CALL_CONV PxRegisterArticulationsReducedCoordinate(physx::PxPhysics& physics);

/**
\brief Enables the usage of the heightfield feature.

This call will link the default 'unified' implementation of heightfields which is identical to the narrow phase of triangle meshes.
This function is called automatically inside PxCreatePhysics().

On resource constrained platforms, it is possible to call PxCreateBasePhysics() and then NOT call this function
to save on code memory if your application does not use heightfields.  In this case the linker should strip out
the relevant implementation code from the library.  If you need to use heightfield but not some other optional
component, you shoud call PxCreateBasePhysics() followed by this call.

You must call this function at a time where no ::PxScene instance exists, typically before calling PxPhysics::createScene().
This is to prevent a change to the heightfield implementation code at runtime which would have undefined results.

Calling PxCreateBasePhysics() and then attempting to create a heightfield shape without first calling
::PxRegisterHeightFields(), will result in an error.
*/
PX_C_EXPORT PX_PHYSX_CORE_API void PX_CALL_CONV PxRegisterHeightFields(physx::PxPhysics& physics);

/**
\brief Creates an instance of the physics SDK with minimal additional components registered

Creates an instance of this class. May not be a class member to avoid name mangling.
Pass the constant #PX_PHYSICS_VERSION as the argument.
There may be only one instance of this class per process. Calling this method after an instance
has been created already will result in an error message and NULL will be returned.

\param version Version number we are expecting(should be #PX_PHYSICS_VERSION)
\param foundation Foundation instance (see PxFoundation)
\param scale values used to determine default tolerances for objects at creation time
\param trackOutstandingAllocations true if you want to track memory allocations
			so a debugger connection partway through your physics simulation will get
			an accurate map of everything that has been allocated so far.  This could have a memory
			and performance impact on your simulation hence it defaults to off.
\param pvd When pvd points to a valid PxPvd instance (PhysX Visual Debugger), a connection to the specified PxPvd instance is created.
			If pvd is NULL no connection will be attempted.
\param omniPvd When omniPvd points to a valid PxOmniPvd instance PhysX will sample its internal structures to the defined OmniPvd output streams
			set in the PxOmniPvd object.
\return PxPhysics instance on success, NULL if operation failed

@see PxPhysics, PxFoundation, PxTolerancesScale, PxPvd
*/
PX_C_EXPORT PX_PHYSX_CORE_API physx::PxPhysics* PX_CALL_CONV PxCreateBasePhysics(	physx::PxU32 version,
																					physx::PxFoundation& foundation,
																					const physx::PxTolerancesScale& scale,
																					bool trackOutstandingAllocations = false,
																					physx::PxPvd* pvd = NULL,
																					physx::PxOmniPvd* omniPvd = NULL);

/**
\brief Creates an instance of the physics SDK.

Creates an instance of this class. May not be a class member to avoid name mangling.
Pass the constant #PX_PHYSICS_VERSION as the argument.
There may be only one instance of this class per process. Calling this method after an instance
has been created already will result in an error message and NULL will be returned.

Calling this will register all optional code modules (Articulations and HeightFields), preparing them for use.
If you do not need some of these modules, consider calling PxCreateBasePhysics() instead and registering needed
modules manually.

\param version Version number we are expecting(should be #PX_PHYSICS_VERSION)
\param foundation Foundation instance (see PxFoundation)
\param scale values used to determine default tolerances for objects at creation time
\param trackOutstandingAllocations true if you want to track memory allocations
			so a debugger connection partway through your physics simulation will get
			an accurate map of everything that has been allocated so far.  This could have a memory
			and performance impact on your simulation hence it defaults to off.
\param pvd When pvd points to a valid PxPvd instance (PhysX Visual Debugger), a connection to the specified PxPvd instance is created.
			If pvd is NULL no connection will be attempted.
\param omniPvd When omniPvd points to a valid PxOmniPvd instance PhysX will sample its internal structures to the defined OmniPvd output streams
			set in the PxOmniPvd object.
\return PxPhysics instance on success, NULL if operation failed

@see PxPhysics, PxCreateBasePhysics,  PxRegisterArticulationsReducedCoordinate, PxRegisterHeightFields
*/
PX_INLINE physx::PxPhysics* PxCreatePhysics(physx::PxU32 version,
											physx::PxFoundation& foundation,
											const physx::PxTolerancesScale& scale,
											bool trackOutstandingAllocations = false,
											physx::PxPvd* pvd = NULL,
											physx::PxOmniPvd* omniPvd = NULL)
{
	physx::PxPhysics* physics = PxCreateBasePhysics(version, foundation, scale, trackOutstandingAllocations, pvd, omniPvd);
	if (!physics)
		return NULL;

	PxRegisterArticulationsReducedCoordinate(*physics);
	PxRegisterHeightFields(*physics);

	return physics;
}


/**
\brief Retrieves the Physics SDK after it has been created.

Before using this function the user must call #PxCreatePhysics().

\note The behavior of this method is undefined if the Physics SDK instance has not been created already.
*/
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
#endif

PX_C_EXPORT PX_PHYSX_CORE_API physx::PxPhysics& PX_CALL_CONV PxGetPhysics();

#ifdef __clang__
#pragma clang diagnostic pop
#endif

/** @} */
#endif
