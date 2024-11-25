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

#ifndef PX_TETRAHEDRON_MESH_H
#define PX_TETRAHEDRON_MESH_H

#include "foundation/PxVec3.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxUserAllocated.h"
#include "common/PxPhysXCommonConfig.h"
#include "common/PxBase.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	struct PxTetrahedronMeshFlag
	{
		enum Enum
		{
			e16_BIT_INDICES = (1 << 1)	//!< The tetrahedron mesh has 16bits vertex indices
		};
	};

	/**
	\brief collection of set bits defined in PxTetrahedronMeshFlag.

	\see PxTetrahedronMeshFlag
	*/
	typedef PxFlags<PxTetrahedronMeshFlag::Enum, PxU8> PxTetrahedronMeshFlags;
	PX_FLAGS_OPERATORS(PxTetrahedronMeshFlag::Enum, PxU8)

	
	/**
	\brief A data container providing mass, rest pose and other information required for deformable simulation

	Stores properties of deformable volume like inverse mass per node, rest pose matrix per tetrahedral element etc.
	Mainly used internally to store runtime data.

	*/
	class PxDeformableVolumeAuxData : public PxRefCounted
	{
	public:
		/**
		\brief Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.

		\see PxPhysics.createTetrahedronMesh()
		*/
		virtual void					release() = 0;

		/**
		\brief Get the inverse mass of each vertex of the tetrahedron mesh.

		\return PxReal* A pointer to an array of inverse mass for each vertex of the tetrahedron mesh. Size: number of vertices * sizeof(PxReal).
		 */
		virtual PxReal*					getGridModelInvMass() = 0;

	protected:
		PX_INLINE						PxDeformableVolumeAuxData(PxType concreteType, PxBaseFlags baseFlags) : PxRefCounted(concreteType, baseFlags) {}
		PX_INLINE						PxDeformableVolumeAuxData(PxBaseFlags baseFlags) : PxRefCounted(baseFlags) {}
		virtual							~PxDeformableVolumeAuxData() {}

		virtual	bool					isKindOf(const char* name) const { PX_IS_KIND_OF(name, "PxDeformableVolumeAuxData", PxRefCounted); }
	};

	typedef PX_DEPRECATED PxDeformableVolumeAuxData PxSoftBodyAuxData;

	/**
	\brief A tetramedron mesh, also called a 'tetrahedron soup'.

	It is represented as an indexed tetrahedron list. There are no restrictions on the
	tetrahedron data.

	To avoid duplicating data when you have several instances of a particular
	mesh positioned differently, you do not use this class to represent a
	mesh object directly. Instead, you create an instance of this mesh via
	the PxTetrahedronMeshGeometry and PxShape classes.

	<h3>Creation</h3>

	To create an instance of this class call PxPhysics::createTetrahedronMesh(),
	and release() to delete it. This is only possible
	once you have released all of its PxShape instances.


	<h3>Visualizations:</h3>
	\li #PxVisualizationParameter::eCOLLISION_AABBS
	\li #PxVisualizationParameter::eCOLLISION_SHAPES
	\li #PxVisualizationParameter::eCOLLISION_AXES
	\li #PxVisualizationParameter::eCOLLISION_FNORMALS
	\li #PxVisualizationParameter::eCOLLISION_EDGES

	\see PxTetrahedronMeshDesc PxTetrahedronMeshGeometry PxShape PxPhysics.createTetrahedronMesh()
	*/
	class PxTetrahedronMesh : public PxRefCounted
	{
	public:
		/**
		\brief Returns the number of vertices.
		\return	number of vertices
		\see getVertices()
		*/
		virtual	PxU32				getNbVertices()									const = 0;

		/**
		\brief Returns the vertices
		\return	array of vertices
		\see getNbVertices()
		*/
		virtual	const PxVec3*			getVertices()									const = 0;


		/**
		\brief Returns the number of tetrahedrons.
		\return	number of tetrahedrons
		\see getTetrahedrons()
		*/
		virtual	PxU32					getNbTetrahedrons()								const = 0;

		/**
		\brief Returns the tetrahedron indices.

		The indices can be 16 or 32bit depending on the number of tetrahedrons in the mesh.
		Call getTetrahedronMeshFlags() to know if the indices are 16 or 32 bits.

		The number of indices is the number of tetrahedrons * 4.

		\return	array of tetrahedrons
		\see getNbTetrahedron() getTetrahedronMeshFlags() getTetrahedraRemap()
		*/
		virtual	const void*				getTetrahedrons()									const = 0;

		/**
		\brief Reads the PxTetrahedronMesh flags.

		See the list of flags #PxTetrahedronMeshFlags

		\return The values of the PxTetrahedronMesh flags.
		*/
		virtual	PxTetrahedronMeshFlags		getTetrahedronMeshFlags()							const = 0;


		/**
		\brief Returns the tetrahedra remapping table.

		The tetrahedra are internally sorted according to various criteria. Hence the internal tetrahedron order
		does not always match the original (user-defined) order. The remapping table helps finding the old
		indices knowing the new ones:

			remapTable[ internalTetrahedronIndex ] = originalTetrahedronIndex

		\return	the remapping table (or NULL if 'PxCookingParams::suppressTriangleMeshRemapTable' has been used)
		\see getNbTetrahedron() getTetrahedrons() PxCookingParams::suppressTriangleMeshRemapTable
		*/
		virtual	const PxU32*	getTetrahedraRemap()	const = 0;

		/**
		\brief Returns the local-space (vertex space) AABB from the tetrahedron mesh.

		\return	local-space bounds
		*/
		virtual	PxBounds3				getLocalBounds()							const = 0;

		/**
		\brief Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.

		\see PxPhysics.createTetrahedronMesh()
		*/
		virtual void					release() = 0;

	protected:
		PX_INLINE						PxTetrahedronMesh(PxType concreteType, PxBaseFlags baseFlags) : PxRefCounted(concreteType, baseFlags) {}
		PX_INLINE						PxTetrahedronMesh(PxBaseFlags baseFlags) : PxRefCounted(baseFlags) {}
		virtual							~PxTetrahedronMesh() {}

		virtual	bool					isKindOf(const char* name) const { PX_IS_KIND_OF(name, "PxTetrahedronMesh", PxRefCounted); }
	};
	
	/**
	\brief A deformable volume mesh, containing structures to store collision shape, simulation shape and deformation state

	The class bundles shapes and deformation state of a deformable volume that is simulated using FEM. The meshes used for 
	collision detection and for the FEM calculations are both tetrahedral meshes. While collision detection requires
	a mesh that matches the surface of the simulated body as exactly as possible, the simulation mesh has more freedom
	such that it can be optimized for tetrahedra without small angles and nodes that aren't shared by too many elements.

	<h3>Creation</h3>

	To create an instance of this class call PxPhysics::createDeformableVolumeMesh(),
	and release() to delete it. This is only possible
	once you have released all of its PxShape instances.

	*/
	class PxDeformableVolumeMesh : public PxRefCounted
	{
	public:
		/**
		\brief Const accecssor to the deformable volume's collision mesh.

		\see PxTetrahedronMesh
		*/
		virtual const PxTetrahedronMesh* getCollisionMesh() const = 0;
		
		/**
		\brief Accecssor to the deformable volume's collision mesh.

		\see PxTetrahedronMesh
		*/
		virtual PxTetrahedronMesh* getCollisionMesh() = 0;

		/**
		\brief Const accessor to the deformable volume's simulation mesh.

		\see PxTetrahedronMesh
		*/
		virtual const PxTetrahedronMesh* getSimulationMesh() const = 0;
		
		/**
		\brief Accecssor to the deformable volume's simulation mesh.

		\see PxTetrahedronMesh
		*/
		virtual PxTetrahedronMesh* getSimulationMesh() = 0;


		/**
		\brief Const accessor to the deformable volume's simulation state.

		\see PxDeformableVolumeAuxData
		*/
		virtual const PxDeformableVolumeAuxData* getDeformableVolumeAuxData() const = 0;

		/**
		\brief Deprecated
		\see getDeformableVolumeAuxData
		*/
		PX_DEPRECATED PX_FORCE_INLINE const PxDeformableVolumeAuxData* getSoftBodyAuxData() const
		{
			return getDeformableVolumeAuxData();
		}

		/**
		\brief Accessor to the deformable volume's auxilary data like mass and rest pose information

		\see PxDeformableVolumeAuxData
		*/
		virtual PxDeformableVolumeAuxData* getDeformableVolumeAuxData() = 0;

		/**
		\brief Deprecated
		\see getDeformableVolumeAuxData
		*/
		PX_DEPRECATED PX_FORCE_INLINE PxDeformableVolumeAuxData* getSoftBodyAuxData()
		{
			return getDeformableVolumeAuxData();
		}

		/**
		\brief Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.

		\see PxPhysics.createTetrahedronMesh()
		*/
		virtual void					release() = 0;


	protected:
		PX_INLINE						PxDeformableVolumeMesh(PxType concreteType, PxBaseFlags baseFlags) : PxRefCounted(concreteType, baseFlags) {}
		PX_INLINE						PxDeformableVolumeMesh(PxBaseFlags baseFlags) : PxRefCounted(baseFlags) {}
		virtual							~PxDeformableVolumeMesh() {}

		virtual	bool					isKindOf(const char* name) const { PX_IS_KIND_OF(name, "PxDeformableVolumeMesh", PxRefCounted); }
	};

	typedef PX_DEPRECATED PxDeformableVolumeMesh PxSoftBodyMesh;


	/**

	\brief Contains information about how to update the collision mesh's vertices given a deformed simulation tetmesh.

	\see PxTetrahedronMeshData
	*/
	class PxCollisionMeshMappingData : public PxUserAllocated
	{
	public:
		virtual void release() = 0;

		virtual ~PxCollisionMeshMappingData() {}
	};

	/**

	\brief Stores data to accelerate collision detection of a tetrahedral mesh

	\see PxTetrahedronMeshData
	*/
	class PxDeformableVolumeCollisionData : public PxUserAllocated
	{

	};

	typedef PX_DEPRECATED PxDeformableVolumeCollisionData PxSoftBodyCollisionData;

	/**

	\brief Contains raw geometry information describing the tetmesh's vertices and its elements (tetrahedra)

	\see PxTetrahedronMeshData
	*/
	class PxTetrahedronMeshData : public PxUserAllocated
	{

	};

	/**

	\brief Stores data to compute and store the state of a deformed tetrahedral mesh

	\see PxTetrahedronMeshData
	*/
	class PxDeformableVolumeSimulationData : public PxUserAllocated
	{

	};

	typedef PX_DEPRECATED PxDeformableVolumeSimulationData PxSoftBodySimulationData;

	/**

	\brief Conbines PxTetrahedronMeshData and PxDeformableVolumeCollisionData

	\see PxTetrahedronMeshData PxDeformableVolumeCollisionData
	*/
	class PxCollisionTetrahedronMeshData : public PxUserAllocated
	{
	public:
		virtual const PxTetrahedronMeshData* getMesh() const = 0;
		virtual PxTetrahedronMeshData* getMesh() = 0;
		virtual const PxDeformableVolumeCollisionData* getData() const = 0;
		virtual PxDeformableVolumeCollisionData* getData() = 0;
		virtual void release() = 0;

		virtual ~PxCollisionTetrahedronMeshData() {}
	};

	/**

	\brief Conbines PxTetrahedronMeshData and PxDeformableVolumeSimulationData

	\see PxTetrahedronMeshData PxDeformableVolumeSimulationData
	*/
	class PxSimulationTetrahedronMeshData : public PxUserAllocated
	{
	public:
		virtual PxTetrahedronMeshData* getMesh() = 0;
		virtual PxDeformableVolumeSimulationData* getData() = 0;
		virtual void release() = 0;

		virtual ~PxSimulationTetrahedronMeshData() {}
	};

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
