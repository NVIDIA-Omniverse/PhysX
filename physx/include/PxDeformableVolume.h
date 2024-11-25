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

#ifndef PX_DEFORMABLE_VOLUME_H
#define PX_DEFORMABLE_VOLUME_H

#include "PxDeformableBody.h"
#include "PxDeformableVolumeFlag.h"
#include "PxConeLimitedConstraint.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4435)
#endif

class PxCudaContextManager;
class PxTetrahedronMesh;
class PxDeformableVolumeAuxData;
class PxDeformableSurface;
class PxParticleBuffer;

/**
\brief The maximum number of tetrahedrons supported in a deformable volume tetrahedron mesh

The current limit is 1'048'575.
*/
#define PX_MAX_NB_DEFORMABLE_VOLUME_TET 0x000fffff

/**
\brief The maximum number of deformable volumes supported in a scene

The current limit is 4095.
*/
#define PX_MAX_NB_DEFORMABLE_VOLUME 0xfff

/**
\brief Represents a deformable volume

The deformable volume feature is exclusively supported on GPU. The full GPU pipeline needs to
be enabled in order to make use of deformable bodies, see #PxSceneFlag::eENABLE_GPU_DYNAMICS,
#PxBroadPhaseType::eGPU.
*/
class PxDeformableVolume : public PxDeformableBody
{
public:

	/**
	\brief Raises or clears a particular deformable volume flag.

	See the list of flags #PxDeformableVolumeFlag

	<b>Default:</b> No flags are set

	\param[in] flag The PxDeformableVolumeFlag to raise(set) or clear. See #PxDeformableVolumeFlag.
	\param[in] val  The new boolean value for the flag.
	*/
	virtual		void						setDeformableVolumeFlag(PxDeformableVolumeFlag::Enum flag, bool val) = 0;
	
	/**
	\brief Sets deformable volume flags.

	See the list of flags #PxDeformableVolumeFlag

	<b>Default:</b> No flags are set

	\param[in] flags The PxDeformableVolumeFlags to set.
	*/
	virtual		void						setDeformableVolumeFlags(PxDeformableVolumeFlags flags) = 0;

	/**
	\brief Reads the deformable volume flags.

	See the list of flags #PxDeformableVolumeFlag

	\return The values of the deformable volume flags.

	\see setDeformableVolumeFlag()
	*/
	virtual		PxDeformableVolumeFlags		getDeformableVolumeFlags() const = 0;

	/**
	\brief Sets the self collision stress tolerance.

	Stress threshold to deactivate collision contacts in case the local stress magnitude exceeds the threshold.
	<b>Default:</b> 0.9

	\param[in] selfCollisionStressTolerance The maximal depenetration velocity
	*/
	virtual		void						setSelfCollisionStressTolerance(const PxReal selfCollisionStressTolerance) = 0;

	/**
	\brief Retrieves the self collision stress tolerance.

	\return The self collision filter distance
	\see setSelfCollisionFilterDistance
	*/
	virtual		PxReal						getSelfCollisionStressTolerance() const = 0;

	/**
	\brief Gets a pointer to a device buffer containing positions and inverse masses of the 
	collision mesh.

	This function returns a pointer to device memory for the positions and inverse masses of
	the deformable volume. This buffer is used to both initialize/update the collision mesh vertices
	of the deformable volume and read the simulation results.

	\note It is mandatory to call PxDeformableVolume::markDirty() with PxDeformableVolumeDataFlag::ePOSITION_INVMASS
	when updating data in this buffer. 

	The simulation expects 4 consecutive floats for each vertex, aligned to a 16B boundary.
	The first 3 floats specify the vertex position and the last float contains the inverse mass of the
	vertex. The size of the buffer is the number of vertices of the collision mesh * sizeof(PxVec4).
	\see PxTetrahedronMesh::getNbVertices().

	The device memory pointed to by this pointer is allocated when a shape is attached to the 
	deformable volume. Calling PxDeformableVolume::detachShape() will deallocate the memory.

	It is not allowed to write to this buffer from the start of the PxScene::simulate() call
	until PxScene::fetchResults() returns. Reading the data is allowed once all the PhysX tasks
	have finished, reading the data during a completion task is explicitly allowed. The 
	simulation will read and write directly from/into this buffer.

	It is the users' responsibility to initialize this buffer with the initial positions of 
	the vertices of the collision mesh. See PxDeformableVolumeExt::allocateAndInitializeHostMirror(),
	PxDeformableVolumeExt::copyToDevice().
	
	\return PxVec4* A pointer to a device buffer containing positions and inverse masses of
	the collision mesh.
	*/
	virtual 	PxVec4*						getPositionInvMassBufferD() = 0;

	/**
	\brief Gets a pointer to a device buffer containing rest positions of the collision mesh vertices.

	This function returns a pointer to device memory for the rest positions of the deformable volume collision
	mesh. This buffer is used to initialize the rest positions of the collision mesh vertices.

	\note It is mandatory to call PxDeformableVolume::markDirty() with PxDeformableVolumeDataFlag::eREST_POSITION when
	updating data in this buffer.

	The simulation expects 4 floats per vertex, aligned to a 16B boundary. The first 3 specify the 
	rest position. The last float is unused. The size of the buffer is the number of vertices in 
	the collision mesh * sizeof(PxVec4). \see PxTetrahedronMesh::getNbVertices().

	The device memory pointed to by this pointer is allocated when a shape is attached to the deformable volume.
	Calling PxDeformableVolume::detachShape() will deallocate the memory.

	It is not allowed to write data into this buffer from the start of PxScene::simulate() until
	PxScene::fetchResults() returns.

	It is the users' responsibility to initialize this buffer with the initial rest positions of the 
	vertices of the collision mesh. See PxDeformableVolumeExt::allocateAndInitializeHostMirror(),
	PxDeformableVolumeExt::copyToDevice().
	
	\return PxVec4* A pointer to a device buffer containing the rest positions of the collision mesh. 
	 */
	virtual 	PxVec4*						getRestPositionBufferD() = 0;

	/**
	\brief Gets a pointer to a device buffer containing the vertex positions of the simulation mesh.

	This function returns a pointer to device memory for the positions and inverse masses of the deformable volume
	simulation mesh. This buffer is used to both initialize/update the simulation mesh vertices
	of the deformable volume and read the simulation results.

	\note It is mandatory to call PxDeformableVolume::markDirty() with PxDeformableVolumeDataFlag::eSIM_POSITION_INVMASS when
	updating data in this buffer.

	The simulation expects 4 consecutive floats for each vertex, aligned to a 16B boundary. The 
	first 3 floats specify the positions and the last float specifies the inverse mass of the vertex.
	The size of the buffer is the number of vertices of the simulation mesh * sizeof(PxVec4).
	\see PxTetrahedronMesh::getNbVertices().

	The device memory pointed to by this pointer is allocated when a simulation mesh is attached to the 
	deformable volume. Calling PxDeformableVolume::detachSimulationMesh() will deallocate the memory.

	It is not allowed to write to this buffer from the start of the PxScene::simulate() call
	until PxScene::fetchResults() returns. Reading the data is allowed once all the PhysX tasks
	have finished, reading the data during a completion task is explicitly allowed. The 
	simulation will read and write directly from/into this buffer.

	It is the users' responsibility to initialize this buffer with the initial positions of 
	the vertices of the simulation mesh. See PxDeformableVolumeExt::allocateAndInitializeHostMirror(),
	PxDeformableVolumeExt::copyToDevice().
	
	\return PxVec4* A pointer to a device buffer containing the vertex positions of the simulation mesh.
	*/
	virtual 	PxVec4*						getSimPositionInvMassBufferD() = 0;

	/**
	\brief Gets a pointer to a device buffer containing the vertex velocities of the simulation mesh.

	This function returns a pointer to device memory for the velocities of the deformable volume simulation mesh 
	vertices. This buffer is used to both initialize/update the simulation mesh vertex velocities
	of the deformable volume and read the simulation results.

	\note It is mandatory to call PxDeformableVolume::markDirty() with PxDeformableVolumeDataFlag::eSIM_VELOCITY when
	updating data in this buffer.

	The simulation expects 4 consecutive floats for each vertex, aligned to a 16B boundary. The 
	first 3 specify the velocities for each vertex. The final float is unused. The size of the 
	buffer is the number of vertices of the simulation mesh * sizeof(PxVec4).
	\see PxTetrahedronMesh::getNbVertices().

	The device memory pointed to by this pointer is allocated when a simulation mesh is attached to the 
	deformable volume. Calling PxDeformableVolume::detachSimulationMesh() will deallocate the memory.

	It is not allowed to write to this buffer from the start of the PxScene::simulate() call
	until PxScene::fetchResults() returns. Reading the data is allowed once all the PhysX tasks
	have finished, reading the data during a completion task is explicitly allowed. The 
	simulation will read and write directly from/into this buffer.

	It is the users' responsibility to initialize this buffer with the initial velocities of 
	the vertices of the simulation mesh. See PxDeformableVolumeExt::allocateAndInitializeHostMirror(),
	PxDeformableVolumeExt::copyToDevice().
	
	\return PxVec4*  A pointer to a device buffer containing the vertex velocities of the simulation mesh.
	*/
	virtual		PxVec4*						getSimVelocityBufferD() = 0;

	/**
	\brief Marks per-vertex simulation state and configuration buffers dirty to signal to the simulation
	that changes have been made.

	Calling this function is mandatory to notify the simulation of changes made in the positionInvMass,
	simPositionInvMass, simVelocity and rest position buffers.

	This function can be called multiple times, and dirty flags are accumulated internally until 
	PxScene::simulate() is called.

	\see getPositionInvMassBufferD, getSimVelocityBufferD, getRestPositionBufferD, getSimPositionInvMassBufferD
	
	\param flags The buffers that have been updated.
	*/
	virtual		void						markDirty(PxDeformableVolumeDataFlags flags) = 0;

	/**
	\brief Sets the device buffer containing the kinematic targets for this deformable volume.

	This function sets the kinematic targets for a deformable volume to a user-provided device buffer. This buffer is
	read by the simulation to obtain the target position for each vertex of the simulation mesh.

	The simulation expects 4 consecutive float for each vertex, aligned to a 16B boundary. The first 3
	floats specify the target positions. The last float determines (together with the flag argument)
	if the target is active or not.
	For a deformable volume with the flag PxDeformableBodyFlag::eKINEMATIC raised, all target positions are considered
	valid. In case a deformable volume has the PxDeformableVolumeFlag::ePARTIALLY_KINEMATIC raised, only target
	positions whose corresponding last float has been set to 0.f are considered valid target positions.
	\see PxConfigureDeformableVolumeKinematicTarget
	Setting the kinematic targets has no effect if neither PxDeformableBodyFlag::eKINEMATIC nor
	PxDeformableVolumeFlag::ePARTIALLY_KINEMATIC is set.

	The size of the buffer is the number of vertices of the simulation mesh * sizeof(PxVec4).
	\see PxTetrahedronMesh::getNbVertices().

	It is the users responsibility to manage the memory pointed to by the input to this function,
	as well as guaranteeing the integrity of the input data. In particular, this means that it is
	not allowed to write this data from from the start of PxScene::simulate() until PxScene::fetchResults()
	returns. The memory is not allowed to be deallocated until PxScene::fetchResults() returns.

	Calling this function with a null pointer for the positions will clear the input and resume normal
	simulation. PxDeformableBodyFlag::eKINEMATIC or PxDeformableVolumeFlag::ePARTIALLY_KINEMATIC are ignored
	if no targets are set.

	This call is persistent across calls to PxScene::simulate(). Once this function is called, the
	simulation will look up the target positions from the same buffer for every call to PxScene::simulate().
	The user is allowed to update the target positions without calling this function again, provided that
	the synchronization requirements are adhered to (no changes between start of PxScene::simulate() until
	PxScene::fetchResults() returns).

	\param positions A pointer to a device buffer containing the kinematic targets for this deformable volume.
	 */
	virtual		void						setKinematicTargetBufferD(const PxVec4* positions) = 0;

	/**
	\brief Deprecated: Sets the device buffer containing the kinematic targets for this deformable volume.

	This function sets the kinematic targets for a deformable volume to a user-provided device buffer. This buffer is
	read by the simulation to obtain the target position for each vertex of the simulation mesh.

	The simulation expects 4 consecutive float for each vertex, aligned to a 16B boundary. The first 3
	floats specify the target positions. The last float determines (together with the flag argument)
	if the target is active or not.
	For a deformable volume with the flag PxDeformableVolumeFlag::eKINEMATIC raised, all target positions are considered
	valid. In case a deformable volume has the PxDeformableVolumeFlag::ePARTIALLY_KINEMATIC raised, only target 
	positions whose corresponding last float has been set to 0.f are considered valid target positions.
	\see PxConfigureDeformableVolumeKinematicTarget

	The size of the buffer is the number of vertices of the simulation mesh * sizeof(PxVec4).
	\see PxTetrahedronMesh::getNbVertices().

	It is the users responsibility to manage the memory pointed to by the input to this function,
	as well as guaranteeing the integrity of the input data. In particular, this means that it is
	not allowed to write this data from from the start of PxScene::simulate() until PxScene::fetchResults()
	returns. The memory is not allowed to be deallocated until PxScene::fetchResults() returns.

	Calling this function with a null pointer for the positions will clear the input and resume normal
	simulation. This will also clear both the PxDeformableVolumeFlag::eKINEMATIC and PxDeformableVolumeFlag::ePARTIALLY_KINEMATIC
	flags of the deformable volume.

	This call is persistent across calls to PxScene::simulate(). Once this function is called, the 
	simulation will look up the target positions from the same buffer for every call to PxScene::simulate().
	The user is allowed to update the target positions without calling this function again, provided that
	the synchronization requirements are adhered to (no changes between start of PxScene::simulate() until 
	PxScene::fetchResults() returns).

	\param positions A pointer to a device buffer containing the kinematic targets for this deformable volume.
	\param flags Flags specifying the type of kinematic deformable volume: this function ignores all flags except PxDeformableVolumeFlag::eKINEMATIC and PxDeformableVolumeFlag::ePARTIALLY_KINEMATIC.
	 */
	PX_DEPRECATED virtual void				setKinematicTargetBufferD(const PxVec4* positions, PxDeformableVolumeFlags flags) = 0;

	/**
	\brief Attaches a simulation mesh

	Attaches the simulation mesh (geometry) and a state containing inverse mass, rest pose
	etc. required to compute the deformation.

	\param[in] simulationMesh The tetrahedral mesh used to compute the deformable's deformation
	\param[in] deformableVolumeAuxData A state that contain a mapping from simulation to collision mesh, volume information etc.

	\return Returns true if the operation was successful
	\see detachSimulationMesh, PxDeformableBody.attachShape
	*/
	virtual		bool						attachSimulationMesh(PxTetrahedronMesh& simulationMesh, PxDeformableVolumeAuxData& deformableVolumeAuxData) = 0;

	/**
	\brief Detaches the simulation mesh

	Detaches the simulation mesh and simulation state used to compute the deformation.

	\see attachSimulationMesh, PxDeformableBody.detachShape
	*/
	virtual		void						detachSimulationMesh() = 0;

	/**
	\brief Retrieves the simulation mesh pointer.

	Allows to access the geometry of the tetrahedral mesh used to compute the object's deformation

	\return Pointer to the simulation mesh
	*/
	virtual		PxTetrahedronMesh*			getSimulationMesh() = 0;

	//! \brief Const version of getSimulationMesh()
	virtual		const PxTetrahedronMesh*	getSimulationMesh() const = 0;

	/**
	\brief Retrieve the collision mesh pointer.

	Allows to access the geometry of the tetrahedral mesh used to perform collision detection

	\return Pointer to the collision mesh
	*/
	virtual		PxTetrahedronMesh*			getCollisionMesh() = 0;

	//! \brief Const version of getCollisionMesh()
	virtual		const PxTetrahedronMesh*	getCollisionMesh() const = 0;

	/**
	\brief Retrieves the simulation state pointer.

	Allows to access the additional data of the simulation mesh (inverse mass, rest state etc.).
	The geometry part of the data is stored in the simulation mesh.

	\return Pointer to the simulation state
	*/
	virtual		PxDeformableVolumeAuxData*	getDeformableVolumeAuxData() = 0;

	//! \brief const version of getDeformableVolumeAuxData()
	virtual		const PxDeformableVolumeAuxData*
											getDeformableVolumeAuxData() const = 0;

	/**
	\brief Returns the GPU deformable volume index.

	\return The GPU index, or 0xFFFFFFFF if the deformable volume is not in a scene.
	*/
	virtual		PxU32						getGpuDeformableVolumeIndex() = 0;

	/**
	\brief Gets the concrete type name.
	\return The name of the concrete type.
	*/
	virtual		const char*					getConcreteTypeName() const PX_OVERRIDE	PX_FINAL { return "PxDeformableVolume"; }

	/**
	\brief Deprecated
	\see setDeformableVolumeFlag
	*/
	PX_DEPRECATED PX_FORCE_INLINE void setSoftBodyFlag(PxDeformableVolumeFlag::Enum flag, bool val) { return setDeformableVolumeFlag(flag, val); }

	/**
	\brief Deprecated
	\see setDeformableVolumeFlags
	*/
	PX_DEPRECATED PX_FORCE_INLINE void setSoftBodyFlags(PxDeformableVolumeFlags flags) { return setDeformableVolumeFlags(flags); }

	/**
	\brief Deprecated
	\see getDeformableVolumeFlags
	*/
	PX_DEPRECATED PX_FORCE_INLINE PxDeformableVolumeFlags getSoftBodyFlag() const { return getDeformableVolumeFlags(); }

	/**
	\brief Deprecated
	\see getDeformableVolumeAuxData
	*/
	PX_DEPRECATED PX_FORCE_INLINE PxDeformableVolumeAuxData* getSoftBodyAuxData() { return getDeformableVolumeAuxData(); }

	/**
	\brief Deprecated
	\see getDeformableVolumeAuxData
	*/
	PX_DEPRECATED PX_FORCE_INLINE const PxDeformableVolumeAuxData* getSoftBodyAuxData() const { return getDeformableVolumeAuxData(); }

	/**
	\brief Deprecated
	<b>Default:</b> 4 position iterations, 1 velocity iteration
	\param[in] minPositionIters Minimal number of position iterations the solver should perform for this body. <b>Range:</b> [1,255]
	\param[in] minVelocityIters Minimal number of velocity iterations the solver should perform for this body. <b>Range:</b> [1,255]
	\see PxDeformableBody.setSolverIterationCounts()
	*/
	PX_DEPRECATED virtual		void	setSolverIterationCounts(PxU32 minPositionIters, PxU32 minVelocityIters = 1) = 0;

	/**
	\brief Deprecated
	\see PxDeformableBody.setSolverIterationCount()
	*/
	PX_DEPRECATED virtual		void	getSolverIterationCounts(PxU32& minPositionIters, PxU32& minVelocityIters) const = 0;

	/**
	\brief Creates a collision filter between a particle and a tetrahedron in the deformable volume's collision mesh.

	\param[in] particlesystem The particle system used for the collision filter
	\param[in] buffer The PxParticleBuffer to which the particle belongs to.
	\param[in] particleId The particle whose collisions with the tetrahedron in the deformable volume are filtered.
	\param[in] tetId The tetradedron in the deformable volume that is filtered. If tetId is PX_MAX_NB_DEFORMABLE_VOLUME_TET, this particle will filter against all tetrahedra in this deformable volume.
	*/
	PX_DEPRECATED	virtual		void	addParticleFilter(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId) = 0;
	
	/**
	\brief Removes a collision filter between a particle and a tetrahedron in the deformable volume's collision mesh.

	\param[in] particlesystem The particle system used for the collision filter
	\param[in] buffer The PxParticleBuffer to which the particle belongs to.
	\param[in] particleId The particle whose collisions with the tetrahedron in the deformable volume are filtered.
	\param[in] tetId The tetrahedron in the deformable volume is filtered.
	*/
	PX_DEPRECATED	virtual		void	removeParticleFilter(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId) = 0;
	
	/**
	\brief Creates an attachment between a particle and a deformable volume.
	Be aware that destroying the particle system before destroying the attachment is illegal and may cause a crash.
	The deformable volume keeps track of these attachments but the particle system does not.

	\param[in] particlesystem The particle system used for the attachment
	\param[in] buffer The PxParticleBuffer to which the particle belongs to.
	\param[in] particleId The particle that is attached to a tetrahedron in the deformable volume's collision mesh.
	\param[in] tetId The tetrahedron in the deformable volume's collision mesh to attach the particle to.
	\param[in] barycentric The barycentric coordinates of the particle attachment position with respect to the tetrahedron specified with tetId.
	\return Returns a handle that identifies the attachment created. This handle can be used to release the attachment later
	*/
	PX_DEPRECATED	virtual		PxU32	addParticleAttachment(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId, const PxVec4& barycentric) = 0;
	
	
	/**
	\brief Removes an attachment between a particle and a deformable volume.
	Be aware that destroying the particle system before destroying the attachment is illegal and may cause a crash.
	The deformable volume keeps track of these attachments but the particle system does not.
	
	\param[in] particlesystem The particle system used for the attachment
	\param[in] handle Index that identifies the attachment. This handle gets returned by the addParticleAttachment when the attachment is created
	*/
	PX_DEPRECATED	virtual		void	removeParticleAttachment(PxPBDParticleSystem* particlesystem, PxU32 handle) = 0;

	/**
	\brief Creates a collision filter between a vertex in a deformable volume and a rigid body.

	\param[in] actor The rigid body actor used for the collision filter
	\param[in] vertId The index of a vertex in the deformable volume's collision mesh whose collisions with the rigid body are filtered.
	*/
	PX_DEPRECATED	virtual		void	addRigidFilter(PxRigidActor* actor, PxU32 vertId) = 0;

	/**
	\brief Removes a collision filter between a vertex in a deformable volume and a rigid body.

	\param[in] actor The rigid body actor used for the collision filter
	\param[in] vertId The index of a vertex in the deformable volume's collision mesh whose collisions with the rigid body are filtered.
	*/
	PX_DEPRECATED	virtual		void	removeRigidFilter(PxRigidActor* actor, PxU32 vertId) = 0;

	/**
	\brief Creates a rigid attachment between a deformable volume and a rigid body.
	Be aware that destroying the rigid body before destroying the attachment is illegal and may cause a crash.
	The deformable volume keeps track of these attachments but the rigid body does not.

	This method attaches a vertex on the deformable volume collision mesh to the rigid body.

	\param[in] actor The rigid body actor used for the attachment
	\param[in] vertId The index of a vertex in the deformable volume's collision mesh that gets attached to the rigid body.
	\param[in] actorSpacePose The location of the attachment point expressed in the rigid body's coordinate system.
	\param[in] constraint The user defined cone distance limit constraint to limit the movement between a vertex in the deformable volume and rigid body.
	\return Returns a handle that identifies the attachment created. This handle can be used to relese the attachment later
	*/
	PX_DEPRECATED	virtual		PxU32	addRigidAttachment(PxRigidActor* actor, PxU32 vertId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint = NULL) = 0;

	/**
	\brief Releases a rigid attachment between a deformable volume and a rigid body.
	Be aware that destroying the rigid body before destroying the attachment is illegal and may cause a crash.
	The deformable volume keeps track of these attachments but the rigid body does not.

	This method removes a previously-created attachment between a vertex of the deformable volume collision mesh and the rigid body.

	\param[in] actor The rigid body actor used for the attachment
	\param[in] handle Index that identifies the attachment. This handle gets returned by the addRigidAttachment when the attachment is created
	*/
	PX_DEPRECATED	virtual		void	removeRigidAttachment(PxRigidActor* actor, PxU32 handle) = 0;

	/**
	\brief Creates collision filter between a tetrahedron in a deformable volume and a rigid body.

	\param[in] actor The rigid body actor used for collision filter
	\param[in] tetIdx The index of a tetrahedron in the deformable volume's collision mesh whose collisions with the rigid body is filtered.
	*/
	PX_DEPRECATED	virtual		void	addTetRigidFilter(PxRigidActor* actor, PxU32 tetIdx) = 0;
	
	/**
	\brief Removes collision filter between a tetrahedron in a deformable volume and a rigid body.

	\param[in] actor The rigid body actor used for collision filter
	\param[in] tetIdx The index of a tetrahedron in the deformable volume's collision mesh whose collisions with the rigid body is filtered.
	*/
	PX_DEPRECATED	virtual		void	removeTetRigidFilter(PxRigidActor* actor, PxU32 tetIdx) = 0;

	/**
	\brief Creates a rigid attachment between a deformable volume and a rigid body.
	Be aware that destroying the rigid body before destroying the attachment is illegal and may cause a crash.
	The deformable volume keeps track of these attachments but the rigid body does not.

	This method attaches a point inside a tetrahedron of the collision to the rigid body.

	\param[in] actor The rigid body actor used for the attachment
	\param[in] tetIdx The index of a tetrahedron in the deformable volume's collision mesh that contains the point to be attached to the rigid body
	\param[in] barycentric The barycentric coordinates of the attachment point inside the tetrahedron specified by tetIdx
	\param[in] actorSpacePose The location of the attachment point expressed in the rigid body's coordinate system.
	\param[in] constraint The user defined cone distance limit constraint to limit the movement between a tet and rigid body.
	\return Returns a handle that identifies the attachment created. This handle can be used to release the attachment later
	*/
	PX_DEPRECATED	virtual		PxU32	addTetRigidAttachment(PxRigidActor* actor, PxU32 tetIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint = NULL) = 0;

	/**
	\brief Creates collision filter between a tetrahedron in a deformable volume and a tetrahedron in another deformable volume.

	\param[in] otherDeformableVolume The other deformable volume actor used for collision filter
	\param[in] otherTetIdx The index of the tetrahedron in the other deformable volume's collision mesh to be filtered.
	\param[in] tetIdx1 The index of the tetrahedron in the deformable volume's collision mesh to be filtered. If tetId1 is PX_MAX_NB_DEFORMABLE_VOLUME_TET, the tetrahedron with index `otherTetIdx' in the other deformable volume will filter against all tetrahedra in this deformable volume.
	*/
	PX_DEPRECATED	virtual		void	addSoftBodyFilter(PxDeformableVolume* otherDeformableVolume, PxU32 otherTetIdx, PxU32 tetIdx1) = 0;

	/**
	\brief Removes collision filter between a tetrahedron in a deformable volume and a tetrahedron in other deformable volume.

	\param[in] otherDeformableVolume The other deformable volume actor used for collision filter
	\param[in] otherTetIdx The index of the other tetrahedron in the other deformable volume's collision mesh whose collision with the tetrahedron with the deformable volume is filtered.
	\param[in] tetIdx1 The index of the tetrahedron in the deformable volume's collision mesh whose collision with the other tetrahedron with the other deformable volume is filtered.
	*/
	PX_DEPRECATED	virtual		void	removeSoftBodyFilter(PxDeformableVolume* otherDeformableVolume, PxU32 otherTetIdx, PxU32 tetIdx1) = 0;

	/**
	\brief Creates collision filters between a tetrahedron in a deformable volume with another deformable volume.

	\param[in] otherDeformableVolume The other deformable volume actor used for collision filter
	\param[in] otherTetIndices The indices of the tetrahedron in the other deformable volume's collision mesh to be filtered.
	\param[in] tetIndices The indices of the tetrahedron of the deformable volume's collision mesh to be filtered.
	\param[in] tetIndicesSize The size of tetIndices.
	*/
	PX_DEPRECATED	virtual		void	addSoftBodyFilters(PxDeformableVolume* otherDeformableVolume, PxU32* otherTetIndices, PxU32* tetIndices, PxU32 tetIndicesSize) = 0;

	/**
	\brief Removes collision filters between a tetrahedron in a deformable volume with another deformable volume.

	\param[in] otherDeformableVolume The other deformable volume actor used for collision filter
	\param[in] otherTetIndices The indices of the tetrahedron in the other deformable volume's collision mesh to be filtered.
	\param[in] tetIndices The indices of the tetrahedron of the deformable volume's collision mesh to be filtered.
	\param[in] tetIndicesSize The size of tetIndices.
	*/
	PX_DEPRECATED	virtual		void	removeSoftBodyFilters(PxDeformableVolume* otherDeformableVolume, PxU32* otherTetIndices, PxU32* tetIndices, PxU32 tetIndicesSize) = 0;

	/**
	\brief Creates an attachment between two deformable volumes.

	This method attaches a point inside a tetrahedron of the collision mesh to a point in another deformable volume's tetrahedron collision mesh.

	\param[in] deformableVolume0 The deformable volume actor used for the attachment
	\param[in] tetIdx0 The index of a tetrahedron in the other deformable volume that contains the point to be attached to the deformable volume
	\param[in] tetBarycentric0 The barycentric coordinates of the attachment point inside the tetrahedron specified by tetIdx0
	\param[in] tetIdx1 The index of a tetrahedron in the deformable volume's collision mesh that contains the point to be attached to the deformableVolume0
	\param[in] tetBarycentric1 The barycentric coordinates of the attachment point inside the tetrahedron specified by tetIdx1
	\param[in] constraint The user defined cone distance limit constraint to limit the movement between tets.
	\param[in] constraintOffset Offsets the cone and distance limit constraint along its axis, in order to specify the location of the cone tip.
	\return Returns a handle that identifies the attachment created. This handle can be used to release the attachment later
	*/
	PX_DEPRECATED	virtual		PxU32	addSoftBodyAttachment(PxDeformableVolume* deformableVolume0, PxU32 tetIdx0, const PxVec4& tetBarycentric0, PxU32 tetIdx1, const PxVec4& tetBarycentric1,
										PxConeLimitedConstraint* constraint = NULL, PxReal constraintOffset = 0.0f) = 0;

	/**
	\brief Releases an attachment between a deformable volume and the other deformable volume.
	Be aware that destroying the deformable volume before destroying the attachment is illegal and may cause a crash.

	This method removes a previously-created attachment between a point inside a tetrahedron of the collision mesh to a point in another deformable volume's tetrahedron collision mesh.

	\param[in] deformableVolume0 The deformable volume actor used for the attachment.
	\param[in] handle Index that identifies the attachment. This handle gets returned by the addSoftBodyAttachment when the attachment is created.
	*/
	PX_DEPRECATED	virtual		void	removeSoftBodyAttachment(PxDeformableVolume* deformableVolume0, PxU32 handle) = 0;

	/**
	\brief Deprecated
	\see getGpuDeformableVolumeIndex
	*/
	PX_DEPRECATED PX_FORCE_INLINE PxU32 getGpuSoftBodyIndex() { return getGpuDeformableVolumeIndex(); }

protected:
	PX_INLINE					PxDeformableVolume(PxType concreteType, PxBaseFlags baseFlags) : PxDeformableBody(concreteType, baseFlags) {}
	PX_INLINE					PxDeformableVolume(PxBaseFlags baseFlags) : PxDeformableBody(baseFlags) {}
	virtual						~PxDeformableVolume() {}
	virtual		bool			isKindOf(const char* name) const PX_OVERRIDE { PX_IS_KIND_OF(name, "PxDeformableVolume", PxDeformableBody); }
};

/**
\brief Adjusts a deformable volume kinematic target such that it is properly set as active or inactive. Inactive targets will not affect vertex position, they are ignored by the solver.

\param[in] target The kinematic target
\param[in] isActive A boolean indicating if the returned target should be marked as active or not
\return The target with adjusted w component
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec4 PxConfigureDeformableVolumeKinematicTarget(const PxVec4& target, bool isActive)
{
	PxVec4 result = target;
	if (isActive)
		result.w = 0.0f;
	else
	{
		//Any non-zero value will mark the target as inactive
		if (result.w == 0.0f)
			result.w = 1.0f;
	}
	return result;
}

/**
\brief Sets up a deformable volume kinematic target such that it is properly set as active or inactive. Inactive targets will not affect vertex position, they are ignored by the solver.

\param[in] target The kinematic target
\param[in] isActive A boolean indicating if the returned target should be marked as active or not
\return The target with configured w component
*/
PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec4 PxConfigureDeformableVolumeKinematicTarget(const PxVec3& target, bool isActive)
{
	return PxConfigureDeformableVolumeKinematicTarget(PxVec4(target, 0.0f), isActive);
}

#if PX_VC
#pragma warning(pop)
#endif


#if !PX_DOXYGEN
} // namespace physx
#endif

#endif // PX_DEFORMABLE_VOLUME_H
