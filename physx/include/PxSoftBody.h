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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PX_SOFT_BODY_H
#define PX_SOFT_BODY_H
/** \addtogroup physics
@{ */

#include "PxFEMParameter.h"
#include "PxActor.h"
#include "PxConeLimitedConstraint.h"
#include "PxSoftBodyFlag.h"

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
	class PxSoftBodyAuxData;
	class PxFEMCloth;
	class PxParticleBuffer;

	/**
	\brief The maximum tetrahedron index supported in the model.
	*/
	#define	PX_MAX_TETID	0x000fffff

	/**
	\brief Flags to enable or disable special modes of a SoftBody
	*/
	struct PxSoftBodyFlag
	{
		enum Enum
		{
			eDISABLE_SELF_COLLISION = 1 << 0,		//!< Determines if self collision will be detected and resolved
			eCOMPUTE_STRESS_TENSOR = 1 << 1,		//!< Enables computation of a Cauchy stress tensor for every tetrahedron in the simulation mesh. The tensors can be accessed through the softbody direct API
			eENABLE_CCD = 1 << 2,					//!< Enables support for continuous collision detection
			eDISPLAY_SIM_MESH = 1 << 3,				//!< Enable debug rendering to display the simulation mesh
			eKINEMATIC = 1 << 4,					//!< Enables support for kinematic motion of the collision and simulation mesh.
			ePARTIALLY_KINEMATIC = 1 << 5			//!< Enables partially kinematic motion of the collision and simulation mesh.
		};
	};

	typedef PxFlags<PxSoftBodyFlag::Enum, PxU32> PxSoftBodyFlags;

	/**
	\brief Represents a FEM softbody including everything to calculate its definition like geometry and material properties
	*/
	class PxSoftBody : public PxActor
	{
	public:

		virtual							~PxSoftBody() {}

		/**
		\brief Set a single softbody flag
		
		\param[in] flag The flag to set or clear
		\param[in] val The new state of the flag
		*/
		virtual		void				setSoftBodyFlag(PxSoftBodyFlag::Enum flag, bool val) = 0;
		
		/**
		\brief Set the softbody flags

		\param[in] flags The new softbody flags
		*/
		virtual		void				setSoftBodyFlags(PxSoftBodyFlags flags) = 0;

		/**
		\brief Get the softbody flags

		\return The softbody flags
		*/
		virtual		PxSoftBodyFlags		getSoftBodyFlag() const = 0;

		/**
		\brief Set parameter for FEM internal solve

		\param[in] parameters The FEM parameters
		*/
		virtual 	void				setParameter(PxFEMParameters parameters) = 0;

		/**
		\brief Get parameter for FEM internal solve

		\return The FEM parameters
		*/
		virtual PxFEMParameters			getParameter() const = 0;

		/**
		\brief Get a pointer to a device buffer containing positions and inverse masses of the 
		collision mesh.

		This function returns a pointer to device memory for the positions and inverse masses of
		the soft body. This buffer is used to both initialize/update the collision mesh vertices
		of the soft body and read the simulation results.

		\note It is mandatory to call PxSoftBody::markDirty() with PxSoftBodyDataFlag::ePOSITION_INVMASS
		when updating data in this buffer. 

		The simulation expects 4 consecutive floats for each vertex, aligned to a 16B boundary.
		The first 3 floats specify the vertex position and the last float contains the inverse mass of the
		vertex. The size of the buffer is the number of vertices of the collision mesh * sizeof(PxVec4).
		@see PxTetrahedronMesh::getNbVertices().

		The device memory pointed to by this pointer is allocated when a shape is attached to the 
		softbody. Calling PxSoftBody::detachShape() will deallocate the memory.

		It is not allowed to write to this buffer from the start of the PxScene::simulate() call
		until PxScene::fetchResults() returns. Reading the data is allowed once all the PhysX tasks
		have finished, reading the data during a completion task is explicitly allowed. The 
		simulation will read and write directly from/into this buffer.

		It is the users' responsibility to initialize this buffer with the initial positions of 
		the vertices of the collision mesh. See PxSoftBodyExt::allocateAndInitializeHostMirror(),
		PxSoftBodyExt::copyToDevice().
		
		\return PxVec4* A pointer to a device buffer containing positions and inverse masses of
		the collision mesh.
		*/
		virtual 	PxVec4*	            getPositionInvMassBufferD() = 0;

		/**
		\brief Get a pointer to a device buffer containing rest positions of the collision mesh vertices.

		This function returns a pointer to device memory for the rest positions of the softbody collision
		mesh. This buffer is used to initialize the rest positions of the collision mesh vertices.

		\note It is mandatory to call PxSoftBody::markDirty() with PxSoftBodyDataFlag::eREST_POSITION when
		updating data in this buffer.

		The simulation expects 4 floats per vertex, aligned to a 16B boundary. The first 3 specify the 
		rest position. The last float is unused. The size of the buffer is the number of vertices in 
		the collision mesh * sizeof(PxVec4). @see PxTetrahedronMesh::getNbVertices().

		The device memory pointed to by this pointer is allocated when a shape is attached to the softbody.
		Calling PxSoftBody::detachShape() will deallocate the memory.

		It is not allowed to write data into this buffer from the start of PxScene::simulate() until
		PxScene::fetchResults() returns.

		It is the users' responsibility to initialize this buffer with the initial rest positions of the 
		vertices of the collision mesh. See PxSoftBodyExt::allocateAndInitializeHostMirror(),
		PxSoftBodyExt::copyToDevice().
		
		\return PxVec4* A pointer to a device buffer containing the rest positions of the collision mesh. 
		 */
		virtual 	PxVec4*	            getRestPositionBufferD() = 0;

		/**
		\brief Get a pointer to a device buffer containing the vertex positions of the simulation mesh.

		This function returns a pointer to device memory for the positions and inverse masses of the softbody
		simulation mesh. This buffer is used to both initialize/update the simulation mesh vertices
		of the softbody and read the simulation results.

		\note It is mandatory to call PxSoftBody::markDirty() with PxSoftBodyDataFlag::eSIM_POSITION_INVMASS when
		updating data in this buffer.

		The simulation expects 4 consecutive floats for each vertex, aligned to a 16B boundary. The 
		first 3 floats specify the positions and the last float specifies the inverse mass of the vertex.
		The size of the buffer is the number of vertices of the simulation mesh * sizeof(PxVec4).
		@see PxTetrahedronMesh::getNbVertices().

		The device memory pointed to by this pointer is allocated when a simulation mesh is attached to the 
		softbody. Calling PxSoftBody::detachSimulationMesh() will deallocate the memory.

		It is not allowed to write to this buffer from the start of the PxScene::simulate() call
		until PxScene::fetchResults() returns. Reading the data is allowed once all the PhysX tasks
		have finished, reading the data during a completion task is explicitly allowed. The 
		simulation will read and write directly from/into this buffer.

		It is the users' responsibility to initialize this buffer with the initial positions of 
		the vertices of the simulation mesh. See PxSoftBodyExt::allocateAndInitializeHostMirror(),
		PxSoftBodyExt::copyToDevice().
		
		\return PxVec4* A pointer to a device buffer containing the vertex positions of the simulation mesh.
		*/
		virtual 	PxVec4*	            getSimPositionInvMassBufferD() = 0;

		/**
		\brief Get a pointer to a device buffer containing the vertex velocities of the simulation mesh.

		This function returns a pointer to device memory for the velocities of the softbody simulation mesh 
		vertices. This buffer is used to both initialize/update the simulation mesh vertex velocities
		of the soft body and read the simulation results.

		\note It is mandatory to call PxSoftBody::markDirty() with PxSoftBodyDataFlag::eSIM_VELOCITY when
		updating data in this buffer.

		The simulation expects 4 consecutive floats for each vertex, aligned to a 16B boundary. The 
		first 3 specify the velocities for each vertex. The final float is unused. The size of the 
		buffer is the number of vertices of the simulation mesh * sizeof(PxVec4).
		@see PxTetrahedronMesh::getNbVertices().

		The device memory pointed to by this pointer is allocated when a simulation mesh is attached to the 
		softbody. Calling PxSoftBody::detachSimulationMesh() will deallocate the memory.

		It is not allowed to write to this buffer from the start of the PxScene::simulate() call
		until PxScene::fetchResults() returns. Reading the data is allowed once all the PhysX tasks
		have finished, reading the data during a completion task is explicitly allowed. The 
		simulation will read and write directly from/into this buffer.

		It is the users' responsibility to initialize this buffer with the initial velocities of 
		the vertices of the simulation mesh. See PxSoftBodyExt::allocateAndInitializeHostMirror(),
		PxSoftBodyExt::copyToDevice().
		
		\return PxVec4*  A pointer to a device buffer containing the vertex velocities of the simulation mesh.
		*/
		virtual 	PxVec4*	            getSimVelocityBufferD() = 0;

		/**
		\brief Marks per-vertex simulation state and configuration buffers dirty to signal to the simulation
		that changes have been made.

		Calling this function is mandatory to notify the simulation of changes made in the positionInvMass,
		simPositionInvMass, simVelocity and rest position buffers.

		This function can be called multiple times, and dirty flags are accumulated internally until 
		PxScene::simulate() is called.

		@see getPositionInvMassBufferD, getSimVelocityBufferD, getRestPositionBufferD, getSimPositionInvMassBufferD
		
		\param flags The buffers that have been updated.
		*/
		virtual     void                markDirty(PxSoftBodyDataFlags flags) = 0;

		/**
		\brief Set the device buffer containing the kinematic targets for this softbody.

		This function sets the kinematic targets for a softbody to a user-provided device buffer. This buffer is
		read by the simulation to obtain the target position for each vertex of the simulation mesh.

		The simulation expects 4 consecutive float for each vertex, aligned to a 16B boundary. The first 3
		floats specify the target positions. The last float determines (together with the flag argument)
		if the target is active or not.
		For a softbody with the flag PxSoftBodyFlag::eKINEMATIC raised, all target positions are considered
		valid. In case a softbody has the PxSoftBodyFlag::ePARTIALLY_KINEMATIC raised, only target 
		positions whose corresponding last float has been set to 0.f are considered valid target positions.
		@see PxConfigureSoftBodyKinematicTarget

		The size of the buffer is the number of vertices of the simulation mesh * sizeof(PxVec4).
		@see PxTetrahedronMesh::getNbVertices().

		It is the users responsibility to manage the memory pointed to by the input to this function,
		as well as guaranteeing the integrity of the input data. In particular, this means that it is
		not allowed to write this data from from the start of PxScene::simulate() until PxScene::fetchResults()
		returns. The memory is not allowed to be deallocated until PxScene::fetchResults() returns.

		Calling this function with a null pointer for the positions will clear the input and resume normal
		simulation. This will also clear both the PxSoftBodyFlag::eKINEMATIC and PxSoftBodyFlag::ePARTIALLY_KINEMATIC
		flags of the softbody.

		This call is persistent across calls to PxScene::simulate(). Once this function is called, the 
		simulation will look up the target positions from the same buffer for every call to PxScene::simulate().
		The user is allowed to update the target positions without calling this function again, provided that
		the synchronization requirements are adhered to (no changes between start of PxScene::simulate() until 
		PxScene::fetchResults() returns).

		\param positions A pointer to a device buffer containing the kinematic targets for this softbody.
		\param flags Flags specifying the type of kinematic softbody: this function ignores all flags except PxSoftBodyFlag::eKINEMATIC and PxSoftBodyFlag::ePARTIALLY_KINEMATIC.
		 */
		virtual 	void                    setKinematicTargetBufferD(const PxVec4* positions, PxSoftBodyFlags flags) = 0;

		/**
		\brief Return the cuda context manager

		\return The cuda context manager
		*/
		virtual		PxCudaContextManager*	getCudaContextManager() const = 0;

		/**
		\brief Sets the wake counter for the soft body.

		The wake counter value determines the minimum amount of time until the soft body can be put to sleep. Please note
		that a soft body will not be put to sleep if any vertex velocity is above the specified threshold
		or if other awake objects are touching it.

		\note Passing in a positive value will wake the soft body up automatically.

		<b>Default:</b> 0.4 (which corresponds to 20 frames for a time step of 0.02)

		\param[in] wakeCounterValue Wake counter value. <b>Range:</b> [0, PX_MAX_F32)

		@see isSleeping() getWakeCounter()
		*/
		virtual		void				setWakeCounter(PxReal wakeCounterValue) = 0;

		/**
		\brief Returns the wake counter of the soft body.

		\return The wake counter of the soft body.

		@see isSleeping() setWakeCounter()
		*/
		virtual		PxReal				getWakeCounter() const = 0;

		/**
		\brief Returns true if this soft body is sleeping.

		When an actor does not move for a period of time, it is no longer simulated in order to save time. This state
		is called sleeping. However, because the object automatically wakes up when it is either touched by an awake object,
		or a sleep-affecting property is changed by the user, the entire sleep mechanism should be transparent to the user.

		A soft body can only go to sleep if all vertices are ready for sleeping. A soft body is guaranteed to be awake
		if at least one of the following holds:

		\li The wake counter is positive (@see setWakeCounter()).
		\li The velocity of any vertex is above the sleep threshold.

		If a soft body is sleeping, the following state is guaranteed:

		\li The wake counter is zero.
		\li The linear velocity of all vertices is zero.

		When a soft body gets inserted into a scene, it will be considered asleep if all the points above hold, else it will
		be treated as awake.

		\note It is invalid to use this method if the soft body has not been added to a scene already.

		\return True if the soft body is sleeping.

		@see isSleeping()
		*/
		virtual		bool				isSleeping() const = 0;

		/**
		\brief Sets the solver iteration counts for the body.

		The solver iteration count determines how accurately deformation and contacts are resolved.
		If you are having trouble with softbodies that are not as stiff as they should be, then
		setting a higher position iteration count may improve the behavior.

		If intersecting bodies are being depenetrated too violently, increase the number of velocity
		iterations.

		<b>Default:</b> 4 position iterations, 1 velocity iteration

		\param[in] minPositionIters Minimal number of position iterations the solver should perform for this body. <b>Range:</b> [1,255]
		\param[in] minVelocityIters Minimal number of velocity iterations the solver should perform for this body. <b>Range:</b> [1,255]

		@see getSolverIterationCounts()
		*/
		virtual		void				setSolverIterationCounts(PxU32 minPositionIters, PxU32 minVelocityIters = 1) = 0;

		/**
		\brief Retrieves the solver iteration counts.

		@see setSolverIterationCounts()
		*/
		virtual		void				getSolverIterationCounts(PxU32& minPositionIters, PxU32& minVelocityIters) const = 0;


		/**
		\brief Retrieves the shape pointer belonging to the actor.

		\return Pointer to the collision mesh's shape
		@see PxShape getNbShapes() PxShape::release()
		*/
		virtual		PxShape*			getShape() = 0;


		/**
		\brief Retrieve the collision mesh pointer.

		Allows to access the geometry of the tetrahedral mesh used to perform collision detection

		\return Pointer to the collision mesh
		*/
		virtual PxTetrahedronMesh*		getCollisionMesh() = 0;

		//! \brief Const version of getCollisionMesh()
		virtual const PxTetrahedronMesh* getCollisionMesh() const = 0;

		/**
		\brief Retrieves the simulation mesh pointer.

		Allows to access the geometry of the tetrahedral mesh used to compute the object's deformation

		\return Pointer to the simulation mesh
		*/
		virtual PxTetrahedronMesh*		getSimulationMesh() = 0;

		//! \brief Const version of getSimulationMesh()
		virtual const PxTetrahedronMesh* getSimulationMesh() const = 0;

		/**
		\brief Retrieves the simulation state pointer.

		Allows to access the additional data of the simulation mesh (inverse mass, rest state etc.).
		The geometry part of the data is stored in the simulation mesh.

		\return Pointer to the simulation state
		*/
		virtual PxSoftBodyAuxData* getSoftBodyAuxData() = 0;

		//! \brief const version of getSoftBodyAuxData()
		virtual const PxSoftBodyAuxData* getSoftBodyAuxData() const = 0;

		/**
		\brief Attaches a shape

		Attaches the shape to use for collision detection

		\param[in] shape The shape to use for collisions

		\return Returns true if the operation was successful
		*/
		virtual		bool				attachShape(PxShape& shape) = 0;

		/**
		\brief Attaches a simulation mesh

		Attaches the simulation mesh (geometry) and a state containing inverse mass, rest pose
		etc. required to compute the softbody deformation.

		\param[in] simulationMesh The tetrahedral mesh used to compute the softbody's deformation
		\param[in] softBodyAuxData A state that contain a mapping from simulation to collision mesh, volume information etc.

		\return Returns true if the operation was successful
		*/
		virtual		bool				attachSimulationMesh(PxTetrahedronMesh& simulationMesh, PxSoftBodyAuxData& softBodyAuxData) = 0;

		/**
		\brief Detaches the shape

		Detaches the shape used for collision detection.

		@see void detachSimulationMesh()
		*/
		virtual     void				detachShape() = 0;

		/**
		\brief Detaches the simulation mesh

		Detaches the simulation mesh and simulation state used to compute the softbody deformation.

		@see void detachShape()
		*/
		virtual		void				detachSimulationMesh() = 0;

		/**
		\brief Releases the softbody

		Releases the softbody and frees its resources.
		*/
		virtual		void				release() = 0;

		/**
		\brief Creates a collision filter between a particle and a tetrahedron in the soft body's collision mesh.

		\param[in] particlesystem The particle system used for the collision filter
		\param[in] buffer The PxParticleBuffer to which the particle belongs to.
		\param[in] particleId The particle whose collisions with the tetrahedron in the soft body are filtered.
		\param[in] tetId The tetradedron in the soft body that is filtered. If tetId is PX_MAX_TETID, this particle will filter against all tetrahedra in this soft body
		*/
		virtual		void				addParticleFilter(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId) = 0;
		
		/**
		\brief Removes a collision filter between a particle and a tetrahedron in the soft body's collision mesh.

		\param[in] particlesystem The particle system used for the collision filter
		\param[in] buffer The PxParticleBuffer to which the particle belongs to.
		\param[in] particleId The particle whose collisions with the tetrahedron in the soft body are filtered.
		\param[in] tetId The tetrahedron in the soft body is filtered.
		*/
		virtual		void				removeParticleFilter(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId) = 0;
		
		/**
		\brief Creates an attachment between a particle and a soft body.
		Be aware that destroying the particle system before destroying the attachment is illegal and may cause a crash.
		The soft body keeps track of these attachments but the particle system does not.

		\param[in] particlesystem The particle system used for the attachment
		\param[in] buffer The PxParticleBuffer to which the particle belongs to.
		\param[in] particleId The particle that is attached to a tetrahedron in the soft body's collision mesh.
		\param[in] tetId The tetrahedron in the soft body's collision mesh to attach the particle to.
		\param[in] barycentric The barycentric coordinates of the particle attachment position with respect to the tetrahedron specified with tetId.
		\return Returns a handle that identifies the attachment created. This handle can be used to release the attachment later
		*/
		virtual		PxU32				addParticleAttachment(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId, const PxVec4& barycentric) = 0;
		
		
		/**
		\brief Removes an attachment between a particle and a soft body.
		Be aware that destroying the particle system before destroying the attachment is illegal and may cause a crash.
		The soft body keeps track of these attachments but the particle system does not.
		
		\param[in] particlesystem The particle system used for the attachment
		\param[in] handle Index that identifies the attachment. This handle gets returned by the addParticleAttachment when the attachment is created
		*/
		virtual		void				removeParticleAttachment(PxPBDParticleSystem* particlesystem, PxU32 handle) = 0;

		/**
		\brief Creates a collision filter between a vertex in a soft body and a rigid body.

		\param[in] actor The rigid body actor used for the collision filter
		\param[in] vertId The index of a vertex in the softbody's collision mesh whose collisions with the rigid body are filtered.
		*/
		virtual		void				addRigidFilter(PxRigidActor* actor, PxU32 vertId) = 0;

		/**
		\brief Removes a collision filter between a vertex in a soft body and a rigid body.

		\param[in] actor The rigid body actor used for the collision filter
		\param[in] vertId The index of a vertex in the softbody's collision mesh whose collisions with the rigid body are filtered.
		*/
		virtual		void				removeRigidFilter(PxRigidActor* actor, PxU32 vertId) = 0;

		/**
		\brief Creates a rigid attachment between a soft body and a rigid body.
		Be aware that destroying the rigid body before destroying the attachment is illegal and may cause a crash.
		The soft body keeps track of these attachments but the rigid body does not.

		This method attaches a vertex on the soft body collision mesh to the rigid body.

		\param[in] actor The rigid body actor used for the attachment
		\param[in] vertId The index of a vertex in the softbody's collision mesh that gets attached to the rigid body.
		\param[in] actorSpacePose The location of the attachment point expressed in the rigid body's coordinate system.
		\param[in] constraint The user defined cone distance limit constraint to limit the movement between a vertex in the soft body and rigid body.
		\return Returns a handle that identifies the attachment created. This handle can be used to relese the attachment later
		*/
		virtual		PxU32					addRigidAttachment(PxRigidActor* actor, PxU32 vertId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint = NULL) = 0;

		/**
		\brief Releases a rigid attachment between a soft body and a rigid body.
		Be aware that destroying the rigid body before destroying the attachment is illegal and may cause a crash.
		The soft body keeps track of these attachments but the rigid body does not.

		This method removes a previously-created attachment between a vertex of the soft body collision mesh and the rigid body.

		\param[in] actor The rigid body actor used for the attachment
		\param[in] handle Index that identifies the attachment. This handle gets returned by the addRigidAttachment when the attachment is created
		*/
		virtual		void					removeRigidAttachment(PxRigidActor* actor, PxU32 handle) = 0;

		/**
		\brief Creates collision filter between a tetrahedron in a soft body and a rigid body.

		\param[in] actor The rigid body actor used for collision filter
		\param[in] tetIdx The index of a tetrahedron in the softbody's collision mesh whose collisions with the rigid body is filtered.
		*/
		virtual		void					addTetRigidFilter(PxRigidActor* actor, PxU32 tetIdx) = 0;
		
		/**
		\brief Removes collision filter between a tetrahedron in a soft body and a rigid body.

		\param[in] actor The rigid body actor used for collision filter
		\param[in] tetIdx The index of a tetrahedron in the softbody's collision mesh whose collisions with the rigid body is filtered.
		*/
		virtual		void					removeTetRigidFilter(PxRigidActor* actor, PxU32 tetIdx) = 0;

		/**
		\brief Creates a rigid attachment between a soft body and a rigid body.
		Be aware that destroying the rigid body before destroying the attachment is illegal and may cause a crash.
		The soft body keeps track of these attachments but the rigid body does not.

		This method attaches a point inside a tetrahedron of the collision to the rigid body.

		\param[in] actor The rigid body actor used for the attachment
		\param[in] tetIdx The index of a tetrahedron in the softbody's collision mesh that contains the point to be attached to the rigid body
		\param[in] barycentric The barycentric coordinates of the attachment point inside the tetrahedron specified by tetIdx
		\param[in] actorSpacePose The location of the attachment point expressed in the rigid body's coordinate system.
		\param[in] constraint The user defined cone distance limit constraint to limit the movement between a tet and rigid body.
		\return Returns a handle that identifies the attachment created. This handle can be used to release the attachment later
		*/
		virtual		PxU32					addTetRigidAttachment(PxRigidActor* actor, PxU32 tetIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint = NULL) = 0;

		/**
		\brief Creates collision filter between a tetrahedron in a soft body and a tetrahedron in another soft body.

		\param[in] otherSoftBody The other soft body actor used for collision filter
		\param[in] otherTetIdx The index of the tetrahedron in the other softbody's collision mesh to be filtered.
		\param[in] tetIdx1 The index of the tetrahedron in the softbody's collision mesh to be filtered.
		*/
		virtual		void					addSoftBodyFilter(PxSoftBody* otherSoftBody, PxU32 otherTetIdx, PxU32 tetIdx1) = 0;

		/**
		\brief Removes collision filter between a tetrahedron in a soft body and a tetrahedron in other soft body.

		\param[in] otherSoftBody The other soft body actor used for collision filter
		\param[in] otherTetIdx The index of the other tetrahedron in the other softbody's collision mesh whose collision with the tetrahedron with the soft body is filtered.
		\param[in] tetIdx1 The index of the tetrahedron in the softbody's collision mesh whose collision with the other tetrahedron with the other soft body is filtered.
		*/
		virtual		void					removeSoftBodyFilter(PxSoftBody* otherSoftBody, PxU32 otherTetIdx, PxU32 tetIdx1) = 0;

		/**
		\brief Creates collision filters between a tetrahedron in a soft body with another soft body.

		\param[in] otherSoftBody The other soft body actor used for collision filter
		\param[in] otherTetIndices The indices of the tetrahedron in the other softbody's collision mesh to be filtered.
		\param[in] tetIndices The indices of the tetrahedron of the softbody's collision mesh to be filtered.
		\param[in] tetIndicesSize The size of tetIndices.
		*/
		virtual		void					addSoftBodyFilters(PxSoftBody* otherSoftBody, PxU32* otherTetIndices, PxU32* tetIndices, PxU32 tetIndicesSize) = 0;

		/**
		\brief Removes collision filters between a tetrahedron in a soft body with another soft body.

		\param[in] otherSoftBody The other soft body actor used for collision filter
		\param[in] otherTetIndices The indices of the tetrahedron in the other softbody's collision mesh to be filtered.
		\param[in] tetIndices The indices of the tetrahedron of the softbody's collision mesh to be filtered.
		\param[in] tetIndicesSize The size of tetIndices.
		*/
		virtual		void					removeSoftBodyFilters(PxSoftBody* otherSoftBody, PxU32* otherTetIndices, PxU32* tetIndices, PxU32 tetIndicesSize) = 0;

		/**
		\brief Creates an attachment between two soft bodies.

		This method attaches a point inside a tetrahedron of the collision mesh to a point in another soft body's tetrahedron collision mesh.

		\param[in] softbody0 The soft body actor used for the attachment
		\param[in] tetIdx0 The index of a tetrahedron in the other soft body that contains the point to be attached to the soft body
		\param[in] tetBarycentric0 The barycentric coordinates of the attachment point inside the tetrahedron specified by tetIdx0
		\param[in] tetIdx1 The index of a tetrahedron in the softbody's collision mesh that contains the point to be attached to the softbody0
		\param[in] tetBarycentric1 The barycentric coordinates of the attachment point inside the tetrahedron specified by tetIdx1
		\param[in] constraint The user defined cone distance limit constraint to limit the movement between tets.
		\param[in] constraintOffset Offsets the cone and distance limit constraint along its axis, in order to specify the location of the cone tip.
		\return Returns a handle that identifies the attachment created. This handle can be used to release the attachment later
		*/
		virtual		PxU32					addSoftBodyAttachment(PxSoftBody* softbody0, PxU32 tetIdx0, const PxVec4& tetBarycentric0, PxU32 tetIdx1, const PxVec4& tetBarycentric1,
											PxConeLimitedConstraint* constraint = NULL, PxReal constraintOffset = 0.0f) = 0;

		/**
		\brief Releases an attachment between a soft body and the other soft body.
		Be aware that destroying the soft body before destroying the attachment is illegal and may cause a crash.

		This method removes a previously-created attachment between a point inside a tetrahedron of the collision mesh to a point in another soft body's tetrahedron collision mesh.

		\param[in] softbody0 The softbody actor used for the attachment.
		\param[in] handle Index that identifies the attachment. This handle gets returned by the addSoftBodyAttachment when the attachment is created.
		*/
		virtual		void					removeSoftBodyAttachment(PxSoftBody* softbody0, PxU32 handle) = 0;

		/**
		\brief Creates collision filter between a tetrahedron in a soft body and a triangle in a cloth.
		\warning Feature under development, only for internal usage.

		\param[in] cloth The cloth actor used for collision filter
		\param[in] triIdx The index of the triangle in the cloth mesh to be filtered.
		\param[in] tetIdx The index of the tetrahedron in the softbody's collision mesh to be filtered.
		*/
		virtual		void					addClothFilter(PxFEMCloth* cloth, PxU32 triIdx, PxU32 tetIdx) = 0;

		/**
		\brief Removes collision filter between a tetrahedron in a soft body and a triangle in a cloth.
		\warning Feature under development, only for internal usage.

		\param[in] cloth The cloth actor used for collision filter
		\param[in] triIdx The index of the triangle in the cloth mesh to be filtered.
		\param[in] tetIdx The index of the tetrahedron in the softbody's collision mesh to be filtered.
		*/
		virtual		void					removeClothFilter(PxFEMCloth* cloth, PxU32 triIdx, PxU32 tetIdx) = 0;

		/**
		\brief Creates an attachment between a soft body and a cloth.
		Be aware that destroying the rigid body before destroying the attachment is illegal and may cause a crash.
		The soft body keeps track of these attachments but the cloth does not.

		This method attaches a point inside a tetrahedron of the collision mesh to a cloth.

		\warning Feature under development, only for internal usage.

		\param[in] cloth The cloth actor used for the attachment
		\param[in] triIdx The index of a triangle in the cloth mesh that contains the point to be attached to the soft body
		\param[in] triBarycentric The barycentric coordinates of the attachment point inside the triangle specified by triangleIdx
		\param[in] tetIdx The index of a tetrahedron in the softbody's collision mesh that contains the point to be attached to the cloth
		\param[in] tetBarycentric The barycentric coordinates of the attachment point inside the tetrahedron specified by tetIdx
		\param[in] constraint The user defined cone distance limit constraint to limit the movement between a triangle in the fem cloth and a tet in the soft body.
		\param[in] constraintOffset Offsets the cone and distance limit constraint along its axis, in order to specify the location of the cone tip.
		\return Returns a handle that identifies the attachment created. This handle can be used to release the attachment later
		*/
		virtual		PxU32					addClothAttachment(PxFEMCloth* cloth, PxU32 triIdx, const PxVec4& triBarycentric, PxU32 tetIdx, const PxVec4& tetBarycentric,
											PxConeLimitedConstraint* constraint = NULL, PxReal constraintOffset = 0.0f) = 0;

		/**
		\brief Releases an attachment between a cloth and a soft body.
		Be aware that destroying the cloth before destroying the attachment is illegal and may cause a crash.
		The soft body keeps track of these attachments but the cloth does not.

		This method removes a previously-created attachment between a point inside a collision mesh tetrahedron and a point inside a cloth mesh.

		\warning Feature under development, only for internal usage.

		\param[in] cloth The cloth actor used for the attachment
		\param[in] handle Index that identifies the attachment. This handle gets returned by the addClothAttachment when the attachment is created
		*/
		virtual		void					removeClothAttachment(PxFEMCloth* cloth, PxU32 handle) = 0;

		/**
		\brief Retrieves the axis aligned bounding box enclosing the soft body.

		\note It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
		in PxContactModifyCallback or in contact report callbacks).

		\param[in] inflation  Scale factor for computed world bounds. Box extents are multiplied by this value.

		\return The soft body's bounding box.

		@see PxBounds3
		*/
		virtual		PxBounds3		getWorldBounds(float inflation = 1.01f) const = 0;

		/**
		\brief Returns the GPU soft body index.

		\return The GPU index, or 0xFFFFFFFF if the soft body is not in a scene.
		*/
		virtual		PxU32			getGpuSoftBodyIndex() = 0;
	
		virtual		const char*		getConcreteTypeName() const PX_OVERRIDE { return "PxSoftBody";  }

	protected:
		PX_INLINE					PxSoftBody(PxType concreteType, PxBaseFlags baseFlags) : PxActor(concreteType, baseFlags) {}
		PX_INLINE					PxSoftBody(PxBaseFlags baseFlags) : PxActor(baseFlags) {}
		virtual		bool			isKindOf(const char* name) const PX_OVERRIDE { return !::strcmp("PxSoftBody", name) || PxActor::isKindOf(name); }
	};

	/**
	\brief Adjusts a softbody kinematic target such that it is properly set as active or inactive. Inactive targets will not affect vertex position, they are ignored by the solver.

	\param[in] target The kinematic target
	\param[in] isActive A boolean indicating if the returned target should be marked as active or not
	\return The target with adjusted w component
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec4 PxConfigureSoftBodyKinematicTarget(const PxVec4& target, bool isActive)
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
	\brief Sets up a softbody kinematic target such that it is properly set as active or inactive. Inactive targets will not affect vertex position, they are ignored by the solver.

	\param[in] target The kinematic target
	\param[in] isActive A boolean indicating if the returned target should be marked as active or not
	\return The target with configured w component
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec4 PxConfigureSoftBodyKinematicTarget(const PxVec3& target, bool isActive)
	{
		return PxConfigureSoftBodyKinematicTarget(PxVec4(target, 0.0f), isActive);
	}

#if PX_VC
#pragma warning(pop)
#endif


#if !PX_DOXYGEN
} // namespace physx
#endif

  /** @} */
#endif
