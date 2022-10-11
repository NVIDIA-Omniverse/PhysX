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

#ifndef PX_PARTICLE_BUFFER_H
#define PX_PARTICLE_BUFFER_H
/** \addtogroup physics
@{ */

#include "PxPhysXConfig.h"
#include "PxParticleSystemFlag.h"
#include "foundation/PxBounds3.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4435)
#endif

class PxCudaContextManager;
struct PxParticleRigidFilterPair;
struct PxParticleRigidAttachment;

/**
\brief Particle volume structure. Used to track the bounding volume of a user-specified set of particles. The particles are required
to be laid out contiguously within the same PxUserParticleBuffer.
*/
PX_ALIGN_PREFIX(16)
struct PxParticleVolume
{
	PxBounds3	bound;					//!< The current bounds of the particles contained in this PxParticleVolume.
	PxU32		particleIndicesOffset;	//!< The index into the particle list of the PxUserParticleBuffer for the first particle of this volume.
	PxU32		numParticles;			//!< The number of particles contained in this PxParticleVolume.
} PX_ALIGN_SUFFIX(16);


/**
\brief The shared base class for all particle user buffers, can be instantiated directly to specify granular and fluid particles.

See PxPhysics::createParticleBuffer.

A particle user buffer is a container that specifies per-particle attributes of a set of particles that will be used during the simulation 
of a particle system. It exposes direct access to the underlying GPU buffers and is independent of the scene and particle system. Particle
user buffers can be added/removed from a particle system at any time between simulation steps, and transferred from one particle system to another.
*/
class PxUserParticleBuffer
{
public:

	/**
	\brief Set positions and inverse masses for this user particle buffer.
	\param[in] pos A pointer to a device buffer containing the positions and inverse mass packed as PxVec4(pos.x, pos.y, pos.z, inverseMass).
	*/
	virtual void				setPositionInvMasses(PxVec4* pos) = 0;

	/**
	\brief Get positions and inverse masses for this user particle buffer.
	\return A pointer to a device buffer containing the positions and inverse mass packed as PxVec4(pos.x, pos.y, pos.z, inverseMass).
	*/
	virtual PxVec4*				getPositionInvMasses() const = 0;

	/**
	\brief Set the number of active particles for this user particle buffer.
	\param[in] nbActiveParticles The number of active particles.

	The number of active particles can be <= PxUserParticleBuffer::getMaxParticles(). The particle system will simulate the first
	x particles in the PxUserParticleBuffer, where x is the number of active particles.
	*/
	virtual void				setNbActiveParticles(const PxU32 nbActiveParticles) = 0;

	/**
	\brief Get the number of active particles for this user particle buffer.
	\return The number of active particles.
	*/
	virtual PxU32				getNbActiveParticles() const = 0;

	/**
	\brief Get the maximum number particles this user particle buffer can hold.

	The maximum number of particles is specified when creating a PxUserParticleBuffer. See PxPhysics::createParticleBuffer.

	\return The maximum number of particles.
	*/
	virtual PxU32				getMaxParticles() const = 0;

	/**
	\brief Set velocities for this user particle buffer.
	\param[in] vel A pointer to a device buffer containing the velocities packed as PxVec4(vel.x, vel.y, vel.z, 0.0f).
	*/
	virtual void				setVelocities(PxVec4* vel) = 0;

	/**
	\brief Get velocities for this user particle buffer.
	\return A pointer to a device buffer containing the velocities packed as PxVec4(vel.x, vel.y, vel.z, 0.0f).
	*/
	virtual PxVec4*				getVelocities() const = 0;

	/**
	\brief Set per-particle phase for this user particle buffer.

	See PxParticlePhase.

	\param[in] phase A pointer to a device buffer containing the per-particle phases.
	*/
	virtual void				setPhase(PxU32* phase) = 0;

	/**
	\brief Get phases for this user particle buffer.
	\return A pointer to a device buffer containing the per-particle phases for this particle buffer.
	*/
	virtual PxU32*				getPhase() const = 0;

	/**
	\brief Set the PxParticleVolumes for this user particle buffer.

	See PxParticleVolume.

	\param[in] volumes A pointer to a device buffer containing the PxParticleVolumes.
	*/
	virtual void				setParticleVolumes(PxParticleVolume* volumes) = 0;

	/**
	\brief Get particle volumes for this user particle buffer.
	\return A pointer to a device buffer containing the PxParticleVolume s for this particle buffer.
	*/
	virtual PxParticleVolume*	getParticleVolumes() const = 0;

	/**
	\brief Get the number of particle volumes in this user particle buffer.
	\return The number of PxParticleVolume s for this particle buffer.
	*/
	virtual PxU32				getNbParticleVolumes() const = 0;

	/**
	\brief Set the number of PxParticleVolumes for this user particle buffer.
	\param[in] nbParticleVolumes The number of particle volumes in this user particle buffer.
	*/
	virtual void				setNbParticleVolumes(const PxU32 nbParticleVolumes) = 0;

	/**
	\brief Get the maximum number of particle volumes this user particle buffer can hold.

	See PxParticleVolume.

	\return The maximum number of particle volumes this user particle buffer can hold.
	*/
	virtual PxU32				getMaxParticleVolumes() const = 0;

	/**
	\brief Set the PxParticleRigidFilterPair s for collision filtering of particles in this buffer with rigid bodies.

	See PxParticleRigidFilterPair

	\param[in] filters A device buffer containing PxParticleRigidFilterPair s.
	\param[in] nbFilters The number of particle-rigid body collision filtering pairs.
	*/
	virtual void				setRigidFilters(PxParticleRigidFilterPair* filters, PxU32 nbFilters) = 0;

	/**
	\brief Set the particle-rigid body attachments for particles in this user particle buffer.

	See PxParticleRigidAttachment

	\param[in] attachments A device buffer containing PxParticleRigidAttachment s.
	\param[in] nbAttachments The number of particle-rigid body attachments.
	*/
	virtual void				setRigidAttachments(PxParticleRigidAttachment* attachments, PxU32 nbAttachments) = 0;

	/**
	\brief Get the start index for the first particle of this user particle buffer in the complete list of
	particles of the particle system this buffer is used in.

	The return value is only correct if the user particle buffer is assigned to a particle system and at least
	one call to simulate() has been performed.

	\return The index of the first particle in the complete particle list.
	*/
	virtual PxU32				getFlatListStartIndex() const = 0;

	/**
	\brief Raise dirty flags on this user particle buffer to communicate that the corresponding data has been updated
	by the user.
	\param[in] flags The flag corresponding to the data that is dirty.

	See PxParticleBufferFlag.
	*/
	virtual void				raiseFlags(PxParticleBufferFlag::Enum flags) = 0;

	/**
	\brief Release this buffer and deallocate all the memory.
	*/
	virtual void				release() = 0;

	/**
	\brief Cleanup helper used in case a particle system is released before the user particle buffers have been removed.
	*/
	virtual void				onParticleSystemDestroy() = 0;

	/**
	\brief Reserved for internal use.
	*/
	virtual void				setInternalData(void* data) = 0;

	/**
	\brief Index of this buffer in the particle system it is assigned to.
	*/
	PxU32						bufferIndex;

	/**
	\brief Unique index that does not change over the lifetime of a PxUserParticleBuffer.
	*/
	const PxU32					bufferUniqueId;

protected:

	virtual						~PxUserParticleBuffer() { }
	PX_INLINE 					PxUserParticleBuffer(PxU32 uniqueId) : bufferIndex(0xffffffff), bufferUniqueId(uniqueId){}

private:
	PX_NOCOPY(PxUserParticleBuffer)
};

/**
\brief Parameters to configure the behavior of diffuse particles
*/
class PxDiffuseParticleParams
{
public:
	/**
	\brief Construct parameters with default values.
	*/
	PX_INLINE PxDiffuseParticleParams()
	{
		threshold = 100.0f;
		lifetime = 5.0f;
		airDrag = 0.0f;
		bubbleDrag = 0.5f;
		buoyancy = 0.8f;
		kineticEnergyWeight = 0.01f;
		pressureWeight = 1.0f;
		divergenceWeight = 5.0f;
		collisionDecay = 0.5f;
		useAccurateVelocity = false;
	}

	/**
	\brief (re)sets the structure to the default.	
	*/
	PX_INLINE void setToDefault()
	{
		*this = PxDiffuseParticleParams();
	}
	
	PxReal	threshold;				//!< Particles with potential value greater than the threshold will spawn diffuse particles
	PxReal	lifetime;				//!< Diffuse particle will be removed after the specified lifetime
	PxReal	airDrag;				//!< Air drag force factor for spray particles
	PxReal	bubbleDrag;				//!< Fluid drag force factor for bubble particles
	PxReal	buoyancy;				//!< Buoyancy force factor for bubble particles
	PxReal	kineticEnergyWeight;	//!< Contribution from kinetic energy when deciding diffuse particle creation.
	PxReal	pressureWeight;			//!< Contribution from pressure when deciding diffuse particle creation.
	PxReal	divergenceWeight;		//!< Contribution from divergence when deciding diffuse particle creation.
	PxReal	collisionDecay;			//!< Decay factor of diffuse particles' lifetime after they collide with shapes.
	bool	useAccurateVelocity;	//!< If true, enables accurate velocity estimation when using PBD solver.
};

/**
\brief A user particle buffer used to simulate diffuse particles.

See PxPhysics::createParticleAndDiffuseBuffer.
*/
class PxUserParticleAndDiffuseBuffer : public PxUserParticleBuffer
{
public:

	/**
	\brief Get a device buffer of positions and remaining lifetimes for the diffuse particles.
	\return A device buffer containing positions and lifetimes of diffuse particles packed as PxVec4(pos.x, pos.y, pos.z, lifetime).
	*/
	virtual PxVec4*					getDiffusePositionLifeTime() const = 0;

	/**
	\brief Get number of currently active diffuse particles.
	\return The number of currently active diffuse particles.
	*/
	virtual PxU32					getNbActiveDiffuseParticles() const = 0;

	/**
	\brief Set the maximum possible number of diffuse particles for this buffer.
	\param[in] maxActiveDiffuseParticles the maximum number of active diffuse particles.

	\note Must be in the range [0, PxParticleAndDiffuseBuffer::getMaxDiffuseParticles()]
	*/
	virtual void					setMaxActiveDiffuseParticles(PxU32 maxActiveDiffuseParticles) = 0;

	/**
	\brief Get maximum possible number of diffuse particles.
	\return The maximum possible number diffuse particles.
	*/
	virtual PxU32					getMaxDiffuseParticles() const = 0;

	/**
	\brief Set the parameters for diffuse particle simulation.
	\param[in] params The diffuse particle parameters.

	See PxDiffuseParticleParams
	*/
	virtual void					setDiffuseParticleParams(const PxDiffuseParticleParams& params) = 0;

	/**
	\brief Get the parameters currently used for diffuse particle simulation.
	\return A PxDiffuseParticleParams structure.
	*/
	virtual PxDiffuseParticleParams	getDiffuseParticleParams() const = 0;

protected:

	virtual 						~PxUserParticleAndDiffuseBuffer() {}
	PX_INLINE 						PxUserParticleAndDiffuseBuffer(PxU32 uniqueId) : PxUserParticleBuffer(uniqueId){}

private:
	PX_NOCOPY(PxUserParticleAndDiffuseBuffer)
};

/**
\brief Holds all the information for a spring constraint between two particles. Used for particle cloth simulation.
*/
struct PX_ALIGN_PREFIX(8) PxParticleSpring
{
	PxU32 ind0;			//!< particle index of first particle
	PxU32 ind1;			//!< particle index of second particle
	PxReal length;		//!< spring length
	PxReal stiffness;	//!< spring stiffness
	PxReal damping;		//!< spring damping factor
	PxReal pad;			//!< padding bytes.
} PX_ALIGN_SUFFIX(8);

/**
\brief Particle cloth structure. Holds information about a single piece of cloth that is part of a PxUserParticleClothBuffer.
*/
struct PxParticleCloth
{
	PxU32	startVertexIndex;	//!< Index of the first particle of this cloth in the position/velocity buffers of the parent PxUserParticleClothBuffer
	PxU32	numVertices;		//!< The number of particles of this piece of cloth
	PxReal	clothBlendScale;	//!< Used internally.
	PxReal	restVolume;			//!< The rest volume of this piece of cloth, used for inflatable simulation.
	PxReal	pressure;			//!< The factor of the rest volume to specify the target volume for this piece of cloth, used for inflatable simulation.
	PxU32	startTriangleIndex;	//!< The index of the first triangle of this piece of cloth in the triangle list.
	PxU32	numTriangles;		//!< The number of triangles of this piece of cloth.

	bool operator <= (const PxParticleCloth& other) const { return startVertexIndex <= other.startVertexIndex; }
	bool operator >= (const PxParticleCloth& other) const { return startVertexIndex >= other.startVertexIndex; }
	bool operator < (const PxParticleCloth& other) const { return startVertexIndex < other.startVertexIndex; }
	bool operator > (const PxParticleCloth& other) const { return startVertexIndex > other.startVertexIndex; }
	bool operator == (const PxParticleCloth& other) const { return startVertexIndex == other.startVertexIndex; }
};

/**
\brief Structure to describe the set of particle cloths in the same PxUserParticleClothBuffer. Used an input for the cloth preprocessing.
*/
struct PxParticleClothDesc
{
	PxParticleClothDesc() : cloths(NULL), triangles(NULL), springs(NULL), restPositions(NULL), 
		nbCloths(0), nbSprings(0), nbTriangles(0), nbParticles(0)
	{
	}

	PxParticleCloth*	cloths;			//!< List of PxParticleCloth s, describes the individual cloths.
	PxU32*				triangles;		//!< List of triangle indices, 3 consecutive PxU32 that map triangle vertices to particles
	PxParticleSpring*	springs;		//!< List of PxParticleSpring s.
	PxVec4*				restPositions;	//!< List of rest positions for all particles
	PxU32				nbCloths;		//!< The number of cloths in described using this cloth descriptor
	PxU32				nbSprings;		//!< The number of springs in this cloth descriptor
	PxU32				nbTriangles;	//!< The number of triangles in this cloth descriptor
	PxU32				nbParticles;	//!< The number of particles in this cloth descriptor
};

/**
\brief Structure to describe the output of the particle cloth preprocessing. Used as an input to specify cloth data for a PxUserParticleClothBuffer.
All the pointers point to pinned host memory.

See PxParticleClothPreProcessor
*/
struct PX_PHYSX_CORE_API PxParticleClothOutput
{
	PxU32*						accumulatedSpringsPerPartitions;	//!< The number of springs in each partition. Size: numPartitions.
	PxU32*						accumulatedCopiesPerParticles;		//!< Start index for each particle in the accumulation buffer. Size: numParticles.
	PxU32*						remapOutput;						//!< Index of the next copy of this particle in the next partition, or in the accumulation buffer. Size: numSprings * 2.
	PxParticleSpring*			orderedSprings;						//!< Springs ordered by partition. Size: numSprings.
	PxU32*						sortedClothStartIndices;			//!< The first particle index into the position buffer of the PxUserParticleClothBuffer for each cloth. Cloths are sorted by start particle index. Size: numCloths.
	PxParticleCloth*			cloths;								//!< The PxParticleCloth s sorted by start particle index.

	PxU32						remapOutputSize;					//!< Size of remapOutput.
	PxU32						nbPartitions;						//!< The number of partitions.
	PxU32						nbSprings;							//!< The number of springs.
	PxU32						nbCloths;							//!< The number of cloths.
	PxU32						maxSpringsPerPartition;				//!< The maximum number of springs in a partition.

	PxCudaContextManager* 		mCudaManager;						//!< A cuda context manager.

	PxParticleClothOutput();
	~PxParticleClothOutput();

	/**
	\brief allocate all the buffers for this PxParticleClothOutput.
	 
	\param[in] nbParticles the number of particles this PxParticleClothOutput will be generated for.
	\param[in] cudaManager a cuda context manager.
	*/
	void allocateBuffers(PxU32 nbParticles, PxCudaContextManager* cudaManager);
};

/**
\brief A user particle buffer used to simulate particle cloth.

See PxPhysics::createParticleClothBuffer.
*/
class PxUserParticleClothBuffer : public PxUserParticleBuffer
{
public:

	/**
	\brief Set rest positions for this user particle buffer.
	\param[in] pos A pointer to a device buffer containing the rest positions packed as PxVec4(pos.x, pos.y, pos.z, 0.0f).
	*/
	virtual void				setRestPositions(PxVec4* pos) = 0;

	/**
	\brief Get rest positions for this user particle buffer.
	\return A pointer to a device buffer containing the rest positions packed as PxVec4(pos.x, pos.y, pos.z, 0.0f).
	*/
	virtual PxVec4*				getRestPositions() = 0;

	/**
	\brief Set triangle indices for this user particle buffer.
	\param[in] triangleIndices A pointer to a device buffer containing triangle indices for this cloth buffer. 3 consecutive PxU32 specify the 3 particle positions that make up a triangle.
	*/
	virtual void				setTriangles(PxU32* triangleIndices) = 0;

	/**
	\brief Get the triangle indices for this user particle buffer.
	\return A pointer to a device buffer containing the triangle indices for this cloth buffer.
	*/
	virtual PxU32*				getTriangles() const = 0;

	/**
	\brief Set the number of triangles for this user particle buffer.
	\param[in] nbTriangles The number of triangles for this user particle cloth buffer.
	*/
	virtual void				setNbTriangles(PxU32 nbTriangles) = 0;

	/**
	\brief Get the number of triangles for this user particle buffer.
	\return The number triangles for this cloth buffer.
	*/
	virtual PxU32				getNbTriangles() const = 0;

	/**
	\brief Get the number of springs in this user particle buffer.
	\return The number of springs in this cloth buffer.
	*/
	virtual PxU32				getNbSprings() const = 0;

	/**
	\brief Get the springs for this user particle buffer.
	\return A pointer to a device buffer containing the springs for this cloth buffer.
	*/
	virtual PxParticleSpring*	getSprings() = 0;

	/**
	\brief Set cloths for this user particle buffer.
	\param[in] cloths A pointer to a PxParticleClothOutput.

	See PxParticleClothOutput, PxParticleClothPreProcessor
	*/
	virtual void				setCloths(PxParticleClothOutput& cloths) = 0;

protected:

	virtual						~PxUserParticleClothBuffer() {}
	PX_INLINE 					PxUserParticleClothBuffer(PxU32 uniqueId) : PxUserParticleBuffer(uniqueId){}

private:
	PX_NOCOPY(PxUserParticleClothBuffer)
};

/**
\brief A user particle buffer used to simulate rigid bodies using shape matching with particles.

See PxPhysics::createParticleRigidBuffer.
*/
class PxUserParticleRigidBuffer : public PxUserParticleBuffer
{
public:
	/**
	\brief Set the particle indices of the first particle for each shape matched rigid body.
	\param[in] offsets Pointer to a device buffer containing the list of particle start indices of each shape matched rigid body.
	*/
	virtual void	setRigidOffsets(const PxU32* offsets) = 0;

	/**
	\brief Get the particle indices of the first particle for each shape matched rigid body.
	\return A device buffer containing the list of particle start indices of each shape matched rigid body.
	*/
	virtual PxU32*	getRigidOffsets() const = 0;

	/**
	\brief Set the stiffness coefficient for all shape-matched rigid bodies in this buffer.

	Stiffness must be in the range [0, 1].

	\param[in] coefficients A pointer to a device buffer containing the stiffness coefficient for each rigid body.
	*/
	virtual void	setRigidCoefficients(const PxReal* coefficients) = 0;

	/**
	\brief Get the stiffness coefficients for all shape matched rigid bodies in this buffer.
	\return A device buffer containing the list of stiffness coefficients for each rigid body.
	*/
	virtual PxReal*	getRigidCoefficients() const = 0;

	/**
	\brief Set the local position of each particle relative to the rigid body's center of mass.
	\param[in] localPositions A pointer to a device buffer containing the local positions.
	*/
	virtual void	setRigidLocalPositions(PxVec4* localPositions) = 0;

	/**
	\brief Get the local position of each particle relative to the rigid body's center of mass.
	\return A pointer to a device buffer containing the local position for each particle.
	*/
	virtual PxVec4*	getRigidLocalPositions() const = 0;

	/**
	\brief Set the world-space translation of each rigid body's center of mass. 
	\param[in] translation A pointer to a device buffer containing the world-space translation for all shape-matched rigid bodies in this buffer. 
	*/
	virtual void	setRigidTranslations(const PxVec4* translation) = 0;

	/**
	\brief Get the world-space translations for all rigid bodies in this buffer. 
	\return A pointer to a device buffer containing the world-space translations for all shape-matched rigid bodies in this buffer. 
	 */
	virtual PxVec4*	getRigidTranslations() const = 0;

	/**
	\brief Set the world-space rotation of every shape-matched rigid body in this buffer.

	Rotations are specified as quaternions.

	\param[in] rotations A pointer to a device buffer containing the world-space rotation for every shape-matched rigid body in this buffer. 
	 */
	virtual void	setRigidRotations(const PxVec4* rotations) = 0;
	
	/**
	\brief Get the world-space rotation of every shape-matched rigid body in this buffer.
	\return A pointer to a device buffer containing the world-space rotation for every shape-matched rigid body in this buffer.
	*/
	virtual PxVec4*	getRigidRotations() const = 0;

	/**
	\brief Set the local space normals for each particle relative to the shape of the corresponding rigid body.

	The 4th component of every PxVec4 should be the negative signed distance of the particle inside its shape.
	 
	\param[in] localNormals A pointer to a device buffer containing the local-space normals for each particle packed as PxVec4(x,y,z,dist).
	*/
	virtual void	setRigidLocalNormals(PxVec4* localNormals) = 0;

	/**
	\brief Set the local space normals for each particle relative to the shape of the corresponding rigid body.
	\return A pointer to a device buffer containing the local-space normals for each particle. 
	 */
	virtual PxVec4*	getRigidLocalNormals() const = 0;
	
	/**
	\brief Set the number of shape matched rigid bodies in this buffer.
	\param[in] nbRigids The number of shape matched rigid bodies
	*/
	virtual void	setNbRigids(const PxU32 nbRigids) = 0;

	/**
	\brief Get the number of shape matched rigid bodies in this buffer.
	\return The number of shape matched rigid bodies in this buffer.
	*/
	virtual PxU32	getNbRigids() const = 0;

protected:

	virtual			~PxUserParticleRigidBuffer() {}
	PX_INLINE 		PxUserParticleRigidBuffer(PxU32 uniqueId) : PxUserParticleBuffer(uniqueId){}

private:
	PX_NOCOPY(PxUserParticleRigidBuffer)
};

/**
\brief Preprocessor to prepare particle cloths for simulation.

Preprocessing is done by calling partitionSprings() on an instance of this class. This will allocate the memory in the 
output object, partition the springs and fill all the members of the ouput object. The output can then be passed without 
any further modifications to PxUserParticleClothBuffer::setCloths().

See PxCreateParticleClothPreprocessor, PxParticleClothDesc, PxParticleClothOutput
*/
class PxParticleClothPreProcessor
{
public:

	/**
	\brief Release this object and deallocate all the memory.
	*/
	virtual void	release() = 0;

	/**
	\brief Partition the spring constraints for particle cloth simulation.
	\param[in] clothDesc Reference to a valid PxParticleClothDesc.
	\param[in] output Reference to a PxParticleClothOutput object. This is the output of the preprocessing and should be passed to a PxUserParticleClothBuffer.
	*/
	virtual void	partitionSprings(const PxParticleClothDesc& clothDesc, PxParticleClothOutput& output) = 0;

protected:
	virtual			~PxParticleClothPreProcessor(){}
};


#if PX_VC
#pragma warning(pop)
#endif


#if !PX_DOXYGEN
} // namespace physx
#endif

/**
\brief Create a particle cloth preprocessor.
\param[in] cudaContextManager A cuda context manager.

See PxParticleClothDesc, PxParticleClothOutput.
*/
PX_C_EXPORT PX_PHYSX_CORE_API physx::PxParticleClothPreProcessor* PX_CALL_CONV PxCreateParticleClothPreProcessor(physx::PxCudaContextManager* cudaContextManager);


  /** @} */
#endif
