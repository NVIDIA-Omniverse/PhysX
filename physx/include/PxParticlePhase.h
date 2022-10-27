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

#ifndef PX_PARTICLE_PHASE_H
#define PX_PARTICLE_PHASE_H
/** \addtogroup physics
@{ */

#include "foundation/PxFlags.h"
#include "foundation/PxSimpleTypes.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4435)
#endif

class PxParticleMaterial;

/**
\brief Identifies per-particle behavior for a PxParticleSystem.

See #PxParticleSystem::createPhase().
*/
struct PxParticlePhaseFlag
{
	enum Enum
	{
		eParticlePhaseGroupMask = 0x000fffff,			//!< Bits [ 0, 19] represent the particle group for controlling collisions
		eParticlePhaseFlagsMask = 0xfff00000,			//!< Bits [20, 23] hold flags about how the particle behave 

		eParticlePhaseSelfCollide = 1 << 20,			//!< If set this particle will interact with particles of the same group
		eParticlePhaseSelfCollideFilter = 1 << 21,		//!< If set this particle will ignore collisions with particles closer than the radius in the rest pose, this flag should not be specified unless valid rest positions have been specified using setRestParticles()
		eParticlePhaseFluid = 1 << 22					//!< If set this particle will generate fluid density constraints for its overlapping neighbors
	};
};

typedef PxFlags<PxParticlePhaseFlag::Enum, PxU32> PxParticlePhaseFlags;

/**
\brief A per particle identifier to define the particles behavior, its group and to reference a material
*/
class PxParticlePhase
{
public:
	/**
	\brief Set phase flags

	Allows to control self collision, etc.

	\param[in] flags The filter flags
	*/
	virtual void					setFlags(PxParticlePhaseFlags flags) = 0;
	
	/**
	\brief Set phase flag

	Allows to control self collision etc.

	\param[in] flag The flag to set
	\param[in] enabled The new value of the flag
	*/
	virtual void					setFlag(PxParticlePhaseFlag::Enum flag, bool enabled) = 0;
	
	/**
	\brief Retrieves the phase flags

	\return The phase flags
	*/
	virtual PxParticlePhaseFlags	getFlags() const = 0;
	
	/**
	\brief Returns the group id

	\return The group id
	*/
	virtual PxU32					getGroupId() const = 0;

	/**
	\brief Returns the pointer to the material used by this phase

	\return The material pointer
	*/
	virtual PxParticleMaterial*		getMaterial() const = 0;
	
	/**
	\brief Sets the material associated referenced by this phase

	\param[in] material The pointer to the material that should be used by this phase	
	*/
	virtual void					setMaterial(PxParticleMaterial* material) = 0;

protected:

	virtual							~PxParticlePhase() {}
};

#if PX_VC
#pragma warning(pop)
#endif


#if !PX_DOXYGEN
} // namespace physx
#endif

  /** @} */
#endif
