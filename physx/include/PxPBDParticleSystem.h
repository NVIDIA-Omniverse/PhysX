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

#ifndef PX_PBD_PARTICLE_SYSTEM_H
#define PX_PBD_PARTICLE_SYSTEM_H
/** \addtogroup physics
@{ */

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"

#include "PxParticleSystem.h"
#include "PxParticlePhase.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxParticleClothBuffer;
class PxParticleRigidBuffer;
class PxPBDMaterial;

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4435)
#endif

/**
\brief A particle system that uses the position based dynamics solver

The position based dynamics solver for particle systems supports several behaviors like
fluid, cloth, inflatables etc. 

*/
class PxPBDParticleSystem : public PxParticleSystem
{
public:

	virtual								~PxPBDParticleSystem() {}


	/**
	\brief Creates combined particle flag with particle material and particle phase flags.
	
	\param[in] material A material instance to associate with the new particle group.
	\param[in] flags The particle phase flags.
	\return The combined particle group index and phase flags.

	See PxParticlePhaseFlag()
	*/
	virtual		PxU32					createPhase(PxPBDMaterial* material, const PxParticlePhaseFlags flags) = 0;

	
	/**
	\brief Set wind direction and intensity

	\param[in] wind The wind direction and intensity
	*/
	virtual		void					setWind(const PxVec3& wind) = 0;

	/**
	\brief Retrieves the wind direction and intensity.

	\return The wind direction and intensity
	*/
	virtual		PxVec3					getWind() const = 0;

	/**
	\brief Set the fluid boundary density scale

	Defines how strong of a contribution the boundary (typically a rigid surface) should have on a fluid particle's density.

	\param[in] fluidBoundaryDensityScale  <b>Range:</b> (0.0, 1.0)
	*/
	virtual		void					setFluidBoundaryDensityScale(const PxReal fluidBoundaryDensityScale) = 0;

	/**
	\brief Return the fluid boundary density scale
	\return the fluid boundary density scale

	See setFluidBoundaryDensityScale()
	*/
	virtual		PxReal					getFluidBoundaryDensityScale() const = 0;

	/**
	\brief Set the fluid rest offset

	Two fluid particles will come to rest at a distance equal to twice the fluidRestOffset value.

	\param[in] fluidRestOffset  <b>Range:</b> (0, particleContactOffset)
	*/
	virtual		void				    setFluidRestOffset(const PxReal fluidRestOffset) = 0;

	/**
	\brief Return the fluid rest offset
	\return the fluid rest offset

	See setFluidRestOffset()
	*/
	virtual		PxReal				    getFluidRestOffset() const = 0;

	/**
	\brief Set the particle system grid size x dimension

	\param[in] gridSizeX x dimension in the particle grid
	*/
	virtual		void					setGridSizeX(const PxU32 gridSizeX) = 0;

	/**
	\brief Set the particle system grid size y dimension

	\param[in] gridSizeY y dimension in the particle grid
	*/
	virtual		void					setGridSizeY(const PxU32 gridSizeY) = 0;

	/**
	\brief Set the particle system grid size z dimension

	\param[in] gridSizeZ z dimension in the particle grid
	*/
	virtual		void					setGridSizeZ(const PxU32 gridSizeZ) = 0;

	/**
	\brief Add an existing cloth buffer to the particle system
	\param[in] particleClothBuffer a PxParticleClothBuffer*.
	*/
	virtual		void					addParticleClothBuffer(PxParticleClothBuffer* particleClothBuffer) = 0;

	/**
	\brief Remove cloth buffer from particle system
	\param[in] particleClothBuffer a PxParticleClothBuffer*.
	*/
	virtual		void					removeParticleClothBuffer(PxParticleClothBuffer* particleClothBuffer) = 0;

	/**
	\brief Add an existing rigid buffer to the particle system
	\param[in] particleRigidBuffer a PxParticleRigidBuffer*.
	*/
	virtual		void					addParticleRigidBuffer(PxParticleRigidBuffer* particleRigidBuffer) = 0;

	/**
	\brief Remove rigid buffer from particle system
	\param[in] particleRigidBuffer a PxParticleRigidBuffer*.
	*/
	virtual		void					removeParticleRigidBuffer(PxParticleRigidBuffer* particleRigidBuffer) = 0;
	
	PX_INLINE							PxPBDParticleSystem(PxType concreteType, PxBaseFlags baseFlags) : PxParticleSystem(concreteType, baseFlags) {}
	PX_INLINE							PxPBDParticleSystem(PxBaseFlags baseFlags) : PxParticleSystem(baseFlags) {}
};

#if PX_VC
#pragma warning(pop)
#endif


#if !PX_DOXYGEN
} // namespace physx
#endif

  /** @} */
#endif
