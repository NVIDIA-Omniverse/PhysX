// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

namespace omni
{
namespace physx
{
namespace ui
{

/**
\brief Particle radius debug visualization option (kSettingDisplayParticlesParticleRadius).
*/
enum class ParticleRadiusType : int
{
    eContactOffset, //!< Particle-nonparticle contact offset
    eRestOffset, //!< Particle-nonparticle rest offset
    eParticleContactOffset, //!< Particle-Particle contact offset
    eParticleRestOffset, //!< Fluid- or solid particle rest offset (applicable radius is auto-determined)
    eAnisotropy, //!< Fluid particles anisotropy
    eRenderGeometry, //!< Render geometry of particle object
};

/**
\brief Smoothed particle fluid visualization option (kSettingDisplayParticlesParticlePositions).
*/
enum class ParticlePositionType : int
{
    eSimPositions, //!< Show simulation particle positions
    eSmoothedPositions, //!< Show smoothing-post-processed particle positions
};

} // namespace ui
} // namespace physx
} // namespace omni
