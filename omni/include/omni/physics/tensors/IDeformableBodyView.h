// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "TensorDesc.h"

#include <cstdint>

namespace omni
{
namespace physics
{
namespace tensors
{

class IDeformableBodyView
{
public:
    // returns the number of deformable bodies in view
    virtual uint32_t getCount() const = 0;

    // return the associated deformable body prim path
    virtual const char* getUsdPrimPath(uint32_t dbIdx) const = 0;

    // return the associated deformable simulation mesh prim path
    virtual const char* getUsdSimulationMeshPrimPath(uint32_t dbIdx) const = 0;

    // return the associated deformable collision mesh prim path
    virtual const char* getUsdCollisionMeshPrimPath(uint32_t dbIdx) const = 0;

    // returns the center of the world bounds as a transform
    virtual bool getTransforms(const TensorDesc* dstTensor) const = 0;

    // returns number of referenced nodes per element (tetmesh: 4, trimesh: 3)
    virtual uint32_t getNumNodesPerElement() const = 0;

    // simulation mesh
    virtual uint32_t getMaxSimulationElementsPerBody() const = 0;
    virtual bool getSimulationElementIndices(const TensorDesc* dstTensor) const = 0;
    virtual uint32_t getMaxSimulationNodesPerBody() const = 0;
    virtual bool getSimulationNodalPositions(const TensorDesc* dstTensor) const = 0;
    virtual bool setSimulationNodalPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool getSimulationNodalVelocities(const TensorDesc* dstTensor) const = 0;
    virtual bool setSimulationNodalVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;
    virtual bool getSimulationNodalKinematicTargets(const TensorDesc* dstTensor) const = 0;
    virtual bool setSimulationNodalKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) = 0;

    // rest shape - elements correspond 1:1 to simulation elements
    virtual bool getRestElementIndices(const TensorDesc* dstTensor) const = 0;
    virtual uint32_t getMaxRestNodesPerBody() const = 0;
    virtual bool getRestNodalPositions(const TensorDesc* dstTensor) const = 0;

    // collision mesh
    virtual uint32_t getMaxCollisionElementsPerBody() const = 0;
    virtual bool getCollisionElementIndices(const TensorDesc* dstTensor) const = 0;
    virtual uint32_t getMaxCollisionNodesPerBody() const = 0;
    virtual bool getCollisionNodalPositions(const TensorDesc* dstTensor) const = 0;
  
    // check view
    virtual bool check() const = 0;

    // release view
    virtual void release() = 0;

protected:
    virtual ~IDeformableBodyView() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
