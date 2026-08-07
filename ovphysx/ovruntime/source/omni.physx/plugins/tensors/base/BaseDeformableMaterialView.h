// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "tensors/base/BaseSimulationData.h"
#include "tensors/GlobalsAreBad.h"
#include <omni/physics/tensors/IDeformableMaterialView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class BaseSimulationView;

class BaseDeformableMaterialView : public omni::physics::tensors::IDeformableMaterialView
{
public:
    BaseDeformableMaterialView(BaseSimulationView* sim, const std::vector<DeformableMaterialEntry>& entries);
    virtual ~BaseDeformableMaterialView() override;

    uint32_t getCount() const override;

    bool getDynamicFriction(const TensorDesc* dstTensor) const;
    bool setDynamicFriction(const TensorDesc* srcTensor, const TensorDesc* indexTensor);
    bool setDynamicFrictionMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;

    bool getYoungsModulus(const TensorDesc* dstTensor) const;
    bool setYoungsModulus(const TensorDesc* srcTensor, const TensorDesc* indexTensor);
    bool setYoungsModulusMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;

    bool getPoissonsRatio(const TensorDesc* dstTensor) const;
    bool setPoissonsRatio(const TensorDesc* srcTensor, const TensorDesc* indexTensor);
    bool setPoissonsRatioMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;

    bool getElasticityDamping(const TensorDesc* dstTensor) const override;
    bool setElasticityDamping(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setElasticityDampingMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;

    bool getBendingStiffness(const TensorDesc* dstTensor) const override;
    bool setBendingStiffness(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setBendingStiffnessMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;

    bool getThickness(const TensorDesc* dstTensor) const override;
    bool setThickness(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setThicknessMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;

    bool getBendingDamping(const TensorDesc* dstTensor) const override;
    bool setBendingDamping(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setBendingDampingMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;

    bool check() const override;
    void release() override;

    //
    // helpers
    //

    void _onParentRelease();

private:
    BaseSimulationView* mSim = nullptr;

protected:

    using MaterialSetter = void (::physx::PxDeformableMaterial::*)(::physx::PxReal);
    using MaterialGetter = ::physx::PxReal (::physx::PxDeformableMaterial::*)() const;
    using SurfaceMaterialSetter = void (::physx::PxDeformableSurfaceMaterial::*)(::physx::PxReal);
    using SurfaceMaterialGetter = ::physx::PxReal (::physx::PxDeformableSurfaceMaterial::*)() const;

    // Pointer to one of the indexed setX(srcTensor, indexTensor) members above.
    using IndexedSetter = bool (BaseDeformableMaterialView::*)(const TensorDesc*, const TensorDesc*);

    // Shared mask -> index-tensor resolution for the setXMasked() wrappers; forwards to indexedSetter.
    bool applyMasked(const TensorDesc* srcTensor,
                     const TensorDesc* maskTensor,
                     const IndexedSetter indexedSetter,
                     const char* callerFunctionName);

    bool getProperty(const TensorDesc* dstTensor,
                     const MaterialGetter materialGetter,
                     const char* parameterName,
                     const char* callerFunctionName) const;

    bool setProperty(const TensorDesc* srcTensor,
                     const TensorDesc* indexTensor,
                     const MaterialSetter materialSetter,
                     const char* parameterName,
                     const char* callerFunctionName);

    // Helpers for surface-only properties: entries whose isSurface==false produce 0.0 on get and are skipped on set.
    bool getSurfaceProperty(const TensorDesc* dstTensor,
                            const SurfaceMaterialGetter getter,
                            const char* parameterName,
                            const char* callerFunctionName) const;

    bool setSurfaceProperty(const TensorDesc* srcTensor,
                            const TensorDesc* indexTensor,
                            const SurfaceMaterialSetter setter,
                            const char* parameterName,
                            const char* callerFunctionName);

    BaseSimulationDataPtr mSimData;
    std::vector<DeformableMaterialEntry> mEntries;
    std::vector<uint32_t> mAllIndices;
};

} // namespace tensors
} // namespace physx
} // namespace omni
