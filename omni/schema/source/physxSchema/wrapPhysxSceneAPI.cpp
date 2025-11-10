//
// Copyright 2016 Pixar
//
// Licensed under the Apache License, Version 2.0 (the "Apache License")
// with the following modification; you may not use this file except in
// compliance with the Apache License and the following modification to it:
// Section 6. Trademarks. is deleted and replaced with:
//
// 6. Trademarks. This License does not grant permission to use the trade
//    names, trademarks, service marks, or product names of the Licensor
//    and its affiliates, except as required to comply with Section 4(c) of
//    the License and to reproduce the content of the NOTICE file.
//
// You may obtain a copy of the Apache License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the Apache License with the above modification is
// distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied. See the Apache License for the specific
// language governing permissions and limitations under the Apache License.
//
#include ".//physxSceneAPI.h"
#include "pxr/usd/usd/schemaBase.h"

#include "pxr/usd/sdf/primSpec.h"

#include "pxr/usd/usd/pyConversions.h"
#include "pxr/base/tf/pyAnnotatedBoolResult.h"
#include "pxr/base/tf/pyContainerConversions.h"
#include "pxr/base/tf/pyResultConversions.h"
#include "pxr/base/tf/pyUtils.h"
#include "pxr/base/tf/wrapTypeHelpers.h"

#include <boost/python.hpp>

#include <string>

using namespace boost::python;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

#define WRAP_CUSTOM                                                     \
    template <class Cls> static void _CustomWrapCode(Cls &_class)

// fwd decl.
WRAP_CUSTOM;

        
static UsdAttribute
_CreateBounceThresholdAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateBounceThresholdAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateFrictionOffsetThresholdAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFrictionOffsetThresholdAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateFrictionCorrelationDistanceAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFrictionCorrelationDistanceAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateMaxBiasCoefficientAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxBiasCoefficientAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateCollisionSystemAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateCollisionSystemAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}
        
static UsdAttribute
_CreateSolverTypeAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSolverTypeAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}
        
static UsdAttribute
_CreateBroadphaseTypeAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateBroadphaseTypeAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}
        
static UsdAttribute
_CreateFrictionTypeAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFrictionTypeAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}
        
static UsdAttribute
_CreateEnableCCDAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateEnableCCDAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateEnableStabilizationAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateEnableStabilizationAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateUpdateTypeAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateUpdateTypeAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Token), writeSparsely);
}
        
static UsdAttribute
_CreateEnableGPUDynamicsAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateEnableGPUDynamicsAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateEnableEnhancedDeterminismAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateEnableEnhancedDeterminismAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateEnableResidualReportingAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateEnableResidualReportingAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateEnableExternalForcesEveryIterationAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateEnableExternalForcesEveryIterationAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateEnableSceneQuerySupportAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateEnableSceneQuerySupportAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateTimeStepsPerSecondAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateTimeStepsPerSecondAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt), writeSparsely);
}
        
static UsdAttribute
_CreateGpuTempBufferCapacityAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGpuTempBufferCapacityAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt64), writeSparsely);
}
        
static UsdAttribute
_CreateGpuMaxRigidContactCountAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGpuMaxRigidContactCountAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt), writeSparsely);
}
        
static UsdAttribute
_CreateGpuMaxRigidPatchCountAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGpuMaxRigidPatchCountAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt), writeSparsely);
}
        
static UsdAttribute
_CreateGpuHeapCapacityAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGpuHeapCapacityAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt), writeSparsely);
}
        
static UsdAttribute
_CreateGpuFoundLostPairsCapacityAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGpuFoundLostPairsCapacityAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt), writeSparsely);
}
        
static UsdAttribute
_CreateGpuFoundLostAggregatePairsCapacityAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGpuFoundLostAggregatePairsCapacityAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt), writeSparsely);
}
        
static UsdAttribute
_CreateGpuTotalAggregatePairsCapacityAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGpuTotalAggregatePairsCapacityAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt), writeSparsely);
}
        
static UsdAttribute
_CreateGpuMaxSoftBodyContactsAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGpuMaxSoftBodyContactsAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt), writeSparsely);
}
        
static UsdAttribute
_CreateGpuMaxDeformableSurfaceContactsAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGpuMaxDeformableSurfaceContactsAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt), writeSparsely);
}
        
static UsdAttribute
_CreateGpuMaxParticleContactsAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGpuMaxParticleContactsAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt), writeSparsely);
}
        
static UsdAttribute
_CreateGpuMaxNumPartitionsAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGpuMaxNumPartitionsAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt), writeSparsely);
}
        
static UsdAttribute
_CreateGpuCollisionStackSizeAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateGpuCollisionStackSizeAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt), writeSparsely);
}
        
static UsdAttribute
_CreateInvertCollisionGroupFilterAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateInvertCollisionGroupFilterAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateReportKinematicKinematicPairsAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateReportKinematicKinematicPairsAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateReportKinematicStaticPairsAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateReportKinematicStaticPairsAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateMinPositionIterationCountAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMinPositionIterationCountAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt), writeSparsely);
}
        
static UsdAttribute
_CreateMaxPositionIterationCountAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxPositionIterationCountAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt), writeSparsely);
}
        
static UsdAttribute
_CreateMinVelocityIterationCountAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMinVelocityIterationCountAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt), writeSparsely);
}
        
static UsdAttribute
_CreateMaxVelocityIterationCountAttr(PhysxSchemaPhysxSceneAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateMaxVelocityIterationCountAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->UInt), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxSceneAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxSceneAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxSceneAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxSceneAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxSceneAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxSceneAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxSceneAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxSceneAPI()
{
    typedef PhysxSchemaPhysxSceneAPI This;

    PhysxSchemaPhysxSceneAPI_CanApplyResult::Wrap<PhysxSchemaPhysxSceneAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxSceneAPI");

    cls
        .def(init<UsdPrim>(arg("prim")))
        .def(init<UsdSchemaBase const&>(arg("schemaObj")))
        .def(TfTypePythonClass())

        .def("Get", &This::Get, (arg("stage"), arg("path")))
        .staticmethod("Get")

        .def("CanApply", &_WrapCanApply, (arg("prim")))
        .staticmethod("CanApply")

        .def("Apply", &This::Apply, (arg("prim")))
        .staticmethod("Apply")

        .def("GetSchemaAttributeNames",
             &This::GetSchemaAttributeNames,
             arg("includeInherited")=true,
             return_value_policy<TfPySequenceToList>())
        .staticmethod("GetSchemaAttributeNames")

        .def("_GetStaticTfType", (TfType const &(*)()) TfType::Find<This>,
             return_value_policy<return_by_value>())
        .staticmethod("_GetStaticTfType")

        .def(!self)

        
        .def("GetBounceThresholdAttr",
             &This::GetBounceThresholdAttr)
        .def("CreateBounceThresholdAttr",
             &_CreateBounceThresholdAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetFrictionOffsetThresholdAttr",
             &This::GetFrictionOffsetThresholdAttr)
        .def("CreateFrictionOffsetThresholdAttr",
             &_CreateFrictionOffsetThresholdAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetFrictionCorrelationDistanceAttr",
             &This::GetFrictionCorrelationDistanceAttr)
        .def("CreateFrictionCorrelationDistanceAttr",
             &_CreateFrictionCorrelationDistanceAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxBiasCoefficientAttr",
             &This::GetMaxBiasCoefficientAttr)
        .def("CreateMaxBiasCoefficientAttr",
             &_CreateMaxBiasCoefficientAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetCollisionSystemAttr",
             &This::GetCollisionSystemAttr)
        .def("CreateCollisionSystemAttr",
             &_CreateCollisionSystemAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSolverTypeAttr",
             &This::GetSolverTypeAttr)
        .def("CreateSolverTypeAttr",
             &_CreateSolverTypeAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetBroadphaseTypeAttr",
             &This::GetBroadphaseTypeAttr)
        .def("CreateBroadphaseTypeAttr",
             &_CreateBroadphaseTypeAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetFrictionTypeAttr",
             &This::GetFrictionTypeAttr)
        .def("CreateFrictionTypeAttr",
             &_CreateFrictionTypeAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetEnableCCDAttr",
             &This::GetEnableCCDAttr)
        .def("CreateEnableCCDAttr",
             &_CreateEnableCCDAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetEnableStabilizationAttr",
             &This::GetEnableStabilizationAttr)
        .def("CreateEnableStabilizationAttr",
             &_CreateEnableStabilizationAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetUpdateTypeAttr",
             &This::GetUpdateTypeAttr)
        .def("CreateUpdateTypeAttr",
             &_CreateUpdateTypeAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetEnableGPUDynamicsAttr",
             &This::GetEnableGPUDynamicsAttr)
        .def("CreateEnableGPUDynamicsAttr",
             &_CreateEnableGPUDynamicsAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetEnableEnhancedDeterminismAttr",
             &This::GetEnableEnhancedDeterminismAttr)
        .def("CreateEnableEnhancedDeterminismAttr",
             &_CreateEnableEnhancedDeterminismAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetEnableResidualReportingAttr",
             &This::GetEnableResidualReportingAttr)
        .def("CreateEnableResidualReportingAttr",
             &_CreateEnableResidualReportingAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetEnableExternalForcesEveryIterationAttr",
             &This::GetEnableExternalForcesEveryIterationAttr)
        .def("CreateEnableExternalForcesEveryIterationAttr",
             &_CreateEnableExternalForcesEveryIterationAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetEnableSceneQuerySupportAttr",
             &This::GetEnableSceneQuerySupportAttr)
        .def("CreateEnableSceneQuerySupportAttr",
             &_CreateEnableSceneQuerySupportAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetTimeStepsPerSecondAttr",
             &This::GetTimeStepsPerSecondAttr)
        .def("CreateTimeStepsPerSecondAttr",
             &_CreateTimeStepsPerSecondAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetGpuTempBufferCapacityAttr",
             &This::GetGpuTempBufferCapacityAttr)
        .def("CreateGpuTempBufferCapacityAttr",
             &_CreateGpuTempBufferCapacityAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetGpuMaxRigidContactCountAttr",
             &This::GetGpuMaxRigidContactCountAttr)
        .def("CreateGpuMaxRigidContactCountAttr",
             &_CreateGpuMaxRigidContactCountAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetGpuMaxRigidPatchCountAttr",
             &This::GetGpuMaxRigidPatchCountAttr)
        .def("CreateGpuMaxRigidPatchCountAttr",
             &_CreateGpuMaxRigidPatchCountAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetGpuHeapCapacityAttr",
             &This::GetGpuHeapCapacityAttr)
        .def("CreateGpuHeapCapacityAttr",
             &_CreateGpuHeapCapacityAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetGpuFoundLostPairsCapacityAttr",
             &This::GetGpuFoundLostPairsCapacityAttr)
        .def("CreateGpuFoundLostPairsCapacityAttr",
             &_CreateGpuFoundLostPairsCapacityAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetGpuFoundLostAggregatePairsCapacityAttr",
             &This::GetGpuFoundLostAggregatePairsCapacityAttr)
        .def("CreateGpuFoundLostAggregatePairsCapacityAttr",
             &_CreateGpuFoundLostAggregatePairsCapacityAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetGpuTotalAggregatePairsCapacityAttr",
             &This::GetGpuTotalAggregatePairsCapacityAttr)
        .def("CreateGpuTotalAggregatePairsCapacityAttr",
             &_CreateGpuTotalAggregatePairsCapacityAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetGpuMaxSoftBodyContactsAttr",
             &This::GetGpuMaxSoftBodyContactsAttr)
        .def("CreateGpuMaxSoftBodyContactsAttr",
             &_CreateGpuMaxSoftBodyContactsAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetGpuMaxDeformableSurfaceContactsAttr",
             &This::GetGpuMaxDeformableSurfaceContactsAttr)
        .def("CreateGpuMaxDeformableSurfaceContactsAttr",
             &_CreateGpuMaxDeformableSurfaceContactsAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetGpuMaxParticleContactsAttr",
             &This::GetGpuMaxParticleContactsAttr)
        .def("CreateGpuMaxParticleContactsAttr",
             &_CreateGpuMaxParticleContactsAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetGpuMaxNumPartitionsAttr",
             &This::GetGpuMaxNumPartitionsAttr)
        .def("CreateGpuMaxNumPartitionsAttr",
             &_CreateGpuMaxNumPartitionsAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetGpuCollisionStackSizeAttr",
             &This::GetGpuCollisionStackSizeAttr)
        .def("CreateGpuCollisionStackSizeAttr",
             &_CreateGpuCollisionStackSizeAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetInvertCollisionGroupFilterAttr",
             &This::GetInvertCollisionGroupFilterAttr)
        .def("CreateInvertCollisionGroupFilterAttr",
             &_CreateInvertCollisionGroupFilterAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetReportKinematicKinematicPairsAttr",
             &This::GetReportKinematicKinematicPairsAttr)
        .def("CreateReportKinematicKinematicPairsAttr",
             &_CreateReportKinematicKinematicPairsAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetReportKinematicStaticPairsAttr",
             &This::GetReportKinematicStaticPairsAttr)
        .def("CreateReportKinematicStaticPairsAttr",
             &_CreateReportKinematicStaticPairsAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMinPositionIterationCountAttr",
             &This::GetMinPositionIterationCountAttr)
        .def("CreateMinPositionIterationCountAttr",
             &_CreateMinPositionIterationCountAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxPositionIterationCountAttr",
             &This::GetMaxPositionIterationCountAttr)
        .def("CreateMaxPositionIterationCountAttr",
             &_CreateMaxPositionIterationCountAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMinVelocityIterationCountAttr",
             &This::GetMinVelocityIterationCountAttr)
        .def("CreateMinVelocityIterationCountAttr",
             &_CreateMinVelocityIterationCountAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetMaxVelocityIterationCountAttr",
             &This::GetMaxVelocityIterationCountAttr)
        .def("CreateMaxVelocityIterationCountAttr",
             &_CreateMaxVelocityIterationCountAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))

        .def("__repr__", ::_Repr)
    ;

    _CustomWrapCode(cls);
}

// ===================================================================== //
// Feel free to add custom code below this line, it will be preserved by 
// the code generator.  The entry point for your custom code should look
// minimally like the following:
//
// WRAP_CUSTOM {
//     _class
//         .def("MyCustomMethod", ...)
//     ;
// }
//
// Of course any other ancillary or support code may be provided.
// 
// Just remember to wrap code in the appropriate delimiters:
// 'namespace {', '}'.
//
// ===================================================================== //
// --(BEGIN CUSTOM CODE)--

namespace {

WRAP_CUSTOM {
}

}
