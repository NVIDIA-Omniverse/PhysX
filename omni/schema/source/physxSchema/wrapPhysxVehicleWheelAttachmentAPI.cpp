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
#include ".//physxVehicleWheelAttachmentAPI.h"
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
_CreateIndexAttr(PhysxSchemaPhysxVehicleWheelAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateIndexAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}
        
static UsdAttribute
_CreateSuspensionTravelDirectionAttr(PhysxSchemaPhysxVehicleWheelAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSuspensionTravelDirectionAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Vector3f), writeSparsely);
}
        
static UsdAttribute
_CreateSuspensionFramePositionAttr(PhysxSchemaPhysxVehicleWheelAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSuspensionFramePositionAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Point3f), writeSparsely);
}
        
static UsdAttribute
_CreateSuspensionFrameOrientationAttr(PhysxSchemaPhysxVehicleWheelAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSuspensionFrameOrientationAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Quatf), writeSparsely);
}
        
static UsdAttribute
_CreateSuspensionForceAppPointOffsetAttr(PhysxSchemaPhysxVehicleWheelAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSuspensionForceAppPointOffsetAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float3), writeSparsely);
}
        
static UsdAttribute
_CreateWheelCenterOfMassOffsetAttr(PhysxSchemaPhysxVehicleWheelAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateWheelCenterOfMassOffsetAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float3), writeSparsely);
}
        
static UsdAttribute
_CreateTireForceAppPointOffsetAttr(PhysxSchemaPhysxVehicleWheelAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateTireForceAppPointOffsetAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float3), writeSparsely);
}
        
static UsdAttribute
_CreateWheelFramePositionAttr(PhysxSchemaPhysxVehicleWheelAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateWheelFramePositionAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Point3f), writeSparsely);
}
        
static UsdAttribute
_CreateWheelFrameOrientationAttr(PhysxSchemaPhysxVehicleWheelAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateWheelFrameOrientationAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Quatf), writeSparsely);
}
        
static UsdAttribute
_CreateDrivenAttr(PhysxSchemaPhysxVehicleWheelAttachmentAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDrivenAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxVehicleWheelAttachmentAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxVehicleWheelAttachmentAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxVehicleWheelAttachmentAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxVehicleWheelAttachmentAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxVehicleWheelAttachmentAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxVehicleWheelAttachmentAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxVehicleWheelAttachmentAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxVehicleWheelAttachmentAPI()
{
    typedef PhysxSchemaPhysxVehicleWheelAttachmentAPI This;

    PhysxSchemaPhysxVehicleWheelAttachmentAPI_CanApplyResult::Wrap<PhysxSchemaPhysxVehicleWheelAttachmentAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxVehicleWheelAttachmentAPI");

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

        
        .def("GetIndexAttr",
             &This::GetIndexAttr)
        .def("CreateIndexAttr",
             &_CreateIndexAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSuspensionTravelDirectionAttr",
             &This::GetSuspensionTravelDirectionAttr)
        .def("CreateSuspensionTravelDirectionAttr",
             &_CreateSuspensionTravelDirectionAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSuspensionFramePositionAttr",
             &This::GetSuspensionFramePositionAttr)
        .def("CreateSuspensionFramePositionAttr",
             &_CreateSuspensionFramePositionAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSuspensionFrameOrientationAttr",
             &This::GetSuspensionFrameOrientationAttr)
        .def("CreateSuspensionFrameOrientationAttr",
             &_CreateSuspensionFrameOrientationAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSuspensionForceAppPointOffsetAttr",
             &This::GetSuspensionForceAppPointOffsetAttr)
        .def("CreateSuspensionForceAppPointOffsetAttr",
             &_CreateSuspensionForceAppPointOffsetAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetWheelCenterOfMassOffsetAttr",
             &This::GetWheelCenterOfMassOffsetAttr)
        .def("CreateWheelCenterOfMassOffsetAttr",
             &_CreateWheelCenterOfMassOffsetAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetTireForceAppPointOffsetAttr",
             &This::GetTireForceAppPointOffsetAttr)
        .def("CreateTireForceAppPointOffsetAttr",
             &_CreateTireForceAppPointOffsetAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetWheelFramePositionAttr",
             &This::GetWheelFramePositionAttr)
        .def("CreateWheelFramePositionAttr",
             &_CreateWheelFramePositionAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetWheelFrameOrientationAttr",
             &This::GetWheelFrameOrientationAttr)
        .def("CreateWheelFrameOrientationAttr",
             &_CreateWheelFrameOrientationAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetDrivenAttr",
             &This::GetDrivenAttr)
        .def("CreateDrivenAttr",
             &_CreateDrivenAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))

        
        .def("GetWheelRel",
             &This::GetWheelRel)
        .def("CreateWheelRel",
             &This::CreateWheelRel)
        
        .def("GetTireRel",
             &This::GetTireRel)
        .def("CreateTireRel",
             &This::CreateTireRel)
        
        .def("GetSuspensionRel",
             &This::GetSuspensionRel)
        .def("CreateSuspensionRel",
             &This::CreateSuspensionRel)
        
        .def("GetCollisionGroupRel",
             &This::GetCollisionGroupRel)
        .def("CreateCollisionGroupRel",
             &This::CreateCollisionGroupRel)
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
