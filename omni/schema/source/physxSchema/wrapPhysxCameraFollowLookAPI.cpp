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
#include ".//physxCameraFollowLookAPI.h"
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
_CreateDownHillGroundAngleAttr(PhysxSchemaPhysxCameraFollowLookAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDownHillGroundAngleAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateDownHillGroundPitchAttr(PhysxSchemaPhysxCameraFollowLookAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDownHillGroundPitchAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateUpHillGroundAngleAttr(PhysxSchemaPhysxCameraFollowLookAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateUpHillGroundAngleAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateUpHillGroundPitchAttr(PhysxSchemaPhysxCameraFollowLookAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateUpHillGroundPitchAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateVelocityBlendTimeConstantAttr(PhysxSchemaPhysxCameraFollowLookAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateVelocityBlendTimeConstantAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateFollowReverseSpeedAttr(PhysxSchemaPhysxCameraFollowLookAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFollowReverseSpeedAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateFollowReverseDistanceAttr(PhysxSchemaPhysxCameraFollowLookAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateFollowReverseDistanceAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxCameraFollowLookAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxCameraFollowLookAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxCameraFollowLookAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxCameraFollowLookAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxCameraFollowLookAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxCameraFollowLookAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxCameraFollowLookAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxCameraFollowLookAPI()
{
    typedef PhysxSchemaPhysxCameraFollowLookAPI This;

    PhysxSchemaPhysxCameraFollowLookAPI_CanApplyResult::Wrap<PhysxSchemaPhysxCameraFollowLookAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxCameraFollowLookAPI");

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

        
        .def("GetDownHillGroundAngleAttr",
             &This::GetDownHillGroundAngleAttr)
        .def("CreateDownHillGroundAngleAttr",
             &_CreateDownHillGroundAngleAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetDownHillGroundPitchAttr",
             &This::GetDownHillGroundPitchAttr)
        .def("CreateDownHillGroundPitchAttr",
             &_CreateDownHillGroundPitchAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetUpHillGroundAngleAttr",
             &This::GetUpHillGroundAngleAttr)
        .def("CreateUpHillGroundAngleAttr",
             &_CreateUpHillGroundAngleAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetUpHillGroundPitchAttr",
             &This::GetUpHillGroundPitchAttr)
        .def("CreateUpHillGroundPitchAttr",
             &_CreateUpHillGroundPitchAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetVelocityBlendTimeConstantAttr",
             &This::GetVelocityBlendTimeConstantAttr)
        .def("CreateVelocityBlendTimeConstantAttr",
             &_CreateVelocityBlendTimeConstantAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetFollowReverseSpeedAttr",
             &This::GetFollowReverseSpeedAttr)
        .def("CreateFollowReverseSpeedAttr",
             &_CreateFollowReverseSpeedAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetFollowReverseDistanceAttr",
             &This::GetFollowReverseDistanceAttr)
        .def("CreateFollowReverseDistanceAttr",
             &_CreateFollowReverseDistanceAttr,
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
