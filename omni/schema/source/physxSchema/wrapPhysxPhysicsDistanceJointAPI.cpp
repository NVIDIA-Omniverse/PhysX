//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxPhysicsDistanceJointAPI.h"
#include "pxr/usd/usd/schemaBase.h"

#include "pxr/usd/sdf/primSpec.h"

#include "pxr/usd/usd/pyConversions.h"
#include "pxr/base/tf/pyAnnotatedBoolResult.h"
#include "pxr/base/tf/pyContainerConversions.h"
#include "pxr/base/tf/pyResultConversions.h"
#include "pxr/base/tf/pyUtils.h"
#include "pxr/base/tf/wrapTypeHelpers.h"

#include "pxr/external/boost/python.hpp"

#include <string>

PXR_NAMESPACE_USING_DIRECTIVE

using namespace pxr_boost::python;

namespace {

#define WRAP_CUSTOM                                                     \
    template <class Cls> static void _CustomWrapCode(Cls &_class)

// fwd decl.
WRAP_CUSTOM;

        
static UsdAttribute
_CreateSpringEnabledAttr(PhysxSchemaPhysxPhysicsDistanceJointAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSpringEnabledAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateSpringStiffnessAttr(PhysxSchemaPhysxPhysicsDistanceJointAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSpringStiffnessAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateSpringDampingAttr(PhysxSchemaPhysxPhysicsDistanceJointAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSpringDampingAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxPhysicsDistanceJointAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxPhysicsDistanceJointAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxPhysicsDistanceJointAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxPhysicsDistanceJointAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxPhysicsDistanceJointAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxPhysicsDistanceJointAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxPhysicsDistanceJointAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxPhysicsDistanceJointAPI()
{
    typedef PhysxSchemaPhysxPhysicsDistanceJointAPI This;

    PhysxSchemaPhysxPhysicsDistanceJointAPI_CanApplyResult::Wrap<PhysxSchemaPhysxPhysicsDistanceJointAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxPhysicsDistanceJointAPI");

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

        
        .def("GetSpringEnabledAttr",
             &This::GetSpringEnabledAttr)
        .def("CreateSpringEnabledAttr",
             &_CreateSpringEnabledAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSpringStiffnessAttr",
             &This::GetSpringStiffnessAttr)
        .def("CreateSpringStiffnessAttr",
             &_CreateSpringStiffnessAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSpringDampingAttr",
             &This::GetSpringDampingAttr)
        .def("CreateSpringDampingAttr",
             &_CreateSpringDampingAttr,
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
