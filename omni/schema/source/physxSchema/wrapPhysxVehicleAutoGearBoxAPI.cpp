//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxVehicleAutoGearBoxAPI.h"
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
_CreateUpRatiosAttr(PhysxSchemaPhysxVehicleAutoGearBoxAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateUpRatiosAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->FloatArray), writeSparsely);
}
        
static UsdAttribute
_CreateDownRatiosAttr(PhysxSchemaPhysxVehicleAutoGearBoxAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateDownRatiosAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->FloatArray), writeSparsely);
}
        
static UsdAttribute
_CreateLatencyAttr(PhysxSchemaPhysxVehicleAutoGearBoxAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateLatencyAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxVehicleAutoGearBoxAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxVehicleAutoGearBoxAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxVehicleAutoGearBoxAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxVehicleAutoGearBoxAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxVehicleAutoGearBoxAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxVehicleAutoGearBoxAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxVehicleAutoGearBoxAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxVehicleAutoGearBoxAPI()
{
    typedef PhysxSchemaPhysxVehicleAutoGearBoxAPI This;

    PhysxSchemaPhysxVehicleAutoGearBoxAPI_CanApplyResult::Wrap<PhysxSchemaPhysxVehicleAutoGearBoxAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxVehicleAutoGearBoxAPI");

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

        
        .def("GetUpRatiosAttr",
             &This::GetUpRatiosAttr)
        .def("CreateUpRatiosAttr",
             &_CreateUpRatiosAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetDownRatiosAttr",
             &This::GetDownRatiosAttr)
        .def("CreateDownRatiosAttr",
             &_CreateDownRatiosAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetLatencyAttr",
             &This::GetLatencyAttr)
        .def("CreateLatencyAttr",
             &_CreateLatencyAttr,
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
