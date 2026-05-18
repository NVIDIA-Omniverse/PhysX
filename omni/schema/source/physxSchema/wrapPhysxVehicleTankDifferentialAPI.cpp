//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxVehicleTankDifferentialAPI.h"
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
_CreateNumberOfWheelsPerTrackAttr(PhysxSchemaPhysxVehicleTankDifferentialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateNumberOfWheelsPerTrackAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->IntArray), writeSparsely);
}
        
static UsdAttribute
_CreateThrustIndexPerTrackAttr(PhysxSchemaPhysxVehicleTankDifferentialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateThrustIndexPerTrackAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->IntArray), writeSparsely);
}
        
static UsdAttribute
_CreateTrackToWheelIndicesAttr(PhysxSchemaPhysxVehicleTankDifferentialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateTrackToWheelIndicesAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->IntArray), writeSparsely);
}
        
static UsdAttribute
_CreateWheelIndicesInTrackOrderAttr(PhysxSchemaPhysxVehicleTankDifferentialAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateWheelIndicesInTrackOrderAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->IntArray), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxVehicleTankDifferentialAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxVehicleTankDifferentialAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxVehicleTankDifferentialAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxVehicleTankDifferentialAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxVehicleTankDifferentialAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxVehicleTankDifferentialAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxVehicleTankDifferentialAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxVehicleTankDifferentialAPI()
{
    typedef PhysxSchemaPhysxVehicleTankDifferentialAPI This;

    PhysxSchemaPhysxVehicleTankDifferentialAPI_CanApplyResult::Wrap<PhysxSchemaPhysxVehicleTankDifferentialAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxVehicleTankDifferentialAPI");

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

        
        .def("GetNumberOfWheelsPerTrackAttr",
             &This::GetNumberOfWheelsPerTrackAttr)
        .def("CreateNumberOfWheelsPerTrackAttr",
             &_CreateNumberOfWheelsPerTrackAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetThrustIndexPerTrackAttr",
             &This::GetThrustIndexPerTrackAttr)
        .def("CreateThrustIndexPerTrackAttr",
             &_CreateThrustIndexPerTrackAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetTrackToWheelIndicesAttr",
             &This::GetTrackToWheelIndicesAttr)
        .def("CreateTrackToWheelIndicesAttr",
             &_CreateTrackToWheelIndicesAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetWheelIndicesInTrackOrderAttr",
             &This::GetWheelIndicesInTrackOrderAttr)
        .def("CreateWheelIndicesInTrackOrderAttr",
             &_CreateWheelIndicesInTrackOrderAttr,
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
