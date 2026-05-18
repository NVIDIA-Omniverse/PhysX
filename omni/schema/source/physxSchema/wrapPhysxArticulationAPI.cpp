//
// Copyright 2016 Pixar
//
// Licensed under the terms set forth in the LICENSE.txt file available at
// https://openusd.org/license.
//
#include ".//physxArticulationAPI.h"
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
_CreateArticulationEnabledAttr(PhysxSchemaPhysxArticulationAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateArticulationEnabledAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}
        
static UsdAttribute
_CreateSolverPositionIterationCountAttr(PhysxSchemaPhysxArticulationAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSolverPositionIterationCountAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}
        
static UsdAttribute
_CreateSolverVelocityIterationCountAttr(PhysxSchemaPhysxArticulationAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSolverVelocityIterationCountAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Int), writeSparsely);
}
        
static UsdAttribute
_CreateSleepThresholdAttr(PhysxSchemaPhysxArticulationAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateSleepThresholdAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateStabilizationThresholdAttr(PhysxSchemaPhysxArticulationAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateStabilizationThresholdAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Float), writeSparsely);
}
        
static UsdAttribute
_CreateEnabledSelfCollisionsAttr(PhysxSchemaPhysxArticulationAPI &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreateEnabledSelfCollisionsAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Bool), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxArticulationAPI &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxArticulationAPI(%s)",
        primRepr.c_str());
}

struct PhysxSchemaPhysxArticulationAPI_CanApplyResult : 
    public TfPyAnnotatedBoolResult<std::string>
{
    PhysxSchemaPhysxArticulationAPI_CanApplyResult(bool val, std::string const &msg) :
        TfPyAnnotatedBoolResult<std::string>(val, msg) {}
};

static PhysxSchemaPhysxArticulationAPI_CanApplyResult
_WrapCanApply(const UsdPrim& prim)
{
    std::string whyNot;
    bool result = PhysxSchemaPhysxArticulationAPI::CanApply(prim, &whyNot);
    return PhysxSchemaPhysxArticulationAPI_CanApplyResult(result, whyNot);
}

} // anonymous namespace

void wrapPhysxSchemaPhysxArticulationAPI()
{
    typedef PhysxSchemaPhysxArticulationAPI This;

    PhysxSchemaPhysxArticulationAPI_CanApplyResult::Wrap<PhysxSchemaPhysxArticulationAPI_CanApplyResult>(
        "_CanApplyResult", "whyNot");

    class_<This, bases<UsdAPISchemaBase> >
        cls("PhysxArticulationAPI");

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

        
        .def("GetArticulationEnabledAttr",
             &This::GetArticulationEnabledAttr)
        .def("CreateArticulationEnabledAttr",
             &_CreateArticulationEnabledAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSolverPositionIterationCountAttr",
             &This::GetSolverPositionIterationCountAttr)
        .def("CreateSolverPositionIterationCountAttr",
             &_CreateSolverPositionIterationCountAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSolverVelocityIterationCountAttr",
             &This::GetSolverVelocityIterationCountAttr)
        .def("CreateSolverVelocityIterationCountAttr",
             &_CreateSolverVelocityIterationCountAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetSleepThresholdAttr",
             &This::GetSleepThresholdAttr)
        .def("CreateSleepThresholdAttr",
             &_CreateSleepThresholdAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetStabilizationThresholdAttr",
             &This::GetStabilizationThresholdAttr)
        .def("CreateStabilizationThresholdAttr",
             &_CreateStabilizationThresholdAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetEnabledSelfCollisionsAttr",
             &This::GetEnabledSelfCollisionsAttr)
        .def("CreateEnabledSelfCollisionsAttr",
             &_CreateEnabledSelfCollisionsAttr,
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
