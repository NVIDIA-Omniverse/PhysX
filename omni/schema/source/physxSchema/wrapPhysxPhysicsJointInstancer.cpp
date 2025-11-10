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
#include ".//physxPhysicsJointInstancer.h"
#include "pxr/usd/usd/schemaBase.h"

#include "pxr/usd/sdf/primSpec.h"

#include "pxr/usd/usd/pyConversions.h"
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
_CreatePhysicsBody0IndicesAttr(PhysxSchemaPhysxPhysicsJointInstancer &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePhysicsBody0IndicesAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->IntArray), writeSparsely);
}
        
static UsdAttribute
_CreatePhysicsBody1IndicesAttr(PhysxSchemaPhysxPhysicsJointInstancer &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePhysicsBody1IndicesAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->IntArray), writeSparsely);
}
        
static UsdAttribute
_CreatePhysicsLocalPos0sAttr(PhysxSchemaPhysxPhysicsJointInstancer &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePhysicsLocalPos0sAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Point3fArray), writeSparsely);
}
        
static UsdAttribute
_CreatePhysicsLocalRot0sAttr(PhysxSchemaPhysxPhysicsJointInstancer &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePhysicsLocalRot0sAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->QuathArray), writeSparsely);
}
        
static UsdAttribute
_CreatePhysicsLocalPos1sAttr(PhysxSchemaPhysxPhysicsJointInstancer &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePhysicsLocalPos1sAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->Point3fArray), writeSparsely);
}
        
static UsdAttribute
_CreatePhysicsLocalRot1sAttr(PhysxSchemaPhysxPhysicsJointInstancer &self,
                                      object defaultVal, bool writeSparsely) {
    return self.CreatePhysicsLocalRot1sAttr(
        UsdPythonToSdfType(defaultVal, SdfValueTypeNames->QuathArray), writeSparsely);
}

static std::string
_Repr(const PhysxSchemaPhysxPhysicsJointInstancer &self)
{
    std::string primRepr = TfPyRepr(self.GetPrim());
    return TfStringPrintf(
        "PhysxSchema.PhysxPhysicsJointInstancer(%s)",
        primRepr.c_str());
}

} // anonymous namespace

void wrapPhysxSchemaPhysxPhysicsJointInstancer()
{
    typedef PhysxSchemaPhysxPhysicsJointInstancer This;

    class_<This, bases<PhysxSchemaPhysxPhysicsInstancer> >
        cls("PhysxPhysicsJointInstancer");

    cls
        .def(init<UsdPrim>(arg("prim")))
        .def(init<UsdSchemaBase const&>(arg("schemaObj")))
        .def(TfTypePythonClass())

        .def("Get", &This::Get, (arg("stage"), arg("path")))
        .staticmethod("Get")

        .def("Define", &This::Define, (arg("stage"), arg("path")))
        .staticmethod("Define")

        .def("GetSchemaAttributeNames",
             &This::GetSchemaAttributeNames,
             arg("includeInherited")=true,
             return_value_policy<TfPySequenceToList>())
        .staticmethod("GetSchemaAttributeNames")

        .def("_GetStaticTfType", (TfType const &(*)()) TfType::Find<This>,
             return_value_policy<return_by_value>())
        .staticmethod("_GetStaticTfType")

        .def(!self)

        
        .def("GetPhysicsBody0IndicesAttr",
             &This::GetPhysicsBody0IndicesAttr)
        .def("CreatePhysicsBody0IndicesAttr",
             &_CreatePhysicsBody0IndicesAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetPhysicsBody1IndicesAttr",
             &This::GetPhysicsBody1IndicesAttr)
        .def("CreatePhysicsBody1IndicesAttr",
             &_CreatePhysicsBody1IndicesAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetPhysicsLocalPos0sAttr",
             &This::GetPhysicsLocalPos0sAttr)
        .def("CreatePhysicsLocalPos0sAttr",
             &_CreatePhysicsLocalPos0sAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetPhysicsLocalRot0sAttr",
             &This::GetPhysicsLocalRot0sAttr)
        .def("CreatePhysicsLocalRot0sAttr",
             &_CreatePhysicsLocalRot0sAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetPhysicsLocalPos1sAttr",
             &This::GetPhysicsLocalPos1sAttr)
        .def("CreatePhysicsLocalPos1sAttr",
             &_CreatePhysicsLocalPos1sAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))
        
        .def("GetPhysicsLocalRot1sAttr",
             &This::GetPhysicsLocalRot1sAttr)
        .def("CreatePhysicsLocalRot1sAttr",
             &_CreatePhysicsLocalRot1sAttr,
             (arg("defaultValue")=object(),
              arg("writeSparsely")=false))

        
        .def("GetPhysicsBody0sRel",
             &This::GetPhysicsBody0sRel)
        .def("CreatePhysicsBody0sRel",
             &This::CreatePhysicsBody0sRel)
        
        .def("GetPhysicsBody1sRel",
             &This::GetPhysicsBody1sRel)
        .def("CreatePhysicsBody1sRel",
             &This::CreatePhysicsBody1sRel)
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
