// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


//
// The macro logic in this header (and the headers CmOmniPvdAutoGenRegisterData.h and 
// and CmOmniPvdAutoGenSetData.h) is meant as a helper to automatically generate a
// structure that stores all PVD class and attribute handles for a module, handles the
// registration logic and adds methods for object creation, setting attribute
// values etc. At the core of the generation logic is a user defined header file
// that describes the classes and attributes as follows:
//
// OMNI_PVD_CLASS_BEGIN(MyClass1)
// OMNI_PVD_ATTRIBUTE(MyClass1, myAttr1, PxReal, OmniPvdDataType::eFLOAT32)
// OMNI_PVD_ATTRIBUTE(MyClass1, myAttr2, PxReal, OmniPvdDataType::eFLOAT32)
// OMNI_PVD_CLASS_END(MyClass1)
//
// OMNI_PVD_CLASS_UNTYPED_BEGIN(MyClass2)
// OMNI_PVD_ATTRIBUTE(MyClass2, myAttr1, PxU32, OmniPvdDataType::eUINT32)
// OMNI_PVD_CLASS_END(MyClass2)
//
// The structure to create from this description will look somewhat like this:
//
// struct MyModulePvdObjectsDescriptor
// {
//
//   struct PvdMyClass1
//   {
//     typedef MyClass1 ObjectType;
//     static OmniPvdObjectHandle getObjectHandle(const ObjectType& objectRef) { return reinterpret_cast<OmniPvdObjectHandle>(&objectRef); }
//
//     OmniPvdClassHandle classHandle;
//
//     void createInstance(OmniPvdWriter& writer, OmniPvdContextHandle contextHandle, const ObjectType& objectRef) const
//     {
//       writer.createObject(contextHandle, classHandle, getObjectHandle(objectRef), NULL);
//     }
//
//     static void destroyInstance(OmniPvdWriter& writer, OmniPvdContextHandle contextHandle, const ObjectType& objectRef)
//     {
//       writer.destroyObject(contextHandle, getObjectHandle(objectRef));
//     }
//
//     OmniPvdAttributeHandle myAttr1;
//     void set_myAttr1_(OmniPvdWriter& writer, OmniPvdContextHandle contextHandle, const ObjectType& objectRef, const PxReal& value) const
//     {
//       writer.setAttribute(contextHandle, getObjectHandle(objectRef), myAttr1, reinterpret_cast<const uint8_t*>(&value), getOmniPvdDataTypeSize<OmniPvdDataType::eFLOAT32>());
//     }
//
//     OmniPvdAttributeHandle myAttr2;
//     void set_myAttr2_(OmniPvdWriter& writer, OmniPvdContextHandle contextHandle, const ObjectType& objectRef, const PxReal& value) const
//     {
//       writer.setAttribute(contextHandle, getObjectHandle(objectRef), myAttr2, reinterpret_cast<const uint8_t*>(&value), getOmniPvdDataTypeSize<OmniPvdDataType::eFLOAT32>());
//     }
//   };
//   PvdMyClass1 pvdMyClass1;
//
//
//   struct PvdMyClass2
//   {
//     typedef OmniPvdObjectHandle ObjectType;
//     static OmniPvdObjectHandle getObjectHandle(const ObjectType& objectHandle) { return objectHandle; }
//
//     OmniPvdClassHandle classHandle;
//
//     void createInstance(OmniPvdWriter& writer, OmniPvdContextHandle contextHandle, const ObjectType& objectRef) const
//     {
//       writer.createObject(contextHandle, classHandle, getObjectHandle(objectRef), NULL);
//     }
//
//     static void destroyInstance(OmniPvdWriter& writer, OmniPvdContextHandle contextHandle, const ObjectType& objectRef)
//     {
//       writer.destroyObject(contextHandle, getObjectHandle(objectRef));
//     }
//
//     OmniPvdAttributeHandle myAttr1;
//     void set_myAttr1_(OmniPvdWriter& writer, OmniPvdContextHandle contextHandle, const ObjectType& objectRef, const PxU32& value) const
//     {
//       writer.setAttribute(contextHandle, getObjectHandle(objectRef), myAttr1, reinterpret_cast<const uint8_t*>(&value), getOmniPvdDataTypeSize<OmniPvdDataType::eUINT32>());
//     }
//   };
//   PvdMyClass2 pvdMyClass2;
//
//
//   void myRegisterDataMethod(OmniPvdWriter& writer)
//   {
//     pvdMyClass1.classHandle = writer.registerClass("MyClass1");
//     pvdMyClass1.myAttr1 = writer.registerAttribute(pvdMyClass1.classHandle, "myAttr1", OmniPvdDataType::eFLOAT32, 1);
//     pvdMyClass1.myAttr2 = writer.registerAttribute(pvdMyClass1.classHandle, "myAttr2", OmniPvdDataType::eFLOAT32, 1);
//
//     pvdMyClass2.classHandle = writer.registerClass("MyClass2");
//     pvdMyClass2.myAttr1 = writer.registerAttribute(pvdMyClass2.classHandle, "myAttr1", OmniPvdDataType::eUINT32, 1);
//   }
//
// };
//
// Assuming the class and attribute definitions are in a file called MyModulePvdObjectDefinitions.h, 
// the described structure can be generated like this:
//
// struct MyModulePvdObjectsDescriptor
// {
//
//   #include "CmOmniPvdAutoGenCreateRegistrationStruct.h"
//   #include "MyModulePvdObjectDefinitions.h"
//   #include "CmOmniPvdAutoGenClearDefines.h"
//
//   // custom registration data related members that are not auto-generated can go here, for example
//
//
//   void myRegisterDataMethod(OmniPvdWriter& writer)
//   {
//     #define OMNI_PVD_WRITER_VAR writer
//
//     #include "CmOmniPvdAutoGenRegisterData.h"
//     #include "MyModulePvdObjectDefinitions.h"
//     #include "CmOmniPvdAutoGenClearDefines.h"
//
//     // custom registration code that is not auto-generated can go here too
//
//     #undef OMNI_PVD_WRITER_VAR
//   }
// };
//
// As can be seen, CmOmniPvdAutoGenCreateRegistrationStruct.h is responsible for generating the structs,
// members and setter methods. CmOmniPvdAutoGenRegisterData.h is responsible for generating the registration
// code (note that defining OMNI_PVD_WRITER_VAR is important in this context since it is used inside
// CmOmniPvdAutoGenRegisterData.h)
// 
// Note that it is the user's responsibility to include the necessary headers before applying these helpers
// (for example, OmniPvdDefines.h etc.).
//
// Last but not least, the helpers in CmOmniPvdAutoGenSetData.h provide a way to use this structure to
// set values of attributes, create class instances etc. An example usage is shown below:
//
// OmniPvdContextHandle contextHandle;  // assuming this holds the context the objects belong to
// MyClass1 myClass1Instance;
// PxReal value;  // assuming this holds the value to set the attribute to
//
// OMNI_PVD_CREATE(contextHandle, MyClass1, myClass1Instance);
// OMNI_PVD_SET(contextHandle, MyClass1, myAttr1, myClass1Instance, value);
//
// To use these helper macros, the following things need to be defined before including CmOmniPvdAutoGenSetData.h:
//
// #define OMNI_PVD_GET_WRITER(writer)
// OmniPvdWriter* writer = GetPvdWriterForMyModule();
//
// #define OMNI_PVD_GET_REGISTRATION_DATA(regData)
// MyModulePvdObjectsDescriptor* regData = GetPvdObjectsDescForMyModule();
//
// #include "CmOmniPvdAutoGenSetData.h"
//
// GetPvdWriterForMyModule() and GetPvdObjectsDescForMyModule() just stand for the logic the user needs
// to provide to access the OmniPvdWriter object and the generated description structure. In the given example,
// the variables "writer" and "regData" need to be assigned but the code to do so will be user specific.
//
//


#define OMNI_PVD_CLASS_INTERNALS                                                                                               \
                                                                                                                               \
OmniPvdClassHandle classHandle;                                                                                                \
                                                                                                                               \
void createInstance(OmniPvdWriter& writer, OmniPvdContextHandle contextHandle, const ObjectType& objectRef) const              \
{                                                                                                                              \
    writer.createObject(contextHandle, classHandle, getObjectHandle(objectRef), NULL);                                         \
}                                                                                                                              \
                                                                                                                               \
static void destroyInstance(OmniPvdWriter& writer, OmniPvdContextHandle contextHandle, const ObjectType& objectRef)            \
{                                                                                                                              \
    writer.destroyObject(contextHandle, getObjectHandle(objectRef));                                                           \
}


//
// Define a PVD class.
//
// Note: has to be paired with OMNI_PVD_CLASS_END
//
// classID: name of the class to register in PVD (note: has to be an existing C++ class)
//
#define OMNI_PVD_CLASS_BEGIN(classID)                                                                                                 \
                                                                                                                                      \
struct Pvd##classID                                                                                                                   \
{                                                                                                                                     \
typedef classID ObjectType;                                                                                                           \
                                                                                                                                      \
static OmniPvdObjectHandle getObjectHandle(const ObjectType& objectRef) { return reinterpret_cast<OmniPvdObjectHandle>(&objectRef); } \
                                                                                                                                      \
OMNI_PVD_CLASS_INTERNALS


//
// Define a PVD class that is derived from another class.
//
// Note: has to be paired with OMNI_PVD_CLASS_END
// 
// classID: see OMNI_PVD_CLASS_BEGIN
// baseClassID: the name of the class to derive from
//
#define OMNI_PVD_CLASS_DERIVED_BEGIN(classID, baseClassID) OMNI_PVD_CLASS_BEGIN(classID)


//
// Define a PVD class.
//
// Note: has to be paired with OMNI_PVD_CLASS_END
//
// classID: name of the class to register in PVD (note: the class does not need to match an actually existing
//          class but still needs to follow C++ naming conventions)
//
#define OMNI_PVD_CLASS_UNTYPED_BEGIN(classID)                                                                     \
                                                                                                                  \
struct Pvd##classID                                                                                               \
{                                                                                                                 \
typedef OmniPvdObjectHandle ObjectType;                                                                           \
                                                                                                                  \
static OmniPvdObjectHandle getObjectHandle(const ObjectType& objectHandle) { return objectHandle; }               \
                                                                                                                  \
OMNI_PVD_CLASS_INTERNALS


//
// Define a PVD class that is derived from another class.
//
// Note: has to be paired with OMNI_PVD_CLASS_END
// 
// classID: see OMNI_PVD_CLASS_UNTYPED_BEGIN
// baseClassID: the name of the class to derive from
//
#define OMNI_PVD_CLASS_UNTYPED_DERIVED_BEGIN(classID, baseClassID) OMNI_PVD_CLASS_UNTYPED_BEGIN(classID)


//
// See OMNI_PVD_CLASS_BEGIN for more info.
//
#define OMNI_PVD_CLASS_END(classID) \
                                    \
};                                  \
Pvd##classID pvd##classID;


//
// Define a PVD enum class.
//
// Note: has to be paired with OMNI_PVD_ENUM_END
//
// enumID: name of the enum class (has to follow C++ naming conventions)
//
#define OMNI_PVD_ENUM_BEGIN(enumID) \
                                    \
struct Pvd##enumID                  \
{                                   \
OmniPvdClassHandle classHandle;


//
// See OMNI_PVD_ENUM_BEGIN
//
#define OMNI_PVD_ENUM_END(enumID) OMNI_PVD_CLASS_END(enumID)


//
// Define a simple PVD attribute.
//
// Note: needs to be placed between a OMNI_PVD_CLASS_BEGIN, OMNI_PVD_CLASS_END
//       sequence
//
// classID: name of the class to add the attribute to (see OMNI_PVD_CLASS_BEGIN)
// attributeID: name of the attribute (has to follow C++ naming conventions)
// valueType: attribute data type (int, float etc.)
// pvdDataType: PVD attribute data type (see OmniPvdDataType)
//
#define OMNI_PVD_ATTRIBUTE(classID, attributeID, valueType, pvdDataType)                                                                                          \
                                                                                                                                                                  \
OmniPvdAttributeHandle attributeID;                                                                                                                               \
void set_##attributeID##_(OmniPvdWriter& writer, OmniPvdContextHandle contextHandle, const ObjectType& objectRef, const valueType& value) const                   \
{                                                                                                                                                                 \
    PX_ASSERT(sizeof(valueType) == getOmniPvdDataTypeSize<pvdDataType>());                                                                                        \
	writer.setAttribute(contextHandle, getObjectHandle(objectRef), attributeID, reinterpret_cast<const uint8_t*>(&value), getOmniPvdDataTypeSize<pvdDataType>()); \
}


//
// Define a fixed size multi-value PVD attribute.
//
// Note: needs to be placed between a OMNI_PVD_CLASS_BEGIN, OMNI_PVD_CLASS_END
//       sequence
//
// The attribute is a fixed size array of values of the given pvd data type.
//
// entryCount: number of entries the array will hold.
// 
// See OMNI_PVD_ATTRIBUTE for the other parameters. Note that valueType is
// expected to hold a type that matches the size of the whole array, i.e.,
// sizeof(valueType) == entryCount * getOmniPvdDataTypeSize<pvdDataType>()
//
#define OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE(classID, attributeID, valueType, pvdDataType, entryCount)                                           \
                                                                                                                                                \
OmniPvdAttributeHandle attributeID;                                                                                                             \
void set_##attributeID##_(OmniPvdWriter& writer, OmniPvdContextHandle contextHandle, const ObjectType& objectRef, const valueType& value) const \
{                                                                                                                                               \
    const uint32_t byteSize = static_cast<uint32_t>(sizeof(valueType));                                                                         \
    PX_ASSERT(byteSize == (entryCount * getOmniPvdDataTypeSize<pvdDataType>()));                                                                \
	writer.setAttribute(contextHandle, getObjectHandle(objectRef), attributeID, reinterpret_cast<const uint8_t*>(&value), byteSize);            \
}


//
// Define a variable size multi-value PVD attribute.
//
// Note: needs to be placed between a OMNI_PVD_CLASS_BEGIN, OMNI_PVD_CLASS_END
//       sequence
//
// The attribute is a variable size array of values of the given pvd data type.
// 
// See OMNI_PVD_ATTRIBUTE for a parameter description. Note that valueType is expected
// to define the type of a single array element, for example, int for an integer array.
//
#define OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE(classID, attributeID, valueType, pvdDataType)                                                                          \
                                                                                                                                                                      \
OmniPvdAttributeHandle attributeID;                                                                                                                                   \
void set_##attributeID##_(OmniPvdWriter& writer, OmniPvdContextHandle contextHandle, const ObjectType& objectRef, const valueType* values, uint32_t valueCount) const \
{                                                                                                                                                                     \
    const uint32_t byteSize = valueCount * getOmniPvdDataTypeSize<pvdDataType>();                                                                                     \
    writer.setAttribute(contextHandle, getObjectHandle(objectRef), attributeID, reinterpret_cast<const uint8_t*>(values), byteSize);                                  \
}


//
// Define a string PVD attribute.
//
// Note: needs to be placed between a OMNI_PVD_CLASS_BEGIN, OMNI_PVD_CLASS_END
//       sequence
// 
// See OMNI_PVD_ATTRIBUTE for a parameter description.
//
#define OMNI_PVD_ATTRIBUTE_STRING(classID, attributeID)                                                                                                          \
                                                                                                                                                                 \
OmniPvdAttributeHandle attributeID;                                                                                                                              \
void set_##attributeID##_(OmniPvdWriter& writer, OmniPvdContextHandle contextHandle, const ObjectType& objectRef, const char* values, uint32_t valueCount) const \
{                                                                                                                                                                \
    const uint32_t byteSize = valueCount;                                                                                                                        \
    writer.setAttribute(contextHandle, getObjectHandle(objectRef), attributeID, reinterpret_cast<const uint8_t*>(values), byteSize);                             \
}


//
// Define a unique list PVD attribute.
//
// Note: needs to be placed between a OMNI_PVD_CLASS_BEGIN, OMNI_PVD_CLASS_END
//       sequence
// 
// See OMNI_PVD_ATTRIBUTE for a parameter description. Note that valueType is expected
// to define the class the list will hold pointers to. If it shall hold pointers to
// instances of class MyClass, then the valueType is MyClass.
//
#define OMNI_PVD_ATTRIBUTE_UNIQUE_LIST(classID, attributeID, valueType)                                                                                \
                                                                                                                                                       \
OmniPvdAttributeHandle attributeID;                                                                                                                    \
                                                                                                                                                       \
void addTo_##attributeID##_(OmniPvdWriter& writer, OmniPvdContextHandle contextHandle, const ObjectType& objectRef, const valueType& value) const      \
{                                                                                                                                                      \
    const OmniPvdObjectHandle objHandle = reinterpret_cast<OmniPvdObjectHandle>(&value);                                                               \
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&objHandle);                                                                                 \
    writer.addToUniqueListAttribute(contextHandle, getObjectHandle(objectRef), attributeID, ptr, sizeof(OmniPvdObjectHandle));                         \
}                                                                                                                                                      \
                                                                                                                                                       \
void removeFrom_##attributeID##_(OmniPvdWriter& writer, OmniPvdContextHandle contextHandle, const ObjectType& objectRef, const valueType& value) const \
{                                                                                                                                                      \
    const OmniPvdObjectHandle objHandle = reinterpret_cast<OmniPvdObjectHandle>(&value);                                                               \
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&objHandle);                                                                                 \
    writer.removeFromUniqueListAttribute(contextHandle, getObjectHandle(objectRef), attributeID, ptr, sizeof(OmniPvdObjectHandle));                    \
}


//
// Define a flag PVD attribute.
//
// Note: needs to be placed between a OMNI_PVD_CLASS_BEGIN, OMNI_PVD_CLASS_END
//       sequence
// 
// enumType: the enum type this attribute refers to
// enumID: the name of the enum class that describes the enum (see OMNI_PVD_ENUM_BEGIN)
//
// See OMNI_PVD_ATTRIBUTE for the other parameters.
//
#define OMNI_PVD_ATTRIBUTE_FLAG(classID, attributeID, enumType, enumID)                                                                        \
                                                                                                                                               \
OmniPvdAttributeHandle attributeID;                                                                                                            \
void set_##attributeID##_(OmniPvdWriter& writer, OmniPvdContextHandle contextHandle, const ObjectType& objectRef, const enumType& value) const \
{                                                                                                                                              \
	writer.setAttribute(contextHandle, getObjectHandle(objectRef), attributeID, reinterpret_cast<const uint8_t*>(&value), sizeof(enumType));   \
}


//
// Define an enum entry.
//
// Note: needs to be placed between a OMNI_PVD_ENUM_BEGIN, OMNI_PVD_ENUM_END
//       sequence
//
// enumID: name of the enum class to add an entry to (see OMNI_PVD_ENUM_BEGIN)
// enumEntryID: the name of the enum entry to add to the enum class (has to follow C++ naming conventions)
// value: the enum value
//
#define OMNI_PVD_ENUM_VALUE_EXPLICIT(enumID, enumEntryID, value)


//
// Define an enum entry.
//
// Note: needs to be placed between a OMNI_PVD_ENUM_BEGIN, OMNI_PVD_ENUM_END
//       sequence
// 
// See OMNI_PVD_ENUM_VALUE_EXPLICIT for a description of the parameters. This shorter form expects the enum to
// have a C++ definition of the form:
//
//       struct <enumID>
//       {
//         enum Enum
//         {
//           <enumEntryID> = ...
//         }
//       }
//
// such that the value can be derived using: <enumID>::<enumEntryID>
//
#define OMNI_PVD_ENUM_VALUE(enumID, enumEntryID) \
                                                 \
OMNI_PVD_ENUM_VALUE_EXPLICIT(enumID, enumEntryID, enumID::enumEntryID)
