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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


//
// This header provides macros to register PVD object instances, to set PVD attribute
// values etc. This only works in combination with a registration structure that was
// defined using the logic in CmOmniPvdAutoGenCreateRegistrationStruct.h.
// OMNI_PVD_GET_WRITER and OMNI_PVD_GET_REGISTRATION_DATA have to be defined before
// including this header. These two macros need to fetch and assign the pointer to
// the OmniPvdWriter instance and the registration structure instance respectively.
// See CmOmniPvdAutoGenCreateRegistrationStruct.h for a more detailed overview of the
// whole approach.
//


#if PX_SUPPORT_OMNI_PVD

//
// It is recommended to use this macro when multiple PVD attributes get written
// in one go since the writer and registration structure is then fetched once only.
//
// Note: has to be paired with OMNI_PVD_WRITE_SCOPE_END
//
// writer: a pointer to the OmniPvdWriter instance will get assigned to a variable
//         named "writer"
// regData: a pointer to the registration structure instance will get assigned to
//          a variable named "regData"
//
// General usage would look like this:
//
// OMNI_PVD_WRITE_SCOPE_BEGIN(writer, regData)
// OMNI_PVD_SET_EXPLICIT(writer, regData, ...)
// OMNI_PVD_SET_EXPLICIT(writer, regData, ...)
// ...
// OMNI_PVD_WRITE_SCOPE_END
//
#define OMNI_PVD_WRITE_SCOPE_BEGIN(writer, regData) \
                                                    \
OMNI_PVD_GET_WRITER(writer)                         \
if (writer != NULL)                                 \
{                                                   \
OMNI_PVD_GET_REGISTRATION_DATA(regData)

//
// See OMNI_PVD_WRITE_SCOPE_BEGIN for more info.
//
#define OMNI_PVD_WRITE_SCOPE_END \
                                 \
}


//
// Create a PVD object instance using the provided pointers to the writer and registration
// structure instance.
// 
// See OMNI_PVD_SET_EXPLICIT and OMNI_PVD_SET for a description of the parameters.
//
#define OMNI_PVD_CREATE_EXPLICIT(writer, regData, contextHandle, classID, objectRef) \
                                                                                     \
PX_ASSERT(writer);                                                                   \
PX_ASSERT(regData);                                                                  \
regData->pvd##classID.createInstance(*writer, contextHandle, objectRef);


//
// Create a PVD object instance.
//
// Note: if attribute values are to be set directly after the object instance registration,
//       it is recommended to use OMNI_PVD_WRITE_SCOPE_BEGIN & OMNI_PVD_CREATE_EXPLICIT etc. instead
//
// See OMNI_PVD_SET_EXPLICIT and OMNI_PVD_SET for a description of the parameters.
//
#define OMNI_PVD_CREATE(contextHandle, classID, objectRef)                          \
                                                                                    \
{                                                                                   \
OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)                                   \
OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, contextHandle, classID, objectRef); \
OMNI_PVD_WRITE_SCOPE_END                                                            \
}


//
// Destroy a PVD object instance using the provided pointer to the writer instance.
//
// See OMNI_PVD_SET_EXPLICIT and OMNI_PVD_SET for a description of the parameters.
//
#define OMNI_PVD_DESTROY_EXPLICIT(writer, regData, contextHandle, classID, objectRef) \
                                                                                      \
PX_ASSERT(writer);                                                                    \
PX_ASSERT(regData);                                                                   \
regData->pvd##classID.destroyInstance(*writer, contextHandle, objectRef);


//
// Destroy a PVD object instance.
//
// See OMNI_PVD_SET_EXPLICIT and OMNI_PVD_SET for a description of the parameters.
//
#define OMNI_PVD_DESTROY(contextHandle, classID, objectRef)                          \
                                                                                     \
{                                                                                    \
OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)                                    \
OMNI_PVD_DESTROY_EXPLICIT(pvdWriter, pvdRegData, contextHandle, classID, objectRef); \
OMNI_PVD_WRITE_SCOPE_END                                                             \
}


//
// Set a PVD attribute value using the provided pointers to the writer and registration
// structure instance.
//
// writer: the variable named "writer" has to hold a pointer to the OmniPvdWriter instance
// regData: the variable named "regData" has to hold a pointer to the registration
//          structure
//
// See OMNI_PVD_SET for a description of the other parameters.
//
#define OMNI_PVD_SET_EXPLICIT(writer, regData, contextHandle, classID, attributeID, objectRef, valueRef) \
                                                                                                         \
PX_ASSERT(writer);                                                                                       \
PX_ASSERT(regData);                                                                                      \
regData->pvd##classID.set_##attributeID##_(*writer, contextHandle, objectRef, valueRef);


//
// Set a PVD attribute value.
//
// Note: if multiple attribute values should get set in a row, it is recommended
//       to use OMNI_PVD_WRITE_SCOPE_BEGIN & OMNI_PVD_SET_EXPLICIT etc. instead
//
// contextHandle: the handle of the context the object instance belongs to
// classID: the name of the class (as defined in OMNI_PVD_CLASS_BEGIN() etc.) the attribute
//          belongs to
// attributeID: the name of the attribute (as defined in OMNI_PVD_ATTRIBUTE() etc.) to set the
//              value for
// objectRef: reference to the class instance to set the attribute for (for untyped classes this shall be
//            a reference to a OmniPvdObjectHandle. For typed classes, the pointer value will be used as the
//            object handle value).
// valueRef: a reference to a variable that holds the value to set the attribute to
//
#define OMNI_PVD_SET(contextHandle, classID, attributeID, objectRef, valueRef)                         \
                                                                                                       \
{                                                                                                      \
OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)                                                      \
OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, contextHandle, classID, attributeID, objectRef, valueRef) \
OMNI_PVD_WRITE_SCOPE_END                                                                               \
}


//
// Set PVD array attribute values (variable size array) using the provided pointers to the writer and registration
// structure instance.
//
// valuesPtr: pointer to the array data to set the attribute to
// valueCount: number of entries in valuePtr
//
// See OMNI_PVD_SET for a description of the other parameters.
//
#define OMNI_PVD_SET_ARRAY_EXPLICIT(writer, regData, contextHandle, classID, attributeID, objectRef, valuesPtr, valueCount) \
                                                                                                                            \
PX_ASSERT(writer);                                                                                                          \
PX_ASSERT(regData);                                                                                                         \
regData->pvd##classID.set_##attributeID##_(*writer, contextHandle, objectRef, valuesPtr, valueCount);


//
// Set PVD array attribute values (variable size array).
//
// Note: if multiple attribute values should get set in a row, it is recommended
//       to use OMNI_PVD_WRITE_SCOPE_BEGIN & OMNI_PVD_SET_EXPLICIT etc. instead
//
// See OMNI_PVD_SET_ARRAY_EXPLICIT for a description of the parameters.
//
#define OMNI_PVD_SET_ARRAY(contextHandle, classID, attributeID, objectRef, valuesPtr, valueCount)                         \
                                                                                                                          \
{                                                                                                                         \
OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)                                                                         \
OMNI_PVD_SET_ARRAY_EXPLICIT(pvdWriter, pvdRegData, contextHandle, classID, attributeID, objectRef, valuesPtr, valueCount) \
OMNI_PVD_WRITE_SCOPE_END                                                                                                  \
}


//
// Add an entry to a PVD unique list attribute using the provided pointers to the writer and registration
// structure instance.
//
// See OMNI_PVD_SET for a description of the parameters.
//
#define OMNI_PVD_ADD_EXPLICIT(writer, regData, contextHandle, classID, attributeID, objectRef, valueRef) \
                                                                                                         \
PX_ASSERT(writer);                                                                                       \
PX_ASSERT(regData);                                                                                      \
regData->pvd##classID.addTo_##attributeID##_(*writer, contextHandle, objectRef, valueRef);


//
// Add an entry to a PVD unique list attribute.
//
// See OMNI_PVD_SET for a description of the parameters.
//
#define OMNI_PVD_ADD(contextHandle, classID, attributeID, objectRef, valueRef)                         \
                                                                                                       \
{                                                                                                      \
OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)                                                      \
OMNI_PVD_ADD_EXPLICIT(pvdWriter, pvdRegData, contextHandle, classID, attributeID, objectRef, valueRef) \
OMNI_PVD_WRITE_SCOPE_END                                                                               \
}


//
// Remove an entry from a PVD unique list attribute using the provided pointers to the writer and registration
// structure instance.
//
// See OMNI_PVD_SET for a description of the parameters.
//
#define OMNI_PVD_REMOVE_EXPLICIT(writer, regData, contextHandle, classID, attributeID, objectRef, valueRef) \
                                                                                                            \
PX_ASSERT(writer);                                                                                          \
PX_ASSERT(regData);                                                                                         \
regData->pvd##classID.removeFrom_##attributeID##_(*writer, contextHandle, objectRef, valueRef);


//
// Remove an entry from a PVD unique list attribute.
//
// See OMNI_PVD_SET for a description of the parameters.
//
#define OMNI_PVD_REMOVE(contextHandle, classID, attributeID, objectRef, valueRef)                         \
                                                                                                          \
{                                                                                                         \
OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)                                                         \
OMNI_PVD_REMOVE_EXPLICIT(pvdWriter, pvdRegData, contextHandle, classID, attributeID, objectRef, valueRef) \
OMNI_PVD_WRITE_SCOPE_END                                                                                  \
}


#else


#define OMNI_PVD_WRITE_SCOPE_BEGIN(writer, regData)
#define OMNI_PVD_WRITE_SCOPE_END
#define OMNI_PVD_CREATE_EXPLICIT(writer, regData, contextHandle, classID, objectRef)
#define OMNI_PVD_CREATE(contextHandle, classID, objectRef)
#define OMNI_PVD_DESTROY_EXPLICIT(writer, regData, contextHandle, classID, objectRef)
#define OMNI_PVD_DESTROY(contextHandle, classID, objectRef)
#define OMNI_PVD_SET_EXPLICIT(writer, regData, contextHandle, classID, attributeID, objectRef, valueRef)
#define OMNI_PVD_SET(contextHandle, classID, attributeID, objectRef, valueRef)
#define OMNI_PVD_SET_ARRAY_EXPLICIT(writer, regData, contextHandle, classID, attributeID, objectRef, valuesPtr, valueCount)
#define OMNI_PVD_SET_ARRAY(contextHandle, classID, attributeID, objectRef, valuesPtr, valueCount)
#define OMNI_PVD_ADD_EXPLICIT(writer, regData, contextHandle, classID, attributeID, objectRef, valueRef)
#define OMNI_PVD_ADD(contextHandle, classID, attributeID, objectRef, valueRef)
#define OMNI_PVD_REMOVE_EXPLICIT(writer, regData, contextHandle, classID, attributeID, objectRef, valueRef)
#define OMNI_PVD_REMOVE(contextHandle, classID, attributeID, objectRef, valueRef)


#endif  // PX_SUPPORT_OMNI_PVD
