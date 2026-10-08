// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


//
// The macro logic in this header will generate the PVD class/attribute registration
// code based on a class/attribute definition file. OMNI_PVD_WRITER_VAR needs to be
// defined before including this header. OMNI_PVD_WRITER_VAR has to represent the
// variable that holds a reference to a OmniPvdWriter instance. See 
// CmOmniPvdAutoGenCreateRegistrationStruct.h for a more detailed overview of the
// whole approach. The various parameters are described there too.
//


#define OMNI_PVD_CLASS_BEGIN(classID) \
                                      \
pvd##classID.classHandle = OMNI_PVD_WRITER_VAR.registerClass(#classID);


#define OMNI_PVD_CLASS_DERIVED_BEGIN(classID, baseClassID) \
                                                           \
pvd##classID.classHandle = OMNI_PVD_WRITER_VAR.registerClass(#classID, pvd##baseClassID.classHandle);


#define OMNI_PVD_CLASS_UNTYPED_BEGIN(classID) OMNI_PVD_CLASS_BEGIN(classID)


#define OMNI_PVD_CLASS_UNTYPED_DERIVED_BEGIN(classID, baseClassID) OMNI_PVD_CLASS_DERIVED_BEGIN(classID, baseClassID)


#define OMNI_PVD_CLASS_END(classID)


#define OMNI_PVD_ENUM_BEGIN(enumID) OMNI_PVD_CLASS_BEGIN(enumID)


#define OMNI_PVD_ENUM_END(enumID) OMNI_PVD_CLASS_END(enumID)


#define OMNI_PVD_ATTRIBUTE(classID, attributeID, valueType, pvdDataType) \
                                                                         \
pvd##classID.attributeID = OMNI_PVD_WRITER_VAR.registerAttribute(pvd##classID.classHandle, #attributeID, pvdDataType, 1);


#define OMNI_PVD_ATTRIBUTE_ARRAY_FIXED_SIZE(classID, attributeID, valueType, pvdDataType, entryCount) \
                                                                                                      \
pvd##classID.attributeID = OMNI_PVD_WRITER_VAR.registerAttribute(pvd##classID.classHandle, #attributeID, pvdDataType, entryCount);


#define OMNI_PVD_ATTRIBUTE_ARRAY_VARIABLE_SIZE(classID, attributeID, valueType, pvdDataType) \
                                                                                             \
pvd##classID.attributeID = OMNI_PVD_WRITER_VAR.registerAttribute(pvd##classID.classHandle, #attributeID, pvdDataType, 0);


#define OMNI_PVD_ATTRIBUTE_STRING(classID, attributeID) \
                                                        \
pvd##classID.attributeID = OMNI_PVD_WRITER_VAR.registerAttribute(pvd##classID.classHandle, #attributeID, OmniPvdDataType::eSTRING, 1);


#define OMNI_PVD_ATTRIBUTE_UNIQUE_LIST(classID, attributeID, valueType) \
                                                                        \
pvd##classID.attributeID = OMNI_PVD_WRITER_VAR.registerUniqueListAttribute(pvd##classID.classHandle, #attributeID, OmniPvdDataType::eOBJECT_HANDLE);


#define OMNI_PVD_ATTRIBUTE_FLAG(classID, attributeID, enumType, enumID) \
                                                                        \
pvd##classID.attributeID = OMNI_PVD_WRITER_VAR.registerFlagsAttribute(pvd##classID.classHandle, #attributeID, pvd##enumID.classHandle);


#define OMNI_PVD_ENUM_VALUE_EXPLICIT(enumID, enumEntryID, value) \
                                                                 \
OMNI_PVD_WRITER_VAR.registerEnumValue(pvd##enumID.classHandle, #enumEntryID, value);


#define OMNI_PVD_ENUM_VALUE(enumID, enumEntryID) \
                                                 \
OMNI_PVD_ENUM_VALUE_EXPLICIT(enumID, enumEntryID, enumID::enumEntryID)
