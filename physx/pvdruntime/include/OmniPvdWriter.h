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

#ifndef OMNI_PVD_WRITER_H
#define OMNI_PVD_WRITER_H

#include "OmniPvdDefines.h"
#include "OmniPvdWriteStream.h"

/**
 * @brief Used to write debug information to an OmniPvdWriteStream
 *
 * Allows the registration of OmniPVD classes and attributes, in a similar fashion to an object oriented language. A registration returns a unique identifier or handle.
 *
 * Once classes and attributes have been registered, one can create for example object instances of a class and set the values of the attributes of a specific object.
 *
 * Objects can be grouped by context using the context handle. The context handle is a user-specified handle which is passed to the set functions and object creation and destruction functions.
 *
 * Each context can have its own notion of time. The current time of a context can be exported with calls to the startFrame and stopFrame functions.
 */
class OmniPvdWriter
{
public:
	virtual ~OmniPvdWriter()
	{
	}

	/**
	 * @brief Sets the log function to print the internal debug messages of the OmniPVD API
	 *
	 * @param logFunction The function pointer to receive the log messages
	 */
	virtual void OMNI_PVD_CALL setLogFunction(OmniPvdLogFunction logFunction) = 0;

	/**
	 * @brief Sets the write stream to receive the API command stream
	 *
	 * @param writeStream The OmniPvdWriteStream to receive the stream of API calls/notifications
	 */
	virtual void OMNI_PVD_CALL setWriteStream(OmniPvdWriteStream& writeStream) = 0;
	
	/**
	 * @brief Gets the pointer to the write stream
	 *
	 * @return A pointer to the write stream
	 */
	virtual OmniPvdWriteStream* OMNI_PVD_CALL getWriteStream() = 0;
	
	/**
	 * @brief Registers an OmniPVD class
	 *
	 * Returns a unique handle to a class, which can be used to register class attributes and express object lifetimes with the createObject and destroyObject functions. Class inheritance can be described by calling registerClass with a base class handle. Derived classes inherit the attributes of the parent classes.
	 *
	 * @param className The class name
	 * @param baseClassHandle The handle to the base class. This handle is obtained by pre-registering the base class. Defaults to 0 which means the class has no parent class
	 * @return A unique class handle for the registered class
	 *
	 * @see OmniPvdWriter::registerAttribute()
	 * @see OmniPvdWriter::registerEnumValue()
	 * @see OmniPvdWriter::registerFlagsAttribute()
	 * @see OmniPvdWriter::registerClassAttribute()
	 * @see OmniPvdWriter::registerUniqueListAttribute()
	 * @see OmniPvdWriter::createObject()
	 */
	virtual OmniPvdClassHandle OMNI_PVD_CALL registerClass(const char* className, OmniPvdClassHandle baseClassHandle = 0) = 0;

	/**
	 * @brief Registers an enum name and corresponding value for a pre-registered class.
	 *
	 * Registering enums happens in two steps. First, registerClass() is called with the name of the enum. This returns a class handle, which is used in a second step for the enum value registration with registerEnumValue(). If an enum has multiple values, registerEnumValue() has to be called with the different values.
	 *
	 * Note that enums differ from usual classes because their attributes, the enum values, do not change over time and there is no need to call setAttribute().
	 *
	 * @param classHandle The handle from the registerClass() call
	 * @param attributeName The name of the enum value
	 * @param value The value of the enum value
	 * @return A unique attribute handle for the registered enum value
	 *
	 * @see OmniPvdWriter::registerClass()
	 */
	virtual OmniPvdAttributeHandle OMNI_PVD_CALL registerEnumValue(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdEnumValueType value) = 0;
	
	/**
	 * @brief Registers an attribute.
	 *
	 * The class handle is obtained from a previous call to registerClass(). After registering an attribute, one gets an attribute handle which can be used to set data values of an attribute with setAttribute(). All attributes are treated as arrays, even if the attribute has only a single data item. Set nbElements to 0 to indicate that the array has a variable length.
	 *
	 * @param classHandle The handle from the registerClass() call
	 * @param attributeName The attribute name
	 * @param attributeDataType The attribute data type
	 * @param nbElements The number of elements in the array. Set this to 0 to indicate a variable length array
	 * @return A unique attribute handle for the registered attribute
	 *
	 * @see OmniPvdWriter::registerClass()
	 * @see OmniPvdWriter::setAttribute()
	 */
	virtual OmniPvdAttributeHandle OMNI_PVD_CALL registerAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdDataType::Enum attributeDataType, uint32_t nbElements) = 0;
	
	/**
	 * @brief Registers an attribute which is a flag.
	 *
	 * Use this function to indicate that a pre-registered class has a flag attribute, i.e., the attribute is a pre-registered enum.
	 *
	 * The returned attribute handle can be used in setAttribute() to set an object's flags.
	 *
	 * @param classHandle The handle from the registerClass() call of the class
	 * @param attributeName The attribute name
	 * @param enumClassHandle The handle from the registerClass() call of the enum
	 * @return A unique attribute handle for the registered flags attribute
	 *
	 * @see OmniPvdWriter::registerClass()
	 * @see OmniPvdWriter::setAttribute()
	 */
	virtual OmniPvdAttributeHandle OMNI_PVD_CALL registerFlagsAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdClassHandle enumClassHandle) = 0;
	
	/**
	 * @brief Registers an attribute which is a class.
	 *
	 * Use this function to indicate that a pre-registered class has an attribute which is a pre-registered class.
	 *
	 * The returned attribute handle can be used in setAttribute() to set an object's class attribute.
	 *
	 * @param classHandle The handle from the registerClass() call of the class
	 * @param attributeName The attribute name
	 * @param classAttributeHandle The handle from the registerClass() call of the class attribute
	 * @return A unique handle for the registered class attribute
	 *
	 * @see OmniPvdWriter::registerClass()
	 * @see OmniPvdWriter::setAttribute()
	 */
	virtual OmniPvdAttributeHandle OMNI_PVD_CALL registerClassAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdClassHandle classAttributeHandle) = 0;
	
	/**
	 * @brief Registers an attribute which can hold a list of unique items.
	 *
	 * The returned attribute handle can be used in calls to addToUniqueListAttribute() and removeFromUniqueListAttribute(), to add an item to and remove it from the list, respectively.
	 *
	 * @param classHandle The handle from the registerClass() call of the class
	 * @param attributeName The attribute name
	 * @param attributeDataType The data type of the items which will get added to the list attribute
	 * @return A unique handle for the registered list attribute
	 *
	 * @see OmniPvdWriter::registerClass()
	 * @see OmniPvdWriter::addToUniqueListAttribute()
	 * @see OmniPvdWriter::removeFromUniqueListAttribute()
	 */
	virtual OmniPvdAttributeHandle OMNI_PVD_CALL registerUniqueListAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdDataType::Enum attributeDataType) = 0;

	/**
	 * @brief Sets an attribute value.
	 *
	 * Since an attribute can be part of a nested construct of class attributes, the method
	 * expects an array of attribute handles as input to uniquely identify the attribute.
	 *
	 * @param contextHandle The user-defined context handle for grouping objects
	 * @param objectHandle The user-defined unique handle of the object. E.g. its physical memory address
	 * @param attributeHandles The attribute handles containing all class attribute handles of a nested class
	 *        construct. The last one has to be the handle from the registerUniqueListAttribute() call.
	 * @param nbAttributeHandles The number of attribute handles provided in attributeHandles
	 * @param data The pointer to the data of the element(s) to remove from the set
	 * @param nbrBytes The number of bytes to be written
	 *
	 * @see OmniPvdWriter::registerAttribute()
	 */
	virtual void OMNI_PVD_CALL setAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle* attributeHandles, uint8_t nbAttributeHandles, const uint8_t *data, uint32_t nbrBytes) = 0;

	/**
	 * @brief Sets an attribute value.
	 *
	 * See other setAttribute method for details. This special version covers the case where the
	 * attribute is not part of a class attribute construct.
	 *
	 * @param contextHandle The user-defined context handle for grouping objects
	 * @param objectHandle The user-defined unique handle of the object. E.g. its physical memory address
	 * @param attributeHandle The handle from the registerAttribute() call
	 * @param data The pointer to the data
	 * @param nbrBytes The number of bytes to be written
	 *
	 * @see OmniPvdWriter::registerAttribute()
	 */
	inline void OMNI_PVD_CALL setAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, OmniPvdAttributeHandle attributeHandle, const uint8_t *data, uint32_t nbrBytes)
	{
		setAttribute(contextHandle, objectHandle, &attributeHandle, 1, data, nbrBytes);
	}
	
	/**
	 * @brief Adds an item to a unique list attribute.
	 *
	 * A unique list attribute is defined like a set in mathematics, where each element must be unique.
	 *
	 * Since an attribute can be part of a nested construct of class attributes, the method
	 * expects an array of attribute handles as input to uniquely identify the attribute.
	 *
	 * @param contextHandle The user-defined context handle for grouping objects
	 * @param objectHandle The user-defined unique handle of the object. E.g. its physical memory address
	 * @param attributeHandles The attribute handles containing all class attribute handles of a nested class
	 *        construct. The last one has to be the handle from the registerUniqueListAttribute() call.
	 * @param nbAttributeHandles The number of attribute handles provided in attributeHandles
	 * @param data The pointer to the data of the item to add to the list
	 * @param nbrBytes The number of bytes to be written
	 *
	 * @see OmniPvdWriter::registerUniqueListAttribute()
	 */
	virtual void OMNI_PVD_CALL addToUniqueListAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle* attributeHandles, uint8_t nbAttributeHandles, const uint8_t* data, uint32_t nbrBytes) = 0;
	
	/**
	 * @brief Adds an item to a unique list attribute.
	 *
	 * See other addToUniqueListAttribute method for details. This special version covers the case where the
	 * attribute is not part of a class attribute construct.
	 *
	 * @param contextHandle The user-defined context handle for grouping objects
	 * @param objectHandle The user-defined unique handle of the object. E.g. its physical memory address
	 * @param attributeHandle The handle from the registerUniqueListAttribute() call
	 * @param data The pointer to the data of the item to add to the list
	 * @param nbrBytes The number of bytes to be written
	 *
	 * @see OmniPvdWriter::registerUniqueListAttribute()
	 */
	inline void OMNI_PVD_CALL addToUniqueListAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, OmniPvdAttributeHandle attributeHandle, const uint8_t* data, uint32_t nbrBytes)
	{
		addToUniqueListAttribute(contextHandle, objectHandle, &attributeHandle, 1, data, nbrBytes);
	}
	
	/**
	 * @brief Removes an item from a uniqe list attribute
	 *
	 * A uniqe list attribute is defined like a set in mathematics, where each element must be unique.
	 *
	 * Since an attribute can be part of a nested construct of class attributes, the method
	 * expects an array of attribute handles as input to uniquely identify the attribute.
	 *
	 * @param contextHandle The user-defined context handle for grouping objects
	 * @param objectHandle The user-defined unique handle of the object. E.g. its physical memory address
	 * @param attributeHandles The attribute handles containing all class attribute handles of a nested class
	 *        construct. The last one has to be the handle from the registerUniqueListAttribute() call.
	 * @param nbAttributeHandles The number of attribute handles provided in attributeHandles
	 * @param data The pointer to the data of the item to remove from the list
	 * @param nbrBytes The number of bytes to be written
	 *
	 * @see OmniPvdWriter::registerUniqueListAttribute()
	 */
	virtual void OMNI_PVD_CALL removeFromUniqueListAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle* attributeHandles, uint8_t nbAttributeHandles, const uint8_t* data, uint32_t nbrBytes) = 0;
	
	/**
	 * @brief Removes an item from a uniqe list attribute
	 *
	 * See other removeFromUniqueListAttribute method for details. This special version covers the case where the
	 * attribute is not part of a class attribute construct.
	 *
	 * @param contextHandle The user-defined context handle for grouping objects
	 * @param objectHandle The user-defined unique handle of the object. E.g. its physical memory address
	 * @param attributeHandle The handle from the registerUniqueListAttribute() call
	 * @param data The pointer to the data of the item to remove from the list
	 * @param nbrBytes The number of bytes to be written
	 *
	 * @see OmniPvdWriter::registerUniqueListAttribute()
	 */
	inline void OMNI_PVD_CALL removeFromUniqueListAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, OmniPvdAttributeHandle attributeHandle, const uint8_t* data, uint32_t nbrBytes)
	{
		removeFromUniqueListAttribute(contextHandle, objectHandle, &attributeHandle, 1, data, nbrBytes);
	}

	/**
	 * @brief Creates an object creation event
	 *
	 * Indicates that an object is created. One can freely choose a context handle for grouping objects.
	 *
	 * The class handle is obtained from a registerClass() call. The object handle should be unique, but as it's not tracked by the OmniPVD API, it's important this is set to a valid handle such as the object's physical memory address.
	 *
	 * The object name can be freely choosen or not set.
	 *
	 * Create about object destruction event by calling destroyObject().
	 *
	 * @param contextHandle The user-defined context handle for grouping objects
	 * @param classHandle The handle from the registerClass() call
	 * @param objectHandle The user-defined unique handle of the object. E.g. its physical memory address
	 * @param objectName The user-defined name of the object. Can be the empty string
	 *
	 * @see OmniPvdWriter::registerClass()
	 * @see OmniPvdWriter::destroyObject()
	 */
	virtual void OMNI_PVD_CALL createObject(OmniPvdContextHandle contextHandle, OmniPvdClassHandle classHandle, OmniPvdObjectHandle objectHandle, const char* objectName) = 0;
	
	/**
	 * @brief Creates an object destruction event
	 *
	 * Use this to indicate that an object is destroyed. Use the same user-defined context and object handles as were used in the create object calls.
	 *
	 * @param contextHandle The user-defined context handle for grouping objects
	 * @param objectHandle The user-defined unique handle of the object. E.g. its physical memory address
	 *
	 * @see OmniPvdWriter::registerClass()
	 * @see OmniPvdWriter::createObject()
	 */
	virtual void OMNI_PVD_CALL destroyObject(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle) = 0;
	
	/**
	 * @brief Creates a frame start event
	 *
	 * Time or frames are counted separatly per user-defined context.
	 *
	 * @param contextHandle The user-defined context handle for grouping objects
	 * @param timeStamp The timestamp of the frame start event
	 */
	virtual void OMNI_PVD_CALL startFrame(OmniPvdContextHandle contextHandle, uint64_t timeStamp) = 0;
	
	/**
	 * @brief Creates a stop frame event
	 *
	 * Time is counted separately per user-defined context.
	 *
	 * @param contextHandle The user-defined context handle for grouping objects
	 * @param timeStamp The timestamp of the frame stop event
	 */
	virtual void OMNI_PVD_CALL stopFrame(OmniPvdContextHandle contextHandle, uint64_t timeStamp) = 0;

	
};

#endif
