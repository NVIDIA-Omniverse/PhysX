// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_WRITER_H
#define OMNI_PVD_WRITER_H

#include "OmniPvdDefines.h"
#include "OmniPvdWriteStream.h"


/**
\brief Flags which report the status of an OmniPvdWriter.
*/
struct OmniPvdWriterStatusFlag
{
	enum Enum
	{
		/**
		 * \brief Set if opening the bound stream or writing its command stream failed
		 *
		 * A lazy-open failure also latches further writer open attempts until clearStatus() or
		 * setWriteStream() starts a new retry epoch.
         *
         * \see OmniPvdWriter.getStatus()
		*/
		eSTREAM_WRITE_FAILURE					= (1<<0)
	};
};

/**
 * \brief Used to write debug information to an OmniPvdWriteStream
 *
 * Allows the registration of OmniPVD classes and attributes, in a similar fashion to an object oriented language. A successful registration returns a unique identifier or handle.
 *
 * Once classes and attributes have been registered, one can create for example object instances of a class and set the values of the attributes of a specific object.
 *
 * Objects can be grouped by context using the context handle. The context handle is a user-specified handle which is passed to the set functions and object creation and destruction functions.
 *
 * Each context can have its own notion of time. The current time of a context can be exported with calls to the startFrame and stopFrame functions.
 *
 * The writer borrows its bound stream and never closes or destroys it. The caller must keep the
 * stream alive while writer access is possible, or rebind the writer to another live stream after
 * writes have quiesced and before closing and destroying/releasing the old stream.
 */
class OmniPvdWriter
{
public:
	virtual ~OmniPvdWriter()
	{
	}

	/**
	 * \brief Sets the log function to print the internal debug messages of the OmniPVD API
	 *
	 * \param logFunction The function pointer to receive the log messages
	 */
	virtual void OMNI_PVD_CALL setLogFunction(OmniPvdLogFunction logFunction) = 0;

	/**
	 * \brief Binds the write stream that receives the OmniPVD command stream.
	 *
	 * Binding is non-owning and performs no I/O: it does not open, write, flush, close, or destroy
	 * the stream. The next writer command lazily calls openStream() and, on success, emits the
	 * version header before its payload. If opening fails, no payload is emitted, registration
	 * calls return OMNI_PVD_INVALID_HANDLE without advancing schema handles, and
	 * OmniPvdWriterStatusFlag::eSTREAM_WRITE_FAILURE is set. While the writer is still waiting for
	 * its first write and that flag remains set, later commands are suppressed without another
	 * openStream() call. Thus lazy open is attempted at most once per status epoch. clearStatus()
	 * performs no I/O and permits one new attempt on the next command; setWriteStream() starts a new
	 * session and retry epoch.
	 *
	 * Every binding resets the writer's per-stream header state, schema handle numbering, and
	 * status flags so subsequent writes form a self-contained versioned segment. Binding itself
	 * does not clear transport bytes or reset its cursor. If the stream is already open, the segment
	 * begins at its current position. If it is closed, the next command first applies that transport's
	 * reopen policy: a file writer truncates, a memory writer preserves its FIFO position, and a TCP
	 * writer reconnects. Decode an appended segment from its preserved boundary. When the complete
	 * destination must contain one standalone recording, use a new or reset transport, or reopen a
	 * file writer so it truncates. Closing and reopening without re-binding does not reset writer state.
	 *
	 * The writer borrows writeStream and never closes or destroys it. Keep it alive until writes
	 * have quiesced or this writer has been rebound to another live stream.
	 *
	 * \param writeStream The borrowed OmniPvdWriteStream that receives API calls/notifications
	 */
	virtual void OMNI_PVD_CALL setWriteStream(OmniPvdWriteStream& writeStream) = 0;
	
	/**
	 * \brief Gets the pointer to the write stream
	 *
	 * \return A pointer to the write stream
	 */
	virtual OmniPvdWriteStream* OMNI_PVD_CALL getWriteStream() = 0;
	
	/**
	 * \brief Registers an OmniPVD class
	 *
	 * Returns a unique handle to a class, which can be used to register class attributes and express object lifetimes with the createObject and destroyObject functions. Class inheritance can be described by calling registerClass with a base class handle. Derived classes inherit the attributes of the parent classes.
	 *
	 * \param className The class name
	 * \param baseClassHandle The handle to the base class. This handle is obtained by pre-registering the base class. Defaults to 0 which means the class has no parent class
	 * \return A unique class handle, or OMNI_PVD_INVALID_HANDLE if no stream is bound or its lazy
	 * open fails.
	 *
	 * \see OmniPvdWriter::registerAttribute()
	 * \see OmniPvdWriter::registerEnumValue()
	 * \see OmniPvdWriter::registerFlagsAttribute()
	 * \see OmniPvdWriter::registerClassAttribute()
	 * \see OmniPvdWriter::registerUniqueListAttribute()
	 * \see OmniPvdWriter::createObject()
	 */
	virtual OmniPvdClassHandle OMNI_PVD_CALL registerClass(const char* className, OmniPvdClassHandle baseClassHandle = 0) = 0;

	/**
	 * \brief Registers an enum name and corresponding value for a pre-registered class.
	 *
	 * Registering enums happens in two steps. First, registerClass() is called with the name of the enum. This returns a class handle, which is used in a second step for the enum value registration with registerEnumValue(). If an enum has multiple values, registerEnumValue() has to be called with the different values.
	 *
	 * Note that enums differ from usual classes because their attributes, the enum values, do not change over time and there is no need to call setAttribute().
	 *
	 * \param classHandle The handle from the registerClass() call
	 * \param attributeName The name of the enum value
	 * \param value The value of the enum value
	 * \return A unique attribute handle, or OMNI_PVD_INVALID_HANDLE if no stream is bound or its
	 * lazy open fails.
	 *
	 * \see OmniPvdWriter::registerClass()
	 */
	virtual OmniPvdAttributeHandle OMNI_PVD_CALL registerEnumValue(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdEnumValueType value) = 0;
	
	/**
	 * \brief Registers an attribute.
	 *
	 * The class handle is obtained from a previous call to registerClass(). After registering an attribute, one gets an attribute handle which can be used to set data values of an attribute with setAttribute(). All attributes are treated as arrays, even if the attribute has only a single data item. Set nbElements to 0 to indicate that the array has a variable length.
	 *
	 * \param classHandle The handle from the registerClass() call
	 * \param attributeName The attribute name
	 * \param attributeDataType The attribute data type
	 * \param nbElements The number of elements in the array. Set this to 0 to indicate a variable length array
	 * \return A unique attribute handle, or OMNI_PVD_INVALID_HANDLE if no stream is bound or its
	 * lazy open fails.
	 *
	 * \see OmniPvdWriter::registerClass()
	 * \see OmniPvdWriter::setAttribute()
	 */
	virtual OmniPvdAttributeHandle OMNI_PVD_CALL registerAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdDataType::Enum attributeDataType, uint32_t nbElements) = 0;
	
	/**
	 * \brief Registers an attribute which is a flag.
	 *
	 * Use this function to indicate that a pre-registered class has a flag attribute, i.e., the attribute is a pre-registered enum.
	 *
	 * The returned attribute handle can be used in setAttribute() to set an object's flags.
	 *
	 * \param classHandle The handle from the registerClass() call of the class
	 * \param attributeName The attribute name
	 * \param enumClassHandle The handle from the registerClass() call of the enum
	 * \return A unique attribute handle, or OMNI_PVD_INVALID_HANDLE if no stream is bound or its
	 * lazy open fails.
	 *
	 * \see OmniPvdWriter::registerClass()
	 * \see OmniPvdWriter::setAttribute()
	 */
	virtual OmniPvdAttributeHandle OMNI_PVD_CALL registerFlagsAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdClassHandle enumClassHandle) = 0;
	
	/**
	 * \brief Registers an attribute which is a class.
	 *
	 * Use this function to indicate that a pre-registered class has an attribute which is a pre-registered class.
	 *
	 * The returned attribute handle can be used in setAttribute() to set an object's class attribute.
	 *
	 * \param classHandle The handle from the registerClass() call of the class
	 * \param attributeName The attribute name
	 * \param classAttributeHandle The handle from the registerClass() call of the class attribute
	 * \return A unique handle, or OMNI_PVD_INVALID_HANDLE if no stream is bound or its lazy open
	 * fails.
	 *
	 * \see OmniPvdWriter::registerClass()
	 * \see OmniPvdWriter::setAttribute()
	 */
	virtual OmniPvdAttributeHandle OMNI_PVD_CALL registerClassAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdClassHandle classAttributeHandle) = 0;
	
	/**
	 * \brief Registers an attribute which can hold a list of unique items.
	 *
	 * The returned attribute handle can be used in calls to addToUniqueListAttribute() and removeFromUniqueListAttribute(), to add an item to and remove it from the list, respectively.
	 *
	 * \param classHandle The handle from the registerClass() call of the class
	 * \param attributeName The attribute name
	 * \param attributeDataType The data type of the items which will get added to the list attribute
	 * \return A unique handle, or OMNI_PVD_INVALID_HANDLE if no stream is bound or its lazy open
	 * fails.
	 *
	 * \see OmniPvdWriter::registerClass()
	 * \see OmniPvdWriter::addToUniqueListAttribute()
	 * \see OmniPvdWriter::removeFromUniqueListAttribute()
	 */
	virtual OmniPvdAttributeHandle OMNI_PVD_CALL registerUniqueListAttribute(OmniPvdClassHandle classHandle, const char* attributeName, OmniPvdDataType::Enum attributeDataType) = 0;

	/**
	 * \brief Sets an attribute value.
	 *
	 * Since an attribute can be part of a nested construct of class attributes, the method
	 * expects an array of attribute handles as input to uniquely identify the attribute.
	 *
	 * \param contextHandle The user-defined context handle for grouping objects
	 * \param objectHandle The user-defined unique handle of the object. E.g. its physical memory address
	 * \param attributeHandles The attribute handles containing all class attribute handles of a nested class
	 *        construct. The last one has to be the handle from the registerUniqueListAttribute() call.
	 * \param nbAttributeHandles The number of attribute handles provided in attributeHandles
	 * \param data The pointer to the data of the element(s) to remove from the set
	 * \param nbrBytes The number of bytes to be written
	 *
	 * \see OmniPvdWriter::registerAttribute()
	 */
	virtual void OMNI_PVD_CALL setAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle* attributeHandles, uint8_t nbAttributeHandles, const uint8_t *data, uint32_t nbrBytes) = 0;

	/**
	 * \brief Sets an attribute value.
	 *
	 * See other setAttribute method for details. This special version covers the case where the
	 * attribute is not part of a class attribute construct.
	 *
	 * \param contextHandle The user-defined context handle for grouping objects
	 * \param objectHandle The user-defined unique handle of the object. E.g. its physical memory address
	 * \param attributeHandle The handle from the registerAttribute() call
	 * \param data The pointer to the data
	 * \param nbrBytes The number of bytes to be written
	 *
	 * \see OmniPvdWriter::registerAttribute()
	 */
	inline void OMNI_PVD_CALL setAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, OmniPvdAttributeHandle attributeHandle, const uint8_t *data, uint32_t nbrBytes)
	{
		setAttribute(contextHandle, objectHandle, &attributeHandle, 1, data, nbrBytes);
	}
	
	/**
	 * \brief Adds an item to a unique list attribute.
	 *
	 * A unique list attribute is defined like a set in mathematics, where each element must be unique.
	 *
	 * Since an attribute can be part of a nested construct of class attributes, the method
	 * expects an array of attribute handles as input to uniquely identify the attribute.
	 *
	 * \param contextHandle The user-defined context handle for grouping objects
	 * \param objectHandle The user-defined unique handle of the object. E.g. its physical memory address
	 * \param attributeHandles The attribute handles containing all class attribute handles of a nested class
	 *        construct. The last one has to be the handle from the registerUniqueListAttribute() call.
	 * \param nbAttributeHandles The number of attribute handles provided in attributeHandles
	 * \param data The pointer to the data of the item to add to the list
	 * \param nbrBytes The number of bytes to be written
	 *
	 * \see OmniPvdWriter::registerUniqueListAttribute()
	 */
	virtual void OMNI_PVD_CALL addToUniqueListAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle* attributeHandles, uint8_t nbAttributeHandles, const uint8_t* data, uint32_t nbrBytes) = 0;
	
	/**
	 * \brief Adds an item to a unique list attribute.
	 *
	 * See other addToUniqueListAttribute method for details. This special version covers the case where the
	 * attribute is not part of a class attribute construct.
	 *
	 * \param contextHandle The user-defined context handle for grouping objects
	 * \param objectHandle The user-defined unique handle of the object. E.g. its physical memory address
	 * \param attributeHandle The handle from the registerUniqueListAttribute() call
	 * \param data The pointer to the data of the item to add to the list
	 * \param nbrBytes The number of bytes to be written
	 *
	 * \see OmniPvdWriter::registerUniqueListAttribute()
	 */
	inline void OMNI_PVD_CALL addToUniqueListAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, OmniPvdAttributeHandle attributeHandle, const uint8_t* data, uint32_t nbrBytes)
	{
		addToUniqueListAttribute(contextHandle, objectHandle, &attributeHandle, 1, data, nbrBytes);
	}
	
	/**
	 * \brief Removes an item from a uniqe list attribute
	 *
	 * A uniqe list attribute is defined like a set in mathematics, where each element must be unique.
	 *
	 * Since an attribute can be part of a nested construct of class attributes, the method
	 * expects an array of attribute handles as input to uniquely identify the attribute.
	 *
	 * \param contextHandle The user-defined context handle for grouping objects
	 * \param objectHandle The user-defined unique handle of the object. E.g. its physical memory address
	 * \param attributeHandles The attribute handles containing all class attribute handles of a nested class
	 *        construct. The last one has to be the handle from the registerUniqueListAttribute() call.
	 * \param nbAttributeHandles The number of attribute handles provided in attributeHandles
	 * \param data The pointer to the data of the item to remove from the list
	 * \param nbrBytes The number of bytes to be written
	 *
	 * \see OmniPvdWriter::registerUniqueListAttribute()
	 */
	virtual void OMNI_PVD_CALL removeFromUniqueListAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, const OmniPvdAttributeHandle* attributeHandles, uint8_t nbAttributeHandles, const uint8_t* data, uint32_t nbrBytes) = 0;
	
	/**
	 * \brief Removes an item from a uniqe list attribute
	 *
	 * See other removeFromUniqueListAttribute method for details. This special version covers the case where the
	 * attribute is not part of a class attribute construct.
	 *
	 * \param contextHandle The user-defined context handle for grouping objects
	 * \param objectHandle The user-defined unique handle of the object. E.g. its physical memory address
	 * \param attributeHandle The handle from the registerUniqueListAttribute() call
	 * \param data The pointer to the data of the item to remove from the list
	 * \param nbrBytes The number of bytes to be written
	 *
	 * \see OmniPvdWriter::registerUniqueListAttribute()
	 */
	inline void OMNI_PVD_CALL removeFromUniqueListAttribute(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle, OmniPvdAttributeHandle attributeHandle, const uint8_t* data, uint32_t nbrBytes)
	{
		removeFromUniqueListAttribute(contextHandle, objectHandle, &attributeHandle, 1, data, nbrBytes);
	}

	/**
	 * \brief Creates an object creation event
	 *
	 * Indicates that an object is created. One can freely choose a context handle for grouping objects.
	 *
	 * The class handle is obtained from a registerClass() call. The object handle should be unique, but as it's not tracked by the OmniPVD API, it's important this is set to a valid handle such as the object's physical memory address.
	 *
	 * The object name can be freely choosen or not set.
	 *
	 * Create about object destruction event by calling destroyObject().
	 *
	 * \param contextHandle The user-defined context handle for grouping objects
	 * \param classHandle The handle from the registerClass() call
	 * \param objectHandle The user-defined unique handle of the object. E.g. its physical memory address
	 * \param objectName The user-defined name of the object. Can be the empty string
	 *
	 * \see OmniPvdWriter::registerClass()
	 * \see OmniPvdWriter::destroyObject()
	 */
	virtual void OMNI_PVD_CALL createObject(OmniPvdContextHandle contextHandle, OmniPvdClassHandle classHandle, OmniPvdObjectHandle objectHandle, const char* objectName) = 0;
	
	/**
	 * \brief Creates an object destruction event
	 *
	 * Use this to indicate that an object is destroyed. Use the same user-defined context and object handles as were used in the create object calls.
	 *
	 * \param contextHandle The user-defined context handle for grouping objects
	 * \param objectHandle The user-defined unique handle of the object. E.g. its physical memory address
	 *
	 * \see OmniPvdWriter::registerClass()
	 * \see OmniPvdWriter::createObject()
	 */
	virtual void OMNI_PVD_CALL destroyObject(OmniPvdContextHandle contextHandle, OmniPvdObjectHandle objectHandle) = 0;
	
	/**
	 * \brief Creates a frame start event
	 *
	 * Time or frames are counted separatly per user-defined context.
	 *
	 * \param contextHandle The user-defined context handle for grouping objects
	 * \param timeStamp The timestamp of the frame start event
	 */
	virtual void OMNI_PVD_CALL startFrame(OmniPvdContextHandle contextHandle, uint64_t timeStamp) = 0;
	
	/**
	 * \brief Creates a stop frame event
	 *
	 * Time is counted separately per user-defined context.
	 *
	 * \param contextHandle The user-defined context handle for grouping objects
	 * \param timeStamp The timestamp of the frame stop event
	 */

	virtual void OMNI_PVD_CALL stopFrame(OmniPvdContextHandle contextHandle, uint64_t timeStamp) = 0;

	/**
	 * \brief Record a message
	 * 
	 * Record a message in the OVD stream. The file, line and type parameters can help locate 
	 * the source of the message when debugging.
	 *
	 * \param contextHandle The user-defined context handle for grouping objects
	 * \param message A character string text message.
	 * \param file A character string containing the name of the source file where the message originated from. 
	 *        NULL is a valid value if a file name is not needed.
	 * \param line The line number in the source file where the message originated from.
	 * \param type An enumerated type describing the message type. If unneeded, any value can be set.
	 * \param handle A handle to an Omni PVD enumerated type that contains all values for the previous type parameter. 
	 *        Setting OMNI_PVD_INVALID_HANDLE will cause this parameter to be ignored. 
	 *        See #registerEnumValue()
	 */
	virtual void OMNI_PVD_CALL recordMessage(OmniPvdContextHandle contextHandle, const char* message, const char* file, uint32_t line, uint32_t type, OmniPvdClassHandle handle = OMNI_PVD_INVALID_HANDLE) = 0;

	/**
	 * @brief Gets the status of the writer
	 *
	 * \return The current status flags of the writer, held in a 32 bit unsigned integer with the flag bits defined by OmniPvdWriterStatusFlag
     *
     * \see OmniPvdWriterStatusFlag
	 */
	virtual uint32_t OMNI_PVD_CALL getStatus() = 0;

	/**
	 * \brief Clears or resets the status of the writer.
	 *
	 * This function performs no stream I/O. If a failed first-write lazy open is latched, clearing
	 * the status permits the next writer command to attempt the open once in a new status epoch.
	 */
	virtual void OMNI_PVD_CALL clearStatus() = 0;

};

#endif
