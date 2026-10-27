// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SN_XML_WRITER_H
#define SN_XML_WRITER_H

#include "foundation/PxSimpleTypes.h"

namespace physx {

	struct PxRepXObject;

	/** 
	 *	Writer used by extensions to write elements to a file or database
	 */
	class PX_DEPRECATED XmlWriter
	{
	protected:
		virtual ~XmlWriter(){}
	public:
		/** Write a key-value pair into the current item */
		virtual void write( const char* inName, const char* inData ) = 0;
		/** Write an object id into the current item */
		virtual void write( const char* inName, const PxRepXObject& inLiveObject ) = 0;
		/** Add a child that then becomes the current context */
		virtual void addAndGotoChild( const char* inName ) = 0;
		/** Leave the current child */
		virtual void leaveChild() = 0;
	};
} 

#endif
