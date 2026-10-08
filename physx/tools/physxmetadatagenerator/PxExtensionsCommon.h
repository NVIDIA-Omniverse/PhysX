// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_EXTENSIONS_COMMON_H
#define PX_EXTENSIONS_COMMON_H

#include <stdint.h>
#include "foundation/PxSimpleTypes.h"
#include "PxPhysXConfig.h"

//Property overrides will output this exact property name instead of the general
//property name that would be used.  The properties need to have no template arguments
//and exactly the same initialization as the classes they are overriding.
struct PropertyOverride
{
	const char* TypeName;
	const char* PropName;
	const char* OverridePropName;
	PropertyOverride( const char* tn, const char* pn, const char* opn )
		: TypeName( tn )
		, PropName( pn )
		, OverridePropName( opn )
	{
	}
};

//The meta data generator ignores properties that are marked as disabled.  Note that in the declaration
//for properties that are defined via getter and setter methods, the 'get' and 'set' prefix can be dropped.
//For protected fields the "m" prefix can be dropped, however for public fields the whole field qualifier is expected.
struct DisabledPropertyEntry
{
	const char* mTypeName;
	const char* mPropertyName;
	DisabledPropertyEntry( const char* inTypeName, const char* inValueName )
		: mTypeName( inTypeName )
		, mPropertyName( inValueName )
	{
	}
};


struct CustomProperty
{
	const char* mPropertyType;
	const char* mTypeName;
	const char* mValueStructDefinition;
	const char* mValueStructInit;

	CustomProperty( const char* inTypeName, const char* inName, const char* inPropertyType, const char* valueStructDefinition, const char* valueStructInit )
		: mPropertyType( inPropertyType )
		, mTypeName( inTypeName )
		, mValueStructDefinition( valueStructDefinition )
		, mValueStructInit( valueStructInit )
	{
	}
};

#endif // PX_EXTENSIONS_COMMON_H
