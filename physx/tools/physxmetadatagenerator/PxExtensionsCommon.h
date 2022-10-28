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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
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
