// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_PVD_OBJECT_MODEL_INTERNAL_TYPES_H
#define PX_PVD_OBJECT_MODEL_INTERNAL_TYPES_H

#include "foundation/PxMemory.h"
#include "PxPvdObjectModelBaseTypes.h"
#include "foundation/PxArray.h"
#include "PxPvdFoundation.h"

namespace physx
{
namespace pvdsdk
{

struct PvdInternalType
{
	enum Enum
	{
		None = 0,
#define DECLARE_INTERNAL_PVD_TYPE(type) type,
#include "PxPvdObjectModelInternalTypeDefs.h"
		Last
#undef DECLARE_INTERNAL_PVD_TYPE
	};
};

PX_COMPILE_TIME_ASSERT(uint32_t(PvdInternalType::Last) <= uint32_t(PvdBaseType::InternalStop));

template <typename T>
struct DataTypeToPvdTypeMap
{
	bool compile_error;
};
template <PvdInternalType::Enum>
struct PvdTypeToDataTypeMap
{
	bool compile_error;
};

#define DECLARE_INTERNAL_PVD_TYPE(type)                                                                                \
	template <>                                                                                                        \
	struct DataTypeToPvdTypeMap<type>                                                                                  \
	{                                                                                                                  \
		enum Enum                                                                                                      \
		{                                                                                                              \
			BaseTypeEnum = PvdInternalType::type                                                                       \
		};                                                                                                             \
	};                                                                                                                 \
	template <>                                                                                                        \
	struct PvdTypeToDataTypeMap<PvdInternalType::type>                                                                 \
	{                                                                                                                  \
		typedef type TDataType;                                                                                        \
	};                                                                                                                 \
	template <>                                                                                                        \
	struct PvdDataTypeToNamespacedNameMap<type>                                                                        \
	{                                                                                                                  \
		NamespacedName Name;                                                                                           \
		PvdDataTypeToNamespacedNameMap() : Name("physx3_debugger_internal", #type)                                     \
		{                                                                                                              \
		}                                                                                                              \
	};
#include "PxPvdObjectModelInternalTypeDefs.h"
#undef DECLARE_INTERNAL_PVD_TYPE

template <typename TDataType, typename TAlloc>
DataRef<TDataType> toDataRef(const PxArray<TDataType, TAlloc>& data)
{
	return DataRef<TDataType>(data.begin(), data.end());
}

static inline bool safeStrEq(const DataRef<String>& lhs, const DataRef<String>& rhs)
{
	uint32_t count = lhs.size();
	if(count != rhs.size())
		return false;
	for(uint32_t idx = 0; idx < count; ++idx)
		if(!safeStrEq(lhs[idx], rhs[idx]))
			return false;
	return true;
}

static inline char* copyStr(const char* str)
{
	str = nonNull(str);
	uint32_t len = static_cast<uint32_t>(strnlen(str, UINT32_MAX - 1));
	char* newData = reinterpret_cast<char*>(PX_ALLOC(len + 1, "string"));
	PxMemCopy(newData, str, len);
	newData[len] = 0;
	return newData;
}

// Used for predictable bit fields.
template <typename TDataType, uint8_t TNumBits, uint8_t TOffset, typename TInputType>
struct BitMaskSetter
{
	// Create a mask that masks out the orginal value shift into place
	static TDataType createOffsetMask()
	{
		return createMask() << TOffset;
	}
	// Create a mask of TNumBits number of tis
	static TDataType createMask()
	{
		return static_cast<TDataType>((1 << TNumBits) - 1);
	}
	void setValue(TDataType& inCurrent, TInputType inData)
	{
		PX_ASSERT(inData < (1 << TNumBits));

		// Create a mask to remove the current value.
		TDataType theMask = ~(createOffsetMask());
		// Clear out current value.
		inCurrent = inCurrent & theMask;
		// Create the new value.
		TDataType theAddition = reinterpret_cast<TDataType>(inData << TOffset);
		// or it into the existing value.
		inCurrent = inCurrent | theAddition;
	}

	TInputType getValue(TDataType inCurrent)
	{
		return static_cast<TInputType>((inCurrent >> TOffset) & createMask());
	}
};

}
}
#endif
