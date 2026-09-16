// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#ifndef EXT_SERIALIZATION_H
#define EXT_SERIALIZATION_H

namespace physx
{
namespace Ext
{
	void RegisterExtensionsSerializers(PxSerializationRegistry& sr);
	void UnregisterExtensionsSerializers(PxSerializationRegistry& sr);
}
}

#endif
