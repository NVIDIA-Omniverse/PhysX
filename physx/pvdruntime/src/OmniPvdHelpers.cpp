// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OmniPvdHelpers.h"

uint8_t OmniPvdCompressInt(uint64_t handle, uint8_t *bytes) {
	uint8_t lastBitGroupIndex = 0;
	uint8_t shiftBits = 0;
	for (int i = 0; i < 8; i++) {
		if ((handle >> shiftBits) & 0x7f) {
			lastBitGroupIndex = static_cast<uint8_t>(i);
		}
		shiftBits += 7;
	}
	shiftBits = 0;
	for (int i = 0; i <= lastBitGroupIndex; i++) {
		uint8_t currentBitGroup = (handle >> shiftBits) & 0x7f;
		if (i < lastBitGroupIndex) {
			currentBitGroup |= 0x80; // Set the continuation flag bit to true
		}
		bytes[i] = currentBitGroup;
		shiftBits += 7;
	}

	return lastBitGroupIndex;
}

uint64_t OmniPvdDeCompressInt(uint8_t *bytes, uint8_t maxBytes) {
	if (maxBytes > 8) {
		maxBytes = 8;
	}
	uint64_t decompressedInt = 0;
	uint8_t continueFlag = 1;
	uint8_t readBytes = 0;
	uint8_t shiftBits = 0;
	while (continueFlag && (readBytes < maxBytes)) {
		const uint8_t currentByte = *bytes;
		uint64_t decompressedBits = currentByte & 0x7f;
		continueFlag = currentByte & 0x80;
		decompressedInt |= (decompressedBits << shiftBits);
		shiftBits += 7;
		if (continueFlag) {
			bytes++;
		}
	}
	return decompressedInt;
}
