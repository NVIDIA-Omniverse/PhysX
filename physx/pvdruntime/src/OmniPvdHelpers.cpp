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
