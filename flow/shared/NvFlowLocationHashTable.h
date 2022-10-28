/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#pragma once

#include "NvFlowTypes.h"
#include "NvFlowArray.h"

struct NvFlowLocationHashTableRange
{
	NvFlowUint64 beginIdx;
	NvFlowUint64 endIdx;
};

struct NvFlowLocationHashTable
{
	NvFlowUint tableDimBits = 0llu;
	NvFlowUint tableDimLessOne = 0llu;
	NvFlowUint tableDim3 = 1u;

	NvFlowArray<NvFlowLocationHashTableRange> ranges;
	NvFlowArray<NvFlowUint64> nextIndices;

	NvFlowArray<NvFlowInt4> locations;
	NvFlowArray<NvFlowUint> masks;

	NvFlowInt4 locationMin = { 0, 0, 0, 0 };
	NvFlowInt4 locationMax = { 0, 0, 0, 0 };

	NvFlowArray<NvFlowInt4> tmpLocations;
	NvFlowArray<NvFlowUint> tmpMasks;

	void reset()
	{
		tableDimBits = 0llu;
		tableDimLessOne = 0llu;
		tableDim3 = 1u;

		ranges.size = 0u;
		nextIndices.size = 0u;
		NvFlowLocationHashTableRange nullRange = { ~0llu, ~0llu };
		ranges.pushBack(nullRange);

		locations.size = 0u;
		masks.size = 0u;
	}

	NvFlowLocationHashTable()
	{
		reset();
	}

	void rebuildTable()
	{
		ranges.size = 0u;
		ranges.reserve(tableDim3);
		ranges.size = tableDim3;

		nextIndices.size = 0u;
		nextIndices.reserve(locations.size);
		nextIndices.size = locations.size;

		// invalidate ranges
		NvFlowLocationHashTableRange nullRange = { ~0llu, ~0llu };
		for (NvFlowUint64 rangeIdx = 0u; rangeIdx < ranges.size; rangeIdx++)
		{
			ranges[rangeIdx] = nullRange;
		}
		for (NvFlowUint64 locationIdx = 0u; locationIdx < locations.size; locationIdx++)
		{
			NvFlowInt4 location = locations[locationIdx];
			NvFlowUint64 baseRangeIdx = (location.x & tableDimLessOne) |
				((location.y & tableDimLessOne) << tableDimBits) |
				((location.z & tableDimLessOne) << (tableDimBits + tableDimBits));

			// reset next for this location
			nextIndices[locationIdx] = ~0llu;

			NvFlowUint64 beginIdx = ranges[baseRangeIdx].beginIdx;
			NvFlowUint64 endIdx = ranges[baseRangeIdx].endIdx;
			if (beginIdx >= endIdx)
			{
				ranges[baseRangeIdx].beginIdx = locationIdx;
				ranges[baseRangeIdx].endIdx = locationIdx + 1u;
			}
			else if (endIdx == locationIdx)
			{
				ranges[baseRangeIdx].endIdx = locationIdx + 1u;
				nextIndices[endIdx - 1u] = locationIdx;
			}
			else
			{
				NvFlowUint64 prevIdx = endIdx - 1u;
				NvFlowUint64 currentIdx = nextIndices[prevIdx];
				while (currentIdx < nextIndices.size)
				{
					prevIdx = currentIdx;
					currentIdx = nextIndices[currentIdx];
				}
				nextIndices[prevIdx] = locationIdx;
			}
		}
	}

	void compactNonZeroWithLimit(NvFlowUint64 maxLocations)
	{
		NvFlowUint64 dstIdx = 0u;
		for (NvFlowUint64 srcIdx = 0u; srcIdx < locations.size && dstIdx < maxLocations; srcIdx++)
		{
			if (masks[srcIdx])
			{
				locations[dstIdx] = locations[srcIdx];
				masks[dstIdx] = masks[srcIdx];
				dstIdx++;
			}
		}
		locations.size = dstIdx;
		masks.size = dstIdx;

		// optimize compacted table dim
		tableDimBits = 0llu;
		tableDimLessOne = 0llu;
		tableDim3 = 1u;
		while (locations.size > tableDim3)
		{
			tableDimBits++;
			tableDimLessOne = (1u << tableDimBits) - 1u;
			tableDim3 = (1 << (tableDimBits + tableDimBits + tableDimBits));
		}

		rebuildTable();
	}

	void sort()
	{
		NvFlowArray_copy(tmpLocations, locations);
		NvFlowArray_copy(tmpMasks, masks);

		NvFlowUint64 globalOffset = 0u;
		for (NvFlowUint64 baseRangeIdx = 0u; baseRangeIdx < ranges.size; baseRangeIdx++)
		{
			NvFlowUint64 beginIdx = ranges[baseRangeIdx].beginIdx;
			NvFlowUint64 endIdx = ranges[baseRangeIdx].endIdx;
			for (NvFlowUint64 currentIdx = beginIdx; currentIdx < endIdx; currentIdx++)
			{
				locations[globalOffset] = tmpLocations[currentIdx];
				masks[globalOffset] = tmpMasks[currentIdx];
				globalOffset++;
			}
			if (beginIdx < endIdx)
			{
				NvFlowUint64 currentIdx = nextIndices[endIdx - 1u];
				while (currentIdx < nextIndices.size)
				{
					locations[globalOffset] = tmpLocations[currentIdx];
					masks[globalOffset] = tmpMasks[currentIdx];
					globalOffset++;

					currentIdx = nextIndices[currentIdx];
				}
			}
		}

		rebuildTable();
	}

	NvFlowUint64 find(NvFlowInt4 location)
	{
		NvFlowUint64 baseRangeIdx = (location.x & tableDimLessOne) |
			((location.y & tableDimLessOne) << tableDimBits) |
			((location.z & tableDimLessOne) << (tableDimBits + tableDimBits));

		NvFlowUint64 beginIdx = ranges[baseRangeIdx].beginIdx;
		NvFlowUint64 endIdx = ranges[baseRangeIdx].endIdx;
		for (NvFlowUint64 currentIdx = beginIdx; currentIdx < endIdx; currentIdx++)
		{
			if (location.x == locations[currentIdx].x &&
				location.y == locations[currentIdx].y &&
				location.z == locations[currentIdx].z &&
				location.w == locations[currentIdx].w)
			{
				return currentIdx;
			}
		}
		if (beginIdx < endIdx)
		{
			NvFlowUint64 currentIdx = nextIndices[endIdx - 1u];
			while (currentIdx < nextIndices.size)
			{
				if (location.x == locations[currentIdx].x &&
					location.y == locations[currentIdx].y &&
					location.z == locations[currentIdx].z &&
					location.w == locations[currentIdx].w)
				{
					return currentIdx;
				}
				currentIdx = nextIndices[currentIdx];
			}
		}
		return ~0llu;
	}

	void pushNoResize(NvFlowInt4 location, NvFlowUint mask)
	{
		NvFlowUint64 baseRangeIdx = (location.x & tableDimLessOne) |
			((location.y & tableDimLessOne) << tableDimBits) |
			((location.z & tableDimLessOne) << (tableDimBits + tableDimBits));

		NvFlowUint64 beginIdx = ranges[baseRangeIdx].beginIdx;
		NvFlowUint64 endIdx = ranges[baseRangeIdx].endIdx;
		for (NvFlowUint64 currentIdx = beginIdx; currentIdx < endIdx; currentIdx++)
		{
			if (location.x == locations[currentIdx].x &&
				location.y == locations[currentIdx].y &&
				location.z == locations[currentIdx].z &&
				location.w == locations[currentIdx].w)
			{
				masks[currentIdx] |= mask;
				return;
			}
		}
		if (beginIdx >= endIdx)
		{
			locations.pushBack(location);
			masks.pushBack(mask);
			nextIndices.pushBack(~0llu);

			ranges[baseRangeIdx].beginIdx = locations.size - 1u;
			ranges[baseRangeIdx].endIdx = locations.size;
		}
		else if (endIdx == locations.size)
		{
			locations.pushBack(location);
			masks.pushBack(mask);
			nextIndices.pushBack(~0llu);

			ranges[baseRangeIdx].endIdx = locations.size;
			nextIndices[endIdx - 1u] = locations.size - 1u;
		}
		else
		{
			NvFlowUint64 prevIdx = endIdx - 1u;
			NvFlowUint64 currentIdx = nextIndices[prevIdx];
			while (currentIdx < nextIndices.size)
			{
				if (location.x == locations[currentIdx].x &&
					location.y == locations[currentIdx].y &&
					location.z == locations[currentIdx].z &&
					location.w == locations[currentIdx].w)
				{
					masks[currentIdx] |= mask;
					return;
				}
				prevIdx = currentIdx;
				currentIdx = nextIndices[currentIdx];
			}

			locations.pushBack(location);
			masks.pushBack(mask);
			nextIndices.pushBack(~0llu);

			nextIndices[prevIdx] = locations.size - 1u;
		}
	}

	void conditionalGrowTable()
	{
		if (locations.size > tableDim3)
		{
			tableDimBits++;
			tableDimLessOne = (1u << tableDimBits) - 1u;
			tableDim3 = (1 << (tableDimBits + tableDimBits + tableDimBits));
			
			rebuildTable();
		}
	}

	void push(NvFlowInt4 location, NvFlowUint mask)
	{
		pushNoResize(location, mask);
		conditionalGrowTable();
	}

	void computeStats()
	{
		locationMin = NvFlowInt4{ 0, 0, 0, 0 };
		locationMax = NvFlowInt4{ 0, 0, 0, 0 };
		if (locations.size > 0)
		{
			locationMin = locations[0];
			locationMax.x = locations[0].x + 1;
			locationMax.y = locations[0].y + 1;
			locationMax.z = locations[0].z + 1;
			locationMax.w = locations[0].w + 1;
		}
		for (NvFlowUint64 locationIdx = 1u; locationIdx < locations.size; locationIdx++)
		{
			NvFlowInt4 location = locations[locationIdx];

			if (location.x < locationMin.x)
			{
				locationMin.x = location.x;
			}
			if (location.y < locationMin.y)
			{
				locationMin.y = location.y;
			}
			if (location.z < locationMin.z)
			{
				locationMin.z = location.z;
			}
			if (location.w < locationMin.w)
			{
				locationMin.w = location.w;
			}

			// plus one, since max is exclusive
			if (location.x + 1 > locationMax.x)
			{
				locationMax.x = location.x + 1;
			}
			if (location.y + 1 > locationMax.y)
			{
				locationMax.y = location.y + 1;
			}
			if (location.z + 1 > locationMax.z)
			{
				locationMax.z = location.z + 1;
			}
			if (location.w + 1 > locationMax.w)
			{
				locationMax.w = location.w + 1;
			}
		}
	}
};