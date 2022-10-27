
groupshared uint sdata0[256u];
groupshared uint sdata1[256u];

groupshared uint stotalCount;

uint blockScan(uint threadIdx, uint val)
{
	uint localVal = val;
	sdata0[threadIdx] = localVal;

	GroupMemoryBarrierWithGroupSync();

	if (threadIdx >= 1) localVal += sdata0[threadIdx - 1];
	if (threadIdx >= 2) localVal += sdata0[threadIdx - 2];
	if (threadIdx >= 3) localVal += sdata0[threadIdx - 3];
	sdata1[threadIdx] = localVal;

	GroupMemoryBarrierWithGroupSync();

	if (threadIdx >= 4) localVal += sdata1[threadIdx - 4];
	if (threadIdx >= 8) localVal += sdata1[threadIdx - 8];
	if (threadIdx >= 12) localVal += sdata1[threadIdx - 12];
	sdata0[threadIdx] = localVal;

	GroupMemoryBarrierWithGroupSync();

	if (threadIdx >= 16) localVal += sdata0[threadIdx - 16];
	if (threadIdx >= 32) localVal += sdata0[threadIdx - 32];
	if (threadIdx >= 48) localVal += sdata0[threadIdx - 48];
	sdata1[threadIdx] = localVal;

	GroupMemoryBarrierWithGroupSync();

	if (threadIdx >= 64) localVal += sdata1[threadIdx - 64];
	if (threadIdx >= 128) localVal += sdata1[threadIdx - 128];
	if (threadIdx >= 192) localVal += sdata1[threadIdx - 192];

	// compute totalCount
	if (threadIdx == 0u)
	{
		stotalCount = sdata1[63] + sdata1[127] + sdata1[191] + sdata1[255];
	}

	return localVal;
}