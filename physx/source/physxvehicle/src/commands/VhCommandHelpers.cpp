// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "vehicle/commands/PxVehicleCommandParams.h"
#include "vehicle/commands/PxVehicleCommandHelpers.h"

namespace physx
{
static float interpolate(const PxReal* speedVals, const PxReal* responseVals, const PxU16 nb, const PxReal speed)
{
	if (1 == nb)
	{
		return responseVals[0];
	}
	else
	{
		const PxReal smallestSpeed = speedVals[0];
		const PxReal largestSpeed = speedVals[nb - 1];
		if (smallestSpeed >= speed)
		{
			return responseVals[0];
		}
		else if (largestSpeed <= speed)
		{
			return responseVals[nb - 1];
		}
		else
		{
			PxU16 speedId = 0;
			while ((speedVals[speedId] < speed) && (speedId < nb))
				speedId++;

			// Make sure that we stay in range.
			PxU16 speedLowerId = speedId - 1;
			PxU16 speeddUpperId = speedId;
			if (nb == speedId)
				speeddUpperId = nb - 1;
			if (0 == speedId)
				speedLowerId = 0;

			return responseVals[speedLowerId] + (speed - speedVals[speedLowerId]) * (responseVals[speeddUpperId] - responseVals[speedLowerId]) / (speedVals[speeddUpperId] - speedVals[speedLowerId]);
		}
	}
}

PxReal PxVehicleNonLinearResponseCompute
(const PxReal commandValue, const PxReal speed, const PxU32 wheelId, const PxVehicleCommandResponseParams& responseParams)
{
	const PxU16 nbResponsesAtSpeeds = responseParams.nonlinearResponse.nbSpeedResponses;
	if (0 == nbResponsesAtSpeeds)
	{
		//Empty response table.
		//Use linear interpolation.
		return PxVehicleLinearResponseCompute(commandValue, wheelId, responseParams);
	}

	const PxReal* commandValues = responseParams.nonlinearResponse.commandValues;
	const PxU16* speedResponsesPerCommandValue = responseParams.nonlinearResponse.speedResponsesPerCommandValue;
	const PxU16* nbSpeedResponsesPerCommandValue = responseParams.nonlinearResponse.nbSpeedResponsesPerCommandValue;
	const PxU16 nbCommandValues = responseParams.nonlinearResponse.nbCommandValues;
	const PxReal* speedResponses = responseParams.nonlinearResponse.speedResponses;

	PxReal normalisedResponse = 0.0f;
	if ((1 == nbCommandValues) || (commandValues[0] >= commandValue))
	{
		//Input command value less than the smallest value in the response table or 
		//there is just a single command value in the response table.
		//No need to interpolate response of two command values.
		const PxReal* speeds = speedResponses + 2*speedResponsesPerCommandValue[0];
		const PxReal* responseValues = speeds + nbSpeedResponsesPerCommandValue[0];
		const PxU16 nb = nbSpeedResponsesPerCommandValue[0];
		normalisedResponse = interpolate(speeds, responseValues, nb, speed);
	}
	else if (commandValues[nbCommandValues - 1] <= commandValue)
	{
		//Input command value greater than the largest value in the response table.
		//No need to interpolate response of two command values.
		const PxReal* speeds = speedResponses + 2*speedResponsesPerCommandValue[nbCommandValues - 1];
		const PxReal* responseValues = speeds + nbSpeedResponsesPerCommandValue[nbCommandValues - 1];
		const PxU16 nb = nbSpeedResponsesPerCommandValue[nbCommandValues - 1];
		normalisedResponse =  interpolate(speeds, responseValues, nb, speed);
	}
	else
	{
		// Find the id of the command value that is immediately above the input command
		PxU16 commandId = 0;
		while ((commandValues[commandId] < commandValue) && (commandId < nbCommandValues))
		{
			commandId++;
		}

		// Make sure that we stay in range.
		PxU16 commandLowerId = commandId - 1;
		PxU16 commandUpperId = commandId;
		if (nbCommandValues == commandId)
			commandUpperId = nbCommandValues - 1;
		if (0 == commandId)
			commandLowerId = 0;

		if (commandUpperId != commandLowerId)
		{
			float zLower;
			{
				const PxReal* speeds = speedResponses + 2*speedResponsesPerCommandValue[commandLowerId];
				const PxReal* responseValues = speeds + nbSpeedResponsesPerCommandValue[commandLowerId];
				const PxU16 nb = nbSpeedResponsesPerCommandValue[commandLowerId];
				zLower = interpolate(speeds, responseValues, nb, speed);
			}
			float zUpper;
			{
				const PxReal* speeds = speedResponses + 2*speedResponsesPerCommandValue[commandUpperId];
				const PxReal* responseValues = speeds + nbSpeedResponsesPerCommandValue[commandUpperId];
				const PxU16 nb = nbSpeedResponsesPerCommandValue[commandUpperId];
				zUpper = interpolate(speeds, responseValues, nb, speed);
			}
			const PxReal commandUpper = commandValues[commandUpperId];
			const PxReal commandLower = commandValues[commandLowerId];
			normalisedResponse = zLower + (commandValue - commandLower) * (zUpper - zLower) / (commandUpper - commandLower);
		}
		else
		{
			const PxReal* speeds = speedResponses + 2*speedResponsesPerCommandValue[commandUpperId];
			const PxReal* responseValues = speeds + nbSpeedResponsesPerCommandValue[commandUpperId];
			const PxU16 nb = nbSpeedResponsesPerCommandValue[commandUpperId];
			normalisedResponse = interpolate(speeds, responseValues, nb, speed);
		}
	}

	return PxVehicleLinearResponseCompute(normalisedResponse, wheelId, responseParams);
}

} // namespace physx
