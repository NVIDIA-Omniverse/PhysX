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

#include "vehicle2/commands/PxVehicleCommandParams.h"
#include "vehicle2/commands/PxVehicleCommandHelpers.h"

namespace physx
{
namespace vehicle2
{
static float interpolate(const PxReal* speedVals, const PxReal* torqueVals, const PxU16 nb, const PxReal speed)
{
	if (1 == nb)
	{
		return torqueVals[0];
	}
	else
	{
		const PxReal smallestSpeed = speedVals[0];
		const PxReal largestSpeed = speedVals[nb - 1];
		if (smallestSpeed >= speed)
		{
			return torqueVals[0];
		}
		else if (largestSpeed <= speed)
		{
			return torqueVals[nb - 1];
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

			return torqueVals[speedLowerId] + (speed - speedVals[speedLowerId]) * (torqueVals[speeddUpperId] - torqueVals[speedLowerId]) / (speedVals[speeddUpperId] - speedVals[speedLowerId]);
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
	const PxU16* nbSpeedResponsesPerCommandValue = responseParams.nonlinearResponse.nbSpeedRenponsesPerCommandValue;
	const PxU16 nbCommandValues = responseParams.nonlinearResponse.nbCommandValues;
	const PxReal* speedResponses = responseParams.nonlinearResponse.speedResponses;

	PxReal normalisedResponse = 0.0f;
	if ((1 == nbCommandValues) || (commandValues[0] >= commandValue))
	{
		//Input command value less than the smallest value in the response table or 
		//there is just a single command value in the response table.
		//No need to interpolate response of two command values.
		const PxReal* speeds = speedResponses + 2*speedResponsesPerCommandValue[0];
		const PxReal* torques = speeds + nbSpeedResponsesPerCommandValue[0];
		const PxU16 nb = nbSpeedResponsesPerCommandValue[0];
		normalisedResponse = interpolate(speeds, torques, nb, speed);
	}
	else if (commandValues[nbCommandValues - 1] <= commandValue)
	{
		//Input command value greater than the largest value in the response table.
		//No need to interpolate response of two command values.
		const PxReal* speeds = speedResponses + 2*speedResponsesPerCommandValue[nbCommandValues - 1];
		const PxReal* torques = speeds + nbSpeedResponsesPerCommandValue[nbCommandValues - 1];
		const PxU16 nb = nbSpeedResponsesPerCommandValue[nbCommandValues - 1];
		normalisedResponse =  interpolate(speeds, torques, nb, speed);
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
				const PxReal* torques = speeds + nbSpeedResponsesPerCommandValue[commandLowerId];
				const PxU16 nb = nbSpeedResponsesPerCommandValue[commandLowerId];
				zLower = interpolate(speeds, torques, nb, speed);
			}
			float zUpper;
			{
				const PxReal* speeds = speedResponses + 2*speedResponsesPerCommandValue[commandUpperId];
				const PxReal* torques = speeds + nbSpeedResponsesPerCommandValue[commandUpperId];
				const PxU16 nb = nbSpeedResponsesPerCommandValue[commandUpperId];
				zUpper = interpolate(speeds, torques, nb, speed);
			}
			const PxReal commandUpper = commandValues[commandUpperId];
			const PxReal commandLower = commandValues[commandLowerId];
			normalisedResponse = zLower + (commandValue - commandLower) * (zUpper - zLower) / (commandUpper - commandLower);
		}
		else
		{
			const PxReal* speeds = speedResponses + 2*speedResponsesPerCommandValue[commandUpperId];
			const PxReal* torques = speeds + nbSpeedResponsesPerCommandValue[commandUpperId];
			const PxU16 nb = nbSpeedResponsesPerCommandValue[commandUpperId];
			normalisedResponse = interpolate(speeds, torques, nb, speed);
		}
	}

	return PxVehicleLinearResponseCompute(normalisedResponse, wheelId, responseParams);
}

} // namespace vehicle2
} // namespace physx
