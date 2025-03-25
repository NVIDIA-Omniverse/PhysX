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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


// ***********************************************************************************************
// This snippet converts raw profiling data to a json file format that can be loaded and viewed
// in Google Chrome.
//
// SnippetProfilerConverter is a simple command-line tool supporting the following options::
//
//  --srcFile=<filename>             Specify the raw profiler source file
//  --dstFile=<filename>             Specify the destination json file
//
// ***********************************************************************************************

#include "extensions/PxDefaultProfiler.h"
#include "foundation/PxString.h"

#include <iostream>
#include <stdio.h>
#include <vector>
#include <map>
#include <algorithm>
#include "string.h"


using namespace physx;

#define TENS_NANO_SECONDS_TO_MICRO_SECONDS	10.0 * 0.001


struct EventData
{
	bool valid;
	PxDefaultProfilerThread thread;
	PxDefaultProfilerHeader header;
	PxDefaultProfilerEvent event;
	PxDefaultProfilerValueEvent value;

	bool operator<(const EventData& rhs) const 
	{ 
		return event.time < rhs.event.time; 
	}
};

struct NameData
{
	PxDefaultProfilerName profilerName;
	char* name;
};

// Parse the file into local data structures.
std::vector<EventData> gEventList;
std::map<PxU64, NameData> gNameMap;

PxU64 gInitialTime = 0;


struct CmdLineParameters
{
	const char* srcFile;
	const char* dstFile;

	CmdLineParameters() : srcFile(NULL), dstFile(NULL)
	{
	}
};



static bool match(const char* opt, const char* ref)
{
	std::string s1(opt);
	std::string s2(ref);
	return !s1.compare(0, s2.length(), s2);
}

static void printHelpMsg()
{
	printf("SnippetProfilerConverter usage:\n"
		"SnippetProfilerConverter "
		"--srcFile=<filename> "
		"--dstFile=<filename> \n");

	printf("--srcFile=<filename>\n");
	printf("  Name of the PhysX profiler input file to convert.\n");

	printf("--dstFile=<filename>\n");
	printf("  Name of the JSON output file to create.\n");
}

static bool parseCommandLine(CmdLineParameters& result, int argc, const char* const* argv)
{
	if (argc <= 1)
	{
		printHelpMsg();
		return false;
	}

#define GET_PARAMETER(v, s)                                             \
    {                                                                   \
	   v = argv[i] + strlen(s);										    \
	   if( v == NULL )													\
	   {																\
	    printf("[ERROR] \"%s\" should have extra parameter\n", argv[i]);\
	    printHelpMsg();													\
	    return false;													\
	  }																	\
	}

	for (int i = 0; i < argc; ++i)
	{
		if (argv[i][0] != '-' || argv[i][1] != '-')
		{
			if (i > 0)
			{
				printf("[ERROR] Unknown command line parameter \"%s\"\n", argv[i]);
				printHelpMsg();
				return false;
			}

			continue;
		}

		if (match(argv[i], "--srcFile="))
		{
			GET_PARAMETER(result.srcFile, "--srcFile=");
		}
		else if (match(argv[i], "--dstFile="))
		{
			GET_PARAMETER(result.dstFile, "--dstFile=");
		}
		else
		{
			printf("[ERROR] Unknown command line parameter \"%s\"\n", argv[i]);
			printHelpMsg();
			return false;
		}
	}

	if (!result.srcFile || !result.dstFile)
	{
		printf("[ERROR] Missing args!! \n");
		printHelpMsg();
		return false;
	}
	return true;
}

bool freadCheck(void* buffer, size_t size, FILE* f)
{
	size_t bytesRead = fread(buffer, 1, size, f);

	if(bytesRead != size)
	{
		return false;
	}

	return true;
}

double convertTimeToMicroSeconds(PxU64 stopTime, PxU64 startTime = 0)
{
	PxU64 initialTime = 0;

	if(startTime == 0)
	{
		initialTime = gInitialTime;
	}

	return (double)(stopTime - initialTime - startTime) * TENS_NANO_SECONDS_TO_MICRO_SECONDS;
}

int snippetMain(int argc, const char* const* argv)
{
	CmdLineParameters result;

	if(!parseCommandLine(result, argc, argv))
	{
		return 1;
	}

	FILE* f = fopen(result.srcFile, "rb");

	if(!f)
	{
		fprintf(stderr, "Could not open input file, \"%s\"!\n", result.srcFile);
		exit(1);
	}

	// Read the version info.
	PxDefaultProfilerVersionInfo versionInfo;
	freadCheck(&versionInfo, sizeof(versionInfo), f);

	if(versionInfo.major > PX_PROFILER_VERSION_MAJOR)
	{
		fprintf(stderr, "This tool cannot load the file. It is version %d.%d and this tool can only read versions %d.%d and earlier.\n", 
				versionInfo.major, versionInfo.minor,
				PX_PROFILER_VERSION_MAJOR, PX_PROFILER_VERSION_MINOR);
		exit(1);
	}
	else if(versionInfo.major == PX_PROFILER_VERSION_MAJOR)
	{
		if(versionInfo.minor > PX_PROFILER_VERSION_MINOR)
		{
			fprintf(stderr, "This tool cannot load the file. It is version %d.%d and this tool can only read versions %d.%d and earlier.\n",
					versionInfo.major, versionInfo.minor, 
					PX_PROFILER_VERSION_MAJOR, PX_PROFILER_VERSION_MINOR);
			exit(1);
		}
	}

	// Loop through all of the events on each thread.
	PxDefaultProfilerHeader header;
	PxDefaultProfilerThread thread;
	EventData event;
	NameData name;

	bool success = true;

	thread.threadId = 0;

	while(success)
	{
		success = freadCheck(&header, sizeof(header), f);

		switch(header.type)
		{
		case PxDefaultProfilerDataType::eTHREAD_BLOCK:
		{
			success = freadCheck(&thread, sizeof(thread), f);
			break;
		}

		case PxDefaultProfilerDataType::eZONE_START:
		case PxDefaultProfilerDataType::eZONE_END:
		case PxDefaultProfilerDataType::eZONE_START_CROSS_THREAD:
		case PxDefaultProfilerDataType::eZONE_END_CROSS_THREAD:
		case PxDefaultProfilerDataType::eFRAME:
		{
			event.valid = true;
			event.header = header;

			// The thread always precedes any event.
			event.thread = thread;

			success = freadCheck(&event.event, sizeof(event.event), f);

			gEventList.push_back(event);
			break;
		}
		
		case PxDefaultProfilerDataType::eNAME_REGISTRATION:
		{
			success = freadCheck(&name.profilerName, sizeof(name.profilerName), f);

			if(gNameMap.find(name.profilerName.key) == gNameMap.end())
			{
				name.name = new char[name.profilerName.size];
				success = freadCheck(name.name, name.profilerName.size, f);

				// Add the name to the name map.
				gNameMap[name.profilerName.key] = name;
			}
			else
			{
				fseek(f, name.profilerName.size, SEEK_CUR);
			}

			break;
		}

		case PxDefaultProfilerDataType::eVALUE_INT:
		case PxDefaultProfilerDataType::eVALUE_FLOAT:
		{
			event.valid = true;
			event.header = header;

			// The thread always precedes any event.
			event.thread = thread;

			success = freadCheck(&event.value, sizeof(event.value), f);

			gEventList.push_back(event);
			break;
		}

		default:
			break;
		}
	}

	fclose(f);

	// Sort all of the events by time.
	std::sort(gEventList.begin(), gEventList.end());

	// All times are relative to the first time in the list.
	if(gEventList.size() > 0)
	{
		gInitialTime = gEventList[0].event.time;
	}

	// Create the output file.
	FILE* w = fopen(result.dstFile, "wt");

	if(!w)
	{
		fprintf(stderr, "Could not open output file, \"%s\"!\n", result.dstFile);
		exit(1);
	}

	fprintf(w, "[\n");

	bool firstLine = true;

	// Process all of the intra threaded zones first.
	for(PxU32 i = 0; i < gEventList.size(); i++)
	{
		EventData startEvent = gEventList[i];

		if(startEvent.valid && startEvent.header.type == PxDefaultProfilerDataType::eZONE_START)
		{
			PxU32 j;
			double startTimeInMicroSeconds = convertTimeToMicroSeconds(startEvent.event.time);

			for(j = i + 1; j < gEventList.size(); j++)
			{
				EventData endEvent = gEventList[j];

				if(endEvent.valid && 
					endEvent.header.type == PxDefaultProfilerDataType::eZONE_END && 
					startEvent.thread.threadId == endEvent.thread.threadId && 
					startEvent.event.nameKey == endEvent.event.nameKey)
				{
					// Found a matching pair.
					double durationInMicroSeconds = convertTimeToMicroSeconds(endEvent.event.time, startEvent.event.time);

					if(firstLine == false)
					{
						fprintf(w, ",\n");
					}

					firstLine = false;

					fprintf(w,
							"{\"name\":\"%s\",\"ph\":\"X\",\"pid\":%llu,\"tid\":%llu,\"ts\":%g,\"dur\":%g,\"args\":{\"sceneId\":\"%llu\"}}",
							gNameMap[startEvent.event.nameKey].name,
							(unsigned long long)0, 
							(unsigned long long)startEvent.thread.threadId,
							startTimeInMicroSeconds,
							durationInMicroSeconds, 
							(unsigned long long)startEvent.event.contextId);

					// Don't process these zones again.
					startEvent.valid = false;
					endEvent.valid = false;
					break;
				}
			}

			if(j == gEventList.size())
			{
				printf("Zone Start %s on thread %llu at time %g is missing a corresponding Zone End.\n", 
						gNameMap[startEvent.event.nameKey].name, (unsigned long long)startEvent.thread.threadId, startTimeInMicroSeconds);
			}
		}
	}

	// Process the remaining cross-threaded zones.
	for(PxU32 i = 0; i < gEventList.size(); i++)
	{
		EventData startEvent = gEventList[i];

		if(startEvent.valid && startEvent.header.type == PxDefaultProfilerDataType::eZONE_START_CROSS_THREAD)
		{
			PxU32 j;
			double startTimeInMicroSeconds = convertTimeToMicroSeconds(startEvent.event.time);

			for(j = i + 1; j < gEventList.size(); j++)
			{
				EventData endEvent = gEventList[j];

				if(endEvent.valid && 
					endEvent.header.type == PxDefaultProfilerDataType::eZONE_END_CROSS_THREAD &&
					startEvent.event.nameKey == endEvent.event.nameKey)
				{
					// Found a matching pair.
					double durationInMicroSeconds = convertTimeToMicroSeconds(endEvent.event.time, startEvent.event.time);

					if(firstLine == false)
					{
						fprintf(w, ",\n");
					}

					firstLine = false;

					fprintf(w,
							"{\"name\":\"%s\",\"ph\":\"X\",\"pid\":%llu,\"tid\":%llu,\"ts\":%g,\"dur\":%g,\"args\":{\"sceneId\":\"%llu\"}}",
							gNameMap[startEvent.event.nameKey].name, 
							(unsigned long long)0, 
							(unsigned long long)startEvent.thread.threadId, 
							startTimeInMicroSeconds,
							durationInMicroSeconds, 
							(unsigned long long)startEvent.event.contextId);

					// Don't process these zones again.
					startEvent.valid = false;
					endEvent.valid = false;
					break;
				}
			}

			if(j == gEventList.size())
			{
				printf("Cross thread Zone Start %s on thread %llu at time %g is missing a corresponding Zone End.\n", 
						gNameMap[startEvent.event.nameKey].name, (unsigned long long)startEvent.thread.threadId, startTimeInMicroSeconds);
			}
		}
	}

	// Write all of the int values.
	for(PxU32 i = 0; i < gEventList.size(); i++)
	{
		EventData valueEvent = gEventList[i];

		if(valueEvent.header.type == PxDefaultProfilerDataType::eVALUE_INT)
		{
			if(firstLine == false)
			{
				fprintf(w, ",\n");
			}

			firstLine = false;

			double timeInMicrosSeconds = convertTimeToMicroSeconds(valueEvent.event.time);

			fprintf(w, 
					"{\"name\":\"%s\",\"ph\":\"C\",\"pid\":%llu,\"tid\":%llu,\"ts\":%g,\"args\":{\"%s\":%d}}",
					gNameMap[valueEvent.event.nameKey].name,
					(unsigned long long)0, 
					(unsigned long long)valueEvent.thread.threadId,
					timeInMicrosSeconds, 
					gNameMap[valueEvent.event.nameKey].name, 
					valueEvent.value.intValue);
		}
	}

	// Write all of the float values.
	for(PxU32 i = 0; i < gEventList.size(); i++)
	{
		EventData valueEvent = gEventList[i];

		if(valueEvent.header.type == PxDefaultProfilerDataType::eVALUE_FLOAT)
		{
			if(firstLine == false)
			{
				fprintf(w, ",\n");
			}

			firstLine = false;

			double timeInMicrosSeconds = convertTimeToMicroSeconds(valueEvent.event.time);

			fprintf(w, 
					"{\"name\":\"%s\",\"ph\":\"C\",\"pid\":%llu,\"tid\":%llu,\"ts\":%g,\"args\":{\"%s\":%g}}",
					gNameMap[valueEvent.event.nameKey].name, 
					(unsigned long long)0, 
					(unsigned long long)valueEvent.thread.threadId, 
					timeInMicrosSeconds, 
					gNameMap[valueEvent.event.nameKey].name, 
					(double)valueEvent.value.floatValue);
		}
	}

	// Write all of the frames.
	for(PxU32 i = 0; i < gEventList.size(); i++)
	{
		EventData frameEvent = gEventList[i];

		if(frameEvent.header.type == PxDefaultProfilerDataType::eFRAME)
		{
			if(firstLine == false)
			{
				fprintf(w, ",\n");
			}

			firstLine = false;

			double timeInMicrosSeconds = convertTimeToMicroSeconds(frameEvent.event.time);

			fprintf(w, 
					"{\"name\":\"%s\",\"ph\":\"i\",\"pid\":%llu,\"tid\":%llu,\"ts\":%g,\"s\":\"t\",\"bp\":\"e\"},\n", 
					gNameMap[frameEvent.event.nameKey].name,
					(unsigned long long)0, 
					(unsigned long long)frameEvent.thread.threadId, 
					timeInMicrosSeconds);

			fprintf(w, 
					"{\"name\":\"Frame %s (%llu)\",\"ph\":\"i\",\"pid\":%llu,\"tid\":%llu,\"ts\":%g,\"s\":\"t\",\"bp\":\"e\"}",
					gNameMap[frameEvent.event.nameKey].name,
					(unsigned long long)frameEvent.event.contextId, 
					(unsigned long long)0,
					(unsigned long long)frameEvent.thread.threadId,
					timeInMicrosSeconds);
		}
	}

	fprintf(w, "\n]\n");
	fclose(w);

	// Clean up.
	std::map<PxU64, NameData>::iterator iterator;

	for(iterator = gNameMap.begin(); iterator != gNameMap.end(); iterator++)
	{
		delete[] iterator->second.name;
	}

	return 1;
}
