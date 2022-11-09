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
// Copyright (c) 2014-2022 NVIDIA Corporation. All rights reserved.


#pragma once

#include "NvFlowReflect.h"
#include "NvFlowArray.h"

#include "NvFlowDeepCopy.h"

#include <string.h>

struct NvFlowDatabaseContext;

struct NvFlowDatabasePrim;

struct NvFlowDatabaseAttr;

struct NvFlowDatabaseInterface
{
	NvFlowDatabasePrim*(NV_FLOW_ABI *createPrim)(
		NvFlowDatabaseContext* context, 
		NvFlowUint64 version,
		NvFlowDatabasePrim* parent,
		const char* displayTypename,
		const char* path,
		const char* name);

	void(NV_FLOW_ABI* updatePrim)(
		NvFlowDatabaseContext* context, 
		NvFlowUint64 version, 
		NvFlowUint64 minActiveVersion, 
		NvFlowDatabasePrim* prim);

	void(NV_FLOW_ABI* markDestroyedPrim)(NvFlowDatabaseContext* context, NvFlowDatabasePrim* prim);

	void(NV_FLOW_ABI* destroyPrim)(NvFlowDatabaseContext* context, NvFlowDatabasePrim* prim);

	NvFlowDatabaseAttr*(NV_FLOW_ABI* createAttr)(
		NvFlowDatabaseContext* context, 
		NvFlowUint64 version,
		NvFlowDatabasePrim* prim, 
		const NvFlowReflectData* reflectData, 
		NvFlowUint8* mappedData);

	void(NV_FLOW_ABI* updateAttr)(
		NvFlowDatabaseContext* context, 
		NvFlowUint64 version, 
		NvFlowUint64 minActiveVersion, 
		NvFlowDatabaseAttr* attr, 
		const NvFlowReflectData* reflectData, 
		NvFlowUint8* mappedData);

	void(NV_FLOW_ABI* markDestroyedAttr)(NvFlowDatabaseContext* context, NvFlowDatabaseAttr* attr);

	void(NV_FLOW_ABI* destroyAttr)(NvFlowDatabaseContext* context, NvFlowDatabaseAttr* attr);
};

struct NvFlowDatabaseString
{
	NvFlowArray<char> data;
	void append(const char* str)
	{
		if (data.size > 0u)
		{
			data.size--;
		}
		if (str)
		{
			NvFlowUint64 idx = 0u;
			while (str[idx])
			{
				data.pushBack(str[idx]);
				idx++;
			}
			data.pushBack('\0');
		}
	}
	void set(const char* str)
	{
		data.size = 0u;
		append(str);
	}
	const char* get()
	{
		return data.data;
	}
};

struct NvFlowDatabaseInstance
{
	struct Prim
	{
		NvFlowDatabasePrim* prim;
		NvFlowArrayPointer<Prim*> childPrims;
		NvFlowArray<NvFlowDatabaseAttr*> attrs;
		NvFlowDatabaseString path;
	};
	struct Data
	{
		NvFlowArray<NvFlowUint8> data;
		NvFlowUint64 version;
		NvFlowReflectDeepCopy* deepCopy = nullptr;
		~Data()
		{
			if (deepCopy)
			{
				NvFlowReflectDeepCopy_destroy(deepCopy);
				deepCopy = nullptr;
			}
		}
	};

	const NvFlowReflectDataType* dataType = nullptr;
	NvFlowDatabaseString displayTypename;
	NvFlowDatabaseString name;
	NvFlowUint64 luid = 0llu;
	NvFlowUint64 luidByteOffset = ~0llu;

	Prim rootPrim;
	NvFlowRingBufferPointer<Data*> datas;
	NvFlowBool32 markedForDestroy = NV_FLOW_FALSE;

	NvFlowArray<NvFlowUint8> defaultData;

	struct StackState
	{
		NvFlowUint64 childIdx;
		const NvFlowReflectDataType* reflectDataType;
		NvFlowUint8* data;
		Prim* prim;
	};

	NvFlowUint8* mapDataVersionAndType(NvFlowUint64 version, const NvFlowReflectDataType** pReflectDataType)
	{
		for (NvFlowUint64 idx = datas.activeCount() - 1u; idx < datas.activeCount(); idx--)
		{
			if (datas[idx]->version == version)
			{
				if (pReflectDataType)
				{
					*pReflectDataType = dataType;
				}
				return datas[idx]->data.data;
			}
		}

		NvFlowDatabaseInstance::Data* data = datas.allocateBackPointer();
		data->version = version;
		data->data.reserve(dataType->elementSize);
		data->data.size = dataType->elementSize;
		if (datas.activeCount() >= 2u)
		{
			NvFlowDatabaseInstance::Data* oldData = datas[datas.activeCount() - 2u];
			memcpy(data->data.data, oldData->data.data, data->data.size);
		}
		else if (dataType->defaultValue)
		{
			memcpy(data->data.data, dataType->defaultValue, data->data.size);
		}
		else
		{
			memset(data->data.data, 0, data->data.size);
		}

		// enforce luid
		if (luidByteOffset < dataType->elementSize)
		{
			*((NvFlowUint64*)(data->data.data + luidByteOffset)) = luid;
		}

		if (pReflectDataType)
		{
			*pReflectDataType = dataType;
		}
		return data->data.data;
	}

	NvFlowUint8* mapDataVersion(NvFlowUint64 version)
	{
		const NvFlowReflectDataType* reflectDataType = nullptr;
		return mapDataVersionAndType(version, &reflectDataType);
	}

	void deepCopyDataVersion(NvFlowUint64 version)
	{
		Data* data = nullptr;
		for (NvFlowUint64 idx = datas.activeCount() - 1u; idx < datas.activeCount(); idx--)
		{
			if (datas[idx]->version == version)
			{
				data = datas[idx];
			}
		}
		if (data)
		{
			if (!data->deepCopy)
			{
				data->deepCopy = NvFlowReflectDeepCopy_create();
			}

			NvFlowUint8* copyData = NvFlowReflectDeepCopy_update(data->deepCopy, data->data.data, dataType);

			// copy root struct over mapped to get safe pointers
			memcpy(data->data.data, copyData, data->data.size);
		}
	}

	NvFlowUint8* mapDataVersionReadOnly(NvFlowUint64 version)
	{
		// TODO: Simply update version to avoid copy
		return mapDataVersion(version);
	}

	template<const NvFlowDatabaseInterface* iface>
	void init(NvFlowDatabaseContext* context, NvFlowUint64 version, NvFlowUint64 luidIn, const NvFlowReflectDataType* dataTypeIn, const char* displayTypenameIn, const char* pathIn, const char* nameIn)
	{
		dataType = dataTypeIn;
		displayTypename.set(displayTypenameIn);
		name.set(nameIn);
		luid = luidIn;
		luidByteOffset = ~0llu;
		// try to find luid offset in root
		for (NvFlowUint64 childIdx = 0u; childIdx < dataType->childReflectDataCount; childIdx++)
		{
			if (strcmp(dataType->childReflectDatas[childIdx].name, "luid") == 0)
			{
				luidByteOffset = dataType->childReflectDatas[childIdx].dataOffset;
				break;
			}
		}

		rootPrim.path.set(pathIn);
		rootPrim.prim = nullptr;
		if (iface->createPrim)
		{
			rootPrim.prim = iface->createPrim(
				context,
				version,
				nullptr,
				displayTypename.get(),
				rootPrim.path.get(),
				name.get());
		}

		NvFlowUint8* mappedData = mapDataVersion(version);

		StackState state = { 0llu, dataType, mappedData, &rootPrim };
		NvFlowArray<StackState, 8u> stateStack;
		for (; state.childIdx < state.reflectDataType->childReflectDataCount; state.childIdx++)
		{
			// push prims
			while (state.reflectDataType->childReflectDatas[state.childIdx].dataType->dataType == eNvFlowType_struct)
			{
				const NvFlowReflectData* childReflectData = state.reflectDataType->childReflectDatas + state.childIdx;
				auto childPrim = state.prim->childPrims.allocateBackPointer();
				state.prim->attrs.pushBack(nullptr);

				// form path
				childPrim->path.set(state.prim->path.get());
				childPrim->path.append("/");
				childPrim->path.append(childReflectData->name);

				childPrim->prim = nullptr;
				if (iface->createPrim)
				{
					childPrim->prim = iface->createPrim(
						context,
						version,
						state.prim->prim,
						NvFlowReflectTrimPrefix(childReflectData->dataType->structTypename),
						childPrim->path.get(),
						childReflectData->name);
				}

				stateStack.pushBack(state);
				state.childIdx = 0u;
				state.reflectDataType = childReflectData->dataType;
				state.data += childReflectData->dataOffset;
				state.prim = childPrim;
			}
			// attributes
			if (state.childIdx < state.reflectDataType->childReflectDataCount)
			{
				const NvFlowReflectData* childReflectData = state.reflectDataType->childReflectDatas + state.childIdx;
				NvFlowDatabaseAttr* attr = nullptr;
				if (iface->createAttr)
				{
					attr = iface->createAttr(context, version, state.prim->prim, childReflectData, state.data);
				}
				state.prim->attrs.pushBack(attr);
				state.prim->childPrims.pushBack(nullptr);
			}
			// pop prims
			while (state.childIdx + 1u >= state.reflectDataType->childReflectDataCount && stateStack.size > 0u)
			{
				state = stateStack.back();
				stateStack.popBack();
			}
		}
	}

	void process(NvFlowUint64 version, NvFlowReflectProcess_t processReflect, void* userdata)
	{
		NvFlowUint8* mappedData = mapDataVersion(version);

		processReflect(mappedData, dataType, userdata);
	}

	template<const NvFlowDatabaseInterface* iface>
	void update(NvFlowDatabaseContext* context, NvFlowUint64 version, NvFlowUint64 minActiveVersion)
	{
		if (!markedForDestroy)
		{
			NvFlowUint8* mappedData = mapDataVersion(version);

			if (rootPrim.prim)
			{
				iface->updatePrim(context, version, minActiveVersion, rootPrim.prim);
			}

			StackState state = { 0llu, dataType, mappedData, &rootPrim };
			NvFlowArray<StackState, 8u> stateStack;
			for (; state.childIdx < state.reflectDataType->childReflectDataCount; state.childIdx++)
			{
				// push prims
				while (state.reflectDataType->childReflectDatas[state.childIdx].dataType->dataType == eNvFlowType_struct)
				{
					const NvFlowReflectData* childReflectData = state.reflectDataType->childReflectDatas + state.childIdx;
					auto childPrim = state.prim->childPrims[state.childIdx];
					if (childPrim->prim)
					{
						iface->updatePrim(context, version, minActiveVersion, childPrim->prim);
					}

					stateStack.pushBack(state);
					state.childIdx = 0u;
					state.reflectDataType = childReflectData->dataType;
					state.data += childReflectData->dataOffset;
					state.prim = childPrim;
				}
				// attributes
				if (state.childIdx < state.reflectDataType->childReflectDataCount)
				{
					const NvFlowReflectData* childReflectData = state.reflectDataType->childReflectDatas + state.childIdx;
					auto attr = state.prim->attrs[state.childIdx];
					if (attr)
					{
						iface->updateAttr(context, version, minActiveVersion, attr, childReflectData, state.data);
					}
				}
				// pop prims
				while (state.childIdx + 1u >= state.reflectDataType->childReflectDataCount && stateStack.size > 0u)
				{
					state = stateStack.back();
					stateStack.popBack();
				}
			}
		}

		NvFlowUint freeTreshold = markedForDestroy ? 0u : 1u;
		while (datas.activeCount() > freeTreshold && datas.front()->version < minActiveVersion)
		{
			datas.popFront();
		}
	}

	template<const NvFlowDatabaseInterface* iface>
	void markForDestroy(NvFlowDatabaseContext* context)
	{
		NvFlowUint8* mappedData = nullptr;
		StackState state = { 0llu, dataType, mappedData, &rootPrim };
		NvFlowArray<StackState, 8u> stateStack;
		for (; state.childIdx < state.reflectDataType->childReflectDataCount; state.childIdx++)
		{
			// push prims
			while (state.reflectDataType->childReflectDatas[state.childIdx].dataType->dataType == eNvFlowType_struct)
			{
				const NvFlowReflectData* childReflectData = state.reflectDataType->childReflectDatas + state.childIdx;
				auto childPrim = state.prim->childPrims[state.childIdx];

				stateStack.pushBack(state);
				state.childIdx = 0u;
				state.reflectDataType = childReflectData->dataType;
				state.data += childReflectData->dataOffset;
				state.prim = childPrim;
			}
			// attributes
			if (state.childIdx < state.reflectDataType->childReflectDataCount)
			{
				auto attr = state.prim->attrs[state.childIdx];
				if (attr)
				{
					iface->markDestroyedAttr(context, attr);
				}
			}
			// pop prims
			while (state.childIdx + 1u >= state.reflectDataType->childReflectDataCount && stateStack.size > 0u)
			{
				if (state.prim->prim)
				{
					iface->markDestroyedPrim(context, state.prim->prim);
				}
				state = stateStack.back();
				stateStack.popBack();
			}
		}
		if (rootPrim.prim)
		{
			iface->markDestroyedPrim(context, rootPrim.prim);
		}

		markedForDestroy = NV_FLOW_TRUE;
	}

	template<const NvFlowDatabaseInterface* iface>
	void destroy(NvFlowDatabaseContext* context)
	{
		NvFlowUint8* mappedData = nullptr;
		StackState state = { 0llu, dataType, mappedData, &rootPrim };
		NvFlowArray<StackState, 8u> stateStack;
		for (; state.childIdx < state.reflectDataType->childReflectDataCount; state.childIdx++)
		{
			// push prims
			while (state.reflectDataType->childReflectDatas[state.childIdx].dataType->dataType == eNvFlowType_struct)
			{
				const NvFlowReflectData* childReflectData = state.reflectDataType->childReflectDatas + state.childIdx;
				auto childPrim = state.prim->childPrims[state.childIdx];

				stateStack.pushBack(state);
				state.childIdx = 0u;
				state.reflectDataType = childReflectData->dataType;
				state.data += childReflectData->dataOffset;
				state.prim = childPrim;
			}
			// attributes
			if (state.childIdx < state.reflectDataType->childReflectDataCount)
			{
				auto attr = state.prim->attrs[state.childIdx];
				if (attr)
				{
					iface->destroyAttr(context, attr);
					attr = nullptr;
				}
			}
			// pop prims
			while (state.childIdx + 1u >= state.reflectDataType->childReflectDataCount && stateStack.size > 0u)
			{
				if (state.prim->prim)
				{
					iface->destroyPrim(context, state.prim->prim);
					state.prim->prim = nullptr;
				}
				state = stateStack.back();
				stateStack.popBack();
			}
		}
		if (rootPrim.prim)
		{
			iface->destroyPrim(context, rootPrim.prim);
			rootPrim.prim = nullptr;
		}
	}
};

struct NvFlowDatabaseType
{
	const NvFlowReflectDataType* dataType = nullptr;
	NvFlowDatabaseString displayTypeName;
	NvFlowArrayPointer<NvFlowDatabaseInstance*> instances;

	struct TypeSnapshot
	{
		NvFlowDatabaseTypeSnapshot snapshot;
		NvFlowArray<NvFlowUint8*> instanceDatas;
	};
	NvFlowRingBufferPointer<TypeSnapshot*> snapshots;

	void init(const NvFlowReflectDataType* dataTypeIn, const char* displayTypeNameIn)
	{
		dataType = dataTypeIn;
		displayTypeName.set(displayTypeNameIn);
	}

	template<const NvFlowDatabaseInterface* iface>
	void update(NvFlowDatabaseContext* context, NvFlowUint64 version, NvFlowUint64 minActiveVersion)
	{
		for (NvFlowUint instanceIdx = 0u; instanceIdx < instances.size; instanceIdx++)
		{
			instances[instanceIdx]->update<iface>(context, version, minActiveVersion);
		}

		// release instances
		{
			NvFlowUint64 keepCount = 0llu;
			for (NvFlowUint instanceIdx = 0u; instanceIdx < instances.size; instanceIdx++)
			{
				if (instances[instanceIdx]->markedForDestroy && instances[instanceIdx]->datas.activeCount() == 0u)
				{
					instances[instanceIdx]->destroy<iface>(context);
					instances.deletePointerAtIndex(instanceIdx);
				}
				else
				{
					instances.swapPointers(keepCount, instanceIdx);
					keepCount++;
				}
			}
			instances.size = keepCount;
		}

		// release snapshots
		while (snapshots.activeCount() > 0u && snapshots.front()->snapshot.version < minActiveVersion)
		{
			snapshots.popFront();
		}
	}

	template<const NvFlowDatabaseInterface* iface>
	void destroy(NvFlowDatabaseContext* context)
	{
		for (NvFlowUint instanceIdx = 0u; instanceIdx < instances.size; instanceIdx++)
		{
			instances[instanceIdx]->destroy<iface>(context);
		}
		instances.deletePointers();
	}

	void getSnapshot(NvFlowDatabaseTypeSnapshot* snapshot, NvFlowUint64 version)
	{
		auto ptr = snapshots.allocateBackPointer();

		ptr->snapshot.version = version;
		ptr->snapshot.dataType = dataType;
		ptr->instanceDatas.size = 0u;

		for (NvFlowUint instanceIdx = 0u; instanceIdx < instances.size; instanceIdx++)
		{
			if (!instances[instanceIdx]->markedForDestroy)
			{
				NvFlowUint8* data = instances[instanceIdx]->mapDataVersionReadOnly(version);

				ptr->instanceDatas.pushBack(data);
			}
		}

		ptr->snapshot.instanceDatas = ptr->instanceDatas.data;
		ptr->snapshot.instanceCount = ptr->instanceDatas.size;

		if (snapshot)
		{
			*snapshot = ptr->snapshot;
		}
	}
};

struct NvFlowDatabase
{
	NvFlowArrayPointer<NvFlowDatabaseType*> types;

	struct Snapshot
	{
		NvFlowDatabaseSnapshot snapshot;
		NvFlowArray<NvFlowDatabaseTypeSnapshot> typeSnapshots;
	};
	NvFlowRingBufferPointer<Snapshot*> snapshots;

	NvFlowUint64 luidCounter = 0llu;

	NvFlowDatabaseType* createType(const NvFlowReflectDataType* dataTypeIn, const char* displayTypeName)
	{
		auto ptr = types.allocateBackPointer();

		ptr->init(dataTypeIn, displayTypeName);

		return ptr;
	}

	template<const NvFlowDatabaseInterface* iface>
	NvFlowDatabaseInstance* createInstance(NvFlowDatabaseContext* context, NvFlowUint64 version, NvFlowDatabaseType* type, const char* pathIn, const char* name)
	{
		auto ptr = type->instances.allocateBackPointer();

		luidCounter++;

		ptr->init<iface>(context, version, luidCounter, type->dataType, type->displayTypeName.get(), pathIn, name);

		return ptr;
	}

	template<const NvFlowDatabaseInterface* iface>
	void update(NvFlowDatabaseContext* context, NvFlowUint64 version, NvFlowUint64 minActiveVersion)
	{
		for (NvFlowUint64 typeIdx = 0u; typeIdx < types.size; typeIdx++)
		{
			types[typeIdx]->update<iface>(context, version, minActiveVersion);
		}

		// release snapshots
		while (snapshots.activeCount() > 0u && snapshots.front()->snapshot.version < minActiveVersion)
		{
			snapshots.popFront();
		}
	}

	template<const NvFlowDatabaseInterface* iface>
	void markInstanceForDestroy(NvFlowDatabaseContext* context, NvFlowDatabaseInstance* ptr)
	{
		ptr->markForDestroy<iface>(context);
	}

	template<const NvFlowDatabaseInterface* iface>
	void markAllInstancesForDestroy(NvFlowDatabaseContext* context)
	{
		for (NvFlowUint64 typeIdx = 0u; typeIdx < types.size; typeIdx++)
		{
			NvFlowDatabaseType* type = types[typeIdx];
			for (NvFlowUint instanceIdx = 0u; instanceIdx < type->instances.size; instanceIdx++)
			{
				type->instances[instanceIdx]->markForDestroy<iface>(context);
			}
		}
	}

	template<const NvFlowDatabaseInterface* iface>
	NvFlowBool32 snapshotPending(NvFlowDatabaseContext* context, NvFlowUint64 version, NvFlowUint64 minActiveVersion)
	{
		update<iface>(context, version, minActiveVersion);

		NvFlowBool32 anySnapshotPending = NV_FLOW_FALSE;
		for (NvFlowUint64 typeIdx = 0u; typeIdx < types.size; typeIdx++)
		{
			if (types[typeIdx]->snapshots.activeCount() > 0u)
			{
				anySnapshotPending = NV_FLOW_TRUE;
			}
		}
		if (snapshots.activeCount() > 0u)
		{
			anySnapshotPending = NV_FLOW_TRUE;
		}
		return anySnapshotPending;
	}

	template<const NvFlowDatabaseInterface* iface>
	void destroy(NvFlowDatabaseContext* context)
	{
		for (NvFlowUint64 typeIdx = 0u; typeIdx < types.size; typeIdx++)
		{
			types[typeIdx]->destroy<iface>(context);
		}
	}

	void getSnapshot(NvFlowDatabaseSnapshot* snapshot, NvFlowUint64 version)
	{
		auto ptr = snapshots.allocateBackPointer();

		ptr->snapshot.version = version;
		ptr->typeSnapshots.size = 0u;

		for (NvFlowUint64 typeIdx = 0u; typeIdx < types.size; typeIdx++)
		{
			NvFlowDatabaseTypeSnapshot typeSnapshot = {};
			types[typeIdx]->getSnapshot(&typeSnapshot, version);

			ptr->typeSnapshots.pushBack(typeSnapshot);
		}

		ptr->snapshot.typeSnapshots = ptr->typeSnapshots.data;
		ptr->snapshot.typeSnapshotCount = ptr->typeSnapshots.size;

		if (snapshot)
		{
			*snapshot = ptr->snapshot;
		}
	}

	void enumerateActiveInstances(NvFlowDatabaseInstance** pInstances, NvFlowUint64* pInstanceCount)
	{
		if (!pInstances && pInstanceCount)
		{
			NvFlowUint64 activeCount = 0llu;
			for (NvFlowUint64 typeIdx = 0u; typeIdx < types.size; typeIdx++)
			{
				NvFlowDatabaseType* type = types[typeIdx];
				for (NvFlowUint instanceIdx = 0u; instanceIdx < type->instances.size; instanceIdx++)
				{
					if (!type->instances[instanceIdx]->markedForDestroy)
					{
						activeCount++;
					}
				}
			}
			*pInstanceCount = activeCount;
		}
		if (pInstances && pInstanceCount)
		{
			NvFlowUint64 activeCount = 0llu;
			for (NvFlowUint64 typeIdx = 0u; typeIdx < types.size; typeIdx++)
			{
				NvFlowDatabaseType* type = types[typeIdx];
				for (NvFlowUint instanceIdx = 0u; instanceIdx < type->instances.size; instanceIdx++)
				{
					if (!type->instances[instanceIdx]->markedForDestroy)
					{
						if (activeCount < (*pInstanceCount))
						{
							pInstances[activeCount] = type->instances[instanceIdx];
							activeCount++;
						}
					}
				}
			}
			*pInstanceCount = activeCount;
		}
	}
};
