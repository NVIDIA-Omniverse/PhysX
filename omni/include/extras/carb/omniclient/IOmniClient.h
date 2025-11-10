// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "carb/Interface.h"
#include "carb/Types.h"
#include "OmniClient.h"

namespace carb
{
namespace omniclient
{

/**
 * Carbonite compatible wrapper around OmniClient
 */
struct IOmniClient
{
    CARB_PLUGIN_INTERFACE("carb::omniclient::IOmniClient", OMNICLIENT_VERSION_MAJOR, OMNICLIENT_VERSION_MINOR)

    decltype(omniClientRegisterConnectionStatusCallback) CARB_ABI* registerConnectionStatusCallback;
    decltype(omniClientGetConnectionStatusString) CARB_ABI* getConnectionStatusString;
    decltype(omniClientRegisterFileStatusCallback) CARB_ABI* registerFileStatusCallback;
    decltype(omniClientGetFileStatusString) CARB_ABI* getFileStatusString;
    decltype(omniClientGetResultString) CARB_ABI* getResultString;
    decltype(omniClientStop) CARB_ABI* stop;
    decltype(omniClientWait) CARB_ABI* wait;
    decltype(omniClientSetAlias) CARB_ABI* setAlias;
    decltype(omniClientUnregisterCallback) CARB_ABI* unregisterCallback;
    decltype(omniClientBreakUrl) CARB_ABI* breakUrl;
    decltype(omniClientFreeUrl) CARB_ABI* freeUrl;
    decltype(omniClientMakeUrl) CARB_ABI* makeUrl;
    decltype(omniClientMakeFileUrl) CARB_ABI* makeFileUrl;
    decltype(omniClientNormalizeUrl) CARB_ABI* normalizeUrl;
    decltype(omniClientCombineUrls) CARB_ABI* combineUrls;
    decltype(omniClientCombineWithBaseUrl) CARB_ABI* combineWithBaseUrl;
    decltype(omniClientMakeRelativeUrl) CARB_ABI* makeRelativeUrl;
    decltype(omniClientMakePrintable) CARB_ABI* makePrintable;
    decltype(omniClientRegisterAuthCallback) CARB_ABI* registerAuthCallback;
    decltype(omniClientSetAuthenticationMessageBoxCallback) CARB_ABI* setAuthenticationMessageBoxCallback;
    decltype(omniClientAuthenticationCancel) CARB_ABI* authenticationCancel;
    decltype(omniClientSignOut) CARB_ABI* signOut;
    decltype(omniClientReconnect) CARB_ABI* reconnect;
    decltype(omniClientGetServerInfo) CARB_ABI* getServerInfo;
    decltype(omniClientStat) CARB_ABI* stat;
    decltype(omniClientStatSubscribe) CARB_ABI* statSubscribe;
    decltype(omniClientResolve) CARB_ABI* resolve;
    decltype(omniClientResolveSubscribe) CARB_ABI* resolveSubscribe;
    decltype(omniClientAddDefaultSearchPath) CARB_ABI* addDefaultSearchPath;
    decltype(omniClientRemoveDefaultSearchPath) CARB_ABI* removeDefaultSearchPath;
    decltype(omniClientGetDefaultSearchPaths) CARB_ABI* getDefaultSearchPaths;
    decltype(omniClientPushBaseUrl) CARB_ABI* pushBaseUrl;
    decltype(omniClientPopBaseUrl) CARB_ABI* popBaseUrl;
    decltype(omniClientList) CARB_ABI* list;
    decltype(omniClientListSubscribe) CARB_ABI* listSubscribe;
    decltype(omniClientDelete) CARB_ABI* _delete;
    decltype(omniClientCopy) CARB_ABI* copy;
    decltype(omniClientCreateFolder) CARB_ABI* createFolder;
    decltype(omniClientWriteFile) CARB_ABI* writeFile;
    decltype(omniClientReadFile) CARB_ABI* readFile;
    decltype(omniClientJoinChannel) CARB_ABI* joinChannel;
    decltype(omniClientSendMessage) CARB_ABI* sendMessage;
    decltype(omniClientGetAcls) CARB_ABI* getAcls;
    decltype(omniClientSetAcls) CARB_ABI* setAcls;
    decltype(omniClientLock) CARB_ABI* lock;
    decltype(omniClientUnlock) CARB_ABI* unlock;
    decltype(omniClientCreateCheckpoint) CARB_ABI* createCheckpoint;
    decltype(omniClientListCheckpoints) CARB_ABI* listCheckpoints;
    decltype(omniClientGetBranchAndCheckpointFromQuery) CARB_ABI* getBranchAndCheckpointFromQuery;
    decltype(omniClientFreeBranchAndCheckpoint) CARB_ABI* freeBranchAndCheckpoint;
    decltype(omniClientMakeQueryFromBranchAndCheckpoint) CARB_ABI* makeQueryFromBranchAndCheckpoint;

    decltype(omniClientLiveSetQueuedCallback) CARB_ABI* liveSetQUeuedCallback;
    decltype(omniClientLiveProcess) CARB_ABI* liveProcess;
    decltype(omniClientLiveProcessUpTo) CARB_ABI* liveProcessUpTo;
    decltype(omniClientLiveGetLatestServerTime) CARB_ABI* liveGetLatestServerTime;
    decltype(omniClientLiveWaitForPendingUpdates) CARB_ABI* liveWaitForPendingUpdates;
    decltype(omniClientLiveConfigureJitterReduction) CARB_ABI* liveConfigureJitterReduction;

    // Called to indicate that omniClientShutdown should be called before unload
    void(CARB_ABI* shutdownOnUnload)();

    // was missing when the omniclient version changed, so it's at the end
    decltype(omniClientGetLocalFile) CARB_ABI* getLocalFile;
    decltype(omniClientBreakUrlReference) CARB_ABI* breakUrlReference;

    decltype(omniClientAllocContent) CARB_ABI* allocContent;
    decltype(omniClientReferenceContent) CARB_ABI* referenceContent;

    // These can't use declype because they have overloads
    OmniClientContent(CARB_ABI* moveContent)(OmniClientContent& content) noexcept;
    OmniClientContent(CARB_ABI* copyContent)(OmniClientContent const& content) noexcept;
    void(CARB_ABI* freeContent)(OmniClientContent& content) noexcept;

    decltype(omniClientGetOmniHubVersion) CARB_ABI* getOmniHubVersion;

    // Deprecated Use blockCachePutOpen() instead
    void(CARB_ABI* kvCacheSet)();
    // Deprecated Use blockCacheGetOpen() instead
    void(CARB_ABI* kvCacheGet)();
    // Deprecated Use blockCacheStat() instead
    void(CARB_ABI* kvCacheStat)();

    decltype(omniClientTraceStart) CARB_ABI* traceStart;
    decltype(omniClientTraceStop) CARB_ABI* traceStop;

    decltype(omniClientBlockCacheGetOpen) CARB_ABI* blockCacheGetOpen;
    decltype(omniClientBlockCacheGetClose) CARB_ABI* blockCacheGetClose;
    decltype(omniClientBlockCachePutOpen) CARB_ABI* blockCachePutOpen;
    decltype(omniClientBlockCachePutCommit) CARB_ABI* blockCachePutCommit;
    decltype(omniClientBlockCachePutAbort) CARB_ABI* blockCachePutAbort;
    decltype(omniClientBlockCacheStat) CARB_ABI* blockCacheStat;

    bool(CARB_ABI* isInsideLiveProcess)();
    
    decltype(omniClientRegisterMetricsCallback) CARB_ABI* registerMetricsCallback;
};

} // namespace omniclient
} // namespace carb
