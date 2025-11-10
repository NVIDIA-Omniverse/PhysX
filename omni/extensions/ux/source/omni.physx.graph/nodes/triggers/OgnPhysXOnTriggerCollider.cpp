// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"


#include <OgnPhysXOnTriggerColliderDatabase.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/physx/IPhysxSimulation.h>

#include <pxr/usd/sdf/path.h>

#include "../plugins/GraphShared.h"

class OgnPhysXOnTriggerCollider
{
    using TriggerNodeType = OgnPhysXOnTriggerCollider;
    using TriggerDatabaseType = OgnPhysXOnTriggerColliderDatabase;

    omni::physx::IPhysxSimulation* mPhysxSimulationInterface = nullptr;
    static constexpr omni::physx::SubscriptionId InvalidSubscriptionId = std::numeric_limits<omni::physx::SubscriptionId>::max();  // 0 is a valid subscription id
    omni::physx::SubscriptionId mTriggerSubscriptionId = InvalidSubscriptionId;
    std::vector<omni::physx::TriggerEventData> bufferedTriggers;
    std::vector<NameToken> pathsToListen;
    std::vector<NameToken> currentPaths;
    std::vector<omni::physx::SubscriptionId> pathsSubscriptions;
    bool registerCatchAll = false;
    bool lastTimeEmittedEnter = false;
    std::string permanentError;

public:
    // Releases all subscriptions this node has with the trigger reports from SimulationInterface
    void releaseSubscription()
    {
        if (!mPhysxSimulationInterface)
        {
            mPhysxSimulationInterface = carb::getCachedInterface<omni::physx::IPhysxSimulation>();
        }
        mPhysxSimulationInterface->unsubscribePhysicsTriggerReportEvents(mTriggerSubscriptionId);
        mTriggerSubscriptionId = InvalidSubscriptionId;
        for (size_t idx = 0; idx < pathsSubscriptions.size(); ++idx)
        {
            mPhysxSimulationInterface->unsubscribePhysicsTriggerReportEvents(pathsSubscriptions[idx]);
        }
        pathsSubscriptions.clear();
    }

    // This is the functiona lled from SimulationInteface to notify us of incoming Trigger data
    // We just filter if it's an enter or leave event and eventually invoke the request compute
    static void OnTriggerEventReportFunction(const omni::physx::TriggerEventData* triggerData, void* userData)
    {
        NodeHandle nodeHandle = (NodeHandle)userData;
        auto iNode = carb::getCachedInterface<omni::graph::core::INode>();
        NodeObj nodeObj = iNode->getNodeFromHandle(nodeHandle);
        if (nodeObj.isValid())
        {
            auto& state = TriggerDatabaseType::template sSharedState<TriggerNodeType>(nodeObj);
            state.bufferedTriggers.push_back(*triggerData);
            nodeObj.iNode->requestCompute(nodeObj);
        }
        else
        {
            // This shouldn't be possible, but a bit of defensive coding doesn't hurt
            CARB_LOG_ERROR(
                "FATAL: Invalid node in Trigger Report Function for subscription %d. Release method on node was not called, forcing unsubscribe from trigger events notification.",
                (int)triggerData->subscriptionId);
            carb::getCachedInterface<omni::physx::IPhysxSimulation>()->unsubscribePhysicsTriggerReportEvents(
                triggerData->subscriptionId);
        }
    }

    static void getInputPaths(TriggerDatabaseType& db, std::vector<NameToken>& pathVector)
    {
        if (db.inputs.triggersPaths().isValid())
        {
            const size_t size = db.inputs.triggersPaths().size();
            pathVector.resize(size);
            for (size_t i = 0; i < size; ++i)
            {
                pathVector[i] = db.inputs.triggersPaths()[i];
            }
        }
        omni::physx::graph::appendRelationshipPrimPathsToNameTokenArray(
            db, pathVector, inputs::triggersRelationships.m_token);
    }


    void subscribe(const GraphContextObj& graphContextObj, const NodeObj& nodeObj, const AttributeObj& attrObj)
    {
        auto& state = TriggerDatabaseType::template sSharedState<TriggerNodeType>(nodeObj);
        uint64_t stageId = graphContextObj.iContext->getStageId(graphContextObj);
        TriggerDatabaseType db(nodeObj);
        pxr::UsdStageWeakPtr stagePtr =
            pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(static_cast<long int>(stageId)));

        permanentError.clear();
        if (state.registerCatchAll)
        {
            mTriggerSubscriptionId = mPhysxSimulationInterface->subscribePhysicsTriggerReportEvents(
                stageId, 0, &OgnPhysXOnTriggerCollider::OnTriggerEventReportFunction, (void*)nodeObj.nodeHandle);
        }
        else
        {
            int numErrors = 0;
            const int MaxNumErrors = 10;
            for (auto it : pathsToListen)
            {
                pxr::SdfPath primPath = pxr::SdfPath(db.tokenToString(it));
                if (stagePtr)
                {
                    pxr::UsdPrim prim = stagePtr->GetPrimAtPath(primPath);
                    if (!prim.IsValid())
                    {
                        if (numErrors < MaxNumErrors)
                        {
                            permanentError +=
                                formatString("Prim '%s' is not valid or doesn't exist\n", primPath.GetText());
                        }
                        numErrors++;
                    }
                    else if (!prim.HasAPI<pxr::PhysxSchemaPhysxTriggerAPI>() &&
                             !prim.HasAPI<pxr::PhysxSchemaPhysxTriggerStateAPI>())
                    {
                        if (numErrors < MaxNumErrors)
                        {
                            permanentError += formatString(
                                "Prim '%s' needs Trigger API for this node to receive trigger notifications (PhysxSchemaPhysxTriggerAPI or PhysxSchemaPhysxTriggerStateAPI)\n",
                                primPath.GetText());
                        }
                        numErrors++;
                    }
                }
                uint64_t triggerUsdPrim = omni::fabric::asInt(primPath).path;
                auto subId = mPhysxSimulationInterface->subscribePhysicsTriggerReportEvents(
                    stageId, triggerUsdPrim, &OgnPhysXOnTriggerCollider::OnTriggerEventReportFunction,
                    (void*)nodeObj.nodeHandle);
                pathsSubscriptions.push_back(subId);
            }
            if (numErrors > MaxNumErrors)
            {
                permanentError += formatString("...\nAnd additional %d errors.", numErrors - MaxNumErrors);
            }
        }
    }

    // Called by OG when our state attrib changes.
    static void onValueChanged(const AttributeObj& attrObj, const void* userData)
    {
        NodeObj nodeObj = attrObj.iAttribute->getNode(attrObj);

        // Just some sanity checks, they should probably not even be necessary
        if (nodeObj.nodeHandle == kInvalidNodeHandle)
            return;

        GraphObj graphObj = nodeObj.iNode->getGraph(nodeObj);
        if (!graphObj.isValid())
            return;

        auto& state = TriggerDatabaseType::template sSharedState<TriggerNodeType>(nodeObj);
        TriggerDatabaseType db(nodeObj);
        refreshSubscriptionsIfNecessary(db);
    }

    // This method might be overridden to set up initial conditions when a node type is registered, or
    // to replace initialization if the auto-generated version has some problem.
    static void initialize(const GraphContextObj& graphContextObj, const NodeObj& nodeObj)
    {
        auto& state = TriggerDatabaseType::template sSharedState<TriggerNodeType>(nodeObj);
        state.mPhysxSimulationInterface = carb::getCachedInterface<omni::physx::IPhysxSimulation>();

        // We need to register valueChangedCallbacks for all the three input parameters

        AttributeObj triggersPathAttribute = nodeObj.iNode->getAttributeByToken(nodeObj, inputs::triggersPaths.m_token);
        triggersPathAttribute.iAttribute->registerValueChangedCallback(triggersPathAttribute, onValueChanged, false);

        AttributeObj triggersRelationshipAttribute =
            nodeObj.iNode->getAttributeByToken(nodeObj, inputs::triggersRelationships.m_token);
        triggersRelationshipAttribute.iAttribute->registerValueChangedCallback(
            triggersRelationshipAttribute, onValueChanged, false);

        AttributeObj listenToAllTriggersAttribute =
            nodeObj.iNode->getAttributeByToken(nodeObj, inputs::listenToAllTriggers.m_token);
        listenToAllTriggersAttribute.iAttribute->registerValueChangedCallback(
            listenToAllTriggersAttribute, onValueChanged, false);

        TriggerDatabaseType db(nodeObj);
        refreshSubscriptionsIfNecessary(db);
    }

    // After a node is removed it will get a release call where anything set up in initialize() can be torn down
    static void release(const NodeObj& nodeObj)
    {
        auto& state = TriggerDatabaseType::template sSharedState<TriggerNodeType>(nodeObj);
        state.releaseSubscription();
    }

    // Reads the input, collects trigger paths to watch and eventually refreshes trigger subscription on
    // SimulationInterface
    static void refreshSubscriptionsIfNecessary(TriggerDatabaseType& db)
    {
        TriggerNodeType& state = db.template sharedState<TriggerNodeType>();
        bool refreshSubscriptions = db.inputs.listenToAllTriggers() != state.registerCatchAll;
        state.registerCatchAll = db.inputs.listenToAllTriggers();
        state.currentPaths.clear();
        if (state.registerCatchAll)
        {
            refreshSubscriptions |= state.mTriggerSubscriptionId == InvalidSubscriptionId;
        }
        else
        {
            getInputPaths(db, state.currentPaths);
            refreshSubscriptions |= !std::equal(state.currentPaths.begin(), state.currentPaths.end(),
                                                state.pathsToListen.begin(), state.pathsToListen.end());
        }
        if (refreshSubscriptions)
        {
            const GraphContextObj& graphContextObj = db.abi_context();
            state.releaseSubscription();
            state.pathsToListen = state.currentPaths;
            const NodeObj& nodeObj = db.abi_node();
            AttributeObj attrObj = nodeObj.iNode->getAttributeByToken(nodeObj, inputs::triggersPaths.m_token);
            state.subscribe(graphContextObj, nodeObj, attrObj);
        }
    }

    static bool compute(TriggerDatabaseType& db)
    {
        using namespace omni::graph::core::ogn;
        auto& state = db.template sharedState<TriggerNodeType>();
        // Here we send out all buffered notifications, one by one at each compute "iteration" until there are none
        if (!state.permanentError.empty())
        {
            db.logError("%s", state.permanentError.c_str());
        }
        auto& nodeObj = db.abi_node();
        const bool isExecutionEnterPinConnected = needOutput(nodeObj, outputs::enterExecOut.m_token);
        const bool isExecutionLeavePinConnected = needOutput(nodeObj, outputs::leaveExecOut.m_token);
        while (!state.bufferedTriggers.empty())
        {
            omni::physx::TriggerEventData tdata = state.bufferedTriggers.back();
            state.bufferedTriggers.pop_back();
            const bool isEnterEvent = tdata.eventType == omni::physx::TriggerEventType::eTRIGGER_ON_ENTER;
            const bool isLeaveEvent = tdata.eventType == omni::physx::TriggerEventType::eTRIGGER_ON_LEAVE;
            if ((isExecutionEnterPinConnected && isEnterEvent) || (isExecutionLeavePinConnected && isLeaveEvent))
            {
                db.outputs.triggerCollider() = omni::physx::graph::asNameToken(tdata.triggerColliderPrimId);
                db.outputs.otherCollider() = omni::physx::graph::asNameToken(tdata.otherColliderPrimId);
                db.outputs.triggerBody() = omni::physx::graph::asNameToken(tdata.triggerBodyPrimId);
                db.outputs.otherBody() = omni::physx::graph::asNameToken(tdata.otherBodyPrimId);
                if (isEnterEvent)
                {
                    db.outputs.enterExecOut() = state.bufferedTriggers.empty() ? kExecutionAttributeStateEnabled :
                                                                                 kExecutionAttributeStateEnabledAndPush;
                    db.outputs.leaveExecOut() = kExecutionAttributeStateDisabled;
                }
                else
                {
                    db.outputs.enterExecOut() = kExecutionAttributeStateDisabled;
                    db.outputs.leaveExecOut() = state.bufferedTriggers.empty() ? kExecutionAttributeStateEnabled :
                                                                                 kExecutionAttributeStateEnabledAndPush;
                }
                return true; // We actually output something, let's stop looping
            }
        }
        // we arrive here either if there's no buffered trigger or if none of the ones was matching type with the
        // currently connected execution pins (enter / leave)
        db.outputs.enterExecOut() = kExecutionAttributeStateDisabled;
        db.outputs.leaveExecOut() = kExecutionAttributeStateDisabled;
        return true;
    }

    inline static bool needOutput(const NodeObj& nodeObj, NameToken attrName)
    {
        const AttributeObj attr = nodeObj.iNode->getAttributeByToken(nodeObj, attrName);
        return attr.iAttribute->getDownstreamConnectionCount(attr);
    }
};

REGISTER_OGN_NODE()
