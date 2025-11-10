// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <omni/fabric/FabricUSD.h>
#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/Framework.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>

#include <PxPhysicsAPI.h>
#include <PxImmediateMode.h>
#include "../../plugins/GraphShared.h"
#include "../../plugins/ContactShared.h"

#include <pxr/usd/usdSkel/bindingAPI.h>
#include <pxr/usd/usdSkel/cache.h>
#include <pxr/usd/usdSkel/skeletonQuery.h>
#include <pxr/usd/usdSkel/utils.h>

using namespace omni::physx;
using namespace omni::physx::graph;
using namespace omni::graph::core;

template <typename ContactEventNodeType, typename ContactEventDatabaseType>
class OgnPhysXOnContactEvent
{
public:
    // This section contains state data. This is preserved between executions but is unique to the node instance.

    // Interface between PhysX contact reporter and this node.
    OgnContactReportData m_contactReportData;

    // Tracking of our current input paths to reduce how often we have to refresh setup.
    std::vector<NameToken> m_nameBodyPaths;
    std::unordered_set<uint64_t> m_nPhysxPaths; // PhysX paths.

    // This value is kept as part of the node state because when executing recursively, we should pick up iterating
    // inputs from where the last execution left off.
    size_t m_nContactHeaderArrayIndex  = 0;

    // Clears all contact data from the node state.
    static void resetContactReports(ContactEventDatabaseType& db)
    {
        auto& state = db.template sharedState<ContactEventNodeType>();
        state.m_contactReportData.mContactHeaders.clear();
        #if PHYSX_OGN_ON_CONTACT_EVENT_REPORT_FULL_DATA
            state.m_contactReportData.mContactData.clear();
        #endif
        state.m_nContactHeaderArrayIndex = 0;
    }

    // Determines if inputs have changed and refreshes subscriptions as relevant.
    static void refreshSubscriptionsIfNecessary(ContactEventDatabaseType& db)
    {
        auto& state = db.template sharedState<ContactEventNodeType>();
        const auto& contextObj = db.abi_context();
        const auto& nodeObj = db.abi_node();

        bool bRefreshSubscription = false;

        // Gather current body inputs.
        std::vector<NameToken> inputBodies;
        if(db.inputs.targetBodies.isValid())
        {
            appendRelationshipPrimPathsToNameTokenArray(db, inputBodies, inputs::targetBodies.m_token);
        }

        if(db.inputs.bodyPaths.isValid())
        {
            const auto& inputBodyPaths = db.inputs.bodyPaths();
            inputBodies.reserve(inputBodies.size() + inputBodyPaths.size());
            for(const auto inputPath : inputBodyPaths)
            {
                inputBodies.push_back(inputPath);
            }
        }

        // Check if any input paths have been updated.
        size_t nPathIndex;
        for(nPathIndex = 0; nPathIndex < std::min(inputBodies.size(), state.m_nameBodyPaths.size()); nPathIndex++)
        {
            if (state.m_nameBodyPaths[nPathIndex] != inputBodies[nPathIndex])
            {
                state.m_nameBodyPaths[nPathIndex] = inputBodies[nPathIndex];
                bRefreshSubscription = true;
            }
        }

        // Check if paths have been added...
        if(inputBodies.size() > state.m_nameBodyPaths.size())
        {
            state.m_nameBodyPaths.resize(inputBodies.size());
            bRefreshSubscription = true;
            for(; nPathIndex < state.m_nameBodyPaths.size(); nPathIndex++)
            {
                state.m_nameBodyPaths[nPathIndex] = inputBodies[nPathIndex];
            }
        }
        // ... or removed.
        else if(inputBodies.size() < state.m_nameBodyPaths.size())
        {
            state.m_nameBodyPaths.resize(inputBodies.size());
            bRefreshSubscription = true;
        }

        // Check if the contact type execution connections have changed.
        uint64_t nTypeFilter = 0;

        AttributeObj outputExecFound = nodeObj.iNode->getAttributeByToken(nodeObj, outputs::foundExecOut.m_token);
        if( outputExecFound.iAttribute->getDownstreamConnectionCount(outputExecFound) > 0)
        {
            nTypeFilter |= ContactFilterFlags::eFOUND;
        }
        AttributeObj outputExecPersists = nodeObj.iNode->getAttributeByToken(nodeObj, outputs::persistsExecOut.m_token);
        if( outputExecPersists.iAttribute->getDownstreamConnectionCount(outputExecPersists) > 0)
        {
            nTypeFilter |= ContactFilterFlags::ePERSIST;
        }
        AttributeObj outputExecLost = nodeObj.iNode->getAttributeByToken(nodeObj, outputs::lostExecOut.m_token);
        if( outputExecLost.iAttribute->getDownstreamConnectionCount(outputExecLost) > 0)
        {
            nTypeFilter |= ContactFilterFlags::eLOST;
        }

        if(nTypeFilter != state.m_contactReportData.getContactTypeFilter())
        {
            state.m_contactReportData.setContactTypeFilter(nTypeFilter);
            bRefreshSubscription = true;
        }

        if(bRefreshSubscription)
        {
            state.m_nPhysxPaths.clear();
            state.m_nPhysxPaths.reserve(inputBodies.size());

            // PhysX paths for contact report subscription.
            uint64_t* subscribePaths = new uint64_t[inputBodies.size()];

            for(size_t n = 0; n < inputBodies.size(); n++)
            {
                uint64_t nPhysxPath = toPhysX(inputBodies[n]);
                state.m_nPhysxPaths.insert(nPhysxPath);
                subscribePaths[n] = nPhysxPath;
            }

            acquireContactEventHandler()->refreshNodeSubscriptions(&state.m_contactReportData, subscribePaths, inputBodies.size());

            delete[] subscribePaths;

            // If any of the conditions have changed, the current data will be outdated.
            resetContactReports(db);
        }
    }

    static void onValueChanged(const AttributeObj& attrObj, const void* userData)
    {
        // Get the new value
        NodeObj nodeObj = attrObj.iAttribute->getNode(attrObj);
        if (nodeObj.nodeHandle == kInvalidNodeHandle)
            return;

        auto graphObj = nodeObj.iNode->getGraph(nodeObj);
        if (!graphObj.isValid())
            return;

        ContactEventDatabaseType db(nodeObj);
        refreshSubscriptionsIfNecessary(db);
    }


    static void onConnectionChanged(AttributeObj const& srcAttr, AttributeObj const& dstAttr, void* userData)
    {
        NodeHandle nodeHandle = reinterpret_cast<NodeHandle>(userData);
        // Check which attribute matches our handle (if any).
        NodeObj nodeObj = srcAttr.iAttribute->getNode(srcAttr);
        if (nodeObj.nodeHandle != nodeHandle)
        {
            nodeObj = dstAttr.iAttribute->getNode(dstAttr);
            if (nodeObj.nodeHandle != nodeHandle)
                return;
        }

        auto graphObj = nodeObj.iNode->getGraph(nodeObj);
        if (!graphObj.isValid())
            return;

        ContactEventDatabaseType db(nodeObj);
        refreshSubscriptionsIfNecessary(db);
    }

    static void initialize(const GraphContextObj& graphContextObj, const NodeObj& nodeObj)
    {
        auto& state = ContactEventDatabaseType::template sSharedState<ContactEventNodeType>(nodeObj);
        state.m_contactReportData.setNodeHandle(nodeObj.nodeHandle);
        state.m_contactReportData.setReportsFullData(PHYSX_OGN_ON_CONTACT_EVENT_REPORT_FULL_DATA);

        AttributeObj inputBodiesAttribute = nodeObj.iNode->getAttributeByToken(nodeObj, inputs::targetBodies.m_token);
        inputBodiesAttribute.iAttribute->registerValueChangedCallback(inputBodiesAttribute, onValueChanged, false);
        AttributeObj inputBodyPathsAttribute = nodeObj.iNode->getAttributeByToken(nodeObj, inputs::bodyPaths.m_token);
        inputBodyPathsAttribute.iAttribute->registerValueChangedCallback(inputBodyPathsAttribute, onValueChanged, false);

        struct ConnectionCallback connectedCallback = {onConnectionChanged, (void*) nodeObj.nodeHandle};
        nodeObj.iNode->registerConnectedCallback(nodeObj, connectedCallback);
        nodeObj.iNode->registerDisconnectedCallback(nodeObj, connectedCallback);

        ContactEventDatabaseType db(nodeObj);
        refreshSubscriptionsIfNecessary(db);
    }

    static void release(const NodeObj& nodeObj)
    {
        auto& state = ContactEventDatabaseType::template sSharedState<ContactEventNodeType>(nodeObj);
        acquireContactEventHandler()->releaseNodeSubscriptions(&state.m_contactReportData);

        auto graphObj = nodeObj.iNode->getGraph(nodeObj);
        if (!graphObj.isValid())
            return;

        ContactEventDatabaseType db(nodeObj);
        resetContactReports(db);
    }

    // This is run both whenever a compute request is sent from the ContactEventHandler but also during authoring the graph (creation, connecting nodes, etc.)
    static bool compute(ContactEventDatabaseType& db)
    {
        auto& state = db.template sharedState<ContactEventNodeType>();
        if(!state.m_contactReportData.mContactHeaders.size() || state.m_contactReportData.mContactHeaders.size() <= state.m_nContactHeaderArrayIndex)
        {
            // This should only happen during authoring.
            CARB_ASSERT(state.m_nContactHeaderArrayIndex == 0 && state.m_contactReportData.mContactHeaders.size() == 0);
            db.outputs.foundExecOut() = ExecutionAttributeState::kExecutionAttributeStateDisabled;
            db.outputs.persistsExecOut() = ExecutionAttributeState::kExecutionAttributeStateDisabled;
            db.outputs.lostExecOut() = ExecutionAttributeState::kExecutionAttributeStateDisabled;
            resetContactReports(db);
            return true;
        }

        bool bSwap;
        if(state.m_nPhysxPaths.find(state.m_contactReportData.mContactHeaders[state.m_nContactHeaderArrayIndex].actor0) != state.m_nPhysxPaths.end())
        {
            bSwap = false;
        }
        else
        {
            // It must be able to find at least one of the actors in the pair, since otherwise we should not have been triggered.
            CARB_ASSERT(state.m_nPhysxPaths.find(state.m_contactReportData.mContactHeaders[state.m_nContactHeaderArrayIndex].actor1) != state.m_nPhysxPaths.end());
            bSwap = true;
        }

        db.outputs.contactingBody() = asNameToken(bSwap ? state.m_contactReportData.mContactHeaders[state.m_nContactHeaderArrayIndex].actor0 : state.m_contactReportData.mContactHeaders[state.m_nContactHeaderArrayIndex].actor1);
        db.outputs.inputBody() = asNameToken(bSwap ? state.m_contactReportData.mContactHeaders[state.m_nContactHeaderArrayIndex].actor1 : state.m_contactReportData.mContactHeaders[state.m_nContactHeaderArrayIndex].actor0);

        #if PHYSX_OGN_ON_CONTACT_EVENT_REPORT_FULL_DATA

            db.outputs.contactingCollider() = asNameToken(bSwap ? state.m_contactReportData.mContactHeaders[state.m_nContactHeaderArrayIndex].collider0 : state.m_contactReportData.mContactHeaders[state.m_nContactHeaderArrayIndex].collider1);
            db.outputs.inputBodyCollider() = asNameToken(bSwap ? state.m_contactReportData.mContactHeaders[state.m_nContactHeaderArrayIndex].collider1 : state.m_contactReportData.mContactHeaders[state.m_nContactHeaderArrayIndex].collider0);

            int nNumContacts = state.m_contactReportData.mContactHeaders[state.m_nContactHeaderArrayIndex].numContactData;
            db.outputs.contactingFaces().resize(nNumContacts);
            db.outputs.inputBodyFaces().resize(nNumContacts);
            db.outputs.contactingMaterials().resize(nNumContacts);
            db.outputs.inputBodyMaterials().resize(nNumContacts);
            db.outputs.contactPoints().resize(nNumContacts);
            db.outputs.contactNormals().resize(nNumContacts);
            db.outputs.contactDepths().resize(nNumContacts);
            db.outputs.contactImpulses().resize(nNumContacts);

            for(int nContact = 0;
                nContact < nNumContacts; nContact++)
            {
                uint32_t nDataIndex = state.m_contactReportData.mContactHeaders[state.m_nContactHeaderArrayIndex].contactDataOffset + nContact;
                db.outputs.contactingFaces()[nContact] = bSwap ? state.m_contactReportData.mContactData[nDataIndex].faceIndex0 : state.m_contactReportData.mContactData[nDataIndex].faceIndex1;
                db.outputs.inputBodyFaces()[nContact] = bSwap ? state.m_contactReportData.mContactData[nDataIndex].faceIndex1 : state.m_contactReportData.mContactData[nDataIndex].faceIndex0;
                db.outputs.contactingMaterials()[nContact] = bSwap ? (int)state.m_contactReportData.mContactData[nDataIndex].material0 : (int)state.m_contactReportData.mContactData[nDataIndex].material1;
                db.outputs.inputBodyMaterials()[nContact] = bSwap ? (int)state.m_contactReportData.mContactData[nDataIndex].material1 : (int)state.m_contactReportData.mContactData[nDataIndex].material0;
                db.outputs.contactPoints()[nContact] = state.m_contactReportData.mContactData[nDataIndex].position;
                db.outputs.contactNormals()[nContact] = state.m_contactReportData.mContactData[nDataIndex].normal;
                db.outputs.contactDepths()[nContact] = state.m_contactReportData.mContactData[nDataIndex].separation;
                db.outputs.contactImpulses()[nContact] = state.m_contactReportData.mContactData[nDataIndex].impulse;
            }
        
        # endif

        ContactEventType::Enum eContactType = state.m_contactReportData.mContactHeaders[state.m_nContactHeaderArrayIndex].type;

        state.m_nContactHeaderArrayIndex++;

        if(state.m_nContactHeaderArrayIndex == state.m_contactReportData.mContactHeaders.size())
        {
            db.outputs.foundExecOut() = (eContactType == ContactEventType::eCONTACT_FOUND ? ExecutionAttributeState::kExecutionAttributeStateEnabled : ExecutionAttributeState::kExecutionAttributeStateDisabled);
            db.outputs.persistsExecOut() = (eContactType == ContactEventType::eCONTACT_PERSIST ? ExecutionAttributeState::kExecutionAttributeStateEnabled : ExecutionAttributeState::kExecutionAttributeStateDisabled);
            db.outputs.lostExecOut() = (eContactType == ContactEventType::eCONTACT_LOST ? ExecutionAttributeState::kExecutionAttributeStateEnabled : ExecutionAttributeState::kExecutionAttributeStateDisabled);
            resetContactReports(db);
        }
        else
        {
            // By using the push variant, we trigger recomputing after executing, allowing us to process the next in the buffer.
            db.outputs.foundExecOut() = (eContactType == ContactEventType::eCONTACT_FOUND ? ExecutionAttributeState::kExecutionAttributeStateEnabledAndPush : ExecutionAttributeState::kExecutionAttributeStateDisabled);
            db.outputs.persistsExecOut() = (eContactType == ContactEventType::eCONTACT_PERSIST ? ExecutionAttributeState::kExecutionAttributeStateEnabledAndPush : ExecutionAttributeState::kExecutionAttributeStateDisabled);
            db.outputs.lostExecOut() = (eContactType == ContactEventType::eCONTACT_LOST ? ExecutionAttributeState::kExecutionAttributeStateEnabledAndPush : ExecutionAttributeState::kExecutionAttributeStateDisabled);
        }

        return true;
    }
};
