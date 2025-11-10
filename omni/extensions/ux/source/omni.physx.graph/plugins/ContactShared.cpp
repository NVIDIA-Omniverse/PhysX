// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "ContactShared.h"

#include <carb/tasking/TaskingUtils.h>

#include <map>
#include <memory> 

namespace omni
{
namespace physx
{
namespace graph
{

OgnContactReportData::~OgnContactReportData()
{
    acquireContactEventHandler()->releaseNodeSubscriptions(this);
    clear();
}

void OgnContactReportData::add(const ContactEventHeader &header, const ContactData* data)
{
    uint32_t nCurrentHeaderSize = (uint32_t)mContactHeaders.size();
    mContactHeaders.push_back(header);
    if(m_bReportFullData)
    {
        uint32_t nCurrentDataSize = (uint32_t)mContactData.size();
        // Make room for copying in additional data.
        mContactData.resize(nCurrentDataSize + (size_t)header.numContactData);

        // Copy in the data after the existing. The nodes themselves are responsible for cleaning this up after reading.
        memcpy(mContactData.data() + nCurrentDataSize, data + header.contactDataOffset, sizeof(ContactData) * header.numContactData);

        // Shift data offsets.
        mContactHeaders[nCurrentHeaderSize].contactDataOffset = nCurrentDataSize;
    }
}

void OgnContactReportData::clear()
{
    mContactHeaders.clear();
    mContactData.clear();
}

ContactEventHandler::ContactEventHandler()
{
    m_contactReportSubscriptionID = kInvalidSubscriptionId;
};

ContactEventHandler::~ContactEventHandler()
{
};

void ContactEventHandler::onContactEvent(const ContactEventHeader* eventHeaders, uint32_t numEventHeaders, const ContactData* contactData,
    uint32_t numContactData, void* userData)
{
    auto iNode = carb::getCachedInterface<omni::graph::core::INode>();

    const ContactEventHandler* handler = acquireContactEventHandler();

    std::lock_guard<carb::tasking::SharedMutexWrapper> shared_lock(handler->m_mutex);
    if(!handler->m_mPhysxPathsToNodeData.empty())
    {
        for(size_t header = 0; header < numEventHeaders; header++)
        {
            for(size_t actor = 0; actor < 2; actor++)
            {
                auto range = handler->m_mPhysxPathsToNodeData.equal_range((actor == 0 ? eventHeaders[header].actor0 : eventHeaders[header].actor1));
                for (auto subscription = range.first; subscription != range.second; subscription++)
                {
                    OgnContactReportData* data = subscription->second;
                    if(data->filter(eventHeaders[header]))
                    {
                        data->add(eventHeaders[header], contactData);
                        NodeObj nodeObj = iNode->getNodeFromHandle(data->getNodeHandle());
                        iNode->requestCompute(nodeObj);
                    }
                }
            }
        }
    }
}

void ContactEventHandler::subscribeNodeToPhysxPathContacts(OgnContactReportData* nodeData, const uint64_t* nPhysxPaths, size_t nNumPaths)
{
    std::lock_guard<carb::tasking::SharedMutexWrapper> lock(m_mutex);

    if(m_contactReportSubscriptionID == kInvalidSubscriptionId)
    {
        omni::physx::IPhysxSimulation* physxInterface = carb::getCachedInterface<omni::physx::IPhysxSimulation>();
        if(physxInterface)
        {
            m_contactReportSubscriptionID = physxInterface->subscribePhysicsContactReportEvents(
                &onContactEvent, nullptr);
        }
    }
    for(size_t n = 0; n < nNumPaths; n++)
    {
        m_mPhysxPathsToNodeData.insert({nPhysxPaths[n], nodeData});
    }
};

void ContactEventHandler::releaseNodeSubscriptions(OgnContactReportData* nodeData)
{
    std::lock_guard<carb::tasking::SharedMutexWrapper> lock(m_mutex);

    // Remove all subscriptions that point to this node.
    for (auto subscription = m_mPhysxPathsToNodeData.begin(); subscription != m_mPhysxPathsToNodeData.end();)
    {
        if(subscription->second == nodeData)
        {
            subscription = m_mPhysxPathsToNodeData.erase(subscription);
        }
        else
        {
            subscription++;
        }
    }

    // If there's no subscriptsions left, delete the handler.
    if(m_mPhysxPathsToNodeData.empty() && m_contactReportSubscriptionID != kInvalidSubscriptionId)
    {
        omni::physx::IPhysxSimulation* physxInterface = carb::getCachedInterface<omni::physx::IPhysxSimulation>();
        if(physxInterface)
        {
            physxInterface->unsubscribePhysicsContactReportEvents(m_contactReportSubscriptionID);
        }
        m_contactReportSubscriptionID = kInvalidSubscriptionId;
    }
};

void ContactEventHandler::refreshNodeSubscriptions(OgnContactReportData* nodeData, const uint64_t* nPhysxPaths, size_t nNumPaths)
{
    releaseNodeSubscriptions(nodeData);
    if(nodeData->getContactTypeFilter())
    {
        ContactEventHandler::subscribeNodeToPhysxPathContacts(nodeData, nPhysxPaths, nNumPaths);
    }
}
} // namespace graph
} // namespace physx
} // namespace omni
