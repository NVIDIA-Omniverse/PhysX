// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/tasking/TaskingUtils.h>

#include "GraphShared.h"
#include <omni/graph/core/Handle.h>

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <private/omni/physx/PhysxUsd.h>

#include <PxPhysicsAPI.h>
#include <PxImmediateMode.h>
namespace omni
{
namespace physx
{
namespace graph
{


static_assert(int(ContactEventType::eCONTACT_FOUND) < 64 && int(ContactEventType::eCONTACT_LOST) < 64 &&
                  int(ContactEventType::eCONTACT_PERSIST) < 64,
              "ContactEventType enums exceeds 63, bitwise flags will be invalid!");


// Bitwise flags to filter types.
struct ContactFilterFlags
{
    enum Enum
    {
        eFOUND = 1 << ContactEventType::eCONTACT_FOUND,
        eLOST = 1 << ContactEventType::eCONTACT_LOST,
        ePERSIST = 1 << ContactEventType::eCONTACT_PERSIST
    };
};

// Each node has an object of this class that the Physx contact reporter callback will write to
// and the node read from on compute. The contact reporter will not replace data but only write to the end
// of the existing.
class OgnContactReportData
{
public:
    OgnContactReportData() : m_contactTypeFilter(0), m_nodeHandle(kInvalidNodeHandle), m_bReportFullData(false){};
    ~OgnContactReportData();

    std::vector<ContactEventHeader> mContactHeaders;
    std::vector<ContactData> mContactData;

    // Adds the input contact event info to the staged contact data.
    void add(const ContactEventHeader& header, const ContactData* data);

    // Clears staged contact headers and data.
    void clear();

    void setContactTypeFilter(uint64_t nFilterFlags)
    {
        m_contactTypeFilter = nFilterFlags;
    };

    uint64_t getContactTypeFilter()
    {
        return m_contactTypeFilter;
    };

    void setNodeHandle(NodeHandle nodeHandle)
    {
        m_nodeHandle = nodeHandle;
    };

    NodeHandle getNodeHandle()
    {
        return m_nodeHandle;
    };

    void setReportsFullData(bool bReportFullData)
    {
        m_bReportFullData = bReportFullData;
    };

    bool filter(const ContactEventHeader& contact)
    {
        return (m_contactTypeFilter & ((uint64_t)1 << contact.type));
    };

private:
    uint64_t m_contactTypeFilter;
    NodeHandle m_nodeHandle;
    bool m_bReportFullData;
};

// Shared object between nodes that handles the physx contact report subscription and writes out contact report data to
// the individual nodes as relevant along with triggering their execution.
class ContactEventHandler
{
public:
    static void onContactEvent(const ContactEventHeader* eventHeaders,
                               uint32_t numEventHeaders,
                               const ContactData* contactData,
                               uint32_t numContactData,
                               void* userData);
    void subscribeNodeToPhysxPathContacts(OgnContactReportData* nodeData,
                                                 const uint64_t* nPhysxPaths,
                                                 size_t nNumPaths = 1);
    void releaseNodeSubscriptions(OgnContactReportData* nodeData);
    void refreshNodeSubscriptions(OgnContactReportData* nodeData, const uint64_t* nPhysxPaths, size_t nNumPaths = 1);

    ContactEventHandler();
    ~ContactEventHandler();

    // Maps between Physx paths and per node contact data.
    std::unordered_multimap<uint64_t, OgnContactReportData*> m_mPhysxPathsToNodeData;
private:
    omni::physx::SubscriptionId m_contactReportSubscriptionID;
    mutable carb::tasking::SharedMutexWrapper m_mutex;
};

// Allows lazy initialization, prevents static initialization fiasco causing the carbonite mutex to fail.
inline ContactEventHandler* acquireContactEventHandler()
{
    static ContactEventHandler handler;
    return &handler;
}

} // namespace graph
} // namespace physx
} // namespace omni
