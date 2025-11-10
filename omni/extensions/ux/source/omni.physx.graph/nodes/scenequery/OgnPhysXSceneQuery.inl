// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//

#include "../plugins/SceneQueryShared.h"
#include <string>
using namespace omni::physx;
using namespace omni::physx::graph;
using namespace omni::graph::core;

template <typename SceneQueryNodeType, typename SceneQueryDatabaseType>
class OgnPhysXSceneQuery
{
public:

    struct DeprecatedAttribute
    {   
        const NameToken attrName;
        const std::string strWarning;
        bool bConnected = false;
        DeprecatedAttribute(const NameToken& attrName, const std::string& strWarning) : attrName(attrName), strWarning(std::string(strWarning)) { };
    };

    std::vector<DeprecatedAttribute> deprecatedAttributes;

    static bool GetIsDeprecatedAttributeConnected(SceneQueryDatabaseType db, const NameToken& attrName)
    {
        auto& state = db.template sharedState<SceneQueryNodeType>();
        for(const DeprecatedAttribute& deprAttrib : state.deprecatedAttributes)
        {
            if(attrName == deprAttrib.attrName)
            {
                return deprAttrib.bConnected;
            }
        }
        return false;
    }

    static void SetAttributeDeprecated(const NodeObj& nodeObj, const NameToken& attrName, const char* strWarning)
    {
        SceneQueryDatabaseType db(nodeObj);
        auto& state = db.template sharedState<SceneQueryNodeType>();
        state.deprecatedAttributes.emplace_back(DeprecatedAttribute(attrName, strWarning));
    }

    static void onConnectionChanged(AttributeObj const& srcAttr, AttributeObj const& dstAttr, void* userData, bool bConnected)
    {
        NodeHandle nodeHandle = reinterpret_cast<NodeHandle>(userData);
        NodeObj nodeObj = srcAttr.iAttribute->getNode(srcAttr);

        // Check that this node is the connection source.
        if (nodeObj.nodeHandle != nodeHandle)
        {
            return;
        }

        SceneQueryDatabaseType db(nodeObj);
        auto& state = db.template sharedState<SceneQueryNodeType>();
        for(DeprecatedAttribute& deprecated : state.deprecatedAttributes)
        {
            const AttributeObj& attribute = nodeObj.iNode->getAttributeByToken(nodeObj, deprecated.attrName);
            if(attribute.attributeHandle == srcAttr.attributeHandle)
            {
                deprecated.bConnected = bConnected;
                if(bConnected) db.logWarning(deprecated.strWarning.c_str());
            }
        }
    }

    static void onConnected(AttributeObj const& srcAttr, AttributeObj const& dstAttr, void* userData)
    {
        onConnectionChanged(srcAttr, dstAttr, userData, true );
    }

    static void onDisconnected(AttributeObj const& srcAttr, AttributeObj const& dstAttr, void* userData)
    {
        onConnectionChanged(srcAttr, dstAttr, userData, false );
    }

    static void SetConnectionCallbacks(const GraphContextObj& context, const NodeObj& nodeObj)
    {
        struct ConnectionCallback connectedCallback = {onConnected, (void*) nodeObj.nodeHandle};
        nodeObj.iNode->registerConnectedCallback(nodeObj, connectedCallback);
        struct ConnectionCallback disconnectedCallback = {onDisconnected, (void*) nodeObj.nodeHandle};
        nodeObj.iNode->registerDisconnectedCallback(nodeObj, disconnectedCallback);
    }

};
