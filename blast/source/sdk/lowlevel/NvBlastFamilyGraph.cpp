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
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.


#include "NvBlastFamilyGraph.h"

#include "NvBlastAssert.h"

#include <vector>
#include <stack>

#define SANITY_CHECKS 0

namespace Nv
{
namespace Blast
{


size_t FamilyGraph::fillMemory(FamilyGraph* familyGraph, uint32_t nodeCount, uint32_t bondCount)
{
    // calculate all offsets, and dataSize as a result
    NvBlastCreateOffsetStart(sizeof(FamilyGraph));
    const size_t NvBlastCreateOffsetAlign16(dirtyNodeLinksOffset, sizeof(NodeIndex) * nodeCount);
    const size_t NvBlastCreateOffsetAlign16(firstDirtyNodeIndicesOffset, sizeof(uint32_t) * nodeCount);
    const size_t NvBlastCreateOffsetAlign16(islandIdsOffset, sizeof(IslandId) * nodeCount);
    const size_t NvBlastCreateOffsetAlign16(fastRouteOffset, sizeof(NodeIndex) * nodeCount);
    const size_t NvBlastCreateOffsetAlign16(hopCountsOffset, sizeof(uint32_t) * nodeCount);
    const size_t NvBlastCreateOffsetAlign16(isEdgeRemovedOffset, FixedBoolArray::requiredMemorySize(bondCount));
    const size_t NvBlastCreateOffsetAlign16(isNodeInDirtyListOffset, FixedBoolArray::requiredMemorySize(nodeCount));
    const size_t dataSize = NvBlastCreateOffsetEndAlign16();

    // fill only if familyGraph was passed (otherwise we just used this function to get dataSize)
    if (familyGraph)
    {
        familyGraph->m_dirtyNodeLinksOffset         = static_cast<uint32_t>(dirtyNodeLinksOffset);
        familyGraph->m_firstDirtyNodeIndicesOffset  = static_cast<uint32_t>(firstDirtyNodeIndicesOffset);
        familyGraph->m_islandIdsOffset              = static_cast<uint32_t>(islandIdsOffset);
        familyGraph->m_fastRouteOffset              = static_cast<uint32_t>(fastRouteOffset);
        familyGraph->m_hopCountsOffset              = static_cast<uint32_t>(hopCountsOffset);
        familyGraph->m_isEdgeRemovedOffset          = static_cast<uint32_t>(isEdgeRemovedOffset);
        familyGraph->m_isNodeInDirtyListOffset      = static_cast<uint32_t>(isNodeInDirtyListOffset);

        new (familyGraph->getIsEdgeRemoved()) FixedBoolArray(bondCount);
        new (familyGraph->getIsNodeInDirtyList()) FixedBoolArray(nodeCount);
    }

    return dataSize;
}


FamilyGraph::FamilyGraph(uint32_t nodeCount, const uint32_t bondCount)
{
    // fill memory with all internal data
    // we need chunks count for size calculation
    fillMemory(this, nodeCount, bondCount);

    // fill arrays with invalid indices / max value (0xFFFFFFFF)
    memset(getIslandIds(), 0xFF, nodeCount*sizeof(uint32_t));
    memset(getFastRoute(), 0xFF, nodeCount*sizeof(uint32_t));
    memset(getHopCounts(), 0xFF, nodeCount*sizeof(uint32_t));       // Initializing to large value
    memset(getDirtyNodeLinks(), 0xFF, nodeCount*sizeof(uint32_t));  // No dirty list initially
    memset(getFirstDirtyNodeIndices(), 0xFF, nodeCount*sizeof(uint32_t));

    getIsNodeInDirtyList()->clear();
    getIsEdgeRemoved()->fill();
}


/**
Graph initialization, reset all internal data to initial state. Marks all nodes dirty for this actor.
First island search probably would be the longest one, as it has to traverse whole graph and set all the optimization stuff like fastRoute and hopCounts for all nodes.
*/
void FamilyGraph::initialize(ActorIndex actorIndex, const SupportGraph* graph)
{
    // used internal data pointers
    NodeIndex* dirtyNodeLinks = getDirtyNodeLinks();
    uint32_t* firstDirtyNodeIndices = getFirstDirtyNodeIndices();

    // link dirty nodes
    for (NodeIndex node = 1; node < graph->m_nodeCount; node++)
    {
        dirtyNodeLinks[node-1] = node;
    }
    firstDirtyNodeIndices[actorIndex] = 0;

    getIsNodeInDirtyList()->fill();
    getIsEdgeRemoved()->clear();
}


void FamilyGraph::addToDirtyNodeList(ActorIndex actorIndex, NodeIndex node)
{
    // used internal data pointers
    FixedBoolArray* isNodeInDirtyList = getIsNodeInDirtyList();
    NodeIndex* dirtyNodeLinks = getDirtyNodeLinks();
    uint32_t* firstDirtyNodeIndices = getFirstDirtyNodeIndices();

    // check for bitmap first for avoid O(n) list search
    if (isNodeInDirtyList->test(node))
        return;

    // add node to dirty node list head
    dirtyNodeLinks[node] = firstDirtyNodeIndices[actorIndex];
    firstDirtyNodeIndices[actorIndex] = node;
    isNodeInDirtyList->set(node);
}


/**
Removes fast routes and marks involved nodes as dirty
*/
bool FamilyGraph::notifyEdgeRemoved(ActorIndex actorIndex, NodeIndex node0, NodeIndex node1, const SupportGraph* graph)
{
    NVBLAST_ASSERT(node0 < graph->m_nodeCount);
    NVBLAST_ASSERT(node1 < graph->m_nodeCount);

    // used internal data pointers
    NodeIndex* fastRoute = getFastRoute();
    const uint32_t* adjacencyPartition = graph->getAdjacencyPartition();
    const uint32_t* adjacentBondIndices = graph->getAdjacentBondIndices();

    // search for bond
    for (uint32_t adjacencyIndex = adjacencyPartition[node0]; adjacencyIndex < adjacencyPartition[node0 + 1]; adjacencyIndex++)
    {
        if (getAdjacentNode(adjacencyIndex, graph) == node1)
        {
            // found bond
            const uint32_t bondIndex = adjacentBondIndices[adjacencyIndex];

            // remove bond
            getIsEdgeRemoved()->set(bondIndex);

            // broke fast route if it goes through this edge:
            if (fastRoute[node0] == node1)
                fastRoute[node0] = invalidIndex<uint32_t>();
            if (fastRoute[node1] == node0)
                fastRoute[node1] = invalidIndex<uint32_t>();

            // mark nodes dirty (add to list if doesn't exist)
            addToDirtyNodeList(actorIndex, node0);
            addToDirtyNodeList(actorIndex, node1);

            // we don't expect to be more than one bond between 2 nodes
            return true;
        }
    }

    return false;
}

bool FamilyGraph::notifyEdgeRemoved(ActorIndex actorIndex, NodeIndex node0, NodeIndex node1, uint32_t bondIndex, const SupportGraph* graph)
{
    NV_UNUSED(graph);
    NVBLAST_ASSERT(node0 < graph->m_nodeCount);
    NVBLAST_ASSERT(node1 < graph->m_nodeCount);

    getIsEdgeRemoved()->set(bondIndex);


    NodeIndex* fastRoute = getFastRoute();

    // broke fast route if it goes through this edge:
    if (fastRoute[node0] == node1)
        fastRoute[node0] = invalidIndex<uint32_t>();
    if (fastRoute[node1] == node0)
        fastRoute[node1] = invalidIndex<uint32_t>();

    // mark nodes dirty (add to list if doesn't exist)
    addToDirtyNodeList(actorIndex, node0);
    addToDirtyNodeList(actorIndex, node1);

    return true;
}

bool FamilyGraph::notifyNodeRemoved(ActorIndex actorIndex, NodeIndex nodeIndex, const SupportGraph* graph)
{
    NVBLAST_ASSERT(nodeIndex < graph->m_nodeCount);

    // used internal data pointers
    NodeIndex* fastRoute = getFastRoute();
    const uint32_t* adjacencyPartition = graph->getAdjacencyPartition();
    const uint32_t* adjacentBondIndices = graph->getAdjacentBondIndices();

    // remove all edges leaving this node
    for (uint32_t adjacencyIndex = adjacencyPartition[nodeIndex]; adjacencyIndex < adjacencyPartition[nodeIndex + 1]; adjacencyIndex++)
    {
        const uint32_t adjacentNodeIndex = getAdjacentNode(adjacencyIndex, graph);
        if (!isInvalidIndex(adjacentNodeIndex))
        {
            const uint32_t bondIndex = adjacentBondIndices[adjacencyIndex];
            getIsEdgeRemoved()->set(bondIndex);

            if (fastRoute[adjacentNodeIndex] == nodeIndex)
                fastRoute[adjacentNodeIndex] = invalidIndex<uint32_t>();
            if (fastRoute[nodeIndex] == adjacentNodeIndex)
                fastRoute[nodeIndex] = invalidIndex<uint32_t>();

            addToDirtyNodeList(actorIndex, adjacentNodeIndex);
        }
    }
    addToDirtyNodeList(actorIndex, nodeIndex);

    // ignore this node in partition (only needed for "chunk deleted from graph")
    // getIslandIds()[nodeIndex] = invalidIndex<uint32_t>();

    return true;
}

void FamilyGraph::unwindRoute(uint32_t traversalIndex, NodeIndex lastNode, uint32_t hopCount, IslandId id, FixedArray<TraversalState>* visitedNodes)
{
    // used internal data pointers
    IslandId* islandIds = getIslandIds();
    NodeIndex* fastRoute = getFastRoute();
    uint32_t* hopCounts = getHopCounts();

    uint32_t currIndex = traversalIndex;
    uint32_t hc = hopCount + 1; //Add on 1 for the hop to the witness/root node.
    do
    {
        TraversalState& state = visitedNodes->at(currIndex);
        hopCounts[state.mNodeIndex] = hc++;
        islandIds[state.mNodeIndex] = id;
        fastRoute[state.mNodeIndex] = lastNode;
        currIndex = state.mPrevIndex;
        lastNode = state.mNodeIndex;
    }
    while(currIndex != invalidIndex<uint32_t>());
}


bool FamilyGraph::tryFastPath(NodeIndex startNode, NodeIndex targetNode, IslandId islandId, FixedArray<TraversalState>* visitedNodes, FixedBitmap* isNodeWitness, const SupportGraph* graph)
{
    NV_UNUSED(graph);

    // used internal data pointers
    IslandId* islandIds = getIslandIds();
    NodeIndex* fastRoute = getFastRoute();

    // prepare for iterating path
    NodeIndex currentNode = startNode;
    uint32_t visitedNotesInitialSize = visitedNodes->size();
    uint32_t depth = 0;

    bool found = false;
    do
    {
        // witness ?
        if (isNodeWitness->test(currentNode))
        {
            // Already visited and not tagged with invalid island == a witness!
            found = islandIds[currentNode] != invalidIndex<uint32_t>();
            break;
        }

        // reached targetNode ?
        if (currentNode == targetNode)
        {
            found = true;
            break;
        }

        TraversalState state(currentNode, visitedNodes->size(), visitedNodes->size() - 1, depth++);
        visitedNodes->pushBack(state);

        NVBLAST_ASSERT(isInvalidIndex(fastRoute[currentNode]) || hasEdge(currentNode, fastRoute[currentNode], graph));

        islandIds[currentNode] = invalidIndex<uint32_t>();
        isNodeWitness->set(currentNode);

        currentNode = fastRoute[currentNode];
    } while (currentNode != invalidIndex<uint32_t>());

    for (uint32_t a = visitedNotesInitialSize; a < visitedNodes->size(); ++a)
    {
        TraversalState& state = visitedNodes->at(a);
        islandIds[state.mNodeIndex] = islandId;
    }

    // if fast path failed we have to remove all isWitness marks on visited nodes and nodes from visited list
    if (!found)
    {
        for (uint32_t a = visitedNotesInitialSize; a < visitedNodes->size(); ++a)
        {
            TraversalState& state = visitedNodes->at(a);
            isNodeWitness->reset(state.mNodeIndex);
        }

        visitedNodes->forceSize_Unsafe(visitedNotesInitialSize);
    }

    return found;
}


bool FamilyGraph::findRoute(NodeIndex startNode, NodeIndex targetNode, IslandId islandId, FixedArray<TraversalState>* visitedNodes, FixedBitmap* isNodeWitness, NodePriorityQueue* priorityQueue, const SupportGraph* graph)
{
    // used internal data pointers
    IslandId* islandIds = getIslandIds();
    NodeIndex* fastRoute = getFastRoute();
    uint32_t* hopCounts = getHopCounts();
    const uint32_t* adjacencyPartition = graph->getAdjacencyPartition();

    // Firstly, traverse the fast path and tag up witnesses. TryFastPath can fail. In that case, no witnesses are left but this node is permitted to report
    // that it is still part of the island. Whichever node lost its fast path will be tagged as dirty and will be responsible for recovering the fast path
    // and tagging up the visited nodes
    if (fastRoute[startNode] != invalidIndex<uint32_t>())
    {
        if (tryFastPath(startNode, targetNode, islandId, visitedNodes, isNodeWitness, graph))
            return true;
    }

    // If we got here, there was no fast path. Therefore, we need to fall back on searching for the root node. This is optimized by using "hop counts".
    // These are per-node counts that indicate the expected number of hops from this node to the root node. These are lazily evaluated and updated
    // as new edges are formed or when traversals occur to re-establish islands. As a result, they may be inaccurate but they still serve the purpose
    // of guiding our search to minimize the chances of us doing an exhaustive search to find the root node.
    islandIds[startNode] = invalidIndex<uint32_t>();
    TraversalState startTraversal(startNode, visitedNodes->size(), invalidIndex<uint32_t>(), 0);
    isNodeWitness->set(startNode);
    QueueElement element(&visitedNodes->pushBack(startTraversal), hopCounts[startNode]);
    priorityQueue->push(element);

    do
    {
        QueueElement currentQE = priorityQueue->pop();

        TraversalState& currentState = *currentQE.mState;
        NodeIndex& currentNode = currentState.mNodeIndex;

        // iterate all edges of currentNode
        for (uint32_t adjacencyIndex = adjacencyPartition[currentNode]; adjacencyIndex < adjacencyPartition[currentNode + 1]; adjacencyIndex++)
        {
            NodeIndex nextIndex = getAdjacentNode(adjacencyIndex, graph);

            if (nextIndex != invalidIndex<uint32_t>())
            {
                if (nextIndex == targetNode)
                {
                    // targetNode found!
                    unwindRoute(currentState.mCurrentIndex, nextIndex, 0, islandId, visitedNodes);
                    return true;
                }

                if (isNodeWitness->test(nextIndex))
                {
                    // We already visited this node. This means that it's either in the priority queue already or we 
                    // visited in on a previous pass. If it was visited on a previous pass, then it already knows what island it's in. 
                    // We now need to test the island id to find out if this node knows the root.
                    // If it has a valid root id, that id *is* our new root. We can guesstimate our hop count based on the node's properties

                    IslandId visitedIslandId = islandIds[nextIndex];
                    if (visitedIslandId != invalidIndex<uint32_t>())
                    {
                        // If we get here, we must have found a node that knows a route to our root node. It must not be a different island
                        // because that would caused me to have been visited already because totally separate islands trigger a full traversal on 
                        // the orphaned side.
                        NVBLAST_ASSERT(visitedIslandId == islandId);
                        unwindRoute(currentState.mCurrentIndex, nextIndex, hopCounts[nextIndex], islandId, visitedNodes);
                        return true;
                    }
                }
                else
                {
                    // This node has not been visited yet, so we need to push it into the stack and continue traversing
                    TraversalState state(nextIndex, visitedNodes->size(), currentState.mCurrentIndex, currentState.mDepth + 1);
                    QueueElement qe(&visitedNodes->pushBack(state), hopCounts[nextIndex]);
                    priorityQueue->push(qe);
                    isNodeWitness->set(nextIndex);
                    NVBLAST_ASSERT(islandIds[nextIndex] == islandId);
                    islandIds[nextIndex] = invalidIndex<uint32_t>(); //Flag as invalid island until we know whether we can find root or an island id.
                }
            }
        }
    } while (priorityQueue->size());

    return false;
}


size_t FamilyGraph::findIslandsRequiredScratch(uint32_t graphNodeCount)
{
    const size_t visitedNodesSize = align16(FixedArray<TraversalState>::requiredMemorySize(graphNodeCount));
    const size_t isNodeWitnessSize = align16(FixedBitmap::requiredMemorySize(graphNodeCount));
    const size_t priorityQueueSize = align16(NodePriorityQueue::requiredMemorySize(graphNodeCount));

    // Aligned and padded
    return 16 + visitedNodesSize
              + isNodeWitnessSize
              + priorityQueueSize;
}


uint32_t FamilyGraph::findIslands(ActorIndex actorIndex, void* scratch, const SupportGraph* graph)
{
    // check if we have at least 1 dirty node for this actor before proceeding
    uint32_t* firstDirtyNodeIndices = getFirstDirtyNodeIndices();
    if (isInvalidIndex(firstDirtyNodeIndices[actorIndex]))
        return 0;

    // used internal data pointers
    IslandId* islandIds = getIslandIds();
    NodeIndex* fastRoute = getFastRoute();
    uint32_t* hopCounts = getHopCounts();
    NodeIndex* dirtyNodeLinks = getDirtyNodeLinks();
    FixedBoolArray* isNodeInDirtyList = getIsNodeInDirtyList();

    // prepare intermediate data on scratch
    scratch = (void*)align16((size_t)scratch); // Bump to 16-byte alignment (see padding in findIslandsRequiredScratch)
    const uint32_t nodeCount = graph->m_nodeCount;
    
    FixedArray<TraversalState>* visitedNodes = new (scratch)FixedArray<TraversalState>();
    scratch = pointerOffset(scratch, align16(FixedArray<TraversalState>::requiredMemorySize(nodeCount)));

    FixedBitmap* isNodeWitness = new (scratch)FixedBitmap(nodeCount);
    scratch = pointerOffset(scratch, align16(FixedBitmap::requiredMemorySize(nodeCount)));

    NodePriorityQueue* priorityQueue = new (scratch)NodePriorityQueue();
    scratch = pointerOffset(scratch, align16(NodePriorityQueue::requiredMemorySize(nodeCount)));

    // reset nodes visited bitmap
    isNodeWitness->clear();

    uint32_t newIslandsCount = 0;

    while (!isInvalidIndex(firstDirtyNodeIndices[actorIndex]))
    {
        // Pop head off of dirty node's list
        const NodeIndex dirtyNode = firstDirtyNodeIndices[actorIndex];
        firstDirtyNodeIndices[actorIndex] = dirtyNodeLinks[dirtyNode];
        dirtyNodeLinks[dirtyNode] = invalidIndex<uint32_t>();
        NVBLAST_ASSERT(isNodeInDirtyList->test(dirtyNode));
        isNodeInDirtyList->reset(dirtyNode);

        // clear PriorityQueue
        priorityQueue->clear();

        // if we already visited this node before in this loop it's not dirty anymore
        if (isNodeWitness->test(dirtyNode))
            continue;

        const IslandId& islandRootNode = islandIds[dirtyNode];
        IslandId islandId = islandRootNode; // the same in this implementation
        
        // if this node is island root node we don't need to do anything
        if (islandRootNode == dirtyNode)
            continue;

        // clear visited notes list (to fill during traverse)
        visitedNodes->clear();

        // try finding island root node from this dirtyNode
        if (findRoute(dirtyNode, islandRootNode, islandId, visitedNodes, isNodeWitness, priorityQueue, graph))
        {
            // We found the root node so let's let every visited node know that we found its root
            // and we can also update our hop counts because we recorded how many hops it took to reach this
            // node

            // We already filled in the path to the root/witness with accurate hop counts. Now we just need to fill in the estimates
            // for the remaining nodes and re-define their islandIds. We approximate their path to the root by just routing them through
            // the route we already found.

            // This loop works because visitedNodes are recorded in the order they were visited and we already filled in the critical path
            // so the remainder of the paths will just fork from that path.
            for (uint32_t b = 0; b < visitedNodes->size(); ++b)
            {
                TraversalState& state = visitedNodes->at(b);
                if (isInvalidIndex(islandIds[state.mNodeIndex]))
                {
                    hopCounts[state.mNodeIndex] = hopCounts[visitedNodes->at(state.mPrevIndex).mNodeIndex] + 1;
                    fastRoute[state.mNodeIndex] = visitedNodes->at(state.mPrevIndex).mNodeIndex;
                    islandIds[state.mNodeIndex] = islandId;
                }
            }
        }
        else
        {
            // NEW ISLAND BORN!

            // If I traversed and could not find the root node, then I have established a new island. In this island, I am the root node
            // and I will point all my nodes towards me. Furthermore, I have established how many steps it took to reach all nodes in my island

            // OK. We need to separate the islands. We have a list of nodes that are part of the new island (visitedNodes) and we know that the 
            // first node in that list is the root node.

#if SANITY_CHECKS
            NVBLAST_ASSERT(!canFindRoot(dirtyNode, islandRootNode, NULL));
#endif

            IslandId newIsland = dirtyNode;
            newIslandsCount++;

            hopCounts[dirtyNode] = 0;
            fastRoute[dirtyNode] = invalidIndex<uint32_t>();
            islandIds[dirtyNode] = newIsland;

            for (uint32_t a = 1; a < visitedNodes->size(); ++a)
            {
                NodeIndex visitedNode = visitedNodes->at(a).mNodeIndex;
                hopCounts[visitedNode] = visitedNodes->at(a).mDepth; //How many hops to root
                fastRoute[visitedNode] = visitedNodes->at(visitedNodes->at(a).mPrevIndex).mNodeIndex;
                islandIds[visitedNode] = newIsland;
            }
        }
    }

    // all dirty nodes processed
    return newIslandsCount;
}


/**
!!! Debug/Test function.
Function to check that root between nodes exists.
*/
bool FamilyGraph::canFindRoot(NodeIndex startNode, NodeIndex targetNode, FixedArray<NodeIndex>* visitedNodes, const SupportGraph* graph)
{
    if (visitedNodes)
        visitedNodes->pushBack(startNode);

    if (startNode == targetNode)
        return true;

    std::vector<bool> visitedState;
    visitedState.resize(graph->m_nodeCount);
    for (uint32_t i = 0; i < graph->m_nodeCount; i++)
        visitedState[i] = false;

    std::stack<NodeIndex> stack;

    stack.push(startNode);
    visitedState[startNode] = true;

    const uint32_t* adjacencyPartition = graph->getAdjacencyPartition();
    do
    {
        NodeIndex currentNode = stack.top();
        stack.pop();

        for (uint32_t adjacencyIndex = adjacencyPartition[currentNode]; adjacencyIndex < adjacencyPartition[currentNode + 1]; adjacencyIndex++)
        {
            NodeIndex nextNode = getAdjacentNode(adjacencyIndex, graph);

            if (isInvalidIndex(nextNode))
                continue;

            if (!visitedState[nextNode])
            {
                if (nextNode == targetNode)
                {
                    return true;
                }

                visitedState[nextNode] = true;
                stack.push(nextNode);

                if (visitedNodes)
                    visitedNodes->pushBack(nextNode);
            }
        }

    } while (!stack.empty());

    return false;
}


/**
!!! Debug/Test function.
Function to check if edge exists.
*/
bool FamilyGraph::hasEdge(NodeIndex node0, NodeIndex node1, const SupportGraph* graph) const
{
    const uint32_t* adjacencyPartition = graph->getAdjacencyPartition();
    uint32_t edges = 0;
    for (uint32_t adjacencyIndex = adjacencyPartition[node0]; adjacencyIndex < adjacencyPartition[node0 + 1]; adjacencyIndex++)
    {
        if (getAdjacentNode(adjacencyIndex, graph) == node1)
        {
            edges++;
            break;
        }
    }
    for (uint32_t adjacencyIndex = adjacencyPartition[node1]; adjacencyIndex < adjacencyPartition[node1 + 1]; adjacencyIndex++)
    {
        if (getAdjacentNode(adjacencyIndex, graph) == node0)
        {
            edges++;
            break;
        }
    }
    return edges > 0;
}


/**
!!! Debug/Test function.
Function to calculate and return edges count
*/
uint32_t FamilyGraph::getEdgesCount(const SupportGraph* graph) const
{
    const uint32_t* adjacencyPartition = graph->getAdjacencyPartition();
    uint32_t edges = 0;
    for (NodeIndex n = 0; n < graph->m_nodeCount; n++)
    {
        for (uint32_t adjacencyIndex = adjacencyPartition[n]; adjacencyIndex < adjacencyPartition[n + 1]; adjacencyIndex++)
        {
            if (getAdjacentNode(adjacencyIndex, graph) != invalidIndex<uint32_t>())
                edges++;
        }
    }
    NVBLAST_ASSERT(edges % 2 == 0);
    return edges / 2;
}




} // namespace Nv
} // namespace Blast

