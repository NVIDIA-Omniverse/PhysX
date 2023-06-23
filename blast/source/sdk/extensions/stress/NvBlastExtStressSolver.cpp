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
// Copyright (c) 2016-2023 NVIDIA Corporation. All rights reserved.


#include "NvBlastExtStressSolver.h"
#include "NvBlast.h"
#include "NvBlastGlobals.h"
#include "NvBlastArray.h"
#include "NvBlastHashMap.h"
#include "NvBlastHashSet.h"
#include "NvBlastAssert.h"
#include "NvBlastIndexFns.h"

#include "NsFPU.h"
#include "NvBlastNvSharedHelpers.h"
#include "NvCMath.h"

#include "stress.h"
#include "buffer.h"
#include "simd/simd_device_query.h"

#include <algorithm>

#define USE_SCALAR_IMPL 0
#define WARM_START 1
#define GRAPH_INTERGRIRY_CHECK 0

#if GRAPH_INTERGRIRY_CHECK
#include <set>
#endif


namespace Nv
{
namespace Blast
{

using namespace nvidia;

static_assert(sizeof(NvVec3) == sizeof(NvcVec3), "sizeof(NvVec3) must equal sizeof(NvcVec3).");
static_assert(offsetof(NvVec3, x) == offsetof(NvcVec3, x) &&
              offsetof(NvVec3, y) == offsetof(NvcVec3, y) &&
              offsetof(NvVec3, z) == offsetof(NvcVec3, z),
              "Elements of NvVec3 and NvcVec3 must have the same struct offset.");


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Conjugate Gradient Solver
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class ConjugateGradientImpulseSolver
{
public:
    ConjugateGradientImpulseSolver(uint32_t nodeCount, uint32_t maxBondCount)
    {
        m_bonds.reserve(maxBondCount);
        m_impulses.reserve(maxBondCount);
        reset(nodeCount);
    }

    void getBondImpulses(uint32_t bond, NvVec3& impulseLinear, NvVec3& impulseAngular) const
    {
        NVBLAST_ASSERT(bond < m_impulses.size());
        const AngLin6& f = m_impulses[bond];
        *(NvcVec3*)&impulseAngular = f.ang;
        *(NvcVec3*)&impulseLinear = f.lin;
    }

    void getBondNodes(uint32_t bond, uint32_t& node0, uint32_t& node1) const
    {
        NVBLAST_ASSERT(bond < m_bonds.size());
        const SolverBond& b = m_bonds[bond];
        node0 = b.nodes[0];
        node1 = b.nodes[1];
    }

    uint32_t getBondCount() const
    {
        return m_bonds.size();
    }

    uint32_t getNodeCount() const
    {
        return m_nodes.size();
    }

    void setNodeMassInfo(uint32_t node, const NvVec3& CoM, float mass, float inertia)
    {
        NVBLAST_ASSERT(node < m_nodes.size());
        SolverNodeS& n = m_nodes[node];
        n.CoM = { CoM.x, CoM.y, CoM.z };
        n.mass = std::max(mass, 0.0f);  // No negative masses, but 0 is meaningful (== infinite)
        n.inertia = std::max(inertia, 0.0f);    // Ditto for inertia
        m_forceColdStart = true;
    }

    void initialize()
    {
        StressProcessor::DataParams params;
        params.centerBonds = true;
        params.equalizeMasses = true;
        m_stressProcessor.prepare(m_nodes.begin(), m_nodes.size(), m_bonds.begin(), m_bonds.size(), params);
    }

    void setNodeVelocities(uint32_t node, const NvVec3& velocityLinear, const NvVec3& velocityAngular)
    {
        NVBLAST_ASSERT(node < m_velocities.size());
        AngLin6& v = m_velocities[node];
        v.ang = { velocityAngular.x, velocityAngular.y, velocityAngular.z };
        v.lin = { velocityLinear.x, velocityLinear.y, velocityLinear.z };
        m_inputsChanged = true;
    }

    uint32_t addBond(uint32_t node0, uint32_t node1, const NvVec3& bondCentroid)
    {
        SolverBond b;
        b.nodes[0] = node0;
        b.nodes[1] = node1;
        b.centroid = { bondCentroid.x, bondCentroid.y, bondCentroid.z };
        m_bonds.pushBack(b);
        m_impulses.push_back({{0,0,0},{0,0,0}});
        m_forceColdStart = true;
        return m_bonds.size() - 1;
    }

    void replaceWithLast(uint32_t bondIndex)
    {
        m_bonds.replaceWithLast(bondIndex);
        if ((size_t)bondIndex + 2 < m_impulses.size())
        {
            m_impulses[bondIndex] = m_impulses.back();
            m_impulses.resize(m_impulses.size() - 1);
        }
        m_stressProcessor.removeBond(bondIndex);
    }

    void reset(uint32_t nodeCount)
    {
        m_nodes.resize(nodeCount);
        memset(m_nodes.begin(), 0, sizeof(SolverNodeS)*nodeCount);
        m_velocities.resize(nodeCount);
        memset(m_velocities.data(), 0, sizeof(AngLin6)*nodeCount);
        clearBonds();
        m_error_sq = {FLT_MAX, FLT_MAX};
        m_converged = false;
        m_forceColdStart = true;
        m_inputsChanged = true;
    }

    void clearBonds()
    {
        m_bonds.clear();
        m_impulses.resize(0);
        m_forceColdStart = true;
    }

    void solve(uint32_t iterationCount, bool warmStart = true)
    {
        StressProcessor::SolverParams params;
        params.maxIter = iterationCount;
        params.tolerance = 0.001f;
        params.warmStart = warmStart && !m_forceColdStart;
        m_converged = (m_stressProcessor.solve(m_impulses.data(), m_velocities.data(), params, &m_error_sq) >= 0);
        m_forceColdStart = false;
        m_inputsChanged = false;
    }

    bool calcError(float& linear, float& angular) const
    {
        linear = sqrtf(m_error_sq.lin);
        angular = sqrtf(m_error_sq.ang);
        return m_converged;
    }

private:
    Array<SolverNodeS>::type    m_nodes;
    Array<SolverBond>::type     m_bonds;
    StressProcessor             m_stressProcessor;
    POD_Buffer<AngLin6>         m_velocities;
    POD_Buffer<AngLin6>         m_impulses;
    AngLin6ErrorSq              m_error_sq;
    bool                        m_converged;
    bool                        m_forceColdStart;
    bool                        m_inputsChanged;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                   Graph Processor
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if GRAPH_INTERGRIRY_CHECK
#define CHECK_GRAPH_INTEGRITY checkGraphIntegrity()
#else
#define CHECK_GRAPH_INTEGRITY ((void)0)
#endif

class SupportGraphProcessor
{

public:
    struct BondData
    {
        uint32_t node0;
        uint32_t node1;
        uint32_t blastBondIndex;
        // linear stresses
        float    stressNormal;  // negative values represent compression pressure, positive represent tension
        float    stressShear;

        // The normal used to compute stress values
        // Can be different than the bond normal if graph reduction is used
        // and multiple bonds are grouped together
        nvidia::NvVec3 normal;

        // Centroid used to compute node offsets, instead of assuming the bond is halfway between node positions.
        // This also allows the bonds to the world node to be drawn
        nvidia::NvVec3 centroid;
    };

    struct NodeData
    {
        float mass;
        float volume;
        NvVec3 localPos;
        NvVec3 localVel;
        uint32_t solverNode;
        uint32_t neighborsCount;
    };

    struct SolverNodeData
    {
        uint32_t supportNodesCount;
        NvVec3 localPos;
        union
        {
            float mass;
            int32_t indexShift;
        };
        float volume;
    };

    struct SolverBondData
    {
        InlineArray<uint32_t, 8>::type blastBondIndices;
    };

    SupportGraphProcessor(uint32_t nodeCount, uint32_t maxBondCount) :
        m_solver(nodeCount, maxBondCount), m_nodesDirty(true), m_bondsDirty(true)
    {
        m_nodesData.resize(nodeCount);
        m_bondsData.reserve(maxBondCount);

        m_solverNodesData.resize(nodeCount);
        m_solverBondsData.reserve(maxBondCount);

        m_solverBondsMap.reserve(maxBondCount);

        m_blastBondIndexMap.resize(maxBondCount);
        memset(m_blastBondIndexMap.begin(), 0xFF, m_blastBondIndexMap.size() * sizeof(uint32_t));

        resetVelocities();
    }

    const NodeData& getNodeData(uint32_t node) const
    {
        return m_nodesData[node];
    }

    const BondData& getBondData(uint32_t bond) const
    {
        return m_bondsData[bond];
    }

    const SolverNodeData& getSolverNodeData(uint32_t node) const
    {
        return m_solverNodesData[node];
    }

    const SolverBondData& getSolverBondData(uint32_t bond) const
    {
        return m_solverBondsData[bond];
    }

    void getSolverInternalBondImpulses(uint32_t bond, NvVec3& impulseLinear, NvVec3& impulseAngular) const
    {
        m_solver.getBondImpulses(bond, impulseLinear, impulseAngular);
    }

    void getSolverInternalBondNodes(uint32_t bond, uint32_t& node0, uint32_t& node1) const
    {
        m_solver.getBondNodes(bond, node0, node1);
    }

    uint32_t getBondCount() const
    {
        return m_bondsData.size();
    }

    uint32_t getNodeCount() const
    {
        return m_nodesData.size();;
    }

    uint32_t getSolverBondCount() const
    {
        return m_solverBondsData.size();
    }

    uint32_t getSolverNodeCount() const
    {
        return m_solverNodesData.size();;
    }

    uint32_t getOverstressedBondCount() const
    {
        return m_overstressedBondCount;
    }

    void calcSolverBondStresses(
        uint32_t bondIdx, float bondArea, float nodeDist, const nvidia::NvVec3& bondNormal,
        float& stressNormal, float& stressShear) const
    {
        if (!canTakeDamage(bondArea))
        {
            stressNormal = stressShear = 0.0f;
            return;
        }

        // impulseLinear in the direction of the bond normal is stressNormal, perpendicular is stressShear
        // ignore impulseAngular for now, not sure how to account for that
        // convert to pressure to factor out area
        NvVec3 impulseLinear, impulseAngular;
        getSolverInternalBondImpulses(bondIdx, impulseLinear, impulseAngular);
        const float normalComponentLinear = impulseLinear.dot(bondNormal);
        stressNormal = normalComponentLinear / bondArea;
        const float impulseLinearMagSqr = impulseLinear.magnitudeSquared();
        stressShear = sqrtf(impulseLinearMagSqr - normalComponentLinear * normalComponentLinear) / bondArea;

        // impulseAngular in the direction of the bond normal is twist, perpendicular is bend
        // take abs() of the dot product because only the magnitude of the twist matters, not direction
        const float normalComponentAngular = abs(impulseAngular.dot(bondNormal));
        const float twist = normalComponentAngular / bondArea;
        const float impulseAngularMagSqr = impulseAngular.magnitudeSquared();
        const float bend = sqrtf(impulseAngularMagSqr - normalComponentAngular * normalComponentAngular) / bondArea;

        // interpret angular pressure as a composition of linear pressures
        // dividing by nodeDist for scaling
        const float twistContribution = twist * 2.0f / nodeDist;
        stressShear += twistContribution;
        const float bendContribution = bend * 2.0f / nodeDist;
        stressNormal += copysignf(bendContribution, stressNormal);
    }

    float mapStressToRange(float stress, float elasticLimit, float fatalLimit) const
    {
        if (stress < elasticLimit)
        {
            return 0.5f * stress / elasticLimit;
        }
        else
        {
            return fatalLimit > elasticLimit ? 0.5f + 0.5f * (stress - elasticLimit) / (fatalLimit - elasticLimit) : 1.0f;
        }
    }

    float getSolverBondStressPct(uint32_t bondIdx, const float* bondHealths, const ExtStressSolverSettings& settings, ExtStressSolver::DebugRenderMode mode) const
    {
        // sum up the stress of all underlying bonds involved in this stress solver bond
        float compressionStress, tensionStress, shearStress;
        float stress = -1.0f;
        const auto& blastBondIndices = m_solverBondsData[bondIdx].blastBondIndices;
        for (const auto blastBondIndex : blastBondIndices)
        {
            // only consider the stress values on bonds that are intact
            if (bondHealths[blastBondIndex] > 0.0f && getBondStress(blastBondIndex, compressionStress, tensionStress, shearStress))
            {
                if (mode == ExtStressSolver::STRESS_PCT_COMPRESSION || mode == ExtStressSolver::STRESS_PCT_MAX)
                {
                    compressionStress = mapStressToRange(compressionStress, settings.compressionElasticLimit, settings.compressionFatalLimit);
                    stress = std::max(compressionStress, stress);
                }

                if (mode == ExtStressSolver::STRESS_PCT_TENSION || mode == ExtStressSolver::STRESS_PCT_MAX)
                {
                    tensionStress = mapStressToRange(tensionStress, settings.tensionElasticLimit, settings.tensionFatalLimit);
                    stress = std::max(tensionStress, stress);
                }

                if (mode == ExtStressSolver::STRESS_PCT_SHEAR || mode == ExtStressSolver::STRESS_PCT_MAX)
                {
                    shearStress = mapStressToRange(shearStress, settings.shearElasticLimit, settings.shearFatalLimit);
                    stress = std::max(shearStress, stress);
                }

                // all bonds in the group share the same stress values, no need to keep iterating
                break;
            }
        }

        // return a value < 0.0f if all bonds are broken
        return stress;
    }

    void setNodeInfo(uint32_t node, float mass, float volume, NvVec3 localPos)
    {
        m_nodesData[node].mass = mass;
        m_nodesData[node].volume = volume;
        m_nodesData[node].localPos = localPos;
        m_nodesDirty = true;
    }

    void setNodeNeighborsCount(uint32_t node, uint32_t neighborsCount)
    {
        // neighbors count is expected to be the number of nodes on 1 island/actor.
        m_nodesData[node].neighborsCount = neighborsCount;

        // check for too huge aggregates (happens after island's split)
        if (!m_nodesDirty)
        {
            m_nodesDirty |= (m_solverNodesData[m_nodesData[node].solverNode].supportNodesCount > neighborsCount / 2);
        }
    }

    void addNodeForce(uint32_t node, const NvVec3& force, ExtForceMode::Enum mode)
    {
        const float mass = m_nodesData[node].mass;
        if (mass > 0)
        {
            // NOTE - passing in acceleration as velocity.  The impulse solver's output will be interpreted as force.
            m_nodesData[node].localVel += (mode == ExtForceMode::FORCE) ? force/mass : force;
        }
    }

    void addBond(uint32_t node0, uint32_t node1, uint32_t blastBondIndex)
    {
        if (isInvalidIndex(m_blastBondIndexMap[blastBondIndex]))
        {
            const BondData data = {
                node0,
                node1,
                blastBondIndex,
                0.0f
            };
            m_bondsData.pushBack(data);
            m_blastBondIndexMap[blastBondIndex] = m_bondsData.size() - 1;
        }
    }

    void removeBondIfExists(uint32_t blastBondIndex)
    {
        const uint32_t bondIndex = m_blastBondIndexMap[blastBondIndex];

        if (!isInvalidIndex(bondIndex))
        {
            const BondData& bond = m_bondsData[bondIndex];
            const uint32_t solverNode0 = m_nodesData[bond.node0].solverNode;
            const uint32_t solverNode1 = m_nodesData[bond.node1].solverNode;
            bool isBondInternal = (solverNode0 == solverNode1);

            if (isBondInternal)
            {
                // internal bond sadly requires graph resync (it never happens on reduction level '0')
                m_nodesDirty = true;
            }
            else if (!m_nodesDirty)
            {
                // otherwise it's external bond, we can remove it manually and keep graph synced
                // we don't need to spend time there if (m_nodesDirty == true), graph will be resynced anyways

                BondKey solverBondKey(solverNode0, solverNode1);
                auto entry = m_solverBondsMap.find(solverBondKey);
                if (entry)
                {
                    const uint32_t solverBondIndex = entry->second;
                    auto& blastBondIndices = m_solverBondsData[solverBondIndex].blastBondIndices;
                    blastBondIndices.findAndReplaceWithLast(blastBondIndex);
                    if (blastBondIndices.empty())
                    {
                        // all bonds associated with this solver bond were removed, so let's remove solver bond

                        m_solverBondsData.replaceWithLast(solverBondIndex);
                        m_solver.replaceWithLast(solverBondIndex);
                        if (m_solver.getBondCount() > 0)
                        {
                            // update 'previously last' solver bond mapping
                            uint32_t node0, node1;
                            m_solver.getBondNodes(solverBondIndex, node0, node1);
                            m_solverBondsMap[BondKey(node0, node1)] = solverBondIndex;
                        }

                        m_solverBondsMap.erase(solverBondKey);
                    }
                }

                CHECK_GRAPH_INTEGRITY;
            }

            // remove bond from graph processor's list
            m_blastBondIndexMap[blastBondIndex] = invalidIndex<uint32_t>();
            m_bondsData.replaceWithLast(bondIndex);
            m_blastBondIndexMap[m_bondsData[bondIndex].blastBondIndex] = m_bondsData.size() > bondIndex ? bondIndex : invalidIndex<uint32_t>();
        }
    }

    void setGraphReductionLevel(uint32_t level)
    {
        m_graphReductionLevel = level;
        m_nodesDirty = true;
    }

    uint32_t getGraphReductionLevel() const
    {
        return m_graphReductionLevel;
    }

    void solve(const ExtStressSolverSettings& settings, const float* bondHealth, const NvBlastBond* bonds, bool warmStart = true)
    {
        sync(bonds);

        for (const NodeData& node : m_nodesData)
        {
            m_solver.setNodeVelocities(node.solverNode, node.localVel, NvVec3(NvZero));
        }

        m_solver.solve(settings.maxSolverIterationsPerFrame, warmStart);

        resetVelocities();

        updateBondStress(settings, bondHealth, bonds);
    }

    bool calcError(float& linear, float& angular) const
    {
        return m_solver.calcError(linear, angular);
    }

    bool getBondStress(uint32_t blastBondIndex, float& compression, float& tension, float& shear) const
    {
        const uint32_t bondIndex = m_blastBondIndexMap[blastBondIndex];
        if (isInvalidIndex(bondIndex))
        {
            return false;
        }

        // compression and tension are mutually exclusive since they operate in opposite directions
        // they both measure stress parallel to the bond normal direction
        // compression is the force resisting two nodes being pushed together (it pushes them apart)
        // tension is the force resisting two nodes being pulled apart (it pulls them together)
        if (m_bondsData[bondIndex].stressNormal <= 0.0f)
        {
            compression = -m_bondsData[bondIndex].stressNormal;
            tension = 0.0f;
        }
        else
        {
            compression = 0.0f;
            tension = m_bondsData[bondIndex].stressNormal;
        }

        // shear is independent and can co-exist with compression and tension
        shear = m_bondsData[bondIndex].stressShear;         // the force perpendicular to the bond normal direction

        return true;
    }

    // Convert from Blast bond index to internal stress solver bond index
    // Will be InvalidIndex if the internal bond was removed from the stress solver
    uint32_t getInternalBondIndex(uint32_t blastBondIndex)
    {
        return m_blastBondIndexMap[blastBondIndex];
    }

private:

    void resetVelocities()
    {
        for (auto& node : m_nodesData)
        {
            node.localVel = NvVec3(NvZero);
        }
    }

    void updateBondStress(const ExtStressSolverSettings& settings, const float* bondHealth, const NvBlastBond* bonds)
    {
        m_overstressedBondCount = 0;

        Array<uint32_t>::type bondIndicesToRemove;
        bondIndicesToRemove.reserve(getBondCount());
        for (uint32_t i = 0; i < m_solverBondsData.size(); ++i)
        {
            // calculate the total area of all bonds involved so pressure can be calculated
            float totalArea = 0.0f;
            // calculate an average normal and centroid for all bonds as well, weighted by their area
            nvidia::NvVec3 bondNormal(NvZero);
            nvidia::NvVec3 bondCentroid(NvZero);
            nvidia::NvVec3 averageNodeDisp(NvZero);
            const auto& blastBondIndices = m_solverBondsData[i].blastBondIndices;
            for (auto blastBondIndex : blastBondIndices)
            {
                if (bondHealth[blastBondIndex] > 0.0f)
                {
                    const uint32_t bondIndex = m_blastBondIndexMap[blastBondIndex];
                    const BondData& bond = m_bondsData[bondIndex];
                    const nvidia::NvVec3 nodeDisp = m_nodesData[bond.node1].localPos - m_nodesData[bond.node0].localPos;

                    // the current health of a bond is the effective area remaining
                    const float remainingArea = bondHealth[blastBondIndex];
                    const NvBlastBond& blastBond = bonds[blastBondIndex];

                    // Align normal(s) with node displacement, so that compressive/tensile distinction is correct
                    const nvidia::NvVec3 assetBondNormal(blastBond.normal[0], blastBond.normal[1], blastBond.normal[2]);
                    const nvidia::NvVec3 blastBondNormal = std::copysignf(1.0f, assetBondNormal.dot(nodeDisp))*assetBondNormal;

                    const nvidia::NvVec3 blastBondCentroid(blastBond.centroid[0], blastBond.centroid[1], blastBond.centroid[2]);

                    if (!canTakeDamage(remainingArea))  // Check unbreakable limit
                    {
                        totalArea = kUnbreakableLimit;  // Don't add this in, in case of overflow
                        bondNormal = blastBondNormal;
                        bondCentroid = blastBondCentroid;
                        averageNodeDisp = nodeDisp;
                        break;
                    }

                    bondNormal += blastBondNormal*remainingArea;
                    bondCentroid += blastBondCentroid*remainingArea;
                    averageNodeDisp += nodeDisp*remainingArea;

                    totalArea += remainingArea;
                }
                else
                {
                    // if the bond is broken, try to remove it after processing is complete
                    bondIndicesToRemove.pushBack(blastBondIndex);
                }
            }

            if (totalArea == 0.0f)
            {
                continue;
            }

            // normalized the aggregate normal now that all contributing bonds have been combined
            bondNormal.normalizeSafe();

            // divide by total area for the weighted position, if the area is valid
            if (canTakeDamage(totalArea))
            {
                bondCentroid /= totalArea;
                averageNodeDisp /= totalArea;
            }

            // bonds are looked at as a whole group,
            // so regardless of the current health of an individual one they are either all over stressed or none are
            float stressNormal, stressShear;
            calcSolverBondStresses(i, totalArea, averageNodeDisp.magnitude(), bondNormal, stressNormal, stressShear);
            NVBLAST_ASSERT(!std::isnan(stressNormal) && !std::isnan(stressShear));
            if (
                -stressNormal > settings.compressionElasticLimit ||
                stressNormal > settings.tensionElasticLimit ||
                stressShear > settings.shearElasticLimit
            )
            {
                m_overstressedBondCount += blastBondIndices.size();
            }

            // store the stress values for all the bonds involved
            for (auto blastBondIndex : blastBondIndices)
            {
                const uint32_t bondIndex = m_blastBondIndexMap[blastBondIndex];
                if (!isInvalidIndex(bondIndex) && bondHealth[blastBondIndex] > 0.0f)
                {
                    BondData& bond = m_bondsData[bondIndex];

                    NVBLAST_ASSERT(getNodeData(bond.node0).solverNode != getNodeData(bond.node1).solverNode);
                    NVBLAST_ASSERT(bond.blastBondIndex == blastBondIndex);

                    bond.stressNormal = stressNormal;
                    bond.stressShear = stressShear;

                    // store the normal used to calc stresses so it can be used later to determine forces
                    bond.normal = bondNormal;

                    // store the bond centroid
                    bond.centroid = bondCentroid;
                }
            }
        }

        // now that processing is done, remove any dead bonds
        for (uint32_t bondIndex : bondIndicesToRemove)
        {
            removeBondIfExists(bondIndex);
        }
    }

    void sync(const NvBlastBond* bonds)
    {
        if (m_nodesDirty)
        {
            syncNodes(bonds);
            m_solver.initialize();
        }
        if (m_bondsDirty)
        {
            syncBonds(bonds);
        }

        CHECK_GRAPH_INTEGRITY;
    }

    void syncNodes(const NvBlastBond* bonds)
    {
        // init with 1<->1 blast nodes to solver nodes mapping
        m_solverNodesData.resize(m_nodesData.size());
        for (uint32_t i = 0; i < m_nodesData.size(); ++i)
        {
            m_nodesData[i].solverNode = i;
            m_solverNodesData[i].supportNodesCount = 1;
            m_solverNodesData[i].indexShift = 0;
        }

        // for static nodes aggregate size per graph reduction level is lower, it
        // falls behind on few levels. (can be made as parameter)
        const uint32_t STATIC_NODES_COUNT_PENALTY = 2 << 2;

        // reducing graph by aggregating nodes level by level
        // NOTE (@anovoselov):  Recently, I found a flow in the algorithm below. In very rare situations aggregate (solver node)
        // can contain more then one connected component. I didn't notice it to produce any visual artifacts and it's
        // unlikely to influence stress solvement a lot. Possible solution is to merge *whole* solver nodes, that
        // will raise complexity a bit (at least will add another loop on nodes for every reduction level.
        for (uint32_t k = 0; k < m_graphReductionLevel; k++)
        {
            const uint32_t maxAggregateSize = 1 << (k + 1);

            for (const BondData& bond : m_bondsData)
            {
                NodeData& node0 = m_nodesData[bond.node0];
                NodeData& node1 = m_nodesData[bond.node1];

                if (node0.solverNode == node1.solverNode)
                    continue;

                SolverNodeData& solverNode0 = m_solverNodesData[node0.solverNode];
                SolverNodeData& solverNode1 = m_solverNodesData[node1.solverNode];

                const int countPenalty = 1;   // This was being set to STATIC_NODES_COUNT_PENALTY for static nodes, may want to revisit
                const uint32_t aggregateSize = std::min<uint32_t>(maxAggregateSize, node0.neighborsCount / 2);

                if (solverNode0.supportNodesCount * countPenalty >= aggregateSize)
                    continue;
                if (solverNode1.supportNodesCount * countPenalty >= aggregateSize)
                    continue;

                if (solverNode0.supportNodesCount >= solverNode1.supportNodesCount)
                {
                    solverNode1.supportNodesCount--;
                    solverNode0.supportNodesCount++;
                    node1.solverNode = node0.solverNode;
                }
                else if (solverNode1.supportNodesCount >= solverNode0.supportNodesCount)
                {
                    solverNode1.supportNodesCount++;
                    solverNode0.supportNodesCount--;
                    node0.solverNode = node1.solverNode;
                }
            }
        }

        // Solver Nodes now sparse, a lot of empty ones. Rearrange them by moving all non-empty to the front
        // 2 passes used for that
        {
            uint32_t currentNode = 0;
            for (; currentNode < m_solverNodesData.size(); ++currentNode)
            {
                if (m_solverNodesData[currentNode].supportNodesCount > 0)
                    continue;

                // 'currentNode' is free

                // search next occupied node
                uint32_t k = currentNode + 1;
                for (; k < m_solverNodesData.size(); ++k)
                {
                    if (m_solverNodesData[k].supportNodesCount > 0)
                    {
                        // replace currentNode and keep indexShift
                        m_solverNodesData[currentNode].supportNodesCount = m_solverNodesData[k].supportNodesCount;
                        m_solverNodesData[k].indexShift = k - currentNode;
                        m_solverNodesData[k].supportNodesCount = 0;
                        break;
                    }
                }

                if (k == m_solverNodesData.size())
                {
                    break;
                }
            }
            for (auto& node : m_nodesData)
            {
                node.solverNode -= m_solverNodesData[node.solverNode].indexShift;
            }

            // now, we know total solver nodes count and which nodes are aggregated into them
            m_solverNodesData.resize(currentNode);
        }


        // calculate all needed data
        for (SolverNodeData& solverNode : m_solverNodesData)
        {
            solverNode.supportNodesCount = 0;
            solverNode.localPos = NvVec3(NvZero);
            solverNode.mass = 0.0f;
            solverNode.volume = 0.0f;
        }

        for (NodeData& node : m_nodesData)
        {
            SolverNodeData& solverNode = m_solverNodesData[node.solverNode];
            solverNode.supportNodesCount++;
            solverNode.localPos += node.localPos;
            solverNode.mass += node.mass;
            solverNode.volume += node.volume;
        }

        for (SolverNodeData& solverNode : m_solverNodesData)
        {
            solverNode.localPos /= (float)solverNode.supportNodesCount;
        }

        m_solver.reset(m_solverNodesData.size());
        for (uint32_t nodeIndex = 0; nodeIndex < m_solverNodesData.size(); ++nodeIndex)
        {
            const SolverNodeData& solverNode = m_solverNodesData[nodeIndex];

            const float R = NvPow(solverNode.volume * 3.0f * NvInvPi / 4.0f, 1.0f / 3.0f); // sphere volume approximation
            const float inertia = solverNode.mass * (R * R * 0.4f); // sphere inertia tensor approximation: I = 2/5 * M * R^2 ; invI = 1 / I;
            m_solver.setNodeMassInfo(nodeIndex, solverNode.localPos, solverNode.mass, inertia);
        }

        m_nodesDirty = false;

        syncBonds(bonds);
    }

    void syncBonds(const NvBlastBond* bonds)
    {
        // traverse all blast bonds and aggregate
        m_solver.clearBonds();
        m_solverBondsMap.clear();
        m_solverBondsData.clear();
        for (BondData& bond : m_bondsData)
        {
            const NodeData& node0 = m_nodesData[bond.node0];
            const NodeData& node1 = m_nodesData[bond.node1];

            // reset stress, bond structure changed and internal bonds stress won't be updated during updateBondStress()
            bond.stressNormal = 0.0f;
            bond.stressShear = 0.0f;

            // initialize normal and centroid using blast values
            bond.normal = *(NvVec3*)bonds[bond.blastBondIndex].normal;
            bond.centroid = *(NvVec3*)bonds[bond.blastBondIndex].centroid;

            // fix normal direction to point from node0 to node1
            bond.normal *= std::copysignf(1.0f, bond.normal.dot(node1.localPos - node1.localPos));

            if (node0.solverNode == node1.solverNode)
                continue; // skip (internal)

            BondKey key(node0.solverNode, node1.solverNode);
            auto entry = m_solverBondsMap.find(key);
            SolverBondData* data;
            if (!entry)
            {
                m_solverBondsData.pushBack(SolverBondData());
                data = &m_solverBondsData.back();
                m_solverBondsMap[key] = m_solverBondsData.size() - 1;

                m_solver.addBond(node0.solverNode, node1.solverNode, bond.centroid);
            }
            else
            {
                data = &m_solverBondsData[entry->second];
            }
            data->blastBondIndices.pushBack(bond.blastBondIndex);
        }

        m_bondsDirty = false;
    }

#if GRAPH_INTERGRIRY_CHECK
    void checkGraphIntegrity()
    {
        NVBLAST_ASSERT(m_solver.getBondCount() == m_solverBondsData.size());
        NVBLAST_ASSERT(m_solver.getNodeCount() == m_solverNodesData.size());

        std::set<uint64_t> solverBonds;
        for (uint32_t i = 0; i < m_solverBondsData.size(); ++i)
        {
            const auto& bondData = m_solver.getBondData(i);
            BondKey key(bondData.node0, bondData.node1);
            NVBLAST_ASSERT(solverBonds.find(key) == solverBonds.end());
            solverBonds.emplace(key);
            auto entry = m_solverBondsMap.find(key);
            NVBLAST_ASSERT(entry != nullptr);
            const auto& solverBond = m_solverBondsData[entry->second];
            for (auto& blastBondIndex : solverBond.blastBondIndices)
            {
                if (!isInvalidIndex(m_blastBondIndexMap[blastBondIndex]))
                {
                    auto& b = m_bondsData[m_blastBondIndexMap[blastBondIndex]];
                    BondKey key2(m_nodesData[b.node0].solverNode, m_nodesData[b.node1].solverNode);
                    NVBLAST_ASSERT(key2 == key);
                }
            }
        }

        for (auto& solverBond : m_solverBondsData)
        {
            for (auto& blastBondIndex : solverBond.blastBondIndices)
            {
                if (!isInvalidIndex(m_blastBondIndexMap[blastBondIndex]))
                {
                    auto& b = m_bondsData[m_blastBondIndexMap[blastBondIndex]];
                    NVBLAST_ASSERT(m_nodesData[b.node0].solverNode != m_nodesData[b.node1].solverNode);
                }
            }
        }
        uint32_t mappedBondCount = 0;
        for (uint32_t i = 0; i < m_blastBondIndexMap.size(); i++)
        {
            const auto& bondIndex = m_blastBondIndexMap[i];
            if (!isInvalidIndex(bondIndex))
            {
                mappedBondCount++;
                NVBLAST_ASSERT(m_bondsData[bondIndex].blastBondIndex == i);
            }
        }
        NVBLAST_ASSERT(m_bondsData.size() == mappedBondCount);
    }
#endif

    struct BondKey
    {
        uint32_t node0;
        uint32_t node1;

        BondKey(uint32_t n0, uint32_t n1) : node0(n0), node1(n1) {}

        operator uint64_t() const
        {
            // Szudzik's function
            return node0 >= node1 ? (uint64_t)node0 * node0 + node0 + node1 : (uint64_t)node1 * node1 + node0;
        }
    };

    ConjugateGradientImpulseSolver      m_solver;
    Array<SolverNodeData>::type         m_solverNodesData;
    Array<SolverBondData>::type         m_solverBondsData;

    uint32_t                            m_graphReductionLevel;

    bool                                m_nodesDirty;
    bool                                m_bondsDirty;

    uint32_t                            m_overstressedBondCount;

    HashMap<BondKey, uint32_t>::type    m_solverBondsMap;
    Array<uint32_t>::type               m_blastBondIndexMap;

    Array<BondData>::type               m_bondsData;
    Array<NodeData>::type               m_nodesData;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           ExtStressSolver
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
*/
class ExtStressSolverImpl final : public ExtStressSolver
{
    NV_NOCOPY(ExtStressSolverImpl)

public:
    ExtStressSolverImpl(const NvBlastFamily& family, const ExtStressSolverSettings& settings);
    virtual void                            release() override;


    //////// ExtStressSolverImpl interface ////////

    virtual void                            setAllNodesInfoFromLL(float density = 1.0f) override;

    virtual void                            setNodeInfo(uint32_t graphNode, float mass, float volume, NvcVec3 localPos) override;

    virtual void                            setSettings(const ExtStressSolverSettings& settings) override
    {
        m_settings = settings;
        inheritSettingsLimits();
    }

    virtual const ExtStressSolverSettings&  getSettings() const override
    {
        return m_settings;
    }

    virtual bool                            addForce(const NvBlastActor& actor, NvcVec3 localPosition, NvcVec3 localForce, ExtForceMode::Enum mode) override;

    virtual void                            addForce(uint32_t graphNode, NvcVec3 localForce, ExtForceMode::Enum mode) override;

    virtual bool                            addGravity(const NvBlastActor& actor, NvcVec3 localGravity) override;
    virtual bool                            addCentrifugalAcceleration(const NvBlastActor& actor, NvcVec3 localCenterMass, NvcVec3 localAngularVelocity) override;

    virtual void                            update() override;

    virtual uint32_t                        getOverstressedBondCount() const override
    {
        return m_graphProcessor->getOverstressedBondCount();
    }

    virtual void                            generateFractureCommands(const NvBlastActor& actor, NvBlastFractureBuffers& commands) override;
    virtual uint32_t                        generateFractureCommandsPerActor(const NvBlastActor** actorBuffer, NvBlastFractureBuffers* commandsBuffer, uint32_t bufferSize) override;


    void                                    reset() override
    {
        m_reset = true;
    }

    virtual float                           getStressErrorLinear() const override
    {
        return m_errorLinear;
    }

    virtual float                           getStressErrorAngular() const override
    {
        return m_errorAngular;
    }

    virtual bool                            converged() const override
    {
        return m_converged;
    }

    virtual uint32_t                        getFrameCount() const override
    {
        return m_framesCount;
    }

    virtual uint32_t                        getBondCount() const override
    {
        return m_graphProcessor->getSolverBondCount();
    }

    virtual bool                            getExcessForces(uint32_t actorIndex, const NvcVec3& com, NvcVec3& force, NvcVec3& torque) override;

    virtual bool                            notifyActorCreated(const NvBlastActor& actor) override;

    virtual void                            notifyActorDestroyed(const NvBlastActor& actor) override;

    virtual const DebugBuffer               fillDebugRender(const uint32_t* nodes, uint32_t nodeCount, DebugRenderMode mode, float scale) override;


private:
    ~ExtStressSolverImpl();


    //////// private methods ////////

    void                                    solve();

    void                                    fillFractureCommands(const NvBlastActor& actor, NvBlastFractureBuffers& commands);

    void                                    initialize();

    void                                    iterate();

    void                                    removeBrokenBonds();

    template<class T>
    T*                                      getScratchArray(uint32_t size);

    bool                                    generateStressDamage(const NvBlastActor& actor, uint32_t bondIndex, uint32_t node0, uint32_t node1);
    void                                    inheritSettingsLimits()
    {
        NVBLAST_ASSERT(m_settings.compressionElasticLimit >= 0.0f && m_settings.compressionFatalLimit >= 0.0f);

        // check if any optional limits need to inherit from the compression values
        if (m_settings.tensionElasticLimit < 0.0f)
        {
            m_settings.tensionElasticLimit = m_settings.compressionElasticLimit;
        }
        if (m_settings.tensionFatalLimit < 0.0f)
        {
            m_settings.tensionFatalLimit = m_settings.compressionFatalLimit;
        }

        if (m_settings.shearElasticLimit < 0.0f)
        {
            m_settings.shearElasticLimit = m_settings.compressionElasticLimit;
        }
        if (m_settings.shearFatalLimit < 0.0f)
        {
            m_settings.shearFatalLimit = m_settings.compressionFatalLimit;
        }
    }


    //////// data ////////

    const NvBlastFamily&                                                m_family;
    HashSet<const NvBlastActor*>::type                                  m_activeActors;
    ExtStressSolverSettings                                             m_settings;
    NvBlastSupportGraph                                                 m_graph;
    bool                                                                m_isDirty;
    bool                                                                m_reset;
    const float*                                                        m_bondHealths;
    const float*                                                        m_cachedBondHealths;
    const NvBlastBond*                                                  m_bonds;
    SupportGraphProcessor*                                              m_graphProcessor;
    float                                                               m_errorAngular;
    float                                                               m_errorLinear;
    bool                                                                m_converged;
    uint32_t                                                            m_framesCount;
    Array<NvBlastBondFractureData>::type                                m_bondFractureBuffer;
    Array<uint8_t>::type                                                m_scratch;
    Array<DebugLine>::type                                              m_debugLineBuffer;
};


template<class T>
NV_INLINE T* ExtStressSolverImpl::getScratchArray(uint32_t size)
{
    const uint32_t scratchSize = sizeof(T) * size;
    if (m_scratch.size() < scratchSize)
    {
        m_scratch.resize(scratchSize);
    }
    return reinterpret_cast<T*>(m_scratch.begin());
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  Creation
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ExtStressSolverImpl::ExtStressSolverImpl(const NvBlastFamily& family, const ExtStressSolverSettings& settings)
    : m_family(family), m_settings(settings), m_isDirty(false), m_reset(false),
    m_errorAngular(std::numeric_limits<float>::max()), m_errorLinear(std::numeric_limits<float>::max()),
    m_converged(false), m_framesCount(0)
{
    // this needs to be called any time settings change, including when they are first set
    inheritSettingsLimits();

    const NvBlastAsset* asset = NvBlastFamilyGetAsset(&m_family, logLL);
    NVBLAST_ASSERT(asset);

    m_graph = NvBlastAssetGetSupportGraph(asset, logLL);
    const uint32_t bondCount = NvBlastAssetGetBondCount(asset, logLL);

    m_bondFractureBuffer.reserve(bondCount);

    {
        NvBlastActor* actor;
        NvBlastFamilyGetActors(&actor, 1, &family, logLL);
        m_bondHealths = NvBlastActorGetBondHealths(actor, logLL);
        m_cachedBondHealths = NvBlastActorGetCachedBondHeaths(actor, logLL);
        m_bonds = NvBlastAssetGetBonds(asset, logLL);
    }

    m_graphProcessor = NVBLAST_NEW(SupportGraphProcessor)(m_graph.nodeCount, bondCount);

    // traverse graph and fill bond info
    for (uint32_t node0 = 0; node0 < m_graph.nodeCount; ++node0)
    {
        for (uint32_t adjacencyIndex = m_graph.adjacencyPartition[node0]; adjacencyIndex < m_graph.adjacencyPartition[node0 + 1]; adjacencyIndex++)
        {
            uint32_t bondIndex = m_graph.adjacentBondIndices[adjacencyIndex];
            if (m_bondHealths[bondIndex] <= 0.0f)
                continue;
            uint32_t node1 = m_graph.adjacentNodeIndices[adjacencyIndex];

            if (node0 < node1)
            {
                m_graphProcessor->addBond(node0, node1, bondIndex);
            }
        }
    }
}

ExtStressSolverImpl::~ExtStressSolverImpl()
{
    NVBLAST_DELETE(m_graphProcessor, SupportGraphProcessor);
}

ExtStressSolver* ExtStressSolver::create(const NvBlastFamily& family, const ExtStressSolverSettings& settings)
{
    return NVBLAST_NEW(ExtStressSolverImpl) (family, settings);
}

void ExtStressSolverImpl::release()
{
    NVBLAST_DELETE(this, ExtStressSolverImpl);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                          Actors & Graph Data
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ExtStressSolverImpl::setAllNodesInfoFromLL(float density)
{
    const NvBlastAsset* asset = NvBlastFamilyGetAsset(&m_family, logLL);
    NVBLAST_ASSERT(asset);

    const uint32_t chunkCount = NvBlastAssetGetChunkCount(asset, logLL);
    const NvBlastChunk* chunks = NvBlastAssetGetChunks(asset, logLL);

    // traverse graph and fill node info
    for (uint32_t node0 = 0; node0 < m_graph.nodeCount; ++node0)
    {
        const uint32_t chunkIndex0 = m_graph.chunkIndices[node0];
        if (chunkIndex0 >= chunkCount)
        {
            // chunkIndex is invalid means it is static node (represents world)
            m_graphProcessor->setNodeInfo(node0, 0.0f, 0.0f, NvVec3(NvZero));
        }
        else
        {
            // fill node info
            const NvBlastChunk& chunk = chunks[chunkIndex0];
            const float volume = chunk.volume;
            const float mass = volume * density;
            const NvVec3 localPos = *reinterpret_cast<const NvVec3*>(chunk.centroid);
            m_graphProcessor->setNodeInfo(node0, mass, volume, localPos);
        }
    }
}

void ExtStressSolverImpl::setNodeInfo(uint32_t graphNode, float mass, float volume, NvcVec3 localPos)
{
    m_graphProcessor->setNodeInfo(graphNode, mass, volume, toNvShared(localPos));
}

bool ExtStressSolverImpl::getExcessForces(uint32_t actorIndex, const NvcVec3& com, NvcVec3& force, NvcVec3& torque)
{
    // otherwise allocate enough space and query the Blast SDK
    const NvBlastActor* actor = NvBlastFamilyGetActorByIndex(&m_family, actorIndex, logLL);
    if (actor == nullptr)
    {
        return false;
    }

    const uint32_t nodeCount = NvBlastActorGetGraphNodeCount(actor, logLL);
    uint32_t* nodeIndices = getScratchArray<uint32_t>(nodeCount);
    const uint32_t retCount = NvBlastActorGetGraphNodeIndices(nodeIndices, nodeCount, actor, logLL);
    NVBLAST_ASSERT(retCount == nodeCount);

    // get the mapping between support chunks and actor indices
    // this is the fastest way to tell if two node/chunks are part of the same actor
    const uint32_t* actorIndices = NvBlastFamilyGetChunkActorIndices(&m_family, logLL);
    if (actorIndices == nullptr)
    {
        return false;
    }

    // walk the visible nodes for the actor looking for bonds that broke this frame
    nvidia::NvVec3 totalForce(0.0f);
    nvidia::NvVec3 totalTorque(0.0f);
    for (uint32_t n = 0; n < nodeCount; n++)
    {
        // find bonds that broke this frame (health <= 0 but internal stress bond index is still valid)
        const uint32_t nodeIdx = nodeIndices[n];
        for (uint32_t i = m_graph.adjacencyPartition[nodeIdx]; i < m_graph.adjacencyPartition[nodeIdx + 1]; i++)
        {
            // check if the bond is broken first of all
            const uint32_t blastBondIndex = m_graph.adjacentBondIndices[i];
            if (m_bondHealths[blastBondIndex] > 0.0f)
            {
                continue;
            }

            // broken bonds that have invalid internal indices broke before this frame
            const uint32_t internalBondIndex = m_graphProcessor->getInternalBondIndex(blastBondIndex);
            if (isInvalidIndex(internalBondIndex))
            {
                continue;
            }

            // make sure the other node in the bond isn't part of the same actor
            // forces should only be applied due to bonds breaking between actors, not within
            const uint32_t chunkIdx = m_graph.chunkIndices[nodeIdx];
            const uint32_t otherNodeIdx = m_graph.adjacentNodeIndices[i];
            const uint32_t otherChunkIdx = m_graph.chunkIndices[otherNodeIdx];
            if (!isInvalidIndex(chunkIdx) && !isInvalidIndex(otherChunkIdx) && actorIndices[chunkIdx] == actorIndices[otherChunkIdx])
            {
                continue;
            }

            // this bond should contribute forces to the output
            const auto bondData = m_graphProcessor->getBondData(internalBondIndex);
            NVBLAST_ASSERT(blastBondIndex == bondData.blastBondIndex);
            uint32_t node0, node1;
            m_graphProcessor->getSolverInternalBondNodes(internalBondIndex, node0, node1);
            NVBLAST_ASSERT(bondData.node0 == internalBondData.node0 && bondData.node1 == internalBondData.node1);

            // accumulators for forces just from this bond
            nvidia::NvVec3 nvLinearPressure(0.0f);
            nvidia::NvVec3 nvAngularPressure(0.0f);

            // deal with linear forces
            const float excessCompression = bondData.stressNormal + m_settings.compressionFatalLimit;
            const float excessTension = bondData.stressNormal - m_settings.tensionFatalLimit;
            if (excessCompression < 0.0f)
            {
                nvLinearPressure += excessCompression * bondData.normal;
            }
            else if (excessTension > 0.0f)
            {
                // tension is in the negative direction of the linear impulse
                nvLinearPressure += excessTension * bondData.normal;
            }

            const float excessShear = bondData.stressShear - m_settings.shearFatalLimit;
            if (excessShear > 0.0f)
            {
                NvVec3 impulseLinear, impulseAngular;
                m_graphProcessor->getSolverInternalBondImpulses(internalBondIndex, impulseLinear, impulseAngular);
                const nvidia::NvVec3 shearDir = impulseLinear - impulseLinear.dot(bondData.normal)*bondData.normal;
                nvLinearPressure += excessShear * shearDir.getNormalized();
            }

            if (nvLinearPressure.magnitudeSquared() > FLT_EPSILON)
            {
                const float* bondCenter = m_bonds[blastBondIndex].centroid;
                const nvidia::NvVec3 forceOffset = nvidia::NvVec3(bondCenter[0], bondCenter[1], bondCenter[3]) - toNvShared(com);
                const nvidia::NvVec3 torqueFromForce = forceOffset.cross(nvLinearPressure);
                nvAngularPressure += torqueFromForce;
            }

            // add the contributions from this bond to the total forces for the actor
            // multiply by the area to convert back to force from pressure
            const float bondRemainingArea = m_cachedBondHealths[blastBondIndex];
            NVBLAST_ASSERT(bondRemainingArea <= m_bonds[blastBondIndex].area);

            const float sign = otherNodeIdx > nodeIdx ? 1.0f : -1.0f;

            totalForce += nvLinearPressure * (sign*bondRemainingArea);
            totalTorque += nvAngularPressure * (sign*bondRemainingArea);
        }
    }

    // convert to the output format and return true if non-zero forces were accumulated
    force = fromNvShared(totalForce);
    torque = fromNvShared(totalTorque);
    return (totalForce.magnitudeSquared() + totalTorque.magnitudeSquared()) > 0.0f;
}

bool ExtStressSolverImpl::notifyActorCreated(const NvBlastActor& actor)
{
    const uint32_t graphNodeCount = NvBlastActorGetGraphNodeCount(&actor, logLL);
    if (graphNodeCount > 1)
    {
        // update neighbors
        {
            uint32_t* graphNodeIndices = getScratchArray<uint32_t>(graphNodeCount);
            const uint32_t nodeCount = NvBlastActorGetGraphNodeIndices(graphNodeIndices, graphNodeCount, &actor, logLL);
            for (uint32_t i = 0; i < nodeCount; ++i)
            {
                m_graphProcessor->setNodeNeighborsCount(graphNodeIndices[i], nodeCount);
            }
        }

        m_activeActors.insert(&actor);
        m_isDirty = true;
        return true;
    }
    return false;
}

void ExtStressSolverImpl::notifyActorDestroyed(const NvBlastActor& actor)
{
    if (m_activeActors.erase(&actor))
    {
        m_isDirty = true;
    }
}

void ExtStressSolverImpl::removeBrokenBonds()
{
    // traverse graph and remove dead bonds
    for (uint32_t node0 = 0; node0 < m_graph.nodeCount; ++node0)
    {
        for (uint32_t adjacencyIndex = m_graph.adjacencyPartition[node0]; adjacencyIndex < m_graph.adjacencyPartition[node0 + 1]; adjacencyIndex++)
        {
            uint32_t node1 = m_graph.adjacentNodeIndices[adjacencyIndex];
            if (node0 < node1)
            {
                uint32_t bondIndex = m_graph.adjacentBondIndices[adjacencyIndex];
                if (m_bondHealths[bondIndex] <= 0.0f)
                {
                    m_graphProcessor->removeBondIfExists(bondIndex);
                }
            }
        }
    }

    m_isDirty = false;
}

void ExtStressSolverImpl::initialize()
{
    if (m_reset)
    {
        m_framesCount = 0;
    }

    if (m_isDirty)
    {
        removeBrokenBonds();
    }

    if (m_settings.graphReductionLevel != m_graphProcessor->getGraphReductionLevel())
    {
        m_graphProcessor->setGraphReductionLevel(m_settings.graphReductionLevel);
    }
}

bool ExtStressSolverImpl::addForce(const NvBlastActor& actor, NvcVec3 localPosition, NvcVec3 localForce, ExtForceMode::Enum mode)
{
    float bestDist = FLT_MAX;
    uint32_t bestNode = invalidIndex<uint32_t>();

    const uint32_t graphNodeCount = NvBlastActorGetGraphNodeCount(&actor, logLL);
    if (graphNodeCount > 1)
    {
        uint32_t* graphNodeIndices = getScratchArray<uint32_t>(graphNodeCount);
        const uint32_t nodeCount = NvBlastActorGetGraphNodeIndices(graphNodeIndices, graphNodeCount, &actor, logLL);

        for (uint32_t i = 0; i < nodeCount; ++i)
        {
            const uint32_t node = graphNodeIndices[i];
            const float sqrDist = (toNvShared(localPosition) - m_graphProcessor->getNodeData(node).localPos).magnitudeSquared();
            if (sqrDist < bestDist)
            {
                bestDist = sqrDist;
                bestNode = node;
            }
        }

        if (!isInvalidIndex(bestNode))
        {
            m_graphProcessor->addNodeForce(bestNode, toNvShared(localForce), mode);
            return true;
        }
    }
    return false;
}

void ExtStressSolverImpl::addForce(uint32_t graphNode, NvcVec3 localForce, ExtForceMode::Enum mode)
{
    m_graphProcessor->addNodeForce(graphNode, toNvShared(localForce), mode);
}

bool ExtStressSolverImpl::addGravity(const NvBlastActor& actor, NvcVec3 localGravity)
{
    const uint32_t graphNodeCount = NvBlastActorGetGraphNodeCount(&actor, logLL);
    if (graphNodeCount > 1)
    {
        uint32_t* graphNodeIndices = getScratchArray<uint32_t>(graphNodeCount);
        const uint32_t nodeCount = NvBlastActorGetGraphNodeIndices(graphNodeIndices, graphNodeCount, &actor, logLL);

        for (uint32_t i = 0; i < nodeCount; ++i)
        {
            const uint32_t node = graphNodeIndices[i];
            m_graphProcessor->addNodeForce(node, toNvShared(localGravity), ExtForceMode::ACCELERATION);
        }
        return true;
    }
    return false;
}

bool ExtStressSolverImpl::addCentrifugalAcceleration(const NvBlastActor& actor, NvcVec3 localCenterMass, NvcVec3 localAngularVelocity)
{
    const uint32_t graphNodeCount = NvBlastActorGetGraphNodeCount(&actor, logLL);
    if (graphNodeCount > 1)
    {
        uint32_t* graphNodeIndices = getScratchArray<uint32_t>(graphNodeCount);
        const uint32_t nodeCount = NvBlastActorGetGraphNodeIndices(graphNodeIndices, graphNodeCount, &actor, logLL);

        // Apply centrifugal force
        for (uint32_t i = 0; i < nodeCount; ++i)
        {
            const uint32_t node = graphNodeIndices[i];
            const auto& localPos = m_graphProcessor->getNodeData(node).localPos;
            // a = w x (w x r)
            const NvVec3 centrifugalAcceleration =
                toNvShared(localAngularVelocity)
                    .cross(toNvShared(localAngularVelocity).cross(localPos - toNvShared(localCenterMass)));
            m_graphProcessor->addNodeForce(node, centrifugalAcceleration, ExtForceMode::ACCELERATION);
        }
        return true;
    }
    return false;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  Update
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ExtStressSolverImpl::update()
{
    initialize();

    solve();

    m_framesCount++;
}

void ExtStressSolverImpl::solve()
{
    NV_SIMD_GUARD;

    m_graphProcessor->solve(m_settings, m_bondHealths, m_bonds, WARM_START && !m_reset);
    m_reset = false;

    m_converged = m_graphProcessor->calcError(m_errorLinear, m_errorAngular);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  Damage
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// check if this bond is over stressed in any way and generate a fracture command if it is
bool ExtStressSolverImpl::generateStressDamage(const NvBlastActor& actor, uint32_t bondIndex, uint32_t node0, uint32_t node1)
{
    const float bondHealth = m_bondHealths[bondIndex];
    float stressCompression, stressTension, stressShear;
    if (bondHealth > 0.0f && m_graphProcessor->getBondStress(bondIndex, stressCompression, stressTension, stressShear))
    {
        // compression and tension are mutually exclusive, only one can be positive at a time since they act in opposite directions
        float stressMultiplier = 0.0f;
        if (stressCompression > m_settings.compressionElasticLimit)
        {
            const float excessStress = stressCompression - m_settings.compressionElasticLimit;
            const float compressionDenom = m_settings.compressionFatalLimit - m_settings.compressionElasticLimit;
            const float compressionMultiplier = excessStress / (compressionDenom > 0.0f ? compressionDenom : 1.0f);
            stressMultiplier += compressionMultiplier;
        }
        else if (stressTension > m_settings.tensionElasticLimit)
        {
            const float excessStress = stressTension - m_settings.tensionElasticLimit;
            const float tensionDenom = m_settings.tensionFatalLimit - m_settings.tensionElasticLimit;
            const float tensionMultiplier = excessStress / (tensionDenom > 0.0f ? tensionDenom : 1.0f);
            stressMultiplier += tensionMultiplier;
        }

        // shear can co-exist with either compression or tension so must be accounted for independently of them
        if (stressShear > m_settings.shearElasticLimit)
        {
            const float excessStress = stressShear - m_settings.shearElasticLimit;
            const float shearDenom = m_settings.shearFatalLimit - m_settings.shearElasticLimit;
            const float shearMultiplier = excessStress / (shearDenom > 0.0f ? shearDenom : 1.0f);
            stressMultiplier += shearMultiplier;
        }

        if (stressMultiplier > 0.0f)
        {
            // bond health/area is reduced by excess pressure to approximate micro bonds in the material breaking
            const float bondDamage = bondHealth * stressMultiplier;
            const NvBlastBondFractureData data = {
                0,
                node0,
                node1,
                bondDamage
            };
            m_bondFractureBuffer.pushBack(data);

            // cache off the current health value for this bond
            // so it can be used to calculate forces to apply if it breaks later
            NvBlastActorCacheBondHeath(&actor, bondIndex, logLL);
            return true;
        }
    }

    return false;
}

void ExtStressSolverImpl::fillFractureCommands(const NvBlastActor& actor, NvBlastFractureBuffers& commands)
{
    const uint32_t graphNodeCount = NvBlastActorGetGraphNodeCount(&actor, logLL);
    uint32_t commandCount = 0;

    if (graphNodeCount > 1 && m_graphProcessor->getOverstressedBondCount() > 0)
    {
        uint32_t* graphNodeIndices = getScratchArray<uint32_t>(graphNodeCount);
        const uint32_t nodeCount = NvBlastActorGetGraphNodeIndices(graphNodeIndices, graphNodeCount, &actor, logLL);

        for (uint32_t i = 0; i < nodeCount; ++i)
        {
            const uint32_t node0 = graphNodeIndices[i];
            for (uint32_t adjacencyIndex = m_graph.adjacencyPartition[node0]; adjacencyIndex < m_graph.adjacencyPartition[node0 + 1]; adjacencyIndex++)
            {
                const uint32_t node1 = m_graph.adjacentNodeIndices[adjacencyIndex];
                if (node0 < node1)
                {
                    const uint32_t bondIndex = m_graph.adjacentBondIndices[adjacencyIndex];
                    if (generateStressDamage(actor, bondIndex, node0, node1))
                    {
                        commandCount++;
                    }
                }
            }
        }
    }

    commands.chunkFractureCount = 0;
    commands.chunkFractures = nullptr;
    commands.bondFractureCount = commandCount;
    commands.bondFractures = commandCount > 0 ? m_bondFractureBuffer.end() - commandCount : nullptr;
}

void ExtStressSolverImpl::generateFractureCommands(const NvBlastActor& actor, NvBlastFractureBuffers& commands)
{
    m_bondFractureBuffer.clear();
    fillFractureCommands(actor, commands);
}

uint32_t ExtStressSolverImpl::generateFractureCommandsPerActor(const NvBlastActor** actorBuffer, NvBlastFractureBuffers* commandsBuffer, uint32_t bufferSize)
{
    if (m_graphProcessor->getOverstressedBondCount() == 0)
        return 0;

    m_bondFractureBuffer.clear();
    uint32_t index = 0;
    for (auto it = m_activeActors.getIterator(); !it.done() && index < bufferSize; ++it)
    {
        const NvBlastActor* actor = *it;
        NvBlastFractureBuffers& nextCommand = commandsBuffer[index];
        fillFractureCommands(*actor, nextCommand);
        if (nextCommand.bondFractureCount > 0)
        {
            actorBuffer[index] = actor;
            index++;
        }
    }
    return index;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  Debug Render
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline uint32_t NvVec4ToU32Color(const NvVec4& color)
{
    return ((uint32_t)(color.w * 255) << 24) | // A
           ((uint32_t)(color.x * 255) << 16) | // R
           ((uint32_t)(color.y * 255) << 8)  | // G
           ((uint32_t)(color.z * 255));        // B
}

static float Lerp(float v0, float v1, float val)
{
    return v0 * (1 - val) + v1 * val;
}

inline float clamp01(float v)
{
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

inline NvVec4 colorConvertHSVAtoRGBA(float h, float s, float v, float a)
{
    const float t = 6.0f * (h - std::floor(h));
    const int n = (int)t;
    const float m = t - (float)n;
    const float c = 1.0f - s;
    const float b[6] = { 1.0f, 1.0f - s * m, c, c, 1.0f - s * (1.0f - m), 1.0f };
    return NvVec4(v * b[n % 6], v * b[(n + 4) % 6], v * b[(n + 2) % 6], a); // n % 6 protects against roundoff errors
}

inline uint32_t bondHealthColor(float stressPct)
{
    stressPct = clamp01(stressPct);

    constexpr float BOND_HEALTHY_HUE = 1.0f/3.0f;   // Green
    constexpr float BOND_ELASTIC_HUE = 0.0f;        // Red
    constexpr float BOND_STRESSED_HUE = 2.0f/3.0f;  // Blue
    constexpr float BOND_FATAL_HUE = 5.0f/6.0f;     // Magenta

    const float hue = stressPct < 0.5f ?
        Lerp(BOND_HEALTHY_HUE, BOND_ELASTIC_HUE, 2.0f * stressPct) : Lerp(BOND_STRESSED_HUE, BOND_FATAL_HUE, 2.0f * stressPct - 1.0f);

    return NvVec4ToU32Color(colorConvertHSVAtoRGBA(hue, 1.0f, 1.0f, 1.0f));
}

const ExtStressSolver::DebugBuffer ExtStressSolverImpl::fillDebugRender(const uint32_t* nodes, uint32_t nodeCount, DebugRenderMode mode, float scale)
{
    NV_UNUSED(scale);

    const uint32_t BOND_UNBREAKABLE_COLOR = NvVec4ToU32Color(NvVec4(0.0f, 0.682f, 1.0f, 1.0f));

    ExtStressSolver::DebugBuffer debugBuffer = { nullptr, 0 };

    if (m_isDirty)
        return debugBuffer;

    m_debugLineBuffer.clear();

    Array<uint8_t>::type& nodesSet = m_scratch;

    nodesSet.resize(m_graphProcessor->getSolverNodeCount());
    memset(nodesSet.begin(), 0, nodesSet.size() * sizeof(uint8_t));
    for (uint32_t i = 0; i < nodeCount; ++i)
    {
        NVBLAST_ASSERT(m_graphProcessor->getNodeData(nodes[i]).solverNode < nodesSet.size());
        nodesSet[m_graphProcessor->getNodeData(nodes[i]).solverNode] = 1;
    }

    const uint32_t bondCount = m_graphProcessor->getSolverBondCount();
    for (uint32_t i = 0; i < bondCount; ++i)
    {
        const auto& bondData = m_graphProcessor->getBondData(i);
        uint32_t node0, node1;
        m_graphProcessor->getSolverInternalBondNodes(i, node0, node1);
        if (nodesSet[node0] != 0)
        {
            //NVBLAST_ASSERT(nodesSet[node1] != 0);
            const auto& solverNode0 = m_graphProcessor->getSolverNodeData(node0);
            const auto& solverNode1 = m_graphProcessor->getSolverNodeData(node1);
            const NvcVec3 p0 = fromNvShared(solverNode0.mass > 0.0f ? solverNode0.localPos : bondData.centroid);
            const NvcVec3 p1 = fromNvShared(solverNode1.mass > 0.0f ? solverNode1.localPos : bondData.centroid);

            // don't render lines for broken bonds
            const float stressPct = m_graphProcessor->getSolverBondStressPct(i, m_bondHealths, m_settings, mode);
            if (stressPct >= 0.0f)
            {
                const uint32_t color = canTakeDamage(m_bondHealths[bondData.blastBondIndex]) ? bondHealthColor(stressPct) : BOND_UNBREAKABLE_COLOR;
                m_debugLineBuffer.pushBack(DebugLine(p0, p1, color));
            }
        }
    }

    debugBuffer.lines = m_debugLineBuffer.begin();
    debugBuffer.lineCount = m_debugLineBuffer.size();

    return debugBuffer;
}


} // namespace Blast
} // namespace Nv
