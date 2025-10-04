#include "stress_bridge.h"

#include <new>
#include <vector>

#include "anglin6.h"
#include "solver_types.h"
#include "stress.h"

struct StressProcessorHandle
{
    StressProcessor solver;
    std::vector<SolverNodeS> nodes;
    std::vector<SolverBond> bonds;
    std::vector<AngLin6> impulses;
    std::vector<AngLin6> velocities;
};

namespace
{
inline AngLin6 make_anglin6()
{
    AngLin6 value;
    value.ang = {0.0f, 0.0f, 0.0f};
    value.lin = {0.0f, 0.0f, 0.0f};
    return value;
}

inline SolverNodeS to_solver_node(const StressNodeDesc& desc)
{
    SolverNodeS node;
    node.CoM = {desc.com.x, desc.com.y, desc.com.z};
    node.mass = desc.mass;
    node.inertia = desc.inertia;
    return node;
}

inline SolverBond to_solver_bond(const StressBondDesc& desc)
{
    SolverBond bond;
    bond.centroid = {desc.centroid.x, desc.centroid.y, desc.centroid.z};
    bond.nodes[0] = desc.node0;
    bond.nodes[1] = desc.node1;
    return bond;
}

inline AngLin6 to_anglin6(const StressImpulse& impulse)
{
    AngLin6 value;
    value.ang = {impulse.ang.x, impulse.ang.y, impulse.ang.z};
    value.lin = {impulse.lin.x, impulse.lin.y, impulse.lin.z};
    return value;
}

inline StressImpulse from_anglin6(const AngLin6& value)
{
    StressImpulse impulse;
    impulse.ang = {value.ang.x, value.ang.y, value.ang.z};
    impulse.lin = {value.lin.x, value.lin.y, value.lin.z};
    return impulse;
}

inline AngLin6 to_anglin6(const StressVelocity& velocity)
{
    AngLin6 value;
    value.ang = {velocity.ang.x, velocity.ang.y, velocity.ang.z};
    value.lin = {velocity.lin.x, velocity.lin.y, velocity.lin.z};
    return value;
}
} // namespace

extern "C" StressProcessorHandle*
stress_processor_create(const StressNodeDesc* nodes,
                        uint32_t node_count,
                        const StressBondDesc* bonds,
                        uint32_t bond_count,
                        StressDataParams params)
{
    if (node_count == 0U || bond_count == 0U || nodes == nullptr || bonds == nullptr)
    {
        return nullptr;
    }

    StressProcessorHandle* handle = new (std::nothrow) StressProcessorHandle();
    if (!handle)
    {
        return nullptr;
    }

    handle->nodes.resize(node_count);
    for (uint32_t i = 0; i < node_count; ++i)
    {
        handle->nodes[i] = to_solver_node(nodes[i]);
    }

    handle->bonds.resize(bond_count);
    for (uint32_t i = 0; i < bond_count; ++i)
    {
        handle->bonds[i] = to_solver_bond(bonds[i]);
    }

    handle->impulses.assign(bond_count, make_anglin6());
    handle->velocities.assign(node_count, make_anglin6());

    StressProcessor::DataParams data_params;
    data_params.centerBonds = params.center_bonds != 0;
    data_params.equalizeMasses = params.equalize_masses != 0;

    handle->solver.prepare(handle->nodes.data(), node_count,
                           handle->bonds.data(), bond_count, data_params);

    return handle;
}

extern "C" void
stress_processor_destroy(StressProcessorHandle* handle)
{
    delete handle;
}

extern "C" uint32_t
stress_processor_node_count(const StressProcessorHandle* handle)
{
    return handle ? handle->solver.getNodeCount() : 0U;
}

extern "C" uint32_t
stress_processor_bond_count(const StressProcessorHandle* handle)
{
    return handle ? handle->solver.getBondCount() : 0U;
}

extern "C" int32_t
stress_processor_solve(StressProcessorHandle* handle,
                       StressImpulse* impulses,
                       const StressVelocity* velocities,
                       StressSolverParams params,
                       StressErrorSq* out_error,
                       uint8_t resume_solver)
{
    if (!handle || !impulses || !velocities)
    {
        return -1;
    }

    const uint32_t node_count = handle->solver.getNodeCount();
    const uint32_t bond_count = handle->solver.getBondCount();

    handle->velocities.resize(node_count);
    for (uint32_t i = 0; i < node_count; ++i)
    {
        handle->velocities[i] = to_anglin6(velocities[i]);
    }

    handle->impulses.resize(bond_count);
    for (uint32_t i = 0; i < bond_count; ++i)
    {
        handle->impulses[i] = to_anglin6(impulses[i]);
    }

    StressProcessor::SolverParams solver_params;
    solver_params.maxIter = params.max_iterations;
    solver_params.tolerance = params.tolerance;
    solver_params.warmStart = params.warm_start != 0;

    AngLin6ErrorSq error_sq{};
    AngLin6ErrorSq* error_sq_ptr = out_error ? &error_sq : nullptr;

    const bool resume = resume_solver != 0;
    const int32_t iterations = handle->solver.solve(handle->impulses.data(),
                                                    handle->velocities.data(),
                                                    solver_params,
                                                    error_sq_ptr,
                                                    resume);

    for (uint32_t i = 0; i < bond_count; ++i)
    {
        impulses[i] = from_anglin6(handle->impulses[i]);
    }

    if (out_error)
    {
        out_error->ang = error_sq.ang;
        out_error->lin = error_sq.lin;
    }

    return iterations;
}

extern "C" uint8_t
stress_processor_remove_bond(StressProcessorHandle* handle, uint32_t bond_index)
{
    if (!handle)
    {
        return 0;
    }

    if (!handle->solver.removeBond(bond_index))
    {
        return 0;
    }

    if (bond_index < handle->bonds.size())
    {
        if (bond_index + 1U < handle->bonds.size())
        {
            handle->bonds[bond_index] = handle->bonds.back();
        }
        handle->bonds.pop_back();
    }

    if (bond_index < handle->impulses.size())
    {
        if (bond_index + 1U < handle->impulses.size())
        {
            handle->impulses[bond_index] = handle->impulses.back();
        }
        handle->impulses.pop_back();
    }

    return 1;
}

extern "C" uint8_t
stress_processor_get_node_desc(const StressProcessorHandle* handle,
                               uint32_t index,
                               StressNodeDesc* out_desc)
{
    if (!handle || !out_desc || index >= handle->nodes.size())
    {
        return 0;
    }

    const SolverNodeS& node = handle->nodes[index];
    out_desc->com = { node.CoM.x, node.CoM.y, node.CoM.z };
    out_desc->mass = node.mass;
    out_desc->inertia = node.inertia;

    return 1;
}

extern "C" uint8_t
stress_processor_get_bond_desc(const StressProcessorHandle* handle,
                               uint32_t index,
                               StressBondDesc* out_desc)
{
    if (!handle || !out_desc || index >= handle->bonds.size())
    {
        return 0;
    }

    const SolverBond& bond = handle->bonds[index];
    out_desc->centroid = { bond.centroid.x, bond.centroid.y, bond.centroid.z };
    out_desc->node0 = bond.nodes[0];
    out_desc->node1 = bond.nodes[1];

    return 1;
}

extern "C" uint8_t
stress_processor_using_simd()
{
    return StressProcessor::usingSIMD() ? 1 : 0;
}

extern "C" uint32_t
stress_sizeof_stress_vec3()
{
    return static_cast<uint32_t>(sizeof(StressVec3));
}

extern "C" uint32_t
stress_sizeof_node_desc()
{
    return static_cast<uint32_t>(sizeof(StressNodeDesc));
}

extern "C" uint32_t
stress_sizeof_bond_desc()
{
    return static_cast<uint32_t>(sizeof(StressBondDesc));
}

extern "C" uint32_t
stress_sizeof_velocity()
{
    return static_cast<uint32_t>(sizeof(StressVelocity));
}

extern "C" uint32_t
stress_sizeof_impulse()
{
    return static_cast<uint32_t>(sizeof(StressImpulse));
}

extern "C" uint32_t
stress_sizeof_data_params()
{
    return static_cast<uint32_t>(sizeof(StressDataParams));
}

extern "C" uint32_t
stress_sizeof_solver_params()
{
    return static_cast<uint32_t>(sizeof(StressSolverParams));
}

extern "C" uint32_t
stress_sizeof_error_sq()
{
    return static_cast<uint32_t>(sizeof(StressErrorSq));
}
