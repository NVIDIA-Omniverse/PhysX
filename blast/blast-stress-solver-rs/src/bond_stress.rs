//! Pure-Rust bond stress computation (no FFI).

use crate::stress_processor::{StressBondDesc, StressImpulse, StressNodeDesc};
use crate::types::BondStressResult;

/// Compute the decomposed stress on a bond given the solver impulse.
///
/// Returns compression, tension, and shear in Pascals.
pub fn compute_bond_stress(
    bond: &StressBondDesc,
    impulse: &StressImpulse,
    nodes: &[StressNodeDesc],
    bond_area: f32,
) -> BondStressResult {
    if bond_area <= 0.0 {
        return BondStressResult::default();
    }

    let node0 = bond.node0 as usize;
    let node1 = bond.node1 as usize;
    if node0 >= nodes.len() || node1 >= nodes.len() {
        return BondStressResult::default();
    }

    let displacement = nodes[node1].com - nodes[node0].com;
    let node_distance = displacement.magnitude().max(1.0e-6);
    let bond_normal = displacement.normalize();

    let linear = impulse.lin;
    let angular = impulse.ang;

    let normal_component_linear = linear.dot(bond_normal);
    let shear_linear_sq =
        (linear.magnitude_squared() - normal_component_linear * normal_component_linear).max(0.0);
    let mut stress_normal = normal_component_linear / bond_area;
    let mut stress_shear = shear_linear_sq.sqrt() / bond_area;

    let normal_component_angular = angular.dot(bond_normal).abs();
    let angular_mag_sq = angular.magnitude_squared();
    let twist = normal_component_angular / bond_area;
    let bend_sq =
        (angular_mag_sq - normal_component_angular * normal_component_angular).max(0.0);
    let bend = bend_sq.sqrt() / bond_area;

    let twist_contribution = twist * 2.0 / node_distance;
    stress_shear += twist_contribution;

    let bend_contribution = bend * 2.0 / node_distance;
    let sign = if stress_normal >= 0.0 { 1.0 } else { -1.0 };
    stress_normal += bend_contribution * sign;

    BondStressResult {
        compression: if stress_normal < 0.0 {
            -stress_normal
        } else {
            0.0
        },
        tension: if stress_normal > 0.0 {
            stress_normal
        } else {
            0.0
        },
        shear: stress_shear,
    }
}
