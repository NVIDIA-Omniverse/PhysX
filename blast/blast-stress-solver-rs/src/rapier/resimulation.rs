use std::collections::HashMap;

use rapier3d::prelude::*;

use super::body_tracker::BodyTracker;
use super::destructible::SplitCohort;

#[derive(Clone, Copy, Debug)]
pub struct ResimulationOptions {
    pub enabled: bool,
    pub max_passes: usize,
}

impl Default for ResimulationOptions {
    fn default() -> Self {
        Self {
            enabled: false,
            max_passes: 0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct BodySnapshot {
    handle: RigidBodyHandle,
    position: Isometry<Real>,
    center_of_mass: Point<Real>,
    linvel: Vector<Real>,
    angvel: AngVector<Real>,
    linear_damping: Real,
    angular_damping: Real,
    user_force: Vector<Real>,
    user_torque: AngVector<Real>,
    sleeping: bool,
    enabled: bool,
}

#[derive(Clone, Copy, Debug)]
struct NodeSnapshot {
    world_centroid: Point<Real>,
}

#[derive(Clone, Debug, Default)]
pub struct BodySnapshots {
    bodies: Vec<BodySnapshot>,
    nodes: HashMap<u32, NodeSnapshot>,
}

impl BodySnapshots {
    pub fn capture(set: &RigidBodySet, tracker: &BodyTracker) -> Self {
        let bodies = set
            .iter()
            .map(|(handle, body)| BodySnapshot {
                handle,
                position: *body.position(),
                center_of_mass: *body.center_of_mass(),
                linvel: *body.linvel(),
                angvel: *body.angvel(),
                linear_damping: body.linear_damping(),
                angular_damping: body.angular_damping(),
                user_force: body.user_force(),
                user_torque: body.user_torque(),
                sleeping: body.is_sleeping(),
                enabled: body.is_enabled(),
            })
            .collect();
        let mut nodes = HashMap::new();
        for node_index in 0..tracker.node_count() as u32 {
            let Some(body_handle) = tracker.node_body(node_index) else {
                continue;
            };
            let Some(local_offset) = tracker.node_local_offset(node_index) else {
                continue;
            };
            let Some(body) = set.get(body_handle) else {
                continue;
            };
            let world_centroid = body.position().transform_point(&point![
                local_offset.x,
                local_offset.y,
                local_offset.z
            ]);
            nodes.insert(node_index, NodeSnapshot { world_centroid });
        }
        Self { bodies, nodes }
    }

    pub fn restore(&self, set: &mut RigidBodySet) {
        for snapshot in &self.bodies {
            let Some(body) = set.get_mut(snapshot.handle) else {
                continue;
            };
            body.set_enabled(snapshot.enabled);
            body.set_position(snapshot.position, true);
            if !body.is_fixed() {
                body.set_linvel(snapshot.linvel, true);
                body.set_angvel(snapshot.angvel, true);
                body.set_linear_damping(snapshot.linear_damping);
                body.set_angular_damping(snapshot.angular_damping);
                body.reset_forces(true);
                body.reset_torques(true);
                if snapshot.user_force.norm_squared() > 0.0 {
                    body.add_force(snapshot.user_force, true);
                }
                if snapshot.user_torque.norm_squared() > 0.0 {
                    body.add_torque(snapshot.user_torque, true);
                }
            }
            if snapshot.sleeping {
                body.sleep();
            } else {
                body.wake_up(true);
            }
        }
    }

    pub fn restore_split_children(
        &self,
        set: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        tracker: &mut BodyTracker,
        split_cohorts: &[SplitCohort],
    ) {
        for cohort in split_cohorts {
            for &target_handle in &cohort.target_bodies {
                let Some(source_handle) = cohort
                    .target_sources
                    .iter()
                    .find_map(|(target, source)| (*target == target_handle).then_some(*source))
                else {
                    continue;
                };
                let Some(source_snapshot) = self
                    .bodies
                    .iter()
                    .find(|snapshot| snapshot.handle == source_handle)
                    .copied()
                else {
                    continue;
                };
                let target_nodes = tracker.body_nodes(target_handle);
                if target_nodes.is_empty() {
                    continue;
                }
                let target_has_support = target_nodes.iter().any(|node| tracker.is_support(*node));
                let mut centroid_sum = Vector::zeros();
                let mut com_sum = Vector::zeros();
                let mut centroid_count = 0usize;
                let mut total_mass = 0.0;
                for &node in &target_nodes {
                    let Some(node_snapshot) = self.nodes.get(&node) else {
                        continue;
                    };
                    centroid_sum += node_snapshot.world_centroid.coords;
                    centroid_count += 1;
                    let mass = tracker.node_mass(node).max(0.0);
                    if mass > 0.0 {
                        com_sum += node_snapshot.world_centroid.coords * mass;
                        total_mass += mass;
                    }
                }
                if centroid_count == 0 {
                    continue;
                }
                let translation = if target_has_support || total_mass <= f32::EPSILON {
                    centroid_sum / centroid_count as Real
                } else {
                    com_sum / total_mass
                };
                let target_pose = Isometry::from_parts(
                    Translation::from(translation),
                    source_snapshot.position.rotation,
                );
                let Some(target_body) = set.get_mut(target_handle) else {
                    continue;
                };
                target_body.set_enabled(source_snapshot.enabled);
                target_body.set_position(target_pose, true);
                let target_pose = *target_body.position();
                for &node in &target_nodes {
                    let Some(node_snapshot) = self.nodes.get(&node) else {
                        continue;
                    };
                    let local = target_pose
                        .inverse_transform_point(&node_snapshot.world_centroid)
                        .coords;
                    tracker.restore_node_local_offset(
                        node,
                        crate::types::Vec3::new(local.x, local.y, local.z),
                        colliders,
                    );
                }
                if !target_body.is_fixed() {
                    target_body.recompute_mass_properties_from_colliders(colliders);
                    let target_com = *target_body.center_of_mass();
                    let com_delta = target_com - source_snapshot.center_of_mass;
                    let linvel = source_snapshot.linvel + source_snapshot.angvel.cross(&com_delta);
                    let angvel = source_snapshot.angvel;
                    target_body.set_linvel(linvel, true);
                    target_body.set_angvel(angvel, true);
                    target_body.set_linear_damping(source_snapshot.linear_damping);
                    target_body.set_angular_damping(source_snapshot.angular_damping);
                    target_body.reset_forces(true);
                    target_body.reset_torques(true);
                    if source_snapshot.user_force.norm_squared() > 0.0 {
                        target_body.add_force(source_snapshot.user_force, true);
                    }
                    if source_snapshot.user_torque.norm_squared() > 0.0 {
                        target_body.add_torque(source_snapshot.user_torque, true);
                    }
                }
                if source_snapshot.sleeping {
                    target_body.sleep();
                } else {
                    target_body.wake_up(true);
                }
            }
        }
    }
}
