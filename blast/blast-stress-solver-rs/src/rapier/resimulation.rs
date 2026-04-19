use rapier3d::prelude::*;

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
    linvel: Vector<Real>,
    angvel: AngVector<Real>,
    linear_damping: Real,
    angular_damping: Real,
    user_force: Vector<Real>,
    user_torque: AngVector<Real>,
    sleeping: bool,
    enabled: bool,
}

#[derive(Clone, Debug, Default)]
pub struct BodySnapshots {
    bodies: Vec<BodySnapshot>,
}

impl BodySnapshots {
    pub fn capture(set: &RigidBodySet) -> Self {
        let bodies = set
            .iter()
            .map(|(handle, body)| BodySnapshot {
                handle,
                position: *body.position(),
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
        Self { bodies }
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

    pub fn restore_split_children(&self, set: &mut RigidBodySet, split_cohorts: &[SplitCohort]) {
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
                let source_motion = set.get(source_handle).map(|source_body| {
                    (
                        *source_body.center_of_mass(),
                        *source_body.linvel(),
                        *source_body.angvel(),
                    )
                });
                let Some(target_body) = set.get_mut(target_handle) else {
                    continue;
                };
                target_body.set_enabled(source_snapshot.enabled);
                target_body.set_position(source_snapshot.position, true);
                if !target_body.is_fixed() {
                    let target_com = *target_body.center_of_mass();
                    let (linvel, angvel) = source_motion
                        .map(|(source_com, source_linvel, source_angvel)| {
                            let com_delta = target_com - source_com;
                            (
                                source_linvel + source_angvel.cross(&com_delta),
                                source_angvel,
                            )
                        })
                        .unwrap_or((source_snapshot.linvel, source_snapshot.angvel));
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
