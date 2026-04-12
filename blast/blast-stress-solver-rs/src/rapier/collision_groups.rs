use rapier3d::prelude::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum DebrisCollisionMode {
    #[default]
    All,
    NoDebrisPairs,
    DebrisGroundOnly,
    DebrisNone,
}

impl DebrisCollisionMode {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::All => "all",
            Self::NoDebrisPairs => "noDebrisPairs",
            Self::DebrisGroundOnly => "debrisGroundOnly",
            Self::DebrisNone => "debrisNone",
        }
    }
}

const GROUP_GROUND: Group = Group::GROUP_1;
const GROUP_DEBRIS: Group = Group::GROUP_2;
const GROUP_MULTI: Group = Group::GROUP_3;

pub fn apply_collision_groups_for_body(
    body_handle: RigidBodyHandle,
    bodies: &RigidBodySet,
    colliders: &mut ColliderSet,
    ground_body_handle: Option<RigidBodyHandle>,
    mode: DebrisCollisionMode,
    debris_collision_active: bool,
    max_colliders_for_debris: usize,
) {
    let Some(body) = bodies.get(body_handle) else {
        return;
    };

    let groups = interaction_groups_for_body(
        body_handle,
        body,
        ground_body_handle,
        mode,
        debris_collision_active,
        max_colliders_for_debris,
    );

    for &collider_handle in body.colliders() {
        if let Some(collider) = colliders.get_mut(collider_handle) {
            collider.set_collision_groups(groups);
            collider.set_solver_groups(groups);
        }
    }
}

fn interaction_groups_for_body(
    body_handle: RigidBodyHandle,
    body: &RigidBody,
    ground_body_handle: Option<RigidBodyHandle>,
    mode: DebrisCollisionMode,
    debris_collision_active: bool,
    max_colliders_for_debris: usize,
) -> InteractionGroups {
    if mode == DebrisCollisionMode::All {
        return InteractionGroups::all();
    }

    if Some(body_handle) == ground_body_handle {
        return InteractionGroups::new(GROUP_GROUND, GROUP_GROUND | GROUP_DEBRIS | GROUP_MULTI);
    }

    let is_dynamic_like = body.is_dynamic() || body.is_kinematic();
    let is_debris = debris_collision_active
        && is_dynamic_like
        && body.colliders().len() <= max_colliders_for_debris;

    match mode {
        DebrisCollisionMode::All => InteractionGroups::all(),
        DebrisCollisionMode::NoDebrisPairs => {
            if is_debris {
                InteractionGroups::new(GROUP_DEBRIS, GROUP_GROUND | GROUP_MULTI)
            } else {
                InteractionGroups::new(GROUP_MULTI, GROUP_GROUND | GROUP_MULTI | GROUP_DEBRIS)
            }
        }
        DebrisCollisionMode::DebrisGroundOnly => {
            if is_debris {
                InteractionGroups::new(GROUP_DEBRIS, GROUP_GROUND)
            } else {
                InteractionGroups::new(GROUP_MULTI, GROUP_GROUND | GROUP_MULTI)
            }
        }
        DebrisCollisionMode::DebrisNone => {
            if is_debris {
                InteractionGroups::none()
            } else {
                InteractionGroups::new(GROUP_MULTI, GROUP_GROUND | GROUP_MULTI | GROUP_DEBRIS)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_body(
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        body: RigidBodyBuilder,
        collider_count: usize,
    ) -> RigidBodyHandle {
        let handle = bodies.insert(body);
        for _ in 0..collider_count {
            colliders.insert_with_parent(ColliderBuilder::ball(0.5), handle, bodies);
        }
        handle
    }

    #[test]
    fn no_debris_pairs_filters_debris_but_not_multi() {
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let ground = create_body(&mut bodies, &mut colliders, RigidBodyBuilder::fixed(), 1);
        let debris = create_body(&mut bodies, &mut colliders, RigidBodyBuilder::dynamic(), 1);
        let multi = create_body(&mut bodies, &mut colliders, RigidBodyBuilder::dynamic(), 3);

        apply_collision_groups_for_body(
            ground,
            &bodies,
            &mut colliders,
            Some(ground),
            DebrisCollisionMode::NoDebrisPairs,
            true,
            2,
        );
        apply_collision_groups_for_body(
            debris,
            &bodies,
            &mut colliders,
            Some(ground),
            DebrisCollisionMode::NoDebrisPairs,
            true,
            2,
        );
        apply_collision_groups_for_body(
            multi,
            &bodies,
            &mut colliders,
            Some(ground),
            DebrisCollisionMode::NoDebrisPairs,
            true,
            2,
        );

        let debris_groups = colliders
            .get(bodies[debris].colliders()[0])
            .unwrap()
            .collision_groups();
        let multi_groups = colliders
            .get(bodies[multi].colliders()[0])
            .unwrap()
            .collision_groups();

        assert_eq!(debris_groups.memberships, GROUP_DEBRIS);
        assert_eq!(debris_groups.filter, GROUP_GROUND | GROUP_MULTI);
        assert_eq!(multi_groups.memberships, GROUP_MULTI);
        assert_eq!(
            multi_groups.filter,
            GROUP_GROUND | GROUP_MULTI | GROUP_DEBRIS
        );
    }

    #[test]
    fn debris_ground_only_only_keeps_ground_for_debris() {
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let ground = create_body(&mut bodies, &mut colliders, RigidBodyBuilder::fixed(), 1);
        let debris = create_body(&mut bodies, &mut colliders, RigidBodyBuilder::dynamic(), 1);

        apply_collision_groups_for_body(
            debris,
            &bodies,
            &mut colliders,
            Some(ground),
            DebrisCollisionMode::DebrisGroundOnly,
            true,
            2,
        );

        let groups = colliders
            .get(bodies[debris].colliders()[0])
            .unwrap()
            .collision_groups();
        assert_eq!(groups.memberships, GROUP_DEBRIS);
        assert_eq!(groups.filter, GROUP_GROUND);
    }

    #[test]
    fn debris_none_disables_debris_collisions() {
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let debris = create_body(&mut bodies, &mut colliders, RigidBodyBuilder::dynamic(), 1);

        apply_collision_groups_for_body(
            debris,
            &bodies,
            &mut colliders,
            None,
            DebrisCollisionMode::DebrisNone,
            true,
            2,
        );

        let groups = colliders
            .get(bodies[debris].colliders()[0])
            .unwrap()
            .collision_groups();
        assert_eq!(groups, InteractionGroups::none());
    }
}
