//! Tests for split migration planning and fracture policy.

#![cfg(feature = "rapier")]

use std::collections::HashSet;
use rapier3d::prelude::*;

use blast_stress_solver::rapier::*;
use blast_stress_solver::types::SplitChild;

fn make_body_set_with_n(n: usize) -> (RigidBodySet, Vec<RigidBodyHandle>) {
    let mut bodies = RigidBodySet::new();
    let mut handles = Vec::new();
    for _ in 0..n {
        let h = bodies.insert(RigidBodyBuilder::dynamic());
        handles.push(h);
    }
    (bodies, handles)
}

// ============================================================================
// Split migration tests
// ============================================================================

#[test]
fn reuses_exact_body_match() {
    let (_, handles) = make_body_set_with_n(2);

    let bodies = vec![
        ExistingBodyState {
            handle: handles[0],
            node_indices: [0, 1, 2].into_iter().collect(),
            is_fixed: false,
        },
        ExistingBodyState {
            handle: handles[1],
            node_indices: [3, 4].into_iter().collect(),
            is_fixed: false,
        },
    ];

    let children = vec![
        SplitChild { actor_index: 10, nodes: vec![0, 1, 2] },
        SplitChild { actor_index: 11, nodes: vec![3, 4] },
    ];

    let plan = plan_split_migration(&bodies, &children);
    assert_eq!(plan.reuse.len(), 2, "both children should reuse");
    assert_eq!(plan.create.len(), 0);
}

#[test]
fn creates_unmatched_children() {
    let (_, handles) = make_body_set_with_n(1);

    let bodies = vec![ExistingBodyState {
        handle: handles[0],
        node_indices: [0, 1].into_iter().collect(),
        is_fixed: false,
    }];

    // Children have completely different node sets
    let children = vec![
        SplitChild { actor_index: 10, nodes: vec![5, 6] },
        SplitChild { actor_index: 11, nodes: vec![7, 8] },
    ];

    let plan = plan_split_migration(&bodies, &children);
    assert_eq!(plan.create.len(), 2, "both should be created");
}

#[test]
fn no_double_assignment() {
    let (_, handles) = make_body_set_with_n(1);

    let bodies = vec![ExistingBodyState {
        handle: handles[0],
        node_indices: [0, 1].into_iter().collect(),
        is_fixed: false,
    }];

    // Two children with the same node set — only one should reuse
    let children = vec![
        SplitChild { actor_index: 10, nodes: vec![0, 1] },
        SplitChild { actor_index: 11, nodes: vec![0, 1] },
    ];

    let plan = plan_split_migration(&bodies, &children);
    assert_eq!(plan.reuse.len(), 1, "only one reuse");
    assert_eq!(plan.create.len(), 1, "second must create");
}

#[test]
fn empty_bodies_all_create() {
    let children = vec![
        SplitChild { actor_index: 10, nodes: vec![0, 1] },
        SplitChild { actor_index: 11, nodes: vec![2, 3] },
    ];

    let plan = plan_split_migration(&[], &children);
    assert_eq!(plan.reuse.len(), 0);
    assert_eq!(plan.create.len(), 2);
}

#[test]
fn empty_children_empty_plan() {
    let (_, handles) = make_body_set_with_n(1);
    let bodies = vec![ExistingBodyState {
        handle: handles[0],
        node_indices: [0, 1].into_iter().collect(),
        is_fixed: false,
    }];

    let plan = plan_split_migration(&bodies, &[]);
    assert_eq!(plan.reuse.len(), 0);
    assert_eq!(plan.create.len(), 0);
}

#[test]
fn partial_overlap_creates_new() {
    let (_, handles) = make_body_set_with_n(1);

    let bodies = vec![ExistingBodyState {
        handle: handles[0],
        node_indices: [0, 1, 2].into_iter().collect(),
        is_fixed: false,
    }];

    // Child has only a subset — not an exact match
    let children = vec![SplitChild {
        actor_index: 10,
        nodes: vec![0, 1],
    }];

    let plan = plan_split_migration(&bodies, &children);
    // Subset doesn't match — should create (hash won't match)
    assert_eq!(plan.create.len(), 1);
}

// ============================================================================
// Fracture policy tests
// ============================================================================

#[test]
fn policy_default_unlimited() {
    let policy = FracturePolicy::default();
    assert!(!policy.should_suppress(100));
    assert_eq!(policy.clamp_fractures(1000), 1000);
    assert_eq!(policy.clamp_new_bodies(1000), 1000);
    assert!(policy.child_qualifies(1));
}

#[test]
fn policy_suppress_at_body_limit() {
    let policy = FracturePolicy {
        max_dynamic_bodies: 10,
        ..FracturePolicy::default()
    };
    assert!(!policy.should_suppress(5));
    assert!(!policy.should_suppress(9));
    assert!(policy.should_suppress(10));
    assert!(policy.should_suppress(20));
}

#[test]
fn policy_clamp_fractures_per_frame() {
    let policy = FracturePolicy {
        max_fractures_per_frame: 3,
        ..FracturePolicy::default()
    };
    assert_eq!(policy.clamp_fractures(1), 1);
    assert_eq!(policy.clamp_fractures(3), 3);
    assert_eq!(policy.clamp_fractures(10), 3);
}

#[test]
fn policy_clamp_new_bodies_per_frame() {
    let policy = FracturePolicy {
        max_new_bodies_per_frame: 2,
        ..FracturePolicy::default()
    };
    assert_eq!(policy.clamp_new_bodies(1), 1);
    assert_eq!(policy.clamp_new_bodies(2), 2);
    assert_eq!(policy.clamp_new_bodies(10), 2);
}

#[test]
fn policy_min_child_node_count() {
    let policy = FracturePolicy {
        min_child_node_count: 3,
        ..FracturePolicy::default()
    };
    assert!(!policy.child_qualifies(1));
    assert!(!policy.child_qualifies(2));
    assert!(policy.child_qualifies(3));
    assert!(policy.child_qualifies(10));
}

#[test]
fn policy_idle_skip_default_true() {
    let policy = FracturePolicy::default();
    assert!(policy.idle_skip);
}
