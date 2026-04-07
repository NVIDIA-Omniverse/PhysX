/**
 * Collision group filtering for debris bodies.
 *
 * Uses Rapier's 16-bit membership/filter interaction groups to control
 * which bodies collide based on their debris classification.
 *
 * Three groups:
 *  - GROUND: The static ground plane
 *  - SINGLE: Small debris (collider count <= threshold)
 *  - MULTI: Larger structural bodies
 */
import type RAPIER from '@dimforge/rapier3d-compat';
import type { SingleCollisionMode, DebrisCollisionMode } from './types';

const GROUP_GROUND = 1 << 0;
const GROUP_SINGLE = 1 << 1;
const GROUP_MULTI = 1 << 2;

/** Pack membership + filter into a single 32-bit interaction group value. */
function packInteractionGroups(memberships: number, filter: number): number {
  return ((memberships & 0xffff) << 16) | (filter & 0xffff);
}

type BodyWithUserData = RAPIER.RigidBody & { userData?: { projectile?: boolean } };

function setColliderGroups(collider: RAPIER.Collider, groups: number) {
  try {
    collider.setCollisionGroups(groups as Parameters<RAPIER.Collider['setCollisionGroups']>[0]);
    collider.setSolverGroups(groups as Parameters<RAPIER.Collider['setSolverGroups']>[0]);
  } catch {
    // Rapier binding may not support setSolverGroups in all versions
  }
}

export type CollisionGroupContext = {
  mode: SingleCollisionMode | DebrisCollisionMode;
  groundBodyHandle: number;
  maxCollidersForDebris: number;
};

/**
 * Compute and apply collision groups for all colliders on a body.
 *
 * Classification logic:
 *  - Projectiles: full collision (0xffff membership + filter)
 *  - Ground body: collides with everything
 *  - Debris bodies (collider count <= threshold):
 *    - noDebrisPairs/noSinglePairs: debris cannot hit other debris
 *    - debrisGroundOnly/singleGround: debris only hits ground
 *    - debrisNone/singleNone: debris has zero collision
 *  - Multi-collider bodies: collide with ground + other multi + debris
 */
export function applyCollisionGroupsForBody(
  body: RAPIER.RigidBody,
  ctx: CollisionGroupContext,
): void {
  if (!body) return;
  const colliderCount = body?.numColliders?.() ?? 0;
  const userData = (body as BodyWithUserData | undefined)?.userData;
  let memberships = 0xffff;
  let filters = 0xffff;

  if (userData?.projectile) {
    // Projectiles collide with everything
    memberships = 0xffff;
    filters = 0xffff;
  } else if (
    ctx.mode === 'noSinglePairs' || ctx.mode === 'noDebrisPairs' ||
    ctx.mode === 'singleGround' || ctx.mode === 'debrisGroundOnly'
  ) {
    if (body.handle === ctx.groundBodyHandle) {
      memberships = GROUP_GROUND;
      filters = GROUP_GROUND | GROUP_SINGLE | GROUP_MULTI;
    } else {
      const isDynamicLike = body.isDynamic() || body.isKinematic();
      const isDebris = isDynamicLike && colliderCount <= ctx.maxCollidersForDebris;
      if (ctx.mode === 'noSinglePairs' || ctx.mode === 'noDebrisPairs') {
        if (isDebris) {
          memberships = GROUP_SINGLE;
          filters = GROUP_GROUND | GROUP_MULTI;
        } else {
          memberships = GROUP_MULTI;
          filters = GROUP_GROUND | GROUP_MULTI | GROUP_SINGLE;
        }
      } else {
        // singleGround / debrisGroundOnly
        if (isDebris) {
          memberships = GROUP_SINGLE;
          filters = GROUP_GROUND;
        } else {
          memberships = GROUP_MULTI;
          filters = GROUP_GROUND | GROUP_MULTI;
        }
      }
    }
  } else if (ctx.mode === 'singleNone' || ctx.mode === 'debrisNone') {
    const isDynamicLike = body.isDynamic() || body.isKinematic();
    const isDebris = isDynamicLike && colliderCount <= ctx.maxCollidersForDebris;
    if (isDebris) {
      memberships = 0;
      filters = 0;
    } else if (body.handle === ctx.groundBodyHandle) {
      memberships = GROUP_GROUND;
      filters = GROUP_GROUND | GROUP_SINGLE | GROUP_MULTI;
    }
  }
  // mode === 'all' → keep 0xffff defaults

  const groups = packInteractionGroups(memberships, filters);
  for (let i = 0; i < colliderCount; i++) {
    try {
      const col = body.collider(i);
      if (col) setColliderGroups(col, groups);
    } catch {
      // Collider may have been removed between count and access
    }
  }
}
