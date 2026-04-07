/**
 * Contact force processing helpers.
 *
 * Provides speed-scaled damage, relative velocity computation,
 * world-to-local coordinate transforms, and projectile momentum boost.
 * Extracted from the monolithic destructible-core for testability.
 */
import type RAPIER from '@dimforge/rapier3d-compat';
import type { Vec3 } from './types';

// ─── Quaternion math (no Three.js dependency) ───────────────────────

type Quat = { x: number; y: number; z: number; w: number };

/** Rotate a vector by a quaternion. Pure function, no mutation. */
export function applyQuatToVec3(v: Vec3, q: Quat): Vec3 {
  const { x, y, z } = v;
  const { x: qx, y: qy, z: qz, w: qw } = q;
  const ix = qw * x + qy * z - qz * y;
  const iy = qw * y + qz * x - qx * z;
  const iz = qw * z + qx * y - qy * x;
  const iw = -qx * x - qy * y - qz * z;
  return {
    x: ix * qw + iw * -qx + iy * -qz - iz * -qy,
    y: iy * qw + iw * -qy + iz * -qx - ix * -qz,
    z: iz * qw + iw * -qz + ix * -qy - iy * -qx,
  };
}

// ─── Speed-scaled damage ────────────────────────────────────────────

export type SpeedScalingOptions = {
  speedMinExternal: number;
  speedMinInternal: number;
  speedMax: number;
  speedExponent: number;
  slowSpeedFactor: number;
  fastSpeedFactor: number;
};

/**
 * Compute a damage multiplier based on relative impact speed.
 *
 * Returns `slowSpeedFactor` when speed <= vMin (suppresses resting contacts),
 * smoothly interpolates to `fastSpeedFactor` at speed >= vMax.
 * The interpolation curve is controlled by `speedExponent`.
 */
export function computeSpeedFactor(
  relSpeed: number,
  isInternal: boolean,
  opts: SpeedScalingOptions,
): number {
  try {
    const vMin = isInternal ? opts.speedMinInternal : opts.speedMinExternal;
    const vMax = opts.speedMax;
    const exp = Math.max(0.01, opts.speedExponent);
    const slow = Math.max(0, Math.min(1, opts.slowSpeedFactor));
    const fast = Math.max(1, opts.fastSpeedFactor);
    const vSpan = Math.max(1e-3, vMax - vMin);
    const t = relSpeed > vMin
      ? Math.min(1, Math.pow((relSpeed - vMin) / vSpan, exp))
      : 0;
    return slow + (fast - slow) * t;
  } catch {
    return 1.0;
  }
}

// ─── Velocity helpers ───────────────────────────────────────────────

/** Compute point velocity on a rigid body (linear + angular cross product). */
export function bodyPointVelocity(
  body: RAPIER.RigidBody | null,
  point: Vec3,
): Vec3 {
  if (!body) return { x: 0, y: 0, z: 0 };
  try {
    const lv = (body as any).linvel?.();
    const av = (body as any).angvel?.();
    const t = body.translation();
    const rx = (point.x ?? 0) - (t.x ?? 0);
    const ry = (point.y ?? 0) - (t.y ?? 0);
    const rz = (point.z ?? 0) - (t.z ?? 0);
    // angular velocity × relative position = rotational contribution
    const cx = (av?.y ?? 0) * rz - (av?.z ?? 0) * ry;
    const cy = (av?.z ?? 0) * rx - (av?.x ?? 0) * rz;
    const cz = (av?.x ?? 0) * ry - (av?.y ?? 0) * rx;
    return {
      x: (lv?.x ?? 0) + cx,
      y: (lv?.y ?? 0) + cy,
      z: (lv?.z ?? 0) + cz,
    };
  } catch {
    return { x: 0, y: 0, z: 0 };
  }
}

/**
 * Get the rigid body that owns a given collider handle.
 * Returns null if the handle is invalid or the collider has no parent.
 */
export function getBodyForColliderHandle(
  world: RAPIER.World,
  handle: number | null | undefined,
): RAPIER.RigidBody | null {
  if (handle == null) return null;
  try {
    const c = world.getCollider(handle);
    const parent = c ? (c as any).parent() : null;
    return parent ?? null;
  } catch {
    return null;
  }
}

/**
 * Compute the relative speed between two colliders at a given world point.
 * Used to modulate damage based on impact velocity.
 */
export function computeRelativeSpeed(
  world: RAPIER.World,
  h1: number | null | undefined,
  h2: number | null | undefined,
  atPoint: Vec3,
): number {
  const b1 = getBodyForColliderHandle(world, h1);
  const b2 = getBodyForColliderHandle(world, h2);
  const v1 = bodyPointVelocity(b1, atPoint);
  const v2 = bodyPointVelocity(b2, atPoint);
  return Math.hypot(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

// ─── Coordinate transforms ──────────────────────────────────────────

/**
 * Transform a world-space point into the local space of a rigid body.
 * Returns null if the body cannot be resolved.
 */
export function worldPointToBodyLocal(
  body: RAPIER.RigidBody,
  worldPoint: Vec3,
): Vec3 {
  const t = body.translation();
  const r = body.rotation();
  const qInv: Quat = { x: -r.x, y: -r.y, z: -r.z, w: r.w };
  const pRel: Vec3 = {
    x: (worldPoint.x ?? 0) - (t.x ?? 0),
    y: (worldPoint.y ?? 0) - (t.y ?? 0),
    z: (worldPoint.z ?? 0) - (t.z ?? 0),
  };
  return applyQuatToVec3(pRel, qInv);
}

/**
 * Compute the world-space center of a chunk given its body transform
 * and baseLocalOffset.
 */
export function chunkWorldCenter(
  body: RAPIER.RigidBody,
  baseLocalOffset: Vec3,
): Vec3 {
  const t = body.translation();
  const r = body.rotation();
  const rotated = applyQuatToVec3(baseLocalOffset, { x: r.x, y: r.y, z: r.z, w: r.w });
  return {
    x: (t.x ?? 0) + rotated.x,
    y: (t.y ?? 0) + rotated.y,
    z: (t.z ?? 0) + rotated.z,
  };
}

/**
 * Get the world position of a collider's parent body as a fallback contact point.
 */
export function fallbackContactPoint(
  world: RAPIER.World,
  handle: number | undefined,
): Vec3 {
  if (handle == null) return { x: 0, y: 0, z: 0 };
  const body = getBodyForColliderHandle(world, handle);
  if (body) {
    const t = body.translation();
    return { x: t.x ?? 0, y: t.y ?? 0, z: t.z ?? 0 };
  }
  return { x: 0, y: 0, z: 0 };
}

// ─── Projectile momentum boost ──────────────────────────────────────

type MassReadableBody = RAPIER.RigidBody & { mass?: () => number };

/**
 * If one of the two bodies involved in a contact is a projectile,
 * compute an alternative effective magnitude from momentum (mass × speed).
 * Returns the larger of the original magnitude and the momentum-derived force,
 * ensuring fast-moving projectiles produce appropriately strong impacts.
 */
export function applyProjectileMomentumBoost(
  body1: RAPIER.RigidBody | null,
  body2: RAPIER.RigidBody | null,
  relSpeed: number,
  dt: number,
  currentEffMag: number,
): number {
  const ud1 = (body1 as any)?.userData;
  const ud2 = (body2 as any)?.userData;
  const projBody = ud1?.projectile ? body1 : (ud2?.projectile ? body2 : null);
  if (!projBody) return currentEffMag;

  let mass = 1;
  try {
    const reader = projBody as MassReadableBody;
    if (typeof reader.mass === 'function') {
      mass = reader.mass();
    }
  } catch { /* fallback to mass=1 */ }

  const impulseEstimate = Math.max(0, mass) * Math.max(0, relSpeed);
  const forceFromMomentum = impulseEstimate / Math.max(1e-6, dt);
  if (Number.isFinite(forceFromMomentum) && forceFromMomentum > 0) {
    return Math.max(currentEffMag, forceFromMomentum);
  }
  return currentEffMag;
}
