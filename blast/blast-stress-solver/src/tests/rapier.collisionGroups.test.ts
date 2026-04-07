import { describe, it, expect, beforeAll } from 'vitest';
import RAPIER from '@dimforge/rapier3d-compat';
import { applyCollisionGroupsForBody, type CollisionGroupContext } from '../rapier/collisionGroups';

beforeAll(async () => {
  await RAPIER.init();
});

function createWorld() {
  return new RAPIER.World({ x: 0, y: -9.81, z: 0 });
}

function createDynamicBody(world: RAPIER.World, colliderCount: number) {
  const body = world.createRigidBody(
    RAPIER.RigidBodyDesc.dynamic().setTranslation(0, 1, 0)
  );
  for (let i = 0; i < colliderCount; i++) {
    world.createCollider(
      RAPIER.ColliderDesc.ball(0.1).setTranslation(i * 0.3, 0, 0),
      body
    );
  }
  return body;
}

function createFixedBody(world: RAPIER.World) {
  const body = world.createRigidBody(
    RAPIER.RigidBodyDesc.fixed().setTranslation(0, -1, 0)
  );
  world.createCollider(
    RAPIER.ColliderDesc.cuboid(10, 0.1, 10),
    body
  );
  return body;
}

function getCollisionGroups(body: RAPIER.RigidBody): number {
  const col = body.collider(0);
  return col.collisionGroups();
}

describe('Collision group filtering', () => {
  it('mode "all" leaves default groups (full collision)', () => {
    const world = createWorld();
    const groundBody = createFixedBody(world);
    const body = createDynamicBody(world, 1);

    const ctx: CollisionGroupContext = {
      mode: 'all',
      groundBodyHandle: groundBody.handle,
      maxCollidersForDebris: 2,
    };

    applyCollisionGroupsForBody(body, ctx);
    const groups = getCollisionGroups(body);
    // 'all' mode should set 0xffff for both membership and filter
    expect(groups).toBe(0xffff_ffff);

    world.free();
  });

  it('mode "noDebrisPairs": debris (<=threshold colliders) cannot hit other debris', () => {
    const world = createWorld();
    const groundBody = createFixedBody(world);
    const debris1 = createDynamicBody(world, 1); // 1 <= 2 = debris
    const debris2 = createDynamicBody(world, 1);
    const multi = createDynamicBody(world, 5); // 5 > 2 = multi

    const ctx: CollisionGroupContext = {
      mode: 'noDebrisPairs',
      groundBodyHandle: groundBody.handle,
      maxCollidersForDebris: 2,
    };

    applyCollisionGroupsForBody(debris1, ctx);
    applyCollisionGroupsForBody(debris2, ctx);
    applyCollisionGroupsForBody(multi, ctx);

    const g1 = getCollisionGroups(debris1);
    const g2 = getCollisionGroups(debris2);
    const gMulti = getCollisionGroups(multi);

    // Debris membership should be GROUP_SINGLE (1<<1=2), filter should be GROUND|MULTI
    const debris1Membership = (g1 >>> 16) & 0xffff;
    const debris1Filter = g1 & 0xffff;
    expect(debris1Membership).toBe(1 << 1); // GROUP_SINGLE
    expect(debris1Filter & (1 << 1)).toBe(0); // cannot hit GROUP_SINGLE

    // Multi membership should be GROUP_MULTI (1<<2=4)
    const multiMembership = (gMulti >>> 16) & 0xffff;
    expect(multiMembership).toBe(1 << 2); // GROUP_MULTI

    // Debris can hit ground (1<<0) and multi (1<<2)
    expect(debris1Filter & (1 << 0)).not.toBe(0); // can hit ground
    expect(debris1Filter & (1 << 2)).not.toBe(0); // can hit multi

    world.free();
  });

  it('mode "debrisGroundOnly": debris can only hit ground', () => {
    const world = createWorld();
    const groundBody = createFixedBody(world);
    const debris = createDynamicBody(world, 1);

    const ctx: CollisionGroupContext = {
      mode: 'debrisGroundOnly',
      groundBodyHandle: groundBody.handle,
      maxCollidersForDebris: 2,
    };

    applyCollisionGroupsForBody(debris, ctx);
    const g = getCollisionGroups(debris);
    const membership = (g >>> 16) & 0xffff;
    const filter = g & 0xffff;

    expect(membership).toBe(1 << 1); // GROUP_SINGLE
    expect(filter).toBe(1 << 0); // only GROUP_GROUND

    world.free();
  });

  it('mode "debrisNone": debris has zero collision', () => {
    const world = createWorld();
    const groundBody = createFixedBody(world);
    const debris = createDynamicBody(world, 1);

    const ctx: CollisionGroupContext = {
      mode: 'debrisNone',
      groundBodyHandle: groundBody.handle,
      maxCollidersForDebris: 2,
    };

    applyCollisionGroupsForBody(debris, ctx);
    const g = getCollisionGroups(debris);
    const membership = (g >>> 16) & 0xffff;
    const filter = g & 0xffff;

    expect(membership).toBe(0);
    expect(filter).toBe(0);

    world.free();
  });

  it('ground body gets full collision regardless of mode', () => {
    const world = createWorld();
    const groundBody = createFixedBody(world);

    const ctx: CollisionGroupContext = {
      mode: 'noDebrisPairs',
      groundBodyHandle: groundBody.handle,
      maxCollidersForDebris: 2,
    };

    applyCollisionGroupsForBody(groundBody, ctx);
    const g = getCollisionGroups(groundBody);
    const membership = (g >>> 16) & 0xffff;
    const filter = g & 0xffff;

    expect(membership).toBe(1 << 0); // GROUP_GROUND
    // Filter includes all groups
    expect(filter & (1 << 0)).not.toBe(0); // ground
    expect(filter & (1 << 1)).not.toBe(0); // single
    expect(filter & (1 << 2)).not.toBe(0); // multi

    world.free();
  });

  it('projectile body keeps full collision in any mode', () => {
    const world = createWorld();
    const groundBody = createFixedBody(world);
    const proj = world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(0, 2, 0)
        .setUserData({ projectile: true })
    );
    world.createCollider(RAPIER.ColliderDesc.ball(0.15), proj);

    const ctx: CollisionGroupContext = {
      mode: 'debrisNone',
      groundBodyHandle: groundBody.handle,
      maxCollidersForDebris: 2,
    };

    applyCollisionGroupsForBody(proj, ctx);
    const g = getCollisionGroups(proj);
    expect(g).toBe(0xffff_ffff);

    world.free();
  });

  it('maxCollidersForDebris threshold correctly classifies bodies', () => {
    const world = createWorld();
    const groundBody = createFixedBody(world);
    const body2 = createDynamicBody(world, 2); // 2 <= 2 = debris
    const body3 = createDynamicBody(world, 3); // 3 > 2 = multi

    const ctx: CollisionGroupContext = {
      mode: 'noDebrisPairs',
      groundBodyHandle: groundBody.handle,
      maxCollidersForDebris: 2,
    };

    applyCollisionGroupsForBody(body2, ctx);
    applyCollisionGroupsForBody(body3, ctx);

    const g2 = getCollisionGroups(body2);
    const g3 = getCollisionGroups(body3);

    const m2 = (g2 >>> 16) & 0xffff;
    const m3 = (g3 >>> 16) & 0xffff;

    expect(m2).toBe(1 << 1); // debris (GROUP_SINGLE)
    expect(m3).toBe(1 << 2); // multi (GROUP_MULTI)

    world.free();
  });
});
