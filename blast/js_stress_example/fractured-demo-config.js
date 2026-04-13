export const FRACTURED_WALL_DEMO_CONFIG = {
  key: 'fractured_wall',
  title: 'Fractured Wall',
  camera: {
    target: { x: 0, y: 1.5, z: 0 },
    distance: 12.1,
  },
  wall: {
    span: 6.0,
    height: 3.0,
    thickness: 0.32,
    fragmentCount: 80,
    deckMass: 10_000,
  },
  projectile: {
    radius: 0.35,
    mass: 1_000,
    speed: 20,
    ttlMs: 6_000,
  },
  solver: {
    gravity: -9.81,
    materialScale: 1e10,
  },
  physics: {
    debrisCollisionMode: 'all',
    friction: 0.25,
    restitution: 0.0,
    contactForceScale: 30,
    skipSingleBodies: false,
  },
  optimization: {
    smallBodyDampingMode: 'always',
    debrisCleanupMode: 'always',
    debrisTtlMs: 10_000,
    maxCollidersForDebris: 2,
  },
};

export const FRACTURED_TOWER_DEMO_CONFIG = {
  key: 'fractured_tower',
  title: 'Fractured Tower',
  camera: {
    target: { x: 0, y: 6, z: 0 },
    distance: 30.3,
  },
  tower: {
    width: 8,
    floorCount: 4,
    floorHeight: 3,
    fragmentCountPerWall: 6,
    fragmentCountPerFloor: 6,
    fragmentCountPerColumn: 3,
    deckMass: 20_000,
    thickness: 0.3,
    floorThickness: 0.2,
    columnSize: 0.6,
    columnsX: 2,
    columnsZ: 2,
  },
  projectile: {
    radius: 0.5,
    mass: 1_000,
    speed: 25,
    ttlMs: 8_000,
  },
  solver: {
    gravity: -9.81,
    materialScale: 1e10,
  },
  physics: {
    debrisCollisionMode: 'noDebrisPairs',
    friction: 0.25,
    restitution: 0.0,
    contactForceScale: 30,
  },
  optimization: {
    smallBodyDampingMode: 'always',
    debrisCleanupMode: 'always',
    debrisTtlMs: 10_000,
    maxCollidersForDebris: 2,
  },
};

export const FRACTURED_BRIDGE_DEMO_CONFIG = {
  key: 'fractured_bridge',
  title: 'Fractured Bridge',
  camera: {
    target: { x: 0, y: 2.5, z: 0 },
    distance: 18.4,
  },
  bridge: {
    span: 12,
    deckWidth: 4,
    deckThickness: 0.5,
    pierHeight: 2.8,
    supportsPerSide: 4,
    postSize: 0.4,
    fragmentCountPerDeck: 25,
    fragmentCountPerPost: 4,
    deckMass: 30_000,
  },
  projectile: {
    radius: 0.4,
    mass: 1_000,
    speed: 20,
    ttlMs: 6_000,
  },
  solver: {
    gravity: -9.81,
    materialScale: 1e10,
  },
  physics: {
    debrisCollisionMode: 'all',
    friction: 0.25,
    restitution: 0.0,
    contactForceScale: 30,
  },
  optimization: {
    smallBodyDampingMode: 'always',
    debrisCleanupMode: 'always',
    debrisTtlMs: 10_000,
    maxCollidersForDebris: 2,
  },
};

export const FRACTURED_DEMO_CONFIGS = {
  wall: FRACTURED_WALL_DEMO_CONFIG,
  tower: FRACTURED_TOWER_DEMO_CONFIG,
  bridge: FRACTURED_BRIDGE_DEMO_CONFIG,
};
