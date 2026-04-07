import * as THREE from 'three';
import type RAPIER from '@dimforge/rapier3d-compat';
import type { ChunkData, DestructibleCore } from '../rapier/types';

const shadowsEnabled = true;

const HEALTHY_COLOR = new THREE.Color(0x2fbf71);
const CRITICAL_COLOR = new THREE.Color(0xd72638);

const KINEMATIC_COLOR = 0x2a6ddb;
const FIXED_COLOR = 0xbababa;
const DYNAMIC_COLOR = 0xff9147;

type ChunkLike = Pick<ChunkData, 'nodeIndex'>;
type RigidBodyLike = Pick<RAPIER.RigidBody, 'isKinematic' | 'isFixed' | 'isDynamic'>;

export type ChunkMeshBuildResult = {
  objects: THREE.Mesh[];
  dispose: () => void;
};

export type ChunkMeshBuildOptions = {
  /** Clone provided geometries before attaching them to meshes. Default: false. */
  cloneGeometries?: boolean;
  /** Remove geometry groups on prepared geometries. Default: true. */
  clearGroups?: boolean;
  /** Dispose external source geometries on `dispose()`. Default: false. */
  disposeSourceGeometries?: boolean;
};

export type BatchedChunkMeshOptions = ChunkMeshBuildOptions & {
  /**
   * Retained for backward compatibility with Vibe City. Standard THREE.BatchedMesh
   * does not support per-instance shader uniforms here, so this flag is ignored.
   */
  enablePerInstanceUniforms?: boolean;
  /** Enable optional prototype-extended BVH helpers if present. Default: true. */
  enableBVH?: boolean;
  /** Margin forwarded to optional `computeBVH` helper. Default: 0.5. */
  bvhMargin?: number;
};

type OptionalBvhApi = {
  computeBVH?: (coordinateSystem?: number, options?: { margin?: number }) => void;
  bvh?: {
    move: (instanceId: number) => void;
  };
};

type ColorWritableBatchedMesh = THREE.BatchedMesh & OptionalBvhApi & {
  setColorAt?: (instanceId: number, color: THREE.Color) => THREE.BatchedMesh;
};

export type BatchedChunkMeshResult = {
  batchedMesh: ColorWritableBatchedMesh;
  chunkToInstanceId: Map<number, number>;
  geometryIds: number[];
  dispose: () => void;
};

const PROJECTILE_MAX_LIFETIME = 12;
const PROJECTILE_MIN_Y = -50;
const nowSeconds = () =>
  (typeof performance !== 'undefined' ? performance.now() : Date.now()) / 1000;

const _matrix = new THREE.Matrix4();
const _position = new THREE.Vector3();
const _quaternion = new THREE.Quaternion();
const _scale = new THREE.Vector3(1, 1, 1);
const _localOffset = new THREE.Vector3();
const _colorTmp = new THREE.Color();
const _batchedDefaultColor = new THREE.Color(0x00ff00);

function toMeshStandardMaterial(material: THREE.Material | THREE.Material[] | undefined): THREE.MeshStandardMaterial | null {
  return material instanceof THREE.MeshStandardMaterial ? material : null;
}

function prepareGeometry(
  geometry: THREE.BufferGeometry,
  options: ChunkMeshBuildOptions | undefined,
  ownedGeometries: Set<THREE.BufferGeometry>,
): THREE.BufferGeometry {
  let prepared = geometry;
  if (options?.cloneGeometries) {
    prepared = geometry.clone();
    ownedGeometries.add(prepared);
  } else if (options?.disposeSourceGeometries) {
    ownedGeometries.add(prepared);
  }

  if (options?.clearGroups ?? true) {
    try {
      prepared.clearGroups();
    } catch {}
  }

  return prepared;
}

function disposeOwnedGeometries(ownedGeometries: Set<THREE.BufferGeometry>) {
  for (const geometry of ownedGeometries) {
    try {
      geometry.dispose();
    } catch {}
  }
  ownedGeometries.clear();
}

function disposeMeshMaterials(meshes: THREE.Mesh[]) {
  for (const mesh of meshes) {
    const material = mesh.material;
    if (Array.isArray(material)) {
      for (const entry of material) {
        try {
          entry.dispose();
        } catch {}
      }
    } else {
      try {
        material?.dispose?.();
      } catch {}
    }
  }
}

function getBatchedMeshCount(geometries: THREE.BufferGeometry[]) {
  let vertexCount = 0;
  let indexCount = 0;

  for (const geometry of geometries) {
    const position = geometry.getAttribute('position');
    if (!position) continue;
    vertexCount += position.count;
    indexCount += geometry.index?.count ?? position.count;
  }

  return { vertexCount, indexCount };
}

function makeDefaultMaterials(materials?: { deck?: THREE.Material; support?: THREE.Material }) {
  return {
    deck:
      materials?.deck ??
      new THREE.MeshStandardMaterial({
        color: 0xbababa,
        roughness: 0.62,
        metalness: 0.05,
      }),
    support:
      materials?.support ??
      new THREE.MeshStandardMaterial({
        color: 0x7a889a,
        roughness: 0.7,
        metalness: 0.15,
      }),
  };
}

/**
 * Apply color based on damage health or rigid body type.
 * Mutates and returns the provided color instance.
 */
export function applyChunkColor(opts: {
  core: DestructibleCore;
  chunk: ChunkLike;
  body: RigidBodyLike;
  color: THREE.Color;
}): THREE.Color {
  const { core, chunk, body, color } = opts;

  const damageEnabled = core.damageEnabled === true;
  const healthGetter = core.getNodeHealth;
  if (damageEnabled && typeof healthGetter === 'function') {
    const info = healthGetter(chunk.nodeIndex);
    if (info && info.maxHealth > 0) {
      const ratio = Math.max(0, Math.min(1, info.health / info.maxHealth));
      color.copy(HEALTHY_COLOR).lerp(CRITICAL_COLOR, 1 - ratio);
      return color;
    }
  }

  if (body.isKinematic()) color.setHex(KINEMATIC_COLOR);
  else if (body.isFixed()) color.setHex(FIXED_COLOR);
  else if (body.isDynamic()) color.setHex(DYNAMIC_COLOR);

  return color;
}

export function buildChunkMeshes(
  core: DestructibleCore,
  materials?: { deck?: THREE.Material; support?: THREE.Material },
): ChunkMeshBuildResult {
  const mats = makeDefaultMaterials(materials);
  const meshes: THREE.Mesh[] = [];
  const ownedGeometries = new Set<THREE.BufferGeometry>();

  for (const chunk of core.chunks) {
    const material = (chunk.isSupport ? mats.support : mats.deck).clone();
    const geometry = new THREE.BoxGeometry(chunk.size.x, chunk.size.y, chunk.size.z);
    ownedGeometries.add(geometry);
    const mesh = new THREE.Mesh(geometry, material);
    mesh.userData.nodeIndex = chunk.nodeIndex;
    mesh.castShadow = shadowsEnabled;
    mesh.receiveShadow = shadowsEnabled;
    meshes.push(mesh);
  }

  return {
    objects: meshes,
    dispose: () => {
      disposeMeshMaterials(meshes);
      disposeOwnedGeometries(ownedGeometries);
    },
  };
}

export function buildChunkMeshesFromGeometries(
  core: DestructibleCore,
  geometries: THREE.BufferGeometry[],
  materials?: { deck?: THREE.Material; support?: THREE.Material },
  options?: ChunkMeshBuildOptions,
): ChunkMeshBuildResult {
  const mats = makeDefaultMaterials(materials);
  const meshes: THREE.Mesh[] = [];
  const ownedGeometries = new Set<THREE.BufferGeometry>();

  for (let i = 0; i < core.chunks.length; i += 1) {
    const chunk = core.chunks[i];
    const geometry = chunk.isSupport
      ? new THREE.BoxGeometry(chunk.size.x, chunk.size.y, chunk.size.z)
      : prepareGeometry(
          geometries[i] ?? new THREE.BoxGeometry(chunk.size.x, chunk.size.y, chunk.size.z),
          options,
          ownedGeometries,
        );

    if (chunk.isSupport || !geometries[i]) {
      ownedGeometries.add(geometry);
      if (options?.clearGroups ?? true) {
        try {
          geometry.clearGroups();
        } catch {}
      }
    }

    const material = (chunk.isSupport ? mats.support : mats.deck).clone();
    const mesh = new THREE.Mesh(geometry, material);
    mesh.userData.nodeIndex = chunk.nodeIndex;
    mesh.castShadow = shadowsEnabled;
    mesh.receiveShadow = shadowsEnabled;
    meshes.push(mesh);
  }

  return {
    objects: meshes,
    dispose: () => {
      disposeMeshMaterials(meshes);
      disposeOwnedGeometries(ownedGeometries);
    },
  };
}

export function buildBatchedChunkMesh(
  core: DestructibleCore,
  options?: BatchedChunkMeshOptions,
): BatchedChunkMeshResult {
  const geometries: THREE.BufferGeometry[] = [];
  const ownedGeometries = new Set<THREE.BufferGeometry>();

  for (const chunk of core.chunks) {
    const geometry = new THREE.BoxGeometry(chunk.size.x, chunk.size.y, chunk.size.z);
    ownedGeometries.add(geometry);
    if (options?.clearGroups ?? true) {
      try {
        geometry.clearGroups();
      } catch {}
    }
    geometries.push(geometry);
  }

  return buildBatchedChunkMeshInternal(core, geometries, ownedGeometries, options);
}

export function buildBatchedChunkMeshFromGeometries(
  core: DestructibleCore,
  geometries: THREE.BufferGeometry[],
  options?: BatchedChunkMeshOptions,
): BatchedChunkMeshResult {
  const finalGeometries: THREE.BufferGeometry[] = [];
  const ownedGeometries = new Set<THREE.BufferGeometry>();

  for (let i = 0; i < core.chunks.length; i += 1) {
    const chunk = core.chunks[i];
    if (chunk.isSupport) {
      const supportGeometry = new THREE.BoxGeometry(chunk.size.x, chunk.size.y, chunk.size.z);
      ownedGeometries.add(supportGeometry);
      if (options?.clearGroups ?? true) {
        try {
          supportGeometry.clearGroups();
        } catch {}
      }
      finalGeometries.push(supportGeometry);
      continue;
    }

    const sourceGeometry = geometries[i] ?? new THREE.BoxGeometry(chunk.size.x, chunk.size.y, chunk.size.z);
    if (!geometries[i]) {
      ownedGeometries.add(sourceGeometry);
    }
    finalGeometries.push(prepareGeometry(sourceGeometry, options, ownedGeometries));
  }

  return buildBatchedChunkMeshInternal(core, finalGeometries, ownedGeometries, options);
}

function buildBatchedChunkMeshInternal(
  core: DestructibleCore,
  geometries: THREE.BufferGeometry[],
  ownedGeometries: Set<THREE.BufferGeometry>,
  options?: BatchedChunkMeshOptions,
): BatchedChunkMeshResult {
  const chunkCount = core.chunks.length;
  const { vertexCount, indexCount } = getBatchedMeshCount(geometries);

  const material = new THREE.MeshStandardMaterial({
    color: 0xbababa,
    roughness: 0.5,
    metalness: 0.1,
  });

  const batchedMesh = new THREE.BatchedMesh(
    Math.max(1, chunkCount),
    Math.max(3, vertexCount),
    Math.max(3, indexCount),
    material,
  ) as ColorWritableBatchedMesh;

  batchedMesh.castShadow = shadowsEnabled;
  batchedMesh.receiveShadow = shadowsEnabled;
  batchedMesh.frustumCulled = false;

  const geometryIds: number[] = [];
  const chunkToInstanceId = new Map<number, number>();

  for (let i = 0; i < chunkCount; i += 1) {
    const chunk = core.chunks[i];
    const geometry = geometries[i];
    const geometryId = batchedMesh.addGeometry(geometry);
    geometryIds.push(geometryId);

    const instanceId = batchedMesh.addInstance(geometryId);
    chunkToInstanceId.set(chunk.nodeIndex, instanceId);

    const body = core.world.getRigidBody(chunk.bodyHandle ?? -1);
    if (body) {
      const translation = body.translation();
      const rotation = body.rotation();
      _position.set(translation.x, translation.y, translation.z);
      _quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
      _localOffset
        .set(chunk.baseLocalOffset.x, chunk.baseLocalOffset.y, chunk.baseLocalOffset.z)
        .applyQuaternion(_quaternion);
      _position.add(_localOffset);
      _matrix.compose(_position, _quaternion, _scale);
    } else {
      _position.set(chunk.baseLocalOffset.x, chunk.baseLocalOffset.y, chunk.baseLocalOffset.z);
      _quaternion.identity();
      _matrix.compose(_position, _quaternion, _scale);
    }

    batchedMesh.setMatrixAt(instanceId, _matrix);
    batchedMesh.setVisibleAt(instanceId, !chunk.destroyed);
    if (typeof batchedMesh.setColorAt === 'function') {
      batchedMesh.setColorAt(instanceId, _batchedDefaultColor);
    }
  }

  try {
    batchedMesh.computeBoundingBox();
  } catch {}
  try {
    batchedMesh.computeBoundingSphere();
  } catch {}

  if ((options?.enableBVH ?? true) && typeof batchedMesh.computeBVH === 'function') {
    try {
      batchedMesh.computeBVH(2000, { margin: options?.bvhMargin ?? 0.5 });
    } catch {}
  }

  return {
    batchedMesh,
    chunkToInstanceId,
    geometryIds,
    dispose: () => {
      try {
        batchedMesh.dispose();
      } catch {}
      try {
        material.dispose();
      } catch {}
      disposeOwnedGeometries(ownedGeometries);
    },
  };
}

export function updateChunkMeshes(core: DestructibleCore, meshes: THREE.Mesh[]) {
  const tmp = new THREE.Vector3();
  const quat = new THREE.Quaternion();
  const isDev = typeof process !== 'undefined' ? process.env.NODE_ENV !== 'production' : true;

  if (isDev && meshes.length !== core.chunks.length) {
    console.error('[Adapter] Chunk mesh count mismatch', {
      meshes: meshes.length,
      chunks: core.chunks.length,
    });
    throw new Error('Chunk mesh count mismatch');
  }

  for (let i = 0; i < core.chunks.length; i += 1) {
    const chunk = core.chunks[i];
    const mesh = meshes[i];
    if (!mesh) continue;

    if (chunk.destroyed) {
      mesh.visible = false;
      continue;
    }
    mesh.visible = true;

    const handle = chunk.bodyHandle;
    if (handle == null) continue;

    const body = core.world.getRigidBody(handle);
    if (!body) continue;

    const translation = body.translation();
    const rotation = body.rotation();
    mesh.position.set(translation.x, translation.y, translation.z);
    quat.set(rotation.x, rotation.y, rotation.z, rotation.w);
    mesh.quaternion.copy(quat);
    tmp
      .set(chunk.baseLocalOffset.x, chunk.baseLocalOffset.y, chunk.baseLocalOffset.z)
      .applyQuaternion(mesh.quaternion);
    mesh.position.add(tmp);

    const material = toMeshStandardMaterial(mesh.material);
    if (material) {
      applyChunkColor({ core, chunk, body, color: material.color });
    }
  }
}

export function updateBatchedChunkMesh(
  core: DestructibleCore,
  batchedMesh: ColorWritableBatchedMesh,
  chunkToInstanceId: Map<number, number>,
  options?: {
    updateBVH?: boolean;
  },
) {
  const updateBVH = options?.updateBVH ?? false;

  for (let i = 0; i < core.chunks.length; i += 1) {
    const chunk = core.chunks[i];
    const instanceId = chunkToInstanceId.get(chunk.nodeIndex);
    if (instanceId == null) continue;

    if (chunk.destroyed) {
      batchedMesh.setVisibleAt(instanceId, false);
      continue;
    }

    const handle = chunk.bodyHandle;
    if (handle == null) {
      batchedMesh.setVisibleAt(instanceId, false);
      continue;
    }

    const body = core.world.getRigidBody(handle);
    if (!body) {
      batchedMesh.setVisibleAt(instanceId, false);
      continue;
    }

    const translation = body.translation();
    const rotation = body.rotation();
    _position.set(translation.x, translation.y, translation.z);
    _quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
    _localOffset
      .set(chunk.baseLocalOffset.x, chunk.baseLocalOffset.y, chunk.baseLocalOffset.z)
      .applyQuaternion(_quaternion);
    _position.add(_localOffset);

    _matrix.compose(_position, _quaternion, _scale);
    batchedMesh.setMatrixAt(instanceId, _matrix);
    batchedMesh.setVisibleAt(instanceId, true);

    if (typeof batchedMesh.setColorAt === 'function') {
      applyChunkColor({ core, chunk, body, color: _colorTmp });
      batchedMesh.setColorAt(instanceId, _colorTmp);
    }

    if (updateBVH && batchedMesh.bvh) {
      try {
        batchedMesh.bvh.move(instanceId);
      } catch {}
    }
  }
}

export { SolverDebugLinesHelper } from './solver-debug-lines';

export function updateProjectileMeshes(
  core: DestructibleCore,
  root: THREE.Group,
) {
  const profilerRecorder = (
    core as unknown as {
      recordProjectileCleanupDuration?: (durationMs: number) => void;
    }
  ).recordProjectileCleanupDuration;
  const hasPerf =
    typeof performance !== 'undefined' && typeof performance.now === 'function';
  const timeNow = hasPerf ? () => performance.now() : () => Date.now();
  const timerStart = profilerRecorder ? timeNow() : null;
  const projectiles = core.projectiles as Array<{
    bodyHandle: number;
    radius: number;
    type: 'ball' | 'box';
    mesh?: THREE.Mesh;
    spawnTime?: number;
  }>;
  const now = nowSeconds();

  for (let i = projectiles.length - 1; i >= 0; i -= 1) {
    const projectile = projectiles[i];
    const body = core.world.getRigidBody(projectile.bodyHandle);
    const lifetime =
      typeof projectile.spawnTime === 'number' ? now - projectile.spawnTime : 0;
    const shouldCullLifetime = lifetime > PROJECTILE_MAX_LIFETIME;
    const shouldCullBody = !body;
    const bodyTranslation = body?.translation();
    const shouldCullFall = bodyTranslation ? bodyTranslation.y < PROJECTILE_MIN_Y : false;

    if (shouldCullLifetime || shouldCullBody || shouldCullFall) {
      if (projectile.mesh) {
        root.remove(projectile.mesh);
        try {
          projectile.mesh.geometry?.dispose?.();
        } catch {}
        try {
          (projectile.mesh.material as THREE.Material | undefined)?.dispose?.();
        } catch {}
      }
      if (body) {
        try {
          core.world.removeRigidBody(body);
        } catch {}
      }
      projectiles.splice(i, 1);
      continue;
    }

    if (!projectile.mesh) {
      const geometry =
        projectile.type === 'ball'
          ? new THREE.SphereGeometry(projectile.radius, 24, 24)
          : new THREE.BoxGeometry(
              projectile.radius * 2,
              projectile.radius * 2,
              projectile.radius * 2,
            );
      const material = new THREE.MeshStandardMaterial({
        color: 0xff9147,
        emissive: 0x331100,
        roughness: 0.4,
        metalness: 0.2,
      });
      const mesh = new THREE.Mesh(geometry, material);
      mesh.castShadow = shadowsEnabled;
      mesh.receiveShadow = shadowsEnabled;
      projectile.mesh = mesh;
      root.add(mesh);
    }

    if (!body || !bodyTranslation) continue;

    const mesh = projectile.mesh as THREE.Mesh;
    mesh.visible = true;
    const rotation = body.rotation();
    mesh.position.set(bodyTranslation.x, bodyTranslation.y, bodyTranslation.z);
    mesh.quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
  }

  if (profilerRecorder && timerStart != null) {
    profilerRecorder(Math.max(0, timeNow() - timerStart));
  }
}
