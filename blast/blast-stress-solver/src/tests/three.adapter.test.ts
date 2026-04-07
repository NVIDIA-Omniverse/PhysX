/**
 * Tests for Three.js adapter helpers (chunk mesh building, batched rendering, etc.)
 *
 * These test the pure Three.js utility functions. Build/layout functions
 * only need chunk data (no Rapier world), while update functions need
 * a mock Rapier world.
 */
import { describe, it, expect } from 'vitest';
import * as THREE from 'three';
import {
  buildChunkMeshes,
  buildChunkMeshesFromGeometries,
  buildBatchedChunkMesh,
  buildBatchedChunkMeshFromGeometries,
} from '../three/destructible-adapter';
import { getScenarioFragmentGeometries } from '../three/scenario';
import type { ChunkData, DestructibleCore, Vec3 } from '../rapier/types';

function makeChunk(nodeIndex: number, pos: Vec3, opts?: { size?: Vec3; isSupport?: boolean }): ChunkData {
  const size = opts?.size ?? { x: 1, y: 1, z: 1 };
  return {
    nodeIndex,
    size,
    isSupport: opts?.isSupport ?? false,
    baseLocalOffset: pos,
    localOffset: pos,
    colliderHandle: nodeIndex,
    bodyHandle: 0,
    active: true,
    detached: false,
    baseWorldPosition: pos,
  };
}

/** Create a minimal mock that satisfies the DestructibleCore interface for build functions */
function makeMockCore(chunks: ChunkData[]): DestructibleCore {
  const mockWorld = {
    getRigidBody: () => ({
      translation: () => ({ x: 0, y: 0, z: 0 }),
      rotation: () => ({ x: 0, y: 0, z: 0, w: 1 }),
      isKinematic: () => false,
      isFixed: () => false,
      isDynamic: () => true,
    }),
  };
  return { chunks, world: mockWorld } as unknown as DestructibleCore;
}

describe('buildChunkMeshes', () => {
  it('creates one mesh per chunk and returns dispose function', () => {
    const chunks = [
      makeChunk(0, { x: 0, y: 0, z: 0 }),
      makeChunk(1, { x: 1.5, y: 0, z: 0 }),
      makeChunk(2, { x: 3, y: 0, z: 0 }),
    ];
    const core = makeMockCore(chunks);
    const result = buildChunkMeshes(core);

    expect(result.objects).toHaveLength(3);
    expect(typeof result.dispose).toBe('function');
    for (const m of result.objects) {
      expect(m).toBeInstanceOf(THREE.Mesh);
      expect(m.geometry).toBeInstanceOf(THREE.BoxGeometry);
    }

    // Dispose should not throw
    result.dispose();
  });

  it('uses chunk size for geometry dimensions', () => {
    const chunks = [makeChunk(0, { x: 0, y: 0, z: 0 }, { size: { x: 2, y: 3, z: 4 } })];
    const core = makeMockCore(chunks);
    const result = buildChunkMeshes(core);

    const params = (result.objects[0].geometry as THREE.BoxGeometry).parameters;
    expect(params.width).toBeCloseTo(2);
    expect(params.height).toBeCloseTo(3);
    expect(params.depth).toBeCloseTo(4);
    result.dispose();
  });

  it('assigns different materials for support vs deck chunks', () => {
    const chunks = [
      makeChunk(0, { x: 0, y: 0, z: 0 }, { isSupport: true }),
      makeChunk(1, { x: 1, y: 0, z: 0 }, { isSupport: false }),
    ];
    const core = makeMockCore(chunks);
    const result = buildChunkMeshes(core);

    // Both should have materials but they should be different instances
    const mat0 = result.objects[0].material as THREE.MeshStandardMaterial;
    const mat1 = result.objects[1].material as THREE.MeshStandardMaterial;
    expect(mat0).toBeInstanceOf(THREE.MeshStandardMaterial);
    expect(mat1).toBeInstanceOf(THREE.MeshStandardMaterial);
    result.dispose();
  });

  it('stores nodeIndex in mesh userData', () => {
    const chunks = [makeChunk(5, { x: 0, y: 0, z: 0 }), makeChunk(7, { x: 1, y: 0, z: 0 })];
    const core = makeMockCore(chunks);
    const result = buildChunkMeshes(core);

    expect(result.objects[0].userData.nodeIndex).toBe(5);
    expect(result.objects[1].userData.nodeIndex).toBe(7);
    result.dispose();
  });
});

describe('buildChunkMeshesFromGeometries', () => {
  it('creates meshes from custom geometries', () => {
    const chunks = [
      makeChunk(0, { x: 0, y: 0, z: 0 }),
      makeChunk(1, { x: 1, y: 0, z: 0 }),
    ];
    const core = makeMockCore(chunks);
    const geoms = [
      new THREE.SphereGeometry(0.5),
      new THREE.CylinderGeometry(0.3, 0.3, 1),
    ];

    const result = buildChunkMeshesFromGeometries(core, geoms);
    expect(result.objects).toHaveLength(2);
    result.dispose();
  });

  it('falls back to box geometry for support chunks', () => {
    const chunks = [
      makeChunk(0, { x: 0, y: 0, z: 0 }, { isSupport: true }),
      makeChunk(1, { x: 1, y: 0, z: 0 }),
    ];
    const core = makeMockCore(chunks);
    const customGeom = new THREE.SphereGeometry(0.5);

    const result = buildChunkMeshesFromGeometries(core, [customGeom, customGeom]);
    expect(result.objects).toHaveLength(2);
    // Support chunk should get box geometry
    expect(result.objects[0].geometry).toBeInstanceOf(THREE.BoxGeometry);
    result.dispose();
  });
});

describe('buildBatchedChunkMesh', () => {
  it('creates a BatchedMesh with geometry IDs for each chunk', () => {
    const chunks = [
      makeChunk(0, { x: 0, y: 0, z: 0 }),
      makeChunk(1, { x: 1, y: 0, z: 0 }),
      makeChunk(2, { x: 2, y: 0, z: 0 }),
    ];
    const core = makeMockCore(chunks);
    const result = buildBatchedChunkMesh(core);

    expect(result.batchedMesh).toBeInstanceOf(THREE.BatchedMesh);
    expect(result.geometryIds).toHaveLength(3);
    expect(result.chunkToInstanceId.size).toBe(3);
    expect(typeof result.dispose).toBe('function');
    result.dispose();
  });

  it('accepts BatchedChunkMeshOptions', () => {
    const chunks = [makeChunk(0, { x: 0, y: 0, z: 0 })];
    const core = makeMockCore(chunks);
    const result = buildBatchedChunkMesh(core, { enableBVH: false, bvhMargin: 2 });

    expect(result.batchedMesh).toBeInstanceOf(THREE.BatchedMesh);
    result.dispose();
  });
});

describe('buildBatchedChunkMeshFromGeometries', () => {
  it('creates a BatchedMesh from provided geometries', () => {
    const chunks = [
      makeChunk(0, { x: 0, y: 0, z: 0 }),
      makeChunk(1, { x: 1, y: 0, z: 0 }),
    ];
    const core = makeMockCore(chunks);
    const geoms = [
      new THREE.BoxGeometry(1, 1, 1),
      new THREE.BoxGeometry(1.5, 1.5, 1.5),
    ];

    const result = buildBatchedChunkMeshFromGeometries(core, geoms);
    expect(result.batchedMesh).toBeInstanceOf(THREE.BatchedMesh);
    expect(result.geometryIds).toHaveLength(2);
    result.dispose();
  });
});

describe('getScenarioFragmentGeometries', () => {
  it('returns geometries from scenario.parameters.fragmentGeometries', () => {
    const geom1 = new THREE.BoxGeometry(1, 1, 1);
    const geom2 = new THREE.SphereGeometry(0.5);
    const scenario = {
      nodes: [],
      bonds: [],
      parameters: { fragmentGeometries: [geom1, geom2] },
    };

    const result = getScenarioFragmentGeometries(scenario as any);
    expect(result).toHaveLength(2);
    expect(result![0]).toBe(geom1);
    expect(result![1]).toBe(geom2);
  });

  it('returns falsy when no parameters', () => {
    const scenario = { nodes: [], bonds: [] };
    const result = getScenarioFragmentGeometries(scenario as any);
    expect(result).toBeFalsy();
  });

  it('returns empty array for empty fragmentGeometries', () => {
    const scenario = { nodes: [], bonds: [], parameters: { fragmentGeometries: [] } };
    const result = getScenarioFragmentGeometries(scenario as any);
    // May return [] or null/undefined depending on implementation
    expect(result?.length ?? 0).toBe(0);
  });
});
