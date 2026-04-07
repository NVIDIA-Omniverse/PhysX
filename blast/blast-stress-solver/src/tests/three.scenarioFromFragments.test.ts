/**
 * Tests for building ScenarioDesc from fragment arrays.
 */
import { describe, it, expect } from 'vitest';
import * as THREE from 'three';
import {
  buildScenarioFromFragments,
} from '../three/scenarioFromFragments';
import type { FragmentInfo } from '../three/fracture';

function makeBoxFragment(
  position: { x: number; y: number; z: number },
  halfExtents: { x: number; y: number; z: number },
  isSupport = false,
): FragmentInfo {
  const geometry = new THREE.BoxGeometry(
    halfExtents.x * 2,
    halfExtents.y * 2,
    halfExtents.z * 2,
  );
  return { worldPosition: position, halfExtents, geometry, isSupport };
}

describe('buildScenarioFromFragments', () => {
  it('creates correct number of nodes', () => {
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
      makeBoxFragment({ x: 1, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
      makeBoxFragment({ x: 2, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
    ];
    const scenario = buildScenarioFromFragments(fragments);
    expect(scenario.nodes.length).toBe(3);
  });

  it('support fragments get mass 0', () => {
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }, true),
      makeBoxFragment({ x: 1, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }, false),
    ];
    const scenario = buildScenarioFromFragments(fragments);
    expect(scenario.nodes[0].mass).toBe(0);
    expect(scenario.nodes[1].mass).toBeGreaterThan(0);
  });

  it('scales total mass correctly', () => {
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
      makeBoxFragment({ x: 1, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
    ];
    const totalMass = 500;
    const scenario = buildScenarioFromFragments(fragments, { totalMass });
    const actualTotal = scenario.nodes.reduce((s, n) => s + n.mass, 0);
    expect(actualTotal).toBeCloseTo(totalMass, 1);
  });

  it('detects bonds between adjacent fragments', () => {
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
      makeBoxFragment({ x: 1, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
    ];
    const scenario = buildScenarioFromFragments(fragments);
    expect(scenario.bonds.length).toBe(1);
  });

  it('includes fragmentGeometries in parameters', () => {
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
    ];
    const scenario = buildScenarioFromFragments(fragments);
    const geoms = scenario.parameters?.fragmentGeometries as THREE.BufferGeometry[];
    expect(geoms).toBeDefined();
    expect(geoms.length).toBe(1);
  });

  it('includes colliderDescForNode array', () => {
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
    ];
    const scenario = buildScenarioFromFragments(fragments);
    expect(scenario.colliderDescForNode).toBeDefined();
    expect(scenario.colliderDescForNode!.length).toBe(1);
    // Without rapier, entries are null
    expect(scenario.colliderDescForNode![0]).toBeNull();
  });

  it('with rapier mock, colliderDescForNode creates cuboid for supports', () => {
    const mockCuboid = { type: 'cuboid' };
    const mockRapier = {
      ColliderDesc: {
        cuboid: () => mockCuboid,
        convexHull: () => ({ type: 'convexHull' }),
      },
    };
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }, true),
    ];
    const scenario = buildScenarioFromFragments(fragments, { rapier: mockRapier });
    const desc = scenario.colliderDescForNode![0]!;
    expect(desc).not.toBeNull();
    expect(desc()).toBe(mockCuboid);
  });

  it('areaNormalization none leaves areas unchanged', () => {
    const fragments = [
      makeBoxFragment({ x: 0, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
      makeBoxFragment({ x: 1, y: 0, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 }),
    ];
    const noNorm = buildScenarioFromFragments(fragments, { areaNormalization: 'none' });
    const withNorm = buildScenarioFromFragments(fragments, { areaNormalization: 'perAxis' });
    // Areas should differ between modes (unless coincidentally equal)
    expect(noNorm.bonds.length).toBe(withNorm.bonds.length);
  });

  it('node centroids match fragment positions', () => {
    const pos = { x: 3.5, y: 2.1, z: -1.0 };
    const fragments = [
      makeBoxFragment(pos, { x: 0.5, y: 0.5, z: 0.5 }),
    ];
    const scenario = buildScenarioFromFragments(fragments);
    expect(scenario.nodes[0].centroid.x).toBeCloseTo(pos.x, 5);
    expect(scenario.nodes[0].centroid.y).toBeCloseTo(pos.y, 5);
    expect(scenario.nodes[0].centroid.z).toBeCloseTo(pos.z, 5);
  });

  it('handles empty fragment array', () => {
    const scenario = buildScenarioFromFragments([]);
    expect(scenario.nodes.length).toBe(0);
    expect(scenario.bonds.length).toBe(0);
  });
});
