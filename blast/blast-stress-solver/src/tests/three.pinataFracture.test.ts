/**
 * Tests for three-pinata integration (Voronoi fracturing).
 *
 * These tests require @dgreenheck/three-pinata to be installed.
 * They are skipped if the dependency is not available.
 */
import { describe, it, expect, beforeAll } from 'vitest';
import * as THREE from 'three';

// Check synchronously at module load time so skipIf works
let pinataAvailable = false;
try {
  require.resolve('@dgreenheck/three-pinata');
  pinataAvailable = true;
} catch {
  pinataAvailable = false;
}

describe('pinataFracture (requires three-pinata)', () => {
  beforeAll(async () => {
    if (pinataAvailable) {
      const { ensurePinataLoaded } = await import('../three/pinataFracture');
      await ensurePinataLoaded();
    }
  });

  it.skipIf(!pinataAvailable)('fractureGeometry produces fragments', async () => {
    const { fractureGeometry } = await import('../three/pinataFracture');

    const geometry = new THREE.BoxGeometry(2, 2, 2, 2, 2, 2);
    const fragments = fractureGeometry(geometry, { fragmentCount: 10 });

    expect(fragments.length).toBeGreaterThan(0);
    for (const f of fragments) {
      expect(f.geometry).toBeInstanceOf(THREE.BufferGeometry);
      expect(f.halfExtents.x).toBeGreaterThan(0);
      expect(f.halfExtents.y).toBeGreaterThan(0);
      expect(f.halfExtents.z).toBeGreaterThan(0);
      expect(f.isSupport).toBe(false);
    }
  });

  it.skipIf(!pinataAvailable)('fractureGeometryAsync works', async () => {
    const { fractureGeometryAsync } = await import('../three/pinataFracture');

    const geometry = new THREE.BoxGeometry(2, 2, 2, 2, 2, 2);
    const fragments = await fractureGeometryAsync(geometry, { fragmentCount: 8 });

    expect(fragments.length).toBeGreaterThan(0);
  });

  it.skipIf(!pinataAvailable)('buildFracturedScenario produces valid ScenarioDesc', async () => {
    const { buildFracturedScenario } = await import('../three/pinataFracture');

    const geometry = new THREE.BoxGeometry(2, 2, 2, 2, 2, 2);
    const scenario = buildFracturedScenario(geometry, {
      fragmentCount: 10,
      totalMass: 1000,
    });

    expect(scenario.nodes.length).toBeGreaterThan(0);
    expect(scenario.bonds.length).toBeGreaterThan(0);
    // All nodes should have positive volume
    for (const node of scenario.nodes) {
      expect(node.volume).toBeGreaterThan(0);
    }
    // Total mass should match
    const totalMass = scenario.nodes.reduce((s, n) => s + n.mass, 0);
    expect(totalMass).toBeCloseTo(1000, 0);
  });

  it.skipIf(!pinataAvailable)('buildFracturedScenario with foundation adds support fragments', async () => {
    const { buildFracturedScenario } = await import('../three/pinataFracture');

    const geometry = new THREE.BoxGeometry(2, 2, 2, 2, 2, 2);
    const scenario = buildFracturedScenario(geometry, {
      fragmentCount: 10,
      foundation: { enabled: true },
    });

    const supports = scenario.nodes.filter((n) => n.mass === 0);
    expect(supports.length).toBeGreaterThan(0);
  });

  it.skipIf(pinataAvailable)('throws descriptive error when three-pinata not installed', async () => {
    // Only runs if three-pinata is NOT installed
    const { fractureGeometry } = await import('../three/pinataFracture');
    const geometry = new THREE.BoxGeometry(1, 1, 1);
    expect(() => fractureGeometry(geometry)).toThrow('three-pinata');
  });
});
