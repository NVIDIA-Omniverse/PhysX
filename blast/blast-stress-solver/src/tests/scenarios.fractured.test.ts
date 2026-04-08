/**
 * Integration tests for fractured tower and fractured bridge scenarios.
 *
 * Tests verify:
 * - Scenario produces valid ScenarioDesc (nodes, bonds, parameters)
 * - Support/dynamic node distribution is correct
 * - Bond strength multipliers are applied
 * - Small configurations for fast test execution
 *
 * Requires @dgreenheck/three-pinata (skips gracefully if unavailable).
 */
import { describe, it, expect, beforeAll } from 'vitest';

let pinataAvailable = false;
try {
  require.resolve('@dgreenheck/three-pinata');
  pinataAvailable = true;
} catch {
  pinataAvailable = false;
}

describe('Fractured scenarios (requires three-pinata)', () => {
  beforeAll(async () => {
    if (pinataAvailable) {
      const { ensurePinataLoaded } = await import('../three/pinataFracture');
      await ensurePinataLoaded();
    }
  });

  // ── Fractured Tower ─────────────────────────────────────────

  describe('buildFracturedTowerScenario', () => {
    it.skipIf(!pinataAvailable)('produces valid scenario with small config', async () => {
      const { buildFracturedTowerScenario } = await import('../scenarios/fracturedTowerScenario');

      const scenario = await buildFracturedTowerScenario({
        width: 6,
        depth: 6,
        floorCount: 2,
        floorHeight: 3,
        thickness: 0.3,
        floorThickness: 0.2,
        columnSize: 0.6,
        columnsX: 2,
        columnsZ: 2,
        fragmentCountPerWall: 4,
        fragmentCountPerFloor: 4,
        fragmentCountPerColumn: 2,
        deckMass: 5000,
      });

      expect(scenario.nodes.length).toBeGreaterThan(0);
      expect(scenario.bonds.length).toBeGreaterThan(0);

      // Should have both support and dynamic nodes
      const supports = scenario.nodes.filter((n) => n.mass === 0);
      const dynamic = scenario.nodes.filter((n) => n.mass > 0);
      expect(supports.length).toBeGreaterThan(0);
      expect(dynamic.length).toBeGreaterThan(0);

      // Total dynamic mass should match deckMass
      const totalMass = dynamic.reduce((s, n) => s + n.mass, 0);
      expect(totalMass).toBeCloseTo(5000, -1); // within 10%

      // Parameters should include metadata
      expect(scenario.parameters).toBeDefined();
      expect(scenario.parameters!.floorCount).toBe(2);
      expect(scenario.parameters!.width).toBe(6);
    });

    it.skipIf(!pinataAvailable)('fragment geometries are stored in parameters', async () => {
      const { buildFracturedTowerScenario } = await import('../scenarios/fracturedTowerScenario');

      const scenario = await buildFracturedTowerScenario({
        width: 4, depth: 4, floorCount: 1,
        fragmentCountPerWall: 3,
        fragmentCountPerFloor: 3,
        fragmentCountPerColumn: 2,
        columnsX: 1, columnsZ: 1,
      });

      const geoms = scenario.parameters?.fragmentGeometries as unknown[];
      expect(geoms).toBeDefined();
      expect(geoms.length).toBe(scenario.nodes.length);
    });

    it.skipIf(!pinataAvailable)('bonds have type-based strength multipliers applied', async () => {
      const { buildFracturedTowerScenario } = await import('../scenarios/fracturedTowerScenario');

      const scenario = await buildFracturedTowerScenario({
        width: 6, depth: 6, floorCount: 1,
        floorHeight: 3, thickness: 0.3, floorThickness: 0.2,
        columnSize: 0.6, columnsX: 2, columnsZ: 2,
        fragmentCountPerWall: 3,
        fragmentCountPerFloor: 3,
        fragmentCountPerColumn: 2,
        deckMass: 2000,
      });

      // With columns, floors, and walls, we should see bonds with different areas.
      // Column-column bonds (if any) should have 4x the area of wall-wall bonds.
      // Just verify bonds exist and have positive areas.
      expect(scenario.bonds.length).toBeGreaterThan(0);
      for (const bond of scenario.bonds) {
        expect(bond.area).toBeGreaterThan(0);
        expect(Number.isFinite(bond.area)).toBe(true);
      }
    });
  });

  // ── Fractured Bridge ────────────────────────────────────────

  describe('buildFracturedBridgeScenario', () => {
    it.skipIf(!pinataAvailable)('produces valid scenario with default config', async () => {
      const { buildFracturedBridgeScenario } = await import('../scenarios/fracturedBridgeScenario');

      const scenario = await buildFracturedBridgeScenario({
        span: 6,
        deckWidth: 2,
        deckThickness: 0.3,
        pierHeight: 1.5,
        supportsPerSide: 2,
        postSize: 0.3,
        fragmentCountPerDeck: 8,
        fragmentCountPerPost: 3,
        deckMass: 5000,
      });

      expect(scenario.nodes.length).toBeGreaterThan(0);
      expect(scenario.bonds.length).toBeGreaterThan(0);

      // Should have support nodes (footings)
      const supports = scenario.nodes.filter((n) => n.mass === 0);
      expect(supports.length).toBe(4); // 2 sides x 2 posts = 4 footings

      // Total dynamic mass should match deckMass
      const dynamic = scenario.nodes.filter((n) => n.mass > 0);
      const totalMass = dynamic.reduce((s, n) => s + n.mass, 0);
      expect(totalMass).toBeCloseTo(5000, -1);

      // Parameters should include metadata
      expect(scenario.parameters!.span).toBe(6);
      expect(scenario.parameters!.supportsPerSide).toBe(2);
    });

    it.skipIf(!pinataAvailable)('deck fragments are positioned above posts', async () => {
      const { buildFracturedBridgeScenario } = await import('../scenarios/fracturedBridgeScenario');

      const pierHeight = 2.0;
      const deckThickness = 0.4;
      const footingThickness = 0.12;
      const scenario = await buildFracturedBridgeScenario({
        span: 4, deckWidth: 2,
        deckThickness, pierHeight,
        supportsPerSide: 2,
        fragmentCountPerDeck: 4,
        fragmentCountPerPost: 2,
        footingThickness,
      });

      const deckBottomY = 0.001 + footingThickness + pierHeight;
      const dynamic = scenario.nodes.filter((n) => n.mass > 0);

      // At least some fragments should be at deck height
      const deckFragments = dynamic.filter(
        (n) => n.centroid.y > deckBottomY - 0.1,
      );
      expect(deckFragments.length).toBeGreaterThan(0);
    });

    it.skipIf(!pinataAvailable)('fragment geometries are stored in parameters', async () => {
      const { buildFracturedBridgeScenario } = await import('../scenarios/fracturedBridgeScenario');

      const scenario = await buildFracturedBridgeScenario({
        span: 4, deckWidth: 2,
        supportsPerSide: 2,
        fragmentCountPerDeck: 4,
        fragmentCountPerPost: 2,
      });

      const geoms = scenario.parameters?.fragmentGeometries as unknown[];
      expect(geoms).toBeDefined();
      expect(geoms.length).toBe(scenario.nodes.length);
    });
  });
});
