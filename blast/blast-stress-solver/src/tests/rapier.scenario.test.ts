import { describe, expect, it } from 'vitest';
import {
  createScenarioNodeSizeResolver,
  getScenarioFragmentSizes,
  resolveScenarioNodeSize,
} from '../rapier/scenario';
import type { ScenarioDesc } from '../rapier/types';

function makeScenario(overrides?: Partial<ScenarioDesc>): ScenarioDesc {
  return {
    nodes: [
      { centroid: { x: 0, y: 0, z: 0 }, mass: 1, volume: 1 },
      { centroid: { x: 1, y: 0, z: 0 }, mass: 1, volume: 1 },
    ],
    bonds: [],
    ...overrides,
  };
}

describe('rapier/scenario', () => {
  it('returns explicit fragment sizes when present', () => {
    const scenario = makeScenario({
      spacing: { x: 9, y: 9, z: 9 },
      parameters: {
        fragmentSizes: [
          { x: 0.25, y: 0.5, z: 0.75 },
          { x: 1, y: 1, z: 1 },
        ],
      },
    });

    expect(getScenarioFragmentSizes(scenario)?.[0]).toEqual({
      x: 0.25,
      y: 0.5,
      z: 0.75,
    });
    expect(resolveScenarioNodeSize(0, scenario)).toEqual({
      x: 0.25,
      y: 0.5,
      z: 0.75,
    });
  });

  it('falls back to spacing and minimum component clamping', () => {
    const scenario = makeScenario({
      spacing: { x: 0.01, y: 0.2, z: 0.03 },
    });

    expect(resolveScenarioNodeSize(0, scenario)).toEqual({
      x: 0.05,
      y: 0.2,
      z: 0.05,
    });
  });

  it('creates reusable resolvers with caller fallback sizes', () => {
    const scenario = makeScenario();
    const resolve = createScenarioNodeSizeResolver({
      fallbackSize: { x: 0.6, y: 0.7, z: 0.8 },
      minimumComponent: 0.1,
    });

    expect(resolve(1, scenario)).toEqual({ x: 0.6, y: 0.7, z: 0.8 });
  });
});
