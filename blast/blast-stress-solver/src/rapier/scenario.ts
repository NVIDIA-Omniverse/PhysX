import type { ScenarioDesc, Vec3 } from './types';

export type ScenarioFragmentParameters = {
  fragmentSizes?: Vec3[];
  fragmentGeometries?: unknown[];
};

export type CreateScenarioNodeSizeResolverOptions = {
  /**
   * Fallback size used when the scenario does not include `parameters.fragmentSizes`
   * and also omits `spacing`.
   */
  fallbackSize?: Vec3;
  /** Clamp each component to at least this value. Default: `0.05`. */
  minimumComponent?: number;
};

const DEFAULT_NODE_SIZE: Vec3 = { x: 0.5, y: 0.5, z: 0.5 };
const DEFAULT_MIN_COMPONENT = 0.05;

function isFinitePositive(value: unknown): value is number {
  return typeof value === 'number' && Number.isFinite(value) && value > 0;
}

function sanitizeSize(size: Vec3, minimumComponent: number): Vec3 {
  return {
    x: Math.max(minimumComponent, size.x),
    y: Math.max(minimumComponent, size.y),
    z: Math.max(minimumComponent, size.z),
  };
}

export function getScenarioFragmentSizes(scenario: ScenarioDesc): Vec3[] | undefined {
  const parameters = scenario.parameters as ScenarioFragmentParameters | undefined;
  const sizes = parameters?.fragmentSizes;
  return Array.isArray(sizes) ? sizes : undefined;
}

export function resolveScenarioNodeSize(
  nodeIndex: number,
  scenario: ScenarioDesc,
  fallbackSize: Vec3 = DEFAULT_NODE_SIZE,
  minimumComponent = DEFAULT_MIN_COMPONENT,
): Vec3 {
  const fragmentSizes = getScenarioFragmentSizes(scenario);
  const explicit = fragmentSizes?.[nodeIndex];
  if (
    explicit &&
    isFinitePositive(explicit.x) &&
    isFinitePositive(explicit.y) &&
    isFinitePositive(explicit.z)
  ) {
    return sanitizeSize(explicit, minimumComponent);
  }

  const spacing = scenario.spacing;
  if (
    spacing &&
    isFinitePositive(spacing.x) &&
    isFinitePositive(spacing.y) &&
    isFinitePositive(spacing.z)
  ) {
    return sanitizeSize(spacing, minimumComponent);
  }

  return sanitizeSize(fallbackSize, minimumComponent);
}

export function createScenarioNodeSizeResolver(
  options?: CreateScenarioNodeSizeResolverOptions,
): (nodeIndex: number, scenario: ScenarioDesc) => Vec3 {
  const minimumComponent = Math.max(
    1e-6,
    options?.minimumComponent ?? DEFAULT_MIN_COMPONENT,
  );
  const fallbackSize = sanitizeSize(
    options?.fallbackSize ?? DEFAULT_NODE_SIZE,
    minimumComponent,
  );

  return (nodeIndex: number, scenario: ScenarioDesc) =>
    resolveScenarioNodeSize(nodeIndex, scenario, fallbackSize, minimumComponent);
}
