import * as THREE from 'three';
import type { DestructibleCore, ScenarioDesc } from '../rapier/types';
import {
  buildBatchedChunkMesh,
  buildBatchedChunkMeshFromGeometries,
  buildChunkMeshes,
  buildChunkMeshesFromGeometries,
  type BatchedChunkMeshOptions,
  type BatchedChunkMeshResult,
  type ChunkMeshBuildOptions,
  type ChunkMeshBuildResult,
} from './destructible-adapter';

export type ScenarioThreeParameters = {
  fragmentGeometries?: THREE.BufferGeometry[];
};

export function getScenarioFragmentGeometries(
  scenario: ScenarioDesc,
): THREE.BufferGeometry[] | undefined {
  const parameters = scenario.parameters as ScenarioThreeParameters | undefined;
  const geometries = parameters?.fragmentGeometries;
  return Array.isArray(geometries) ? geometries : undefined;
}

export function buildChunkMeshesFromScenario(
  core: DestructibleCore,
  scenario: ScenarioDesc,
  materials?: { deck?: THREE.Material; support?: THREE.Material },
  options?: ChunkMeshBuildOptions,
): ChunkMeshBuildResult {
  const geometries = getScenarioFragmentGeometries(scenario);
  return geometries?.length
    ? buildChunkMeshesFromGeometries(core, geometries, materials, options)
    : buildChunkMeshes(core, materials);
}

export function buildBatchedChunkMeshFromScenario(
  core: DestructibleCore,
  scenario: ScenarioDesc,
  options?: BatchedChunkMeshOptions,
): BatchedChunkMeshResult {
  const geometries = getScenarioFragmentGeometries(scenario);
  return geometries?.length
    ? buildBatchedChunkMeshFromGeometries(core, geometries, options)
    : buildBatchedChunkMesh(core, options);
}
