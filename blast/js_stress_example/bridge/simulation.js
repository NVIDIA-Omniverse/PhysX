import * as THREE from 'three';
import RAPIER from '@dimforge/rapier3d-compat';
import { vec3, computeBondStress } from '../stress.js';
import { updateProjectiles } from './spawning.js';
import { updateBondTable, pushEvent } from './ui.js';
import { BRIDGE_DIMENSIONS } from './constants.js';
import { createColliderForChunk } from './factory.js';
import { resolveParentBody, computeWorldPose, computeLinearVelocityAtPoint, identityQuaternion } from './transformUtils.js';

export const stressColors = {
  low: new THREE.Color('#3fb3ff'),
  medium: new THREE.Color('#ff9f43'),
  high: new THREE.Color('#ff4f67')
};
const severityScratchColor = new THREE.Color();

export function updateBridge(world, bridge, delta) {
  const { stressProcessor, chunks, bonds, limits } = bridge;

  const velocities = chunks.map((chunk) => {
    const body = world.getRigidBody(chunk.bodyHandle);
    if (!body) {
      return { lin: vec3(), ang: vec3() };
    }
    const linVel = body.linvel();
    const angVel = body.angvel();
    return {
      lin: vec3(linVel.x, linVel.y, linVel.z),
      ang: vec3(angVel.x, angVel.y, angVel.z)
    };
  });

  let solveResult;
  try {
    solveResult = stressProcessor.solve({ velocities, resume: false });
  } catch (err) {
    pushEvent(`Stress solver failed: ${err.message ?? err}`);
    return;
  }
  const { iterations, error, impulses } = solveResult;
  if (!Number.isFinite(iterations)) {
    pushEvent('Stress solver failed: non-finite iteration count');
    return;
  }
  const solverNodes = stressProcessor.getNodes();

  bonds.forEach((bond, index) => {
    if (!bond.active) {
      return;
    }
    const stress = computeBondStress(
      stressProcessor.bondDesc(index),
      impulses[index],
      solverNodes,
      bond.area
    );
    const severity = limits.severity(stress);
    bond.severity = severity;
    severity.max = Math.max(severity.compression, severity.tension, severity.shear);
    const mode = limits.failureMode(stress);
    if (mode) {
      fractureBond(world, bridge, index, bond, mode);
    }
  });

  syncMeshes(world, bridge);
  updateProjectiles(world, bridge, delta);
  updateLoadVehicle(world, bridge, delta);
  updateBondTable(bonds);
  pushEvent(`Bridge solve: iter ${iterations}, error=(${error.lin.toFixed(3)}, ${error.ang.toFixed(3)})`);
}

export function fractureBond(world, bridge, bondIndex, bond, mode) {
  const chunkA = bridge.chunks.find((chunk) => chunk.nodeId === bond.node0);
  const chunkB = bridge.chunks.find((chunk) => chunk.nodeId === bond.node1);
  if (!chunkA || !chunkB) {
    return;
  }
  bond.active = false;
  pushEvent(`${bond.name} failed due to ${mode}`);

  const targetChunk = chunkB.active ? chunkB : chunkA;
  if (!targetChunk || !targetChunk.active) {
    return;
  }
  splitChunk(world, bridge, targetChunk);
}

export function splitChunk(world, bridge, chunk) {
  if (!chunk.active) {
    return;
  }
  const parentBody = resolveParentBody(world, chunk);
  if (!parentBody) {
    return;
  }
  const collider = world.getCollider(chunk.colliderHandle);
  if (!collider) {
    return;
  }

  const pose = computeWorldPose(parentBody, chunk);
  if (!pose) {
    return;
  }

  const parentTranslation = parentBody.translation();
  const offsetWorld = pose.position.clone().sub(new THREE.Vector3(parentTranslation.x, parentTranslation.y, parentTranslation.z));
  const worldLinVel = computeLinearVelocityAtPoint(parentBody, {
    x: offsetWorld.x,
    y: offsetWorld.y,
    z: offsetWorld.z
  });
  const parentAngVel = parentBody.angvel();

  // FIXME: After this errors with "Uncaught RuntimeError: unreachable""
  return;

  chunk.active = false;
  world.removeCollider(collider, false);

  const newBodyDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(pose.position.x, pose.position.y, pose.position.z)
    .setRotation(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w)
    .setLinearDamping(parentBody.linearDamping())
    .setAngularDamping(parentBody.angularDamping());

  if (Number.isFinite(worldLinVel.x) && Number.isFinite(worldLinVel.y) && Number.isFinite(worldLinVel.z)) {
    newBodyDesc.setLinvel(worldLinVel.x, worldLinVel.y, worldLinVel.z);
  }
  if (
    Number.isFinite(parentAngVel.x) &&
    Number.isFinite(parentAngVel.y) &&
    Number.isFinite(parentAngVel.z)
  ) {
    newBodyDesc.setAngvel(parentAngVel.x, parentAngVel.y, parentAngVel.z);
  }

  const newBody = world.createRigidBody(newBodyDesc);

  chunk.localOffset.set(0, 0, 0);
  chunk.localQuat.copy(identityQuaternion());

  const newColliderDesc = createColliderForChunk(chunk);
  const newCollider = world.createCollider(newColliderDesc, newBody);

  chunk.bodyHandle = newBody.handle;
  chunk.parentBodyHandle = newBody.handle;
  chunk.colliderHandle = newCollider.handle;
  chunk.active = true;
  if (chunk.mesh) {
    chunk.mesh.position.set(pose.position.x, pose.position.y, pose.position.z);
    chunk.mesh.quaternion.copy(pose.rotation);
  }

  if (Array.isArray(bridge.splitBodies)) {
    bridge.splitBodies.push(newBody.handle);
  }
}

function syncMeshes(world, bridge) {
  bridge.chunks.forEach((chunk) => {
    if (!chunk.mesh) {
      return;
    }
    const body = world.getRigidBody(chunk.bodyHandle);
    if (!body) {
      return;
    }
    const position = body.translation();
    const rotation = body.rotation();

    chunk.mesh.position.set(position.x, position.y, position.z);
    chunk.mesh.quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
    chunk.mesh.position.add(chunk.localOffset.clone().applyQuaternion(chunk.mesh.quaternion));

    const severity = Math.max(
      chunk.severity?.max ?? 0,
      ...bridge.bonds
        .filter((bond) => bond.node0 === chunk.nodeId || bond.node1 === chunk.nodeId)
        .map((bond) => bond.severity.max)
    );

    let targetColor = stressColors.low;
    if (severity >= 0.9) {
      targetColor = stressColors.high;
    } else if (severity >= 0.5) {
      targetColor = stressColors.medium;
    }

    chunk.mesh.material.color.copy(targetColor);
  });
}

function updateLoadVehicle(world, bridge, delta) {
  if (!bridge.loadVehicle) {
    return;
  }
  const vehicle = bridge.loadVehicle;
  const body = world.getRigidBody(vehicle.bodyHandle);
  if (!body) {
    vehicle.mesh.removeFromParent();
    bridge.loadVehicle = undefined;
    return;
  }
  const translation = body.translation();
  if (translation.x > BRIDGE_DIMENSIONS.deckSegmentLength * 1.6) {
    body.setTranslation({ x: -BRIDGE_DIMENSIONS.deckSegmentLength * 1.6, y: translation.y, z: translation.z }, true);
  }
  vehicle.mesh.position.set(translation.x, translation.y, translation.z);
  const rotation = body.rotation();
  vehicle.mesh.quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
}

