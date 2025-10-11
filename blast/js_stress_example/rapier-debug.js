import initRapierWasm, * as RAPierModule from '../../deps/rapier.js/builds/rapier3d/pkg/rapier_wasm3d.js';

let initPromise = null;

async function ensureInitialized(options) {
  if (!initPromise) {
    initPromise = initRapierWasm(options);
  }
  await initPromise;
  return RAPierModule;
}

const compat = {
  async init(options) {
    await ensureInitialized(options);
  }
};

Object.assign(compat, RAPierModule);

export async function init(options) {
  await ensureInitialized(options);
}

export { RAPierModule as rawModule };

export default compat;


