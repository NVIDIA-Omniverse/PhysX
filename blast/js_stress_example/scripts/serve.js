/**
 * Dev server for the wall-demolition / tower-collapse HTML demos.
 *
 * Maps import-map paths to the actual files:
 *   /vendor/three/*              → blast-stress-solver/node_modules/three/*
 *   /vendor/rapier/*             → blast-stress-solver/node_modules/@dimforge/rapier3d-compat/*
 *   /vendor/blast-stress-solver/* → blast-stress-solver/dist/*
 *
 * Usage:  node scripts/serve.js [port]
 */
import { createServer } from 'node:http';
import { readFile, stat } from 'node:fs/promises';
import { resolve, extname, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = dirname(fileURLToPath(import.meta.url));
const projectRoot = resolve(__dirname, '..');
const libRoot = resolve(projectRoot, '..', 'blast-stress-solver');
const PORT = parseInt(process.argv[2] || '3000', 10);

const MIME = {
  '.html': 'text/html',
  '.js':   'application/javascript',
  '.mjs':  'application/javascript',
  '.css':  'text/css',
  '.wasm': 'application/wasm',
  '.json': 'application/json',
  '.map':  'application/json',
  '.png':  'image/png',
  '.svg':  'image/svg+xml',
};

/** Map URL paths to filesystem paths. */
function resolveUrl(url) {
  // Vendor: three.js
  if (url.startsWith('/vendor/three/')) {
    return resolve(libRoot, 'node_modules/three', url.slice('/vendor/three/'.length));
  }
  // Vendor: rapier
  if (url.startsWith('/vendor/rapier/')) {
    return resolve(libRoot, 'node_modules/@dimforge/rapier3d-compat', url.slice('/vendor/rapier/'.length));
  }
  // Vendor: blast-stress-solver library
  if (url.startsWith('/vendor/blast-stress-solver/')) {
    return resolve(libRoot, 'dist', url.slice('/vendor/blast-stress-solver/'.length));
  }
  // Demo files (html, css, dist/*.js)
  return resolve(projectRoot, url.startsWith('/') ? url.slice(1) : url);
}

const server = createServer(async (req, res) => {
  let url = req.url.split('?')[0];
  if (url === '/') url = '/demo-index.html';

  const filePath = resolveUrl(url);

  // Security: don't serve outside expected roots
  if (!filePath.startsWith(projectRoot) && !filePath.startsWith(libRoot)) {
    res.writeHead(403);
    res.end('Forbidden');
    return;
  }

  try {
    const info = await stat(filePath);
    if (!info.isFile()) throw new Error('Not a file');
    const data = await readFile(filePath);
    const ext = extname(filePath);
    res.writeHead(200, {
      'Content-Type': MIME[ext] || 'application/octet-stream',
      'Cache-Control': 'no-cache',
    });
    res.end(data);
  } catch {
    res.writeHead(404);
    res.end(`Not found: ${url}`);
  }
});

server.listen(PORT, () => {
  console.log(`Demo server: http://localhost:${PORT}`);
  console.log(`  wall-demolition: http://localhost:${PORT}/wall-demolition.html`);
  console.log(`  tower-collapse:  http://localhost:${PORT}/tower-collapse.html`);
  console.log();
  console.log('Vendor mappings:');
  console.log(`  /vendor/three/             → ${resolve(libRoot, 'node_modules/three')}`);
  console.log(`  /vendor/rapier/            → ${resolve(libRoot, 'node_modules/@dimforge/rapier3d-compat')}`);
  console.log(`  /vendor/blast-stress-solver/ → ${resolve(libRoot, 'dist')}`);
});
