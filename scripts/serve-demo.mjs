import { createServer } from 'node:http'
import { createReadStream } from 'node:fs'
import { constants as fsConstants } from 'node:fs'
import { access, stat } from 'node:fs/promises'
import { extname, join, resolve, sep } from 'node:path'
import { fileURLToPath } from 'node:url'

const MIME_TYPES = {
    '.html': 'text/html; charset=UTF-8',
    '.css': 'text/css; charset=UTF-8',
    '.js': 'text/javascript; charset=UTF-8',
    '.mjs': 'text/javascript; charset=UTF-8',
    '.json': 'application/json; charset=UTF-8',
    '.wasm': 'application/wasm',
    '.xml': 'application/xml; charset=UTF-8',
    '.png': 'image/png',
    '.jpg': 'image/jpeg',
    '.jpeg': 'image/jpeg',
    '.gif': 'image/gif',
    '.svg': 'image/svg+xml',
    '.ico': 'image/x-icon'
}

const scriptDir = resolve(fileURLToPath(new URL('.', import.meta.url)))
const projectRoot = resolve(scriptDir, '..')
const nodeModulesDir = resolve(projectRoot, 'node_modules')

const DEFAULT_PORT = 8000

function parseArgs() {
    const args = new Map()
    for (let i = 2; i < process.argv.length; i++) {
        const arg = process.argv[i]
        if (arg.startsWith('--')) {
            const [key, value] = arg.slice(2).split('=')
            args.set(key, value ?? process.argv[++i])
        }
    }
    return args
}

const args = parseArgs()
const port = Number(args.get('port') ?? process.env.PORT ?? DEFAULT_PORT)

if (!Number.isInteger(port) || port <= 0 || port > 65535) {
    console.error('Invalid port provided. Expected an integer between 1 and 65535.')
    process.exit(1)
}

const STATIC_ALIASES = [
    { prefix: '/vendor/three/', directory: resolve(nodeModulesDir, 'three') },
    { prefix: '/vendor/rapier/', directory: resolve(nodeModulesDir, '@dimforge/rapier3d-compat') }
]

function resolveAliasedPath(pathname) {
    for (const { prefix, directory } of STATIC_ALIASES) {
        if (pathname.startsWith(prefix)) {
            const relativePart = pathname.slice(prefix.length)
            const candidate = resolve(directory, relativePart)
            const normalizedDir = directory.endsWith(sep) ? directory : directory + sep
            if (candidate.startsWith(normalizedDir) || candidate === directory) {
                return candidate
            }
        }
    }
    return null
}

const server = createServer(async (req, res) => {
    try {
        const url = new URL(req.url ?? '/', `http://${req.headers.host ?? 'localhost'}`)
        const pathname = decodeURIComponent(url.pathname)

        let resolvedPath = resolve(projectRoot, `.${pathname}`)
        let fileStat

        try {
            await access(resolvedPath, fsConstants.R_OK)
            fileStat = await stat(resolvedPath)
        } catch {
            const aliasPath = resolveAliasedPath(pathname)
            if (!aliasPath) {
                res.writeHead(404, { 'Content-Type': 'text/plain; charset=UTF-8' })
                res.end('404 Not Found')
                return
            }

            try {
                await access(aliasPath, fsConstants.R_OK)
                fileStat = await stat(aliasPath)
                resolvedPath = aliasPath
            } catch {
                res.writeHead(404, { 'Content-Type': 'text/plain; charset=UTF-8' })
                res.end('404 Not Found')
                return
            }
        }

        if (fileStat.isDirectory()) {
            resolvedPath = join(resolvedPath, 'index.html')
            try {
                await access(resolvedPath, fsConstants.R_OK)
                fileStat = await stat(resolvedPath)
            } catch {
                res.writeHead(403, { 'Content-Type': 'text/plain; charset=UTF-8' })
                res.end('Directory listing is disabled')
                return
            }
        }

        const ext = extname(resolvedPath).toLowerCase()
        const contentType = MIME_TYPES[ext] ?? 'application/octet-stream'

        res.writeHead(200, {
            'Content-Type': contentType,
            'Content-Length': fileStat.size
        })

        createReadStream(resolvedPath).pipe(res)
    } catch (error) {
        console.error(error)
        res.writeHead(500, { 'Content-Type': 'text/plain; charset=UTF-8' })
        res.end('500 Internal Server Error')
    }
})

server.listen(port, () => {
    console.log(`Serving ${projectRoot} at http://localhost:${port}`)
    console.log('Press Ctrl+C to stop.')
})

