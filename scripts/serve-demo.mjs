import { createServer } from 'node:http'
import { createReadStream } from 'node:fs'
import { constants as fsConstants } from 'node:fs'
import { access, readdir, stat } from 'node:fs/promises'
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

const INDEX_FILES = ['index.html', 'index.htm']

function escapeHtml(value) {
    return value.replace(/[&<>"']/g, (char) => {
        switch (char) {
            case '&':
                return '&amp;'
            case '<':
                return '&lt;'
            case '>':
                return '&gt;'
            case '"':
                return '&quot;'
            case "'":
                return '&#39;'
            default:
                return char
        }
    })
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

const rapierCompatDir = resolve(nodeModulesDir, '@dimforge/rapier3d-compat')
const rapierDebugDir = resolve(projectRoot, 'deps/rapier.js/rapier-compat/builds/3d/pkg')

const STATIC_ALIASES = [
    { prefix: '/vendor/three/', directory: resolve(nodeModulesDir, 'three') },
    { prefix: '/vendor/rapier/', directory: rapierCompatDir },
    { prefix: '/vendor/rapier-debug/', directory: rapierDebugDir }
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
        const rawPathname = url.pathname
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
            let indexPath
            for (const indexName of INDEX_FILES) {
                const candidate = join(resolvedPath, indexName)
                try {
                    await access(candidate, fsConstants.R_OK)
                    indexPath = candidate
                    break
                } catch {}
            }

            if (indexPath) {
                resolvedPath = indexPath
                fileStat = await stat(resolvedPath)
            } else {
                try {
                    const directoryEntries = await readdir(resolvedPath, { withFileTypes: true })
                    const encodedDirPath = rawPathname.endsWith('/') ? rawPathname : `${rawPathname}/`
                    const displayPath = pathname.endsWith('/') ? pathname : `${pathname}/`

                    const sortedEntries = directoryEntries.sort((a, b) => a.name.localeCompare(b.name, undefined, { sensitivity: 'base' }))
                    let items = ''

                    if (encodedDirPath !== '/') {
                        const parentUrl = new URL('..', `http://localhost${encodedDirPath}`)
                        const parentPathname = parentUrl.pathname.endsWith('/') ? parentUrl.pathname : `${parentUrl.pathname}/`
                        items += `<li><a href="${parentPathname}">../</a></li>`
                    }

                    for (const entry of sortedEntries) {
                        const name = entry.name + (entry.isDirectory() ? '/' : '')
                        const href = `${encodedDirPath}${encodeURIComponent(entry.name)}${entry.isDirectory() ? '/' : ''}`
                        items += `<li><a href="${href}">${escapeHtml(name)}</a></li>`
                    }

                    const body = `<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8"><title>Index of ${escapeHtml(displayPath)}</title></head><body><h1>Index of ${escapeHtml(displayPath)}</h1><ul>${items}</ul></body></html>`

                    res.writeHead(200, { 'Content-Type': 'text/html; charset=UTF-8' })
                    res.end(body)
                } catch {
                    res.writeHead(403, { 'Content-Type': 'text/plain; charset=UTF-8' })
                    res.end('Directory listing is disabled')
                }
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

