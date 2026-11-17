// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Small WebDriver adapter for the worker qualification harness; no shared browser/profile is used.
import { spawn } from 'node:child_process';
import { mkdtemp, rm, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import path from 'node:path';
import net from 'node:net';

export async function launchFirefox({ logPath, evaluationTransport = 'poll' } = {}) {
    if (!['poll', 'event'].includes(evaluationTransport)) throw new TypeError('Unknown Firefox evaluation transport');
    const socket = net.createServer();
    await new Promise(resolve => socket.listen(0, '127.0.0.1', resolve));
    const port = socket.address().port;
    await new Promise(resolve => socket.close(resolve));
    const profiles = await mkdtemp(path.join(tmpdir(), 'ovphysx-firefox-profiles-'));
    const driver = spawn(process.env.GECKODRIVER || '/snap/firefox/current/usr/lib/firefox/geckodriver',
        ['--host', '127.0.0.1', '--port', String(port), '--profile-root', profiles], { stdio: ['ignore', 'pipe', 'pipe'] });
    let log = '', startupError, session, capabilities;
    const append = chunk => { log = (log + chunk).slice(-262144); };
    driver.stdout.on('data', append); driver.stderr.on('data', append); driver.on('error', error => { startupError = error; });
    const request = async (method, endpoint, body, timeout = 90000) => {
        const response = await fetch('http://127.0.0.1:' + port + endpoint, { method,
            headers: { 'Content-Type': 'application/json' }, body: body === undefined ? undefined : JSON.stringify(body),
            signal: AbortSignal.timeout(timeout) });
        const result = await response.json();
        if (!response.ok) throw Error((result.value?.error || response.status) + ': ' + (result.value?.message || JSON.stringify(result)));
        return result.value;
    };
    const close = async () => {
        try { if (session) await request('DELETE', '/session/' + session, undefined, 10000); }
        finally {
            if (driver.exitCode === null && !driver.killed) driver.kill('SIGTERM');
            await Promise.race([new Promise(resolve => driver.once('exit', resolve)), new Promise(resolve => setTimeout(resolve, 3000))]);
            if (driver.exitCode === null) driver.kill('SIGKILL');
            if (logPath) await writeFile(logPath, log);
            await rm(profiles, { recursive: true, force: true });
        }
    };
    try {
        let ready = false;
        for (let attempt = 0; attempt < 100; ++attempt) {
            if (startupError) throw startupError;
            if (driver.exitCode !== null) throw Error('Geckodriver exited: ' + log);
            try { await request('GET', '/status', undefined, 500); ready = true; break; } catch {}
            await new Promise(resolve => setTimeout(resolve, 100));
        }
        if (!ready) throw Error('Geckodriver startup timeout: ' + log);
        const created = await request('POST', '/session', { capabilities: { alwaysMatch: {
            browserName: 'firefox', 'moz:firefoxOptions': {
                binary: process.env.FIREFOX_BIN || '/snap/firefox/current/usr/lib/firefox/firefox', args: ['-headless'],
            },
        } } });
        session = created.sessionId; capabilities = created.capabilities;
        await request('POST', '/session/' + session + '/timeouts', { script: 90000, pageLoad: 30000, implicit: 0 });
        const endpoint = '/session/' + session;
        return {
            evaluationTransport,
            version: async () => capabilities.browserVersion,
            close,
            async newPage() {
                const listeners = new Map();
                return {
                    on(name, handler) { listeners.set(name, handler); },
                    async goto(url) {
                        await request('POST', endpoint + '/url', { url });
                        await request('POST', endpoint + '/execute/sync', { script: "window.__qualificationErrors = []; addEventListener('error', e => window.__qualificationErrors.push(e.message));", args: [] });
                    },
                    async evaluate(callback, argument) {
                        let callbackSource = callback.toString();
                        if (evaluationTransport === 'event') {
                            if (typeof callback !== 'function') throw new TypeError('Firefox evaluation requires a serializable function');
                            callbackSource = Function.prototype.toString.call(callback);
                            // Validate serialization without executing it, before creating page state.
                            new Function('return (' + callbackSource + ');');
                        }
                        // Gecko's WebDriver sandbox does not inherit the document's import map.
                        // Execute in a real page module and read its serialized result through the DOM.
                        const script = evaluationTransport === 'event' ? `
                            const done = arguments[arguments.length - 1];
                            const node = document.createElement('script'); node.type = 'module';
                            const result = document.createElement('div'); result.hidden = true;
                            result.id = 'qualification-result-' + crypto.randomUUID();
                            const eventName = 'qualification-complete-' + crypto.randomUUID();
                            let completed = false, deadline;
                            const finish = value => {
                                if (completed) return;
                                completed = true; clearTimeout(deadline);
                                result.removeEventListener(eventName, onResult);
                                node.removeEventListener('error', onModuleError);
                                node.remove(); result.remove(); done(value);
                            };
                            const onResult = () => {
                                try { finish(JSON.parse(result.textContent)); }
                                catch (error) { finish({ok:false,error:String(error)}); }
                            };
                            const onModuleError = () => finish({ok:false,error:'Document evaluation module failed to load'});
                            result.addEventListener(eventName, onResult);
                            node.addEventListener('error', onModuleError);
                            deadline = setTimeout(() => finish({ok:false,error:'Document evaluation timed out'}), 88000);
                            document.body.appendChild(result);
                            const target = JSON.stringify(result.id), event = JSON.stringify(eventName);
                            node.textContent = 'const send = value => { const target = document.getElementById(' + target + '); if (!target) return; try { target.textContent = JSON.stringify(value); } catch (error) { target.textContent = JSON.stringify({ok:false,error:String(error),stack:error?.stack}); } target.dispatchEvent(new Event(' + event + ')); }; Promise.resolve().then(() => (' + arguments[0] + ')(' + JSON.stringify(arguments[1]) + ')).then(value => send({ok:true,value}), error => send({ok:false,error:String(error),stack:error?.stack}));';
                            document.body.appendChild(node);
                        ` : `
                            const done = arguments[arguments.length - 1];
                            const node = document.createElement('script'); node.type = 'module';
                            const result = document.createElement('div'); result.hidden = true;
                            result.id = 'qualification-result-' + crypto.randomUUID(); document.body.appendChild(result);
                            const target = JSON.stringify(result.id);
                            node.textContent = 'Promise.resolve((' + arguments[0] + ')(' + JSON.stringify(arguments[1]) + ')).then(value => document.getElementById(' + target + ').textContent = JSON.stringify({ok:true,value}), error => document.getElementById(' + target + ').textContent = JSON.stringify({ok:false,error:String(error),stack:error?.stack}));';
                            const poll = setInterval(() => {
                                if (!result.textContent) return;
                                clearInterval(poll); const value = JSON.parse(result.textContent);
                                node.remove(); result.remove(); done(value);
                            }, 10);
                            document.body.appendChild(node);
                        `;
                        const result = await request('POST', endpoint + '/execute/async', { script, args: [callbackSource, argument] });
                        if (!result.ok) throw Error(result.error + '\n' + (result.stack || ''));
                        const errors = await request('POST', endpoint + '/execute/sync', { script: 'return window.__qualificationErrors;', args: [] });
                        for (const error of errors || []) listeners.get('pageerror')?.(Error(error));
                        return result.value;
                    },
                };
            },
        };
    } catch (error) { await close(); throw error; }
}
