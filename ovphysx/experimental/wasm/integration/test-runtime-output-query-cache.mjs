// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFile, writeFile } from 'node:fs/promises';
import { pathToFileURL } from 'node:url';
import { runOutputPublication } from './output-runner.mjs';

const [modulePath, sidePath, reportPath] = process.argv.slice(2);
if (!modulePath || !sidePath) throw Error('Usage: node test-runtime-output-query-cache.mjs main.mjs side.wasm [report.json]');
const moduleUrl = pathToFileURL(modulePath);
const { default: createModule } = await import(moduleUrl);
const create = () => createModule({ dynamicLibraries:[pathToFileURL(sidePath).href], locateFile:name => new URL(name,moduleUrl).href });
const source = await readFile(new URL('./output.usda', import.meta.url), 'utf8');
const publication = await runOutputPublication(await create(), source);
const cases = [];
for (const kind of ['rigid', 'articulation']) {
    const runtime = await create();
    const error = physics => runtime.UTF8ToString(Number(physics ? runtime._ovp_wasm_error() : runtime._ovs_wasm_error()));
    const check = (status, physics = false) => assert.equal(status,0,error(physics));
    const withText = (text, action) => {
        const bytes = new TextEncoder().encode(text), pointer = BigInt(runtime._malloc(bytes.length+1));
        try { new Uint8Array(runtime.wasmMemory.buffer,Number(pointer),bytes.length).set(bytes); return action(pointer,bytes.length); }
        finally { runtime._free(pointer); }
    };
    let stage = 0, ordinal = 1n;
    const paths = kind === 'rigid' ? ['/World/Mirrored','/World/Parent/Body'] : ['/World/Pendulum/Base','/World/Pendulum/Arm'];
    const matrix = path => withText(path,(pathPointer,pathSize) => withText('omni:fabric:worldMatrix',(namePointer,nameSize) => {
        check(runtime._ovs_wasm_read_attribute(stage,pathPointer,pathSize,namePointer,nameSize,ordinal));
        assert.equal(runtime._ovs_wasm_result_count(),16);
        assert.equal(runtime._ovs_wasm_result_bits(),64);
        return [...new Float64Array(runtime.wasmMemory.buffer,Number(runtime._ovs_wasm_result_data()),16)];
    }));
    const publish = cached => { check((cached ? runtime._ovp_wasm_publish : runtime._ovp_test_publish_uncached)(stage,++ordinal),true); check(runtime._ovs_wasm_seal(stage,ordinal)); };
    const compare = label => {
        publish(true);
        const cached = paths.map(matrix);
        publish(false);
        const fresh = paths.map(matrix);
        assert.deepEqual(cached,fresh,`${label}: reused selectors must match fresh-query publication without stepping`);
        assert.ok(fresh.flat().every(Number.isFinite));
        return { label, paths:[...paths] };
    };
    const count = () => {
        check((kind === 'rigid' ? runtime._ovp_wasm_read_vectors : runtime._ovp_wasm_read_links)(0),true);
        return runtime._ovp_wasm_value_count()/3;
    };
    const edit = command => withText(JSON.stringify(command),(ptr,size) => check(runtime._ovs_wasm_edit_usd(stage,ptr,size)));
    const sync = () => {
        check(runtime._ovp_test_refresh_output_cache(),true);
        const authored = ++ordinal;
        check(runtime._ovs_wasm_apply_usd_changes(stage,authored));
        check(runtime._ovs_wasm_seal(stage,authored));
        check(runtime._ovp_test_update_from_stage(authored,authored),true);
    };
    const createBody = path => {
        edit({operation:'createPrim',path,type:'Cube'});
        edit({operation:'configurePhysics',path,kind:'collider',enabled:true});
        edit({operation:'configurePhysics',path,kind:'rigidBody',enabled:true});
    };
    try {
        stage = runtime._ovs_wasm_create(); assert.ok(stage,error(false));
        const text = kind === 'rigid' ? source : await readFile(new URL('./articulation.usda',import.meta.url),'utf8');
        withText(text,(ptr,size) => check(runtime._ovs_wasm_open_usd_string(stage,ptr,size,ordinal,0)));
        check(runtime._ovs_wasm_seal(stage,ordinal)); check(runtime._ovp_wasm_create(stage,ordinal),true);
        assert.equal(count(),2);
        for(let tick=0;tick<8;tick++) { check(runtime._ovp_wasm_step(1/60),true); cases.push(compare(`${kind} tick ${tick}`)); }
        check(runtime._ovp_test_refresh_output_cache(),true);
        check(runtime._ovp_test_refresh_output_cache(),true);
        cases.push(compare(`${kind} repeated refresh`));
        assert.notEqual(runtime._ovp_wasm_publish(stage,ordinal),0,'Sealed ordinal must still reject');
        cases.push(compare(`${kind} recovery after rejected publication`));
        if (kind === 'rigid') {
            createBody('/World/Added'); sync(); paths.push('/World/Added');
            assert.equal(count(),3); cases.push(compare('add body within same attachment'));
            const checkpointPointer = BigInt(runtime._malloc(4));
            let checkpoint = 0;
            try {
                check(runtime._ovs_wasm_capture_usd_checkpoint(stage, checkpointPointer));
                checkpoint = new Uint32Array(runtime.wasmMemory.buffer, Number(checkpointPointer), 1)[0];
                assert.ok(checkpoint);
                edit({operation:'deletePrims',paths:['/World/Mirrored']}); sync(); paths.splice(paths.indexOf('/World/Mirrored'),1);
                assert.equal(count(),2); cases.push(compare('delete body within same attachment'));
                // USD deletion deactivates the source prim; undo is the supported same-path recreation.
                check(runtime._ovs_wasm_restore_usd_checkpoint(stage, checkpoint)); sync(); paths.push('/World/Mirrored');
                assert.equal(count(),3); cases.push(compare('recreate same prim path through undo within same attachment'));
            } finally {
                try { if (checkpoint) check(runtime._ovs_wasm_release_usd_checkpoint(stage, checkpoint)); }
                finally { runtime._free(checkpointPointer); }
            }
            edit({operation:'deletePrims',paths:[...paths]}); sync(); paths.length=0;
            assert.equal(count(),0); cases.push(compare('empty rigid selector'));
            createBody('/World/Repopulated'); sync(); paths.push('/World/Repopulated');
            assert.equal(count(),1); cases.push(compare('empty selector becomes populated'));
        }
    } finally {
        try {
            if (stage) {
                try { check(runtime._ovp_wasm_destroy(), true); }
                finally { check(runtime._ovs_wasm_destroy(stage)); }
            }
        } finally { runtime.PThread.terminateAllThreads(); }
    }
}
const mainWasm = new URL(moduleUrl); mainWasm.pathname = mainWasm.pathname.replace(/\.m?js$/,'.wasm');
const artifactSha256 = Object.fromEntries(await Promise.all([['ovstage.mjs',moduleUrl],['ovstage.wasm',mainWasm],['ovphysx.wasm',pathToFileURL(sidePath)]].map(async([name,file]) => [name,createHash('sha256').update(await readFile(file)).digest('hex')])));
const result = {passed:true,artifactSha256,publication,cases,scope:'Fresh-query parity, fresh stepping, rigid/articulation selectors, refresh, in-attachment add/delete/recreate/empty transitions, rejected output and ownership cleanup'};
if(reportPath) await writeFile(reportPath,JSON.stringify(result,null,2)+'\n');
console.log(JSON.stringify({passed:true,cases:cases.length}));
