#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
"""Generate the opt-in WASM publication selector cache; all read sessions stay fresh."""
import argparse
import hashlib
import json
from pathlib import Path


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError("Publication query-cache anchor changed: " + old[:90])
    return text.replace(old, new)


def prepare_output_query_cache(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    original = source.read_bytes()
    text = original.decode("utf-8")
    text = replace_once(text, "#include <vector>\n", """#include <vector>

#if !defined(__EMSCRIPTEN__)
#error This generated publication selector cache is an isolated WASM experiment
#endif
""")
    text = replace_once(text, "struct PhysicsRead\n{", """struct PhysicsQuery
{
    ovphysx_instance_handle_t instance = 0;
    ovphysx_query_handle_t query = 0;

    ~PhysicsQuery()
    {
        if (query)
            cleanupPhysics(instance, ovphysx_release_query(instance, query));
    }
    void reset()
    {
        if (query)
            finishPhysicsRelease(instance, query, ovphysx_release_query(instance, query));
        instance = 0;
    }
};

struct PhysicsRead
{""")
    text = replace_once(text, """    explicit PhysicsRead(ovphysx_instance_handle_t value) : instance(value)
    {
    }
""", """    PhysicsQuery* retained = nullptr;

    explicit PhysicsRead(ovphysx_instance_handle_t value, PhysicsQuery* cached = nullptr)
        : instance(value), retained(cached)
    {
        if (retained && retained->query)
        {
            require(retained->instance == instance, "Publication query belongs to another instance");
            query = retained->query;
        }
    }
    bool ownsQuery() const
    {
        return !retained || retained->query != query;
    }
""")
    text = replace_once(text, """        if (read)
            cleanupPhysics(instance, ovphysx_release_read(instance, read));
        if (query)
            cleanupPhysics(instance, ovphysx_release_query(instance, query));
""", """        if (read)
            cleanupPhysics(instance, ovphysx_release_read(instance, read));
        if (query && ownsQuery())
            cleanupPhysics(instance, ovphysx_release_query(instance, query));
""")
    text = replace_once(text, """        if (read)
            finishPhysicsRelease(instance, read, ovphysx_release_read(instance, read));
        if (query)
            finishPhysicsRelease(instance, query, ovphysx_release_query(instance, query));
""", """        if (read)
            finishPhysicsRelease(instance, read, ovphysx_release_read(instance, read));
        if (query && ownsQuery())
            finishPhysicsRelease(instance, query, ovphysx_release_query(instance, query));
""")
    text = replace_once(text, """    waitPhysics(session.instance, ovphysx_query(session.instance, type, OVPHYSX_SCOPE_ALL, &session.query, nullptr));
    if (!session.query)
        return;
""", """    if (!session.query)
    {
        waitPhysics(session.instance, ovphysx_query(session.instance, type, OVPHYSX_SCOPE_ALL, &session.query, nullptr));
        if (session.retained && session.query)
        {
            session.retained->instance = session.instance;
            session.retained->query = session.query;
        }
    }
    if (!session.query)
        return;
""")
    text = replace_once(text, """    std::vector<std::unique_ptr<CacheEntry>> entries;
    std::vector<ovx_primpath_t> paths;
""", """    std::vector<std::unique_ptr<CacheEntry>> entries;
    std::vector<ovx_primpath_t> paths;
    PhysicsQuery rigidQuery;
    PhysicsQuery linkQuery;
""")
    text = replace_once(text, """void OvStageOutputCache::refresh()
{
    m_impl->entries.clear();
}""", """void OvStageOutputCache::refresh()
{
    m_impl->rigidQuery.reset();
    m_impl->linkQuery.reset();
    m_impl->entries.clear();
}""")
    text = replace_once(text, """        PhysicsRead rigid(instance);
        PhysicsRead links(instance);""", """        PhysicsRead rigid(instance, cache ? &state.rigidQuery : nullptr);
        PhysicsRead links(instance, cache ? &state.linkQuery : nullptr);""")
    output.mkdir(parents=True, exist_ok=True)
    baseline = output / "OvStageOutput.baseline.cpp"
    adapted = output / "OvStageOutput.cpp"
    if source in (baseline, adapted):
        raise ValueError("The source and generated output paths must be distinct")
    baseline.write_bytes(original)
    adapted.write_text(text)
    receipt = {
        "adaptation": "attachment-bound ALL pose selectors; fresh reads each publication",
        "source": str(source), "sourceSha256": hashlib.sha256(original).hexdigest(),
        "baseline": str(baseline), "compiledSource": str(adapted),
        "compiledSourceSha256": hashlib.sha256(adapted.read_bytes()).hexdigest(),
        "recipeSha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "defaultEnabled": False,
        "retained": ["one ALL rigid-body selector", "one ALL articulation-link selector"],
        "notRetained": ["read sessions", "read groups", "tensor pointers", "prim-list handles", "stage queries"],
        "invalidation": "refresh releases selectors; existing instance/stage/attachment binding remains; destroy before detach",
    }
    (output / "output-query-cache-overlay.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return adapted


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(prepare_output_query_cache(args.source, args.output))


if __name__ == "__main__":
    main()
