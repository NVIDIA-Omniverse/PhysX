use std::env;
use std::fs;
use std::path::{Path, PathBuf};

const LIB_BASENAME: &str = "blast_stress_solver_ffi";
const BUNDLED_NATIVE_ROOT: &str = "native/blast";

const BRIDGE_SOURCES: &[&str] = &[
    "rust_stress_example/ffi/stress_bridge.cpp",
    "rust_stress_example/ffi/ext_stress_bridge.cpp",
];

const BLAST_SOURCES: &[&str] = &[
    "source/shared/stress_solver/stress.cpp",
    "source/sdk/common/NvBlastAssert.cpp",
    "source/sdk/common/NvBlastAtomic.cpp",
    "source/sdk/common/NvBlastTime.cpp",
    "source/sdk/common/NvBlastTimers.cpp",
    "source/sdk/globals/NvBlastGlobals.cpp",
    "source/sdk/globals/NvBlastInternalProfiler.cpp",
    "source/sdk/lowlevel/NvBlastAsset.cpp",
    "source/sdk/lowlevel/NvBlastAssetHelper.cpp",
    "source/sdk/lowlevel/NvBlastFamily.cpp",
    "source/sdk/lowlevel/NvBlastFamilyGraph.cpp",
    "source/sdk/lowlevel/NvBlastActor.cpp",
    "source/sdk/lowlevel/NvBlastActorSerializationBlock.cpp",
    "source/sdk/extensions/stress/NvBlastExtStressSolver.cpp",
];

const INCLUDE_DIRS: &[&str] = &[
    "include",
    "include/globals",
    "include/lowlevel",
    "include/extensions/stress",
    "include/shared/NvFoundation",
    "source/shared",
    "source/shared/stress_solver",
    "source/sdk/common",
    "source/sdk/lowlevel",
    "source/shared/NsFoundation/include",
    "rust_stress_example/ffi",
];

fn main() {
    println!("cargo:rerun-if-env-changed=BLAST_STRESS_SOLVER_LIB_DIR");
    println!("cargo:rerun-if-env-changed=BLAST_STRESS_SOLVER_STATIC_LIB_PATH");
    println!("cargo:rerun-if-env-changed=BLAST_STRESS_SOLVER_FORCE_SOURCE_BUILD");

    if env::var_os("DOCS_RS").is_some() {
        println!("cargo:rustc-cfg=docsrs");
        println!("cargo:rerun-if-env-changed=DOCS_RS");
        return;
    }

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let target = env::var("TARGET").expect("TARGET");
    let force_source_build = env_truthy("BLAST_STRESS_SOLVER_FORCE_SOURCE_BUILD");

    if let Some(lib_path) = env::var_os("BLAST_STRESS_SOLVER_STATIC_LIB_PATH") {
        let lib_path = PathBuf::from(lib_path);
        println!("cargo:rerun-if-changed={}", lib_path.display());
        emit_link_for_path(&lib_path);
        emit_cpp_runtime(&target);
        return;
    }

    if let Some(lib_dir) = env::var_os("BLAST_STRESS_SOLVER_LIB_DIR") {
        let lib_dir = PathBuf::from(lib_dir);
        println!("cargo:rustc-link-search=native={}", lib_dir.display());
        println!("cargo:rustc-link-lib=static={LIB_BASENAME}");
        emit_cpp_runtime(&target);
        return;
    }

    let packaged_dir = manifest_dir.join("artifacts").join(&target);
    emit_rerun_if_changed(&packaged_dir);

    if !force_source_build {
        if let Some(path) = find_packaged_library(&packaged_dir) {
            emit_link_for_path(&path);
            emit_cpp_runtime(&target);
            return;
        }
    }

    if native_source_build_supported(&target) {
        let bundled_native_root = manifest_dir.join(BUNDLED_NATIVE_ROOT);
        emit_rerun_if_changed(&bundled_native_root);
        if bundled_native_root.exists() {
            build_backend_sources(&bundled_native_root);
            return;
        }

        if let Some(monorepo_blast_root) = find_monorepo_blast_root(&manifest_dir) {
            emit_rerun_if_changed(&monorepo_blast_root);
            build_backend_sources(&monorepo_blast_root);
            return;
        }
    }

    let supported = supported_targets(manifest_dir.join("artifacts"));
    let native_help = if find_monorepo_blast_root(&manifest_dir).is_some() {
        " Native Apple/Linux builds from the monorepo checkout are supported; wasm still requires packaged artifacts."
    } else if manifest_dir.join(BUNDLED_NATIVE_ROOT).exists() {
        " Bundled source fallback is available for Apple/Linux native targets with a C++17 toolchain."
    } else {
        ""
    };

    if force_source_build {
        panic!(
            "blast-stress-solver source build was forced, but target {} has no supported source-build fallback.{}",
            target, native_help
        );
    }

    panic!(
        "blast-stress-solver has no packaged backend for target {}. Supported packaged targets: {}.{}",
        target, supported, native_help
    );
}

fn env_truthy(name: &str) -> bool {
    matches!(
        env::var(name).ok().as_deref(),
        Some("1" | "true" | "TRUE" | "yes" | "YES" | "on" | "ON")
    )
}

fn native_source_build_supported(target: &str) -> bool {
    !target.contains("wasm32") && (target.contains("apple") || target.contains("linux"))
}

fn build_backend_sources(blast_root: &Path) {
    let mut build = cc::Build::new();
    build.cpp(true);
    build.std("c++17");

    for rel in BRIDGE_SOURCES.iter().chain(BLAST_SOURCES.iter()) {
        build.file(blast_root.join(rel));
    }

    for rel in INCLUDE_DIRS {
        build.include(blast_root.join(rel));
    }

    build.define("STRESS_SOLVER_FORCE_SCALAR", None);
    build.define("STRESS_SOLVER_NO_SIMD", None);
    build.define("STRESS_SOLVER_NO_DEVICE_QUERY", None);
    build.define("NDEBUG", None);
    build.flag_if_supported("-fno-exceptions");
    build.flag_if_supported("-fvisibility=hidden");
    build.flag_if_supported("-Wno-unknown-pragmas");
    build.flag_if_supported("-Wno-unused-parameter");
    build.flag_if_supported("-Wno-unused-variable");
    build.flag_if_supported("-Wno-deprecated-declarations");
    build.compile(LIB_BASENAME);
}

fn find_monorepo_blast_root(manifest_dir: &Path) -> Option<PathBuf> {
    let blast_root = manifest_dir
        .parent()
        .and_then(Path::parent)
        .map(|root| root.join("blast"))?;
    if blast_root.join("include").exists()
        && blast_root.join("source").exists()
        && blast_root.join("rust_stress_example/ffi").exists()
    {
        Some(blast_root)
    } else {
        None
    }
}

fn find_packaged_library(dir: &Path) -> Option<PathBuf> {
    for candidate in [
        format!("lib{LIB_BASENAME}.a"),
        format!("{LIB_BASENAME}.lib"),
        format!("lib{LIB_BASENAME}.dylib"),
        format!("lib{LIB_BASENAME}.so"),
    ] {
        let path = dir.join(candidate);
        if path.exists() {
            return Some(path);
        }
    }
    None
}

fn emit_link_for_path(lib_path: &Path) {
    let dir = lib_path.parent().expect("library parent");
    let stem = lib_path
        .file_stem()
        .and_then(|stem| stem.to_str())
        .expect("library stem");
    let lib_name = stem.trim_start_matches("lib");

    println!("cargo:rustc-link-search=native={}", dir.display());
    if matches!(
        lib_path.extension().and_then(|ext| ext.to_str()),
        Some("a") | Some("lib")
    ) {
        println!("cargo:rustc-link-lib=static={lib_name}");
    } else {
        println!("cargo:rustc-link-lib={lib_name}");
    }
}

fn emit_cpp_runtime(target: &str) {
    if target.contains("wasm32") {
        return;
    }

    if target.contains("apple") {
        println!("cargo:rustc-link-lib=c++");
    } else if target.contains("linux") {
        println!("cargo:rustc-link-lib=stdc++");
    }
}

fn supported_targets(artifacts_dir: PathBuf) -> String {
    let Ok(entries) = fs::read_dir(artifacts_dir) else {
        return "<none>".to_owned();
    };

    let mut targets: Vec<String> = entries
        .filter_map(|entry| entry.ok())
        .filter_map(|entry| {
            entry
                .file_type()
                .ok()
                .filter(|kind| kind.is_dir())
                .and_then(|_| entry.file_name().into_string().ok())
        })
        .collect();
    targets.sort();

    if targets.is_empty() {
        "<none>".to_owned()
    } else {
        targets.join(", ")
    }
}

fn emit_rerun_if_changed(path: &Path) {
    if let Ok(metadata) = fs::metadata(path) {
        if metadata.is_file() {
            println!("cargo:rerun-if-changed={}", path.display());
            return;
        }
        if metadata.is_dir() {
            let mut entries: Vec<PathBuf> = fs::read_dir(path)
                .expect("read_dir")
                .filter_map(|entry| entry.ok().map(|entry| entry.path()))
                .collect();
            entries.sort();
            for entry in entries {
                emit_rerun_if_changed(&entry);
            }
        }
    }
}
