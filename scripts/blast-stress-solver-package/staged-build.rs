use std::env;
use std::fs;
use std::path::{Path, PathBuf};

const LIB_BASENAME: &str = "blast_stress_solver_ffi";

fn main() {
    println!("cargo:rerun-if-env-changed=BLAST_STRESS_SOLVER_LIB_DIR");
    println!("cargo:rerun-if-env-changed=BLAST_STRESS_SOLVER_STATIC_LIB_PATH");

    if env::var_os("DOCS_RS").is_some() {
        println!("cargo:rustc-cfg=docsrs");
        println!("cargo:rerun-if-env-changed=DOCS_RS");
        return;
    }

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let target = env::var("TARGET").expect("TARGET");

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

    if let Some(path) = find_packaged_library(&packaged_dir) {
        emit_link_for_path(&path);
        emit_cpp_runtime(&target);
        return;
    }

    let supported = supported_targets(manifest_dir.join("artifacts"));
    panic!(
        "blast-stress-solver has no packaged backend for target {}. Supported packaged targets: {}",
        target, supported
    );
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
