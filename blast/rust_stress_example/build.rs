use std::env;
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let repo_root = manifest_dir
        .parent()
        .expect("crate parent")
        .parent()
        .expect("repo root");

    let stress_cpp = repo_root.join("blast/source/shared/stress_solver/stress.cpp");
    let bridge_cpp = manifest_dir.join("ffi/stress_bridge.cpp");

    println!("cargo:rerun-if-changed={}", bridge_cpp.display());
    println!(
        "cargo:rerun-if-changed={}",
        manifest_dir.join("ffi/stress_bridge.h").display()
    );
    println!("cargo:rerun-if-changed={}", stress_cpp.display());

    let mut build = cc::Build::new();
    build.cpp(true);
    build.file(bridge_cpp);
    build.file(stress_cpp);
    build.include(repo_root.join("blast/include"));
    build.include(repo_root.join("blast/source/shared"));
    build.include(repo_root.join("blast/source/shared/stress_solver"));
    build.include(repo_root.join("blast/include/globals"));
    build.include(repo_root.join("blast/include/shared/NvFoundation"));
    build.flag_if_supported("-std=c++17");
    build.define("STRESS_SOLVER_FORCE_SCALAR", None);
    build.define("NDEBUG", None);
    build.flag_if_supported("-fno-exceptions");
    build.flag_if_supported("-fvisibility=hidden");
    build.compile("stress_solver_ffi");
}
