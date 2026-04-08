use std::env;
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let blast_root = manifest_dir.parent().expect("blast dir");
    let repo_root = blast_root.parent().expect("repo root");

    let blast = repo_root.join("blast");
    let ffi_dir = blast.join("rust_stress_example/ffi");

    // --- Source files ---
    let bridge_sources = [
        ffi_dir.join("stress_bridge.cpp"),
        ffi_dir.join("ext_stress_bridge.cpp"),
    ];

    let blast_sources = [
        // Core stress solver
        blast.join("source/shared/stress_solver/stress.cpp"),
        // SDK common
        blast.join("source/sdk/common/NvBlastAssert.cpp"),
        blast.join("source/sdk/common/NvBlastAtomic.cpp"),
        blast.join("source/sdk/common/NvBlastTime.cpp"),
        blast.join("source/sdk/common/NvBlastTimers.cpp"),
        // SDK globals
        blast.join("source/sdk/globals/NvBlastGlobals.cpp"),
        blast.join("source/sdk/globals/NvBlastInternalProfiler.cpp"),
        // SDK lowlevel
        blast.join("source/sdk/lowlevel/NvBlastAsset.cpp"),
        blast.join("source/sdk/lowlevel/NvBlastAssetHelper.cpp"),
        blast.join("source/sdk/lowlevel/NvBlastFamily.cpp"),
        blast.join("source/sdk/lowlevel/NvBlastFamilyGraph.cpp"),
        blast.join("source/sdk/lowlevel/NvBlastActor.cpp"),
        blast.join("source/sdk/lowlevel/NvBlastActorSerializationBlock.cpp"),
        // Extension: stress solver
        blast.join("source/sdk/extensions/stress/NvBlastExtStressSolver.cpp"),
    ];

    // --- Rerun-if-changed ---
    for src in bridge_sources.iter().chain(blast_sources.iter()) {
        println!("cargo:rerun-if-changed={}", src.display());
    }
    println!("cargo:rerun-if-changed={}", ffi_dir.join("stress_bridge.h").display());
    println!("cargo:rerun-if-changed={}", ffi_dir.join("ext_stress_bridge.h").display());

    // --- Build ---
    let mut build = cc::Build::new();
    build.cpp(true);

    for src in bridge_sources.iter().chain(blast_sources.iter()) {
        build.file(src);
    }

    // Include paths
    build.include(blast.join("include"));
    build.include(blast.join("include/globals"));
    build.include(blast.join("include/lowlevel"));
    build.include(blast.join("include/extensions/stress"));
    build.include(blast.join("include/shared/NvFoundation"));
    build.include(blast.join("source/shared"));
    build.include(blast.join("source/shared/stress_solver"));
    build.include(blast.join("source/sdk/common"));
    build.include(blast.join("source/sdk/lowlevel"));
    build.include(blast.join("source/shared/NsFoundation/include"));
    // The FFI dir for stress_bridge.h
    build.include(&ffi_dir);

    // C++17
    build.flag_if_supported("-std=c++17");
    // Scalar math (no SIMD)
    build.define("STRESS_SOLVER_FORCE_SCALAR", None);
    build.define("STRESS_SOLVER_NO_SIMD", None);
    build.define("STRESS_SOLVER_NO_DEVICE_QUERY", None);
    build.define("NDEBUG", None);
    build.flag_if_supported("-fno-exceptions");
    build.flag_if_supported("-fvisibility=hidden");

    build.compile("blast_stress_solver_ffi");
}
