use std::{
    collections::HashMap,
    fs,
    process::Command,
    sync::{Mutex, OnceLock},
};

fn headless_process_lock() -> &'static Mutex<()> {
    static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
    LOCK.get_or_init(|| Mutex::new(()))
}

fn run_headless_scenario_with_envs(
    scenario: &str,
    shot_script: &str,
    frames: u32,
    extra_envs: &[(&str, &str)],
) -> (String, HashMap<String, String>) {
    let _guard = headless_process_lock().lock().expect("lock poisoned");
    let mut command = Command::new(env!("CARGO_BIN_EXE_blast-stress-demo"));
    command
        .env("BLAST_STRESS_DEMO_HEADLESS", "1")
        .env("BLAST_STRESS_DEMO_HEADLESS_FRAMES", frames.to_string())
        .env("BLAST_STRESS_DEMO_SCENARIO", scenario)
        .env("BLAST_STRESS_DEMO_GIZMOS", "0")
        .env("BLAST_STRESS_DEMO_SHOW_MESHES", "0")
        .env("BLAST_STRESS_DEMO_HEADLESS_SHOT_SCRIPT", shot_script);
    for (key, value) in extra_envs {
        command.env(key, value);
    }
    let output = command
        .output()
        .expect("failed to run blast-stress-demo headless benchmark");

    assert!(
        output.status.success(),
        "demo failed for scenario={scenario} script={shot_script}\nstdout:\n{}\nstderr:\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );

    let stdout = String::from_utf8_lossy(&output.stdout);
    let log_path = stdout
        .lines()
        .rev()
        .find(|line| line.contains("blast-stress-demo-perf-") && line.ends_with(".log"))
        .unwrap_or_else(|| panic!("missing perf log path in stdout:\n{stdout}"));
    let log_text = fs::read_to_string(log_path)
        .unwrap_or_else(|err| panic!("failed to read perf log {log_path}: {err}"));
    let summary_line = log_text
        .lines()
        .rev()
        .find(|line| line.starts_with("[summary] "))
        .unwrap_or_else(|| panic!("missing summary line in {log_path}\n{log_text}"))
        .to_string();

    (log_text, parse_summary(summary_line))
}

fn run_headless_scenario(
    scenario: &str,
    shot_script: &str,
    frames: u32,
) -> (String, HashMap<String, String>) {
    run_headless_scenario_with_envs(scenario, shot_script, frames, &[])
}

fn parse_summary(line: impl AsRef<str>) -> HashMap<String, String> {
    line.as_ref()
        .split_whitespace()
        .skip(1)
        .filter_map(|token| {
            let (key, value) = token.split_once('=')?;
            Some((key.to_string(), value.to_string()))
        })
        .collect()
}

fn get_u64(summary: &HashMap<String, String>, key: &str) -> u64 {
    summary
        .get(key)
        .unwrap_or_else(|| panic!("missing summary key {key}"))
        .parse::<u64>()
        .unwrap_or_else(|err| panic!("failed to parse summary key {key}: {err}"))
}

fn get_f32(summary: &HashMap<String, String>, key: &str) -> f32 {
    summary
        .get(key)
        .unwrap_or_else(|| panic!("missing summary key {key}"))
        .parse::<f32>()
        .unwrap_or_else(|err| panic!("failed to parse summary key {key}: {err}"))
}

#[test]
fn headless_smoke_scripts_emit_summary_and_fracture() {
    let cases = [
        ("wall", "wall_smoke", 120u32),
        ("tower", "tower_smoke", 180u32),
        ("bridge", "bridge_smoke", 180u32),
    ];

    for (scenario, script, frames) in cases {
        let (log_text, summary) = run_headless_scenario(scenario, script, frames);
        assert_eq!(summary.get("scenario").map(String::as_str), Some(scenario));
        assert_eq!(summary.get("shot_script").map(String::as_str), Some(script));
        assert_eq!(get_u64(&summary, "shots_planned"), 3);
        assert_eq!(get_u64(&summary, "shots_fired"), 3);
        assert!(get_u64(&summary, "total_frames") >= u64::from(frames));
        assert!(
            get_u64(&summary, "total_fractures") > 0,
            "expected fractures for {scenario}"
        );
        assert!(get_f32(&summary, "max_physics_ms") > 0.0);
        assert!(get_f32(&summary, "max_rapier_ms") > 0.0);
        assert!(get_f32(&summary, "max_solver_ms") > 0.0);
        assert!(get_u64(&summary, "peak_world_bodies") > 0);
        assert_eq!(
            log_text
                .lines()
                .filter(|line| line.starts_with("[shot-fired] "))
                .count(),
            3,
            "expected exactly three fired shots for {scenario}"
        );
    }
}

#[test]
fn fractured_scene_packs_load_and_fracture_in_headless_mode() {
    let cases = [
        (
            "fractured_wall",
            "fractured-wall",
            "auto_smoke",
            160u32,
            3u64,
        ),
        (
            "fractured_tower",
            "fractured-tower",
            "auto_benchmark",
            260u32,
            5u64,
        ),
        (
            "fractured_bridge",
            "fractured-bridge",
            "auto_smoke",
            220u32,
            3u64,
        ),
        (
            "brick_building",
            "brick-building",
            "auto_smoke",
            220u32,
            3u64,
        ),
    ];

    for (scenario_env, scenario_slug, shot_script, frames, expected_shots) in cases {
        let (log_text, summary) = run_headless_scenario(scenario_env, shot_script, frames);
        assert_eq!(
            summary.get("scenario").map(String::as_str),
            Some(scenario_slug)
        );
        assert_eq!(
            summary.get("shot_script").map(String::as_str),
            Some(shot_script)
        );
        assert_eq!(get_u64(&summary, "shots_planned"), expected_shots);
        assert_eq!(get_u64(&summary, "shots_fired"), expected_shots);
        assert!(get_u64(&summary, "total_frames") >= u64::from(frames));
        assert!(
            get_u64(&summary, "total_fractures") > 0,
            "expected fractures for {scenario_slug}"
        );
        assert!(get_f32(&summary, "max_physics_ms") > 0.0);
        assert!(get_u64(&summary, "peak_world_bodies") > 0);
        assert!(
            log_text.lines().any(|line| line.starts_with("[summary] ")),
            "expected summary line for {scenario_slug}"
        );
    }
}

#[test]
fn tower_benchmark_script_emits_rich_stats() {
    let (log_text, summary) = run_headless_scenario("tower", "tower_benchmark", 220);
    assert_eq!(summary.get("scenario").map(String::as_str), Some("tower"));
    assert_eq!(
        summary.get("shot_script").map(String::as_str),
        Some("tower_benchmark")
    );
    assert_eq!(get_u64(&summary, "shots_planned"), 5);
    assert_eq!(get_u64(&summary, "shots_fired"), 5);
    assert!(get_u64(&summary, "total_fractures") > 0);
    assert!(get_f32(&summary, "max_solver_ms") > 0.0);
    let max_split_plan_ms = get_f32(&summary, "max_split_plan_ms");
    let max_split_apply_ms = get_f32(&summary, "max_split_apply_ms");
    assert!(max_split_plan_ms >= 0.0);
    assert!(max_split_apply_ms >= 0.0);
    assert!(get_u64(&summary, "peak_active_contact_pairs") > 0);
    assert!(get_u64(&summary, "peak_contact_manifolds") > 0);
    assert!(
        log_text
            .lines()
            .any(|line| line.starts_with("[fracture-frame] ")),
        "expected fracture-frame lines in tower benchmark log"
    );
    assert!(
        log_text.lines().any(|line| line.starts_with("[summary] ")),
        "expected summary line in tower benchmark log"
    );
}

#[test]
fn building_benchmark_script_emits_rich_stats() {
    let (log_text, summary) = run_headless_scenario("brick_building", "building_benchmark", 260);
    assert_eq!(
        summary.get("scenario").map(String::as_str),
        Some("brick-building")
    );
    assert_eq!(
        summary.get("shot_script").map(String::as_str),
        Some("building_benchmark")
    );
    assert_eq!(get_u64(&summary, "shots_planned"), 5);
    assert_eq!(get_u64(&summary, "shots_fired"), 5);
    assert!(get_u64(&summary, "total_fractures") > 0);
    assert!(get_u64(&summary, "total_splits") > 0);
    assert!(get_f32(&summary, "max_solver_ms") > 0.0);
    assert!(get_f32(&summary, "max_split_plan_ms") >= 0.0);
    assert!(get_f32(&summary, "max_split_apply_ms") >= 0.0);
    assert!(get_u64(&summary, "peak_world_bodies") > 0);
    assert!(
        log_text
            .lines()
            .any(|line| line.starts_with("[fracture-frame] ")),
        "expected fracture-frame lines in building benchmark log"
    );
}

#[test]
fn bridge_benchmark_script_emits_rich_stats() {
    let (log_text, summary) = run_headless_scenario("bridge", "bridge_benchmark", 220);
    assert_eq!(summary.get("scenario").map(String::as_str), Some("bridge"));
    assert_eq!(
        summary.get("shot_script").map(String::as_str),
        Some("bridge_benchmark")
    );
    assert_eq!(get_u64(&summary, "shots_planned"), 6);
    assert_eq!(get_u64(&summary, "shots_fired"), 6);
    assert!(get_u64(&summary, "total_fractures") > 0);
    assert!(get_u64(&summary, "total_splits") > 0);
    assert!(get_u64(&summary, "total_new_bodies") > 0);
    assert!(get_u64(&summary, "total_moved_colliders") > 0);
    assert!(get_f32(&summary, "max_solver_ms") > 0.0);
    assert!(get_f32(&summary, "max_split_plan_ms") > 0.0);
    assert!(get_f32(&summary, "max_split_apply_ms") > 0.0);
    assert!(get_u64(&summary, "peak_active_contact_pairs") > 0);
    assert!(get_u64(&summary, "peak_contact_manifolds") > 0);
    assert!(
        log_text
            .lines()
            .any(|line| line.starts_with("[fracture-frame] ")),
        "expected fracture-frame lines in bridge benchmark log"
    );
    assert_eq!(
        log_text
            .lines()
            .filter(|line| line.starts_with("[shot-fired] "))
            .count(),
        6,
        "expected exactly six fired shots for bridge benchmark"
    );
}

#[test]
fn wall_face_heavy_shatters_without_resim() {
    let (_log_text, summary) = run_headless_scenario_with_envs(
        "wall",
        "wall_face_heavy",
        160,
        &[
            ("BLAST_STRESS_DEMO_PROJECTILE_CCD", "0"),
            ("BLAST_STRESS_DEMO_RESIM", "0"),
            ("BLAST_STRESS_DEMO_SPLIT_RECENTER_CHILDREN", "0"),
            ("BLAST_STRESS_DEMO_SPLIT_VELOCITY_FIT", "0"),
            ("BLAST_STRESS_DEMO_SIBLING_GRACE_STEPS", "0"),
            ("BLAST_STRESS_DEMO_PROJECTILE_FRACTURE_GRACE_STEPS", "0"),
        ],
    );
    assert_eq!(summary.get("scenario").map(String::as_str), Some("wall"));
    assert_eq!(
        summary.get("shot_script").map(String::as_str),
        Some("wall_face_heavy")
    );
    assert_eq!(get_u64(&summary, "shots_planned"), 1);
    assert_eq!(get_u64(&summary, "shots_fired"), 1);
    assert!(get_u64(&summary, "total_fractures") > 0);
    assert!(get_u64(&summary, "total_splits") > 0);
    assert_eq!(get_u64(&summary, "projectile_passed_through_count"), 0);
}

#[test]
fn wall_face_heavy_with_resim_blasts_through_without_ccd() {
    let (_log_text, summary) = run_headless_scenario_with_envs(
        "wall",
        "wall_face_heavy",
        160,
        &[
            // This is the production contract for resimulation: the projectile should
            // pass through the wall because the replay frame resolves against the
            // freshly fractured topology, not because CCD tunnels it.
            ("BLAST_STRESS_DEMO_PROJECTILE_CCD", "0"),
            ("BLAST_STRESS_DEMO_RESIM", "1"),
            ("BLAST_STRESS_DEMO_MAX_RESIM_PASSES", "2"),
            ("BLAST_STRESS_DEMO_SPLIT_RECENTER_CHILDREN", "0"),
            ("BLAST_STRESS_DEMO_SPLIT_VELOCITY_FIT", "0"),
            ("BLAST_STRESS_DEMO_SIBLING_GRACE_STEPS", "0"),
            ("BLAST_STRESS_DEMO_PROJECTILE_FRACTURE_GRACE_STEPS", "0"),
        ],
    );
    assert_eq!(summary.get("scenario").map(String::as_str), Some("wall"));
    assert_eq!(
        summary.get("shot_script").map(String::as_str),
        Some("wall_face_heavy")
    );
    assert_eq!(get_u64(&summary, "shots_planned"), 1);
    assert_eq!(get_u64(&summary, "shots_fired"), 1);
    assert!(get_u64(&summary, "total_fractures") > 0);
    assert!(get_u64(&summary, "total_splits") > 0);
    assert!(
        get_u64(&summary, "projectile_passed_through_count") > 0,
        "resimulation should replay the heavy impact against the broken wall and let the projectile pass through without CCD: {summary:?}"
    );
}

#[test]
fn wall_face_heavy_passes_through_when_wall_starts_prefractured() {
    let (_log_text, summary) = run_headless_scenario_with_envs(
        "wall",
        "wall_face_heavy",
        160,
        &[
            ("BLAST_STRESS_DEMO_PREFRACTURE_ALL_BONDS", "1"),
            ("BLAST_STRESS_DEMO_PROJECTILE_CCD", "0"),
            ("BLAST_STRESS_DEMO_RESIM", "0"),
        ],
    );
    assert_eq!(summary.get("scenario").map(String::as_str), Some("wall"));
    assert_eq!(
        summary.get("shot_script").map(String::as_str),
        Some("wall_face_heavy")
    );
    assert_eq!(get_u64(&summary, "shots_planned"), 1);
    assert_eq!(get_u64(&summary, "shots_fired"), 1);
    assert_eq!(get_u64(&summary, "active_bonds_after"), 0);
    assert!(
        get_u64(&summary, "projectile_passed_through_count") > 0,
        "the same heavy projectile should pass through the wall once all bonds are pre-fractured: {summary:?}"
    );
}
