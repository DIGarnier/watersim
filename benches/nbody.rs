// Headless benchmark that exercises the REAL physics engine (lolballs::physics::Physics),
// not a reimplementation. Run with:
//
//   cargo bench --no-default-features
//
// Prints a markdown table so results can be redirected into docs/benchmarks/.

use std::sync::mpsc::channel;
use std::time::Instant;

use glam::Vec2;
use lolballs::constants::{BALL_SIZE, HEIGHT, WIDTH};
use lolballs::physics::{Physics, ShareData, PHYS_TIME_STEP};

#[derive(Clone, Copy, Debug, PartialEq)]
enum ForcePath {
    /// Barnes-Hut quadtree forces (default configuration)
    BarnesHut,
    /// Verlet neighbor lists over the spatial hash
    VerletLists,
    /// Plain spatial-hash forces
    SpatialHash,
}

impl ForcePath {
    fn label(self) -> &'static str {
        match self {
            ForcePath::BarnesHut => "barnes-hut",
            ForcePath::VerletLists => "verlet-lists",
            ForcePath::SpatialHash => "spatial-hash",
        }
    }
}

/// Engine knobs under study in this pass (docs/benchmarks/12+). `None` in
/// run_config leaves the engine at its adopted defaults; sweeps pin them
/// explicitly so results don't shift when defaults are re-tuned.
#[derive(Clone, Copy)]
struct Knobs {
    theta: f32,
    fint: usize, // far-field force refresh interval (substeps)
    iters: usize,
    omega: f32,
    par_min: Option<usize>, // None = engine default crossover
    substeps: usize,
}

/// The configuration this optimization pass started from (stage-11 behavior):
/// θ = 0.5, forces recomputed every substep, 4 solver iterations at ω = 1.
const REF_KNOBS: Knobs = Knobs {
    theta: 0.5,
    fint: 1,
    iters: 4,
    omega: 1.0,
    par_min: None,
    substeps: 1,
};

/// The defaults adopted at the end of the pass (stages 13–15) — keep in sync
/// with the constants in physics.rs. Used by sweeps that pin everything but
/// the knob under study.
const ADOPTED_KNOBS: Knobs = Knobs {
    theta: 0.9,
    fint: 4,
    iters: 3,
    omega: 1.0,
    par_min: None,
    substeps: 1,
};

struct RunResult {
    particles: usize,
    path: ForcePath,
    steps: usize,
    /// mean µs/step: integrate, grid build, tree build, forces, solver
    phase_means_us: [f64; 5],
    mean_us: f64,
    median_us: f64,
    p95_us: f64,
    min_us: f64,
    max_us: f64,
    nan_count: usize,
    escaped_count: usize,
    quality: ContactQuality,
    final_pos: Vec<Vec2>,
}

/// Coarse density histogram (20 px bins) of a particle state. Robust to the
/// engine's internal array reordering, unlike per-index position diffs.
const DBIN: f32 = 20.0;
const DW: usize = (WIDTH / DBIN) as usize + 1;
const DH: usize = (HEIGHT / DBIN) as usize + 1;

fn density_hist(positions: &[Vec2]) -> Vec<u32> {
    let mut hist = vec![0u32; DW * DH];
    for p in positions {
        let x = ((p.x / DBIN) as usize).min(DW - 1);
        let y = ((p.y / DBIN) as usize).min(DH - 1);
        hist[y * DW + x] += 1;
    }
    hist
}

/// % of particles that ended up in a different 20 px bin than the reference
/// run: L1 histogram distance / 2n. 0 % = identical end state.
fn density_drift_pct(a: &[Vec2], b: &[Vec2]) -> f64 {
    let (ha, hb) = (density_hist(a), density_hist(b));
    let l1: u64 = ha
        .iter()
        .zip(&hb)
        .map(|(x, y)| (*x as i64 - *y as i64).unsigned_abs())
        .sum();
    l1 as f64 / (2.0 * a.len() as f64) * 100.0
}

/// Contact quality of the final state, all as % of the contact distance
/// (2·BALL_SIZE): worst pair overlap, mean overlap across overlapping pairs,
/// and the number of "deep" pairs (>50 % overlap — includes coincident pairs,
/// which the solver's degenerate-pair mask can never separate again).
/// Max-pen alone is a single-worst-pair metric; mean + deep count tell
/// whether contacts are globally mushy or one pair got stuck.
struct ContactQuality {
    max_pen_pct: f64,
    mean_pen_pct: f64,
    deep_pairs: usize,
}

fn contact_quality(positions: &[Vec2]) -> ContactQuality {
    let contact = 2.0 * BALL_SIZE;
    let mut max_pen = 0.0f32;
    let mut pen_sum = 0.0f64;
    let mut pen_count = 0usize;
    let mut deep_pairs = 0usize;
    for i in 0..positions.len() {
        for j in i + 1..positions.len() {
            let dist_sq = (positions[i] - positions[j]).length_squared();
            if dist_sq < contact * contact {
                let pen = contact - dist_sq.sqrt();
                max_pen = max_pen.max(pen);
                pen_sum += pen as f64;
                pen_count += 1;
                if pen > 0.5 * contact {
                    deep_pairs += 1;
                }
            }
        }
    }
    ContactQuality {
        max_pen_pct: (max_pen / contact) as f64 * 100.0,
        mean_pen_pct: pen_sum / pen_count.max(1) as f64 / contact as f64 * 100.0,
        deep_pairs,
    }
}

/// Dense block of particles resting on the floor, spaced slightly apart so the
/// collision solver and force kernels both have real work to do. Deterministic.
fn init_particles(n: usize) -> Vec<Vec2> {
    let spacing = BALL_SIZE * 2.25; // 9.0 px
    let cols = 150usize;
    let mut positions = Vec::with_capacity(n);
    for i in 0..n {
        let col = i % cols;
        let row = i / cols;
        positions.push(Vec2::new(
            12.0 + col as f32 * spacing,
            12.0 + row as f32 * spacing,
        ));
    }
    positions
}

fn run_config(
    particles: usize,
    steps: usize,
    warmup: usize,
    path: ForcePath,
    knobs: Option<Knobs>,
) -> RunResult {
    run_opts(particles, steps, warmup, path, knobs, knobs, false)
}

/// Full-control runner. `warmup_knobs` govern the warmup/settle phase,
/// `knobs` the measured phase (sweeps settle every run under identical
/// reference physics so variants A/B from the same state). `perturb` nudges
/// one particle by 1e-3 px after warmup — the chaos-floor control: how much
/// end-state drift does a physically meaningless perturbation cause?
fn run_opts(
    particles: usize,
    steps: usize,
    warmup: usize,
    path: ForcePath,
    warmup_knobs: Option<Knobs>,
    knobs: Option<Knobs>,
    perturb: bool,
) -> RunResult {
    let positions = init_particles(particles);

    let (_tx, rx) = channel();
    let mut physics = Physics::new(
        positions.clone(), // c_opos == c_pos -> particles start at rest
        vec![Vec2::ZERO; particles],
        rx,
        2000.0,
    );

    // Defaults are: barnes-hut ON, verlet ON, adaptive dt ON.
    // Adaptive dt is always disabled for deterministic timing.
    physics.toggle_adaptive_dt();
    match path {
        ForcePath::BarnesHut => {}
        ForcePath::VerletLists => {
            physics.toggle_barnes_hut();
        }
        ForcePath::SpatialHash => {
            physics.toggle_barnes_hut();
            physics.toggle_verlet_lists();
        }
    }
    let apply = |physics: &mut Physics, k: Knobs| {
        physics.set_bh_theta(k.theta);
        physics.set_force_interval(k.fint);
        physics.set_solver_iterations(k.iters);
        physics.set_solver_omega(k.omega);
        if let Some(pm) = k.par_min {
            physics.set_par_min_particles(pm);
        }
        physics.set_substeps(k.substeps);
    };
    if let Some(k) = warmup_knobs {
        apply(&mut physics, k);
    }

    let mut share = ShareData {
        c_pos: positions,
        c_color: vec![0.0; particles],
        ..Default::default()
    };

    for _ in 0..warmup {
        physics.step(PHYS_TIME_STEP, &mut share);
    }
    if perturb {
        share.c_pos[0].x += 1e-3;
    }
    if let Some(k) = knobs {
        apply(&mut physics, k);
    }

    let mut step_times_us = Vec::with_capacity(steps);
    let mut phase_sums_us = [0u64; 5]; // integrate, grid build, tree build, forces, solver
    for _ in 0..steps {
        let t = Instant::now();
        physics.step(PHYS_TIME_STEP, &mut share);
        step_times_us.push(t.elapsed().as_secs_f64() * 1e6);
        let ps = &share.perf_stats;
        phase_sums_us[0] += ps.integration_time_us;
        phase_sums_us[1] += ps.neighbor_rebuild_time_us;
        phase_sums_us[2] += ps.tree_build_time_us;
        phase_sums_us[3] += ps.force_calc_time_us;
        phase_sums_us[4] += ps.collision_time_us;
    }

    // Sanity checks: the optimizations must not blow up the simulation.
    let nan_count = share.c_pos.iter().filter(|p| !p.x.is_finite() || !p.y.is_finite()).count();
    let escaped_count = share
        .c_pos
        .iter()
        .filter(|p| p.x < -50.0 || p.x > WIDTH + 50.0 || p.y < -50.0 || p.y > HEIGHT + 50.0)
        .count();

    step_times_us.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mean_us = step_times_us.iter().sum::<f64>() / steps as f64;
    let pct = |q: f64| step_times_us[((steps as f64 - 1.0) * q) as usize];

    RunResult {
        particles,
        path,
        steps,
        phase_means_us: phase_sums_us.map(|v| v as f64 / steps as f64),
        mean_us,
        median_us: pct(0.5),
        p95_us: pct(0.95),
        min_us: step_times_us[0],
        max_us: step_times_us[steps - 1],
        nan_count,
        escaped_count,
        quality: contact_quality(&share.c_pos),
        final_pos: share.c_pos,
    }
}

/// θ accuracy study: settle a state under reference physics, then compare
/// Barnes-Hut forces at several opening angles against the exact O(n²) sum.
/// Error metric: relative RMS, sqrt(Σ|ΔF|² / Σ|F_exact|²).
fn force_error_mode() {
    println!("| particles | theta | rel RMS force error | serial traversal ms |");
    println!("|---|---|---|---|");
    for &n in &[3_000usize, 12_000] {
        let positions = init_particles(n);
        let (_tx, rx) = channel();
        let mut physics = Physics::new(positions.clone(), vec![Vec2::ZERO; n], rx, 2000.0);
        physics.toggle_adaptive_dt();
        let mut share = ShareData {
            c_pos: positions,
            c_color: vec![0.0; n],
            ..Default::default()
        };
        // Settle under reference knobs so the state is representative.
        physics.set_bh_theta(REF_KNOBS.theta);
        physics.set_force_interval(REF_KNOBS.fint);
        physics.set_solver_iterations(REF_KNOBS.iters);
        physics.set_solver_omega(REF_KNOBS.omega);
        for _ in 0..60 {
            physics.step(PHYS_TIME_STEP, &mut share);
        }

        let exact = physics.forces_direct(&share.c_pos);
        let exact_sq: f64 = exact.iter().map(|f| f.length_squared() as f64).sum();

        for &theta in &[0.3f32, 0.5, 0.7, 0.9, 1.1] {
            let t = Instant::now();
            let approx = physics.forces_barnes_hut_at(&share.c_pos, theta);
            let ms = t.elapsed().as_secs_f64() * 1e3;
            let err_sq: f64 = approx
                .iter()
                .zip(&exact)
                .map(|(a, e)| (*a - *e).length_squared() as f64)
                .sum();
            println!(
                "| {} | {:.1} | {:.2e} | {:.1} |",
                n,
                theta,
                (err_sq / exact_sq).sqrt(),
                ms
            );
        }
    }
}

/// Small Steps experiment (Macklin et al. 2019): S substeps × fewer solver
/// iterations vs 1 substep × 3 iterations, at a held far-field refresh rate
/// (force_interval scales with S). Quality read like sweep_mode: settle 1 s
/// under the adopted defaults, then measure each variant from that state.
fn small_steps_mode() {
    const STEPS: usize = 240;
    const WARMUP: usize = 480;
    let variants: &[(&str, Knobs)] = &[
        ("S=1 it=3 (adopted)", ADOPTED_KNOBS),
        ("S=1 it=1 (iteration cut only)", Knobs { iters: 1, ..ADOPTED_KNOBS }),
        ("S=2 it=1 K=8", Knobs { substeps: 2, iters: 1, fint: 8, ..ADOPTED_KNOBS }),
        ("S=2 it=2 K=8", Knobs { substeps: 2, iters: 2, fint: 8, ..ADOPTED_KNOBS }),
        ("S=3 it=1 K=12", Knobs { substeps: 3, iters: 1, fint: 12, ..ADOPTED_KNOBS }),
    ];
    let configs: &[(usize, ForcePath)] = &[
        (12_000, ForcePath::BarnesHut),
        (12_000, ForcePath::VerletLists),
        (24_000, ForcePath::SpatialHash),
    ];

    println!("| particles | path | variant | mean µs | p95 µs | Δmean vs adopted | max pen % | mean pen % | deep pairs | density drift % | sane? |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|");
    for &(n, path) in configs {
        let run = |knobs: Knobs| {
            run_opts(n, STEPS, WARMUP, path, Some(ADOPTED_KNOBS), Some(knobs), false)
        };
        let reference = run(ADOPTED_KNOBS);
        for &(label, knobs) in variants {
            let r = run(knobs);
            let sane = if r.nan_count == 0 && r.escaped_count == 0 {
                "ok".to_string()
            } else {
                format!("{} NaN, {} escaped", r.nan_count, r.escaped_count)
            };
            println!(
                "| {} | {} | {} | {:.0} | {:.0} | {:+.0}% | {:.1} | {:.2} | {} | {:.2} | {} |",
                n,
                path.label(),
                label,
                r.mean_us,
                r.p95_us,
                (r.mean_us / reference.mean_us - 1.0) * 100.0,
                r.quality.max_pen_pct,
                r.quality.mean_pen_pct,
                r.quality.deep_pairs,
                density_drift_pct(&r.final_pos, &reference.final_pos),
                sane
            );
        }
    }
}

/// Serial Gauss-Seidel vs packed Jacobi engine crossover: run the grid paths
/// at several sizes with the engine forced each way (everything else at the
/// adopted defaults) to locate where the packed engine starts winning —
/// PAR_MIN_PARTICLES. Re-run whenever the per-step cost profile changes.
fn engine_sweep_mode() {
    println!("| particles | path | serial mean µs | packed mean µs | packed wins? |");
    println!("|---|---|---|---|---|");
    for &n in &[8_000usize, 12_000, 16_000, 20_000, 24_000] {
        for path in [ForcePath::VerletLists, ForcePath::SpatialHash] {
            let run = |pm: usize| {
                run_config(
                    n,
                    150,
                    30,
                    path,
                    Some(Knobs { par_min: Some(pm), ..ADOPTED_KNOBS }),
                )
            };
            let serial = run(usize::MAX);
            let packed = run(0);
            println!(
                "| {} | {} | {:.0} | {:.0} | {} |",
                n,
                path.label(),
                serial.mean_us,
                packed.mean_us,
                if packed.mean_us < serial.mean_us { "yes" } else { "no" },
            );
        }
    }
}

/// Long-horizon stability check of the engine's adopted defaults: 2400 steps
/// (5 s simulated), sanity + contact quality at the end. Guards against slow
/// degradation modes (creeping penetration, drift into walls, NaN) that the
/// short timing windows can't see.
fn soak_mode() {
    println!("| particles | path | steps | mean µs | max pen % | mean pen % | deep pairs | sane? |");
    println!("|---|---|---|---|---|---|---|---|");
    for &(n, path) in &[
        (12_000usize, ForcePath::BarnesHut),
        (12_000, ForcePath::VerletLists),
        (16_000, ForcePath::SpatialHash),
    ] {
        let r = run_config(n, 2_400, 0, path, None);
        let sane = if r.nan_count == 0 && r.escaped_count == 0 {
            "ok".to_string()
        } else {
            format!("{} NaN, {} escaped", r.nan_count, r.escaped_count)
        };
        println!(
            "| {} | {} | {} | {:.0} | {:.1} | {:.2} | {} | {} |",
            n,
            path.label(),
            r.steps,
            r.mean_us,
            r.quality.max_pen_pct,
            r.quality.mean_pen_pct,
            r.quality.deep_pairs,
            sane
        );
    }
}

/// Knob sweep with quality metrics: each variant is compared against the
/// stage-11 reference configuration on the same scenario — median step time,
/// worst penetration, and end-state density drift.
fn sweep_mode() {
    const STEPS: usize = 240; // 0.5 s of simulated time, measured
    // Settle 1 s under identical reference physics before switching knobs,
    // so variants are compared from the same near-equilibrium state instead
    // of mid-collapse (where chaotic divergence swamps systematic error).
    const WARMUP: usize = 480;

    let bh_variants: &[(&str, Knobs)] = &[
        ("θ=0.5 K=4", Knobs { theta: 0.5, fint: 4, ..REF_KNOBS }),
        ("θ=0.7 K=1", Knobs { theta: 0.7, ..REF_KNOBS }),
        ("θ=0.9 K=1", Knobs { theta: 0.9, ..REF_KNOBS }),
        ("θ=0.9 K=4", Knobs { theta: 0.9, fint: 4, ..REF_KNOBS }),
        ("θ=0.9 K=8", Knobs { theta: 0.9, fint: 8, ..REF_KNOBS }),
        (
            "combo-safe θ=0.9 K=4 it=3 ω=1.0",
            Knobs { theta: 0.9, fint: 4, iters: 3, omega: 1.0, ..REF_KNOBS },
        ),
        (
            "combo-fast θ=0.9 K=4 it=2 ω=1.25",
            Knobs { theta: 0.9, fint: 4, iters: 2, omega: 1.25, ..REF_KNOBS },
        ),
    ];
    let grid_variants: &[(&str, Knobs)] = &[
        ("K=4", Knobs { fint: 4, ..REF_KNOBS }),
        ("it=3 ω=1.0", Knobs { iters: 3, ..REF_KNOBS }),
        ("it=2 ω=1.0", Knobs { iters: 2, ..REF_KNOBS }),
        ("it=2 ω=1.25", Knobs { iters: 2, omega: 1.25, ..REF_KNOBS }),
        ("it=2 ω=1.5", Knobs { iters: 2, omega: 1.5, ..REF_KNOBS }),
        ("it=1 ω=1.5", Knobs { iters: 1, omega: 1.5, ..REF_KNOBS }),
        ("it=1 ω=2.0", Knobs { iters: 1, omega: 2.0, ..REF_KNOBS }),
        (
            "combo-safe K=4 it=3 ω=1.0",
            Knobs { fint: 4, iters: 3, omega: 1.0, ..REF_KNOBS },
        ),
        (
            "combo-fast K=4 it=2 ω=1.25",
            Knobs { fint: 4, iters: 2, omega: 1.25, ..REF_KNOBS },
        ),
    ];

    // 16k is the smallest size on the packed-Jacobi solver engine
    // (PAR_MIN_PARTICLES) whose scenario still fits the box — the clean
    // quality read for that engine. The 24k scenario over-fills the box (160
    // rows × 9 px > 1200 px): overflow rows wall-clamp onto identical
    // coordinates, creating permanently-coincident pairs, so its pen/deep
    // numbers measure the scenario, not the solver. 24k stays for timing
    // continuity with the earlier series.
    let configs: &[(usize, ForcePath, &[(&str, Knobs)])] = &[
        (3_000, ForcePath::BarnesHut, bh_variants),
        (12_000, ForcePath::BarnesHut, bh_variants),
        (12_000, ForcePath::VerletLists, grid_variants),
        (16_000, ForcePath::SpatialHash, grid_variants),
        (24_000, ForcePath::SpatialHash, grid_variants),
    ];

    println!("| particles | path | variant | mean µs | median µs | p95 µs | Δmean vs ref | max pen % | mean pen % | deep pairs | density drift % | sane? |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|---|");
    for &(n, path, variants) in configs {
        let run = |knobs: Knobs, perturb: bool| {
            run_opts(n, STEPS, WARMUP, path, Some(REF_KNOBS), Some(knobs), perturb)
        };
        let reference = run(REF_KNOBS, false);
        let print_row = |label: &str, r: &RunResult| {
            let sane = if r.nan_count == 0 && r.escaped_count == 0 {
                "ok".to_string()
            } else {
                format!("{} NaN, {} escaped", r.nan_count, r.escaped_count)
            };
            println!(
                "| {} | {} | {} | {:.0} | {:.0} | {:.0} | {:+.0}% | {:.1} | {:.2} | {} | {:.2} | {} |",
                n,
                path.label(),
                label,
                r.mean_us,
                r.median_us,
                r.p95_us,
                (r.mean_us / reference.mean_us - 1.0) * 100.0,
                r.quality.max_pen_pct,
                r.quality.mean_pen_pct,
                r.quality.deep_pairs,
                density_drift_pct(&r.final_pos, &reference.final_pos),
                sane
            );
        };
        print_row("reference (stage-11)", &reference);
        // Chaos-floor control: identical physics, one particle nudged 1e-3 px.
        // Any variant whose drift is near this row is indistinguishable from
        // measurement noise on this chaotic system.
        print_row("chaos floor (ref + 1e-3 nudge)", &run(REF_KNOBS, true));
        for &(label, knobs) in variants {
            print_row(label, &run(knobs, false));
        }
    }
}

fn main() {
    // The physics thread targets 480 substeps/s, so a step must finish in
    // 1/480 s = 2083 µs to run in real time.
    const REALTIME_BUDGET_US: f64 = 1e6 / 480.0;

    let quick = std::env::args().any(|a| a == "--quick");
    let phases = std::env::args().any(|a| a == "--phases");
    if std::env::args().any(|a| a == "--force-error") {
        force_error_mode();
        return;
    }
    if std::env::args().any(|a| a == "--sweep") {
        sweep_mode();
        return;
    }
    if std::env::args().any(|a| a == "--soak") {
        soak_mode();
        return;
    }
    if std::env::args().any(|a| a == "--engine-sweep") {
        engine_sweep_mode();
        return;
    }
    if std::env::args().any(|a| a == "--small-steps") {
        small_steps_mode();
        return;
    }
    if phases {
        println!("| particles | force path | integrate | grid build | tree build | forces | solver | total step (mean) |");
        println!("|---|---|---|---|---|---|---|---|");
        for &(particles, steps) in if quick {
            &[(1_000usize, 120usize), (6_000, 40)][..]
        } else {
            &[(3_000, 200), (12_000, 60), (24_000, 40)][..]
        } {
            for path in [ForcePath::BarnesHut, ForcePath::VerletLists, ForcePath::SpatialHash] {
                let r = run_config(particles, steps, steps / 10 + 5, path, None);
                let p = r.phase_means_us;
                println!(
                    "| {} | {} | {:.0} | {:.0} | {:.0} | {:.0} | {:.0} | {:.0} |",
                    particles, path.label(), p[0], p[1], p[2], p[3], p[4], r.mean_us,
                );
            }
        }
        return;
    }
    let configs: &[(usize, usize)] = if quick {
        &[(1_000, 120), (6_000, 40)]
    } else {
        &[(1_000, 300), (3_000, 200), (6_000, 120), (12_000, 60), (24_000, 40)]
    };

    println!("| particles | force path | steps | mean µs/step | median | p95 | min | max | 480 Hz real-time? | max pen % | sane? |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|");

    for &(particles, steps) in configs {
        for path in [ForcePath::BarnesHut, ForcePath::VerletLists, ForcePath::SpatialHash] {
            let r = run_config(particles, steps, steps / 10 + 5, path, None);
            let realtime = if r.median_us <= REALTIME_BUDGET_US { "yes" } else { "NO" };
            let sane = if r.nan_count == 0 && r.escaped_count == 0 {
                "ok".to_string()
            } else {
                format!("{} NaN, {} escaped", r.nan_count, r.escaped_count)
            };
            println!(
                "| {} | {} | {} | {:.0} | {:.0} | {:.0} | {:.0} | {:.0} | {} | {:.1} | {} |",
                r.particles, r.path.label(), r.steps,
                r.mean_us, r.median_us, r.p95_us, r.min_us, r.max_us,
                realtime, r.quality.max_pen_pct, sane,
            );
        }
    }
}
