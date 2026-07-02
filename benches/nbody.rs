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
    max_penetration_pct: f64,
}

/// Worst pair overlap in the final state, as % of the contact distance
/// (2·BALL_SIZE). Measures constraint-solver quality: lower is better.
fn max_penetration_pct(positions: &[Vec2]) -> f64 {
    let contact = 2.0 * BALL_SIZE;
    let mut max_pen = 0.0f32;
    for i in 0..positions.len() {
        for j in i + 1..positions.len() {
            let dist_sq = (positions[i] - positions[j]).length_squared();
            if dist_sq < contact * contact {
                max_pen = max_pen.max(contact - dist_sq.sqrt());
            }
        }
    }
    (max_pen / contact) as f64 * 100.0
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

fn run_config(particles: usize, steps: usize, warmup: usize, path: ForcePath) -> RunResult {
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

    let mut share = ShareData {
        c_pos: positions,
        c_color: vec![0.0; particles],
        ..Default::default()
    };

    for _ in 0..warmup {
        physics.step(PHYS_TIME_STEP, &mut share);
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
        max_penetration_pct: max_penetration_pct(&share.c_pos),
    }
}

fn main() {
    // The physics thread targets 480 substeps/s, so a step must finish in
    // 1/480 s = 2083 µs to run in real time.
    const REALTIME_BUDGET_US: f64 = 1e6 / 480.0;

    let quick = std::env::args().any(|a| a == "--quick");
    let phases = std::env::args().any(|a| a == "--phases");
    if phases {
        println!("| particles | force path | integrate | grid build | tree build | forces | solver | total step (mean) |");
        println!("|---|---|---|---|---|---|---|---|");
        for &(particles, steps) in if quick {
            &[(1_000usize, 120usize), (6_000, 40)][..]
        } else {
            &[(3_000, 200), (12_000, 60), (24_000, 40)][..]
        } {
            for path in [ForcePath::BarnesHut, ForcePath::VerletLists, ForcePath::SpatialHash] {
                let r = run_config(particles, steps, steps / 10 + 5, path);
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
            let r = run_config(particles, steps, steps / 10 + 5, path);
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
                realtime, r.max_penetration_pct, sane,
            );
        }
    }
}
