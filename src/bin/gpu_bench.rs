//! Small end-to-end CPU/GPU crossover benchmark for the live solver paths.
//! Usage: cargo run --release --no-default-features --features gpu --bin gpu_bench -- 50000 480

use std::sync::mpsc::channel;
use std::time::Instant;

use glam::Vec2;
use lolballs::constants::{HEIGHT, WIDTH};
use lolballs::physics::{Physics, ShareData, Strategy, PHYS_TIME_STEP};

fn particles(n: usize) -> Vec<Vec2> {
    // Fill the available box instead of spilling high-count rows through the
    // wall (which would benchmark a pathological pile of coincident points).
    let columns = ((n as f32 * WIDTH / HEIGHT).sqrt().ceil() as usize).max(1);
    let rows = n.div_ceil(columns).max(1);
    let sx = (WIDTH - 24.0) / columns.saturating_sub(1).max(1) as f32;
    let sy = (HEIGHT - 24.0) / rows.saturating_sub(1).max(1) as f32;
    (0..n)
        .map(|i| {
            Vec2::new(
                12.0 + (i % columns) as f32 * sx,
                12.0 + (i / columns) as f32 * sy,
            )
        })
        .collect()
}

fn run(strategy: Strategy, initial: &[Vec2], steps: usize, readback: usize) -> (f64, bool, usize) {
    std::env::set_var("WATERSIM_GPU_READBACK_INTERVAL", readback.to_string());
    let (_tx, rx) = channel();
    let mut physics = Physics::new(
        initial.to_vec(),
        vec![Vec2::ZERO; initial.len()],
        rx,
        2000.0,
    );
    physics.set_strategy(strategy);
    physics.set_adaptive_dt(false);
    let mut share = ShareData {
        c_pos: initial.to_vec(),
        c_color: vec![0.0; initial.len()],
        ..Default::default()
    };
    for _ in 0..16 {
        physics.step(PHYS_TIME_STEP, &mut share);
    }
    let started = Instant::now();
    for _ in 0..steps {
        physics.step(PHYS_TIME_STEP, &mut share);
    }
    let invalid = share
        .c_pos
        .iter()
        .filter(|p| !p.is_finite() || p.x < 0.0 || p.x > WIDTH || p.y < 0.0 || p.y > HEIGHT)
        .count();
    (
        started.elapsed().as_secs_f64() * 1e6 / steps as f64,
        share.perf_stats.gpu_enabled,
        invalid,
    )
}

fn main() {
    let mut args = std::env::args().skip(1);
    let n = args.next().and_then(|v| v.parse().ok()).unwrap_or(50_000);
    let steps = args.next().and_then(|v| v.parse().ok()).unwrap_or(240);
    let initial = particles(n);
    let (cpu, _, cpu_bad) = run(Strategy::Granular, &initial, steps, 1);
    let (gpu_sync, active, sync_bad) = run(Strategy::Gpu, &initial, steps, 1);
    let (gpu_live, _, live_bad) = run(Strategy::Gpu, &initial, steps, 8);
    println!("| particles | CPU | GPU sync/step | GPU live (sync/8) | live speedup | invalid | GPU active |");
    println!("|---:|---:|---:|---:|---:|---:|:---:|");
    println!(
        "| {n} | {cpu:.1} µs | {gpu_sync:.1} µs | {gpu_live:.1} µs | {:.2}x | {}/{}/{} | {} |",
        cpu / gpu_live,
        cpu_bad,
        sync_bad,
        live_bad,
        if active { "yes" } else { "no" }
    );
}
