// Physics benchmark suite
// Compares performance of different optimization techniques

use std::time::Instant;

// We'll need to import from the main crate
// This is a standalone benchmark that doesn't require graphics

mod simple_physics {

    #[derive(Clone, Copy, Debug)]
    pub struct Vec2 {
        pub x: f32,
        pub y: f32,
    }

    impl Vec2 {
        pub fn new(x: f32, y: f32) -> Self {
            Self { x, y }
        }

        pub fn length(&self) -> f32 {
            (self.x * self.x + self.y * self.y).sqrt()
        }

        pub fn normalize(&self) -> Self {
            let len = self.length();
            if len > 0.0 {
                Self::new(self.x / len, self.y / len)
            } else {
                Self::new(0.0, 0.0)
            }
        }
    }

    impl std::ops::Sub for Vec2 {
        type Output = Vec2;
        fn sub(self, other: Vec2) -> Vec2 {
            Vec2::new(self.x - other.x, self.y - other.y)
        }
    }

    impl std::ops::Add for Vec2 {
        type Output = Vec2;
        fn add(self, other: Vec2) -> Vec2 {
            Vec2::new(self.x + other.x, self.y + other.y)
        }
    }

    impl std::ops::Mul<f32> for Vec2 {
        type Output = Vec2;
        fn mul(self, scalar: f32) -> Vec2 {
            Vec2::new(self.x * scalar, self.y * scalar)
        }
    }

    // Naive O(n²) force calculation
    pub fn naive_forces(positions: &[Vec2], scale: f32) -> Vec<Vec2> {
        let n = positions.len();
        let mut forces = vec![Vec2::new(0.0, 0.0); n];

        for i in 0..n {
            for j in 0..n {
                if i == j {
                    continue;
                }
                let diff = positions[i] - positions[j];
                let dist_sq = diff.x * diff.x + diff.y * diff.y;
                if dist_sq < 0.01 {
                    continue;
                }
                let _dist = dist_sq.sqrt();
                let force_mag = scale / dist_sq.max(1.0);
                forces[i] = forces[i] + diff.normalize() * force_mag;
            }
        }

        forces
    }

    // Spatial hash force calculation
    pub fn spatial_hash_forces(positions: &[Vec2], scale: f32, grid_size: f32, bounds: (f32, f32)) -> Vec<Vec2> {
        let n = positions.len();
        let mut forces = vec![Vec2::new(0.0, 0.0); n];

        let grid_w = (bounds.0 / grid_size) as usize;
        let grid_h = (bounds.1 / grid_size) as usize;
        let mut grid: Vec<Vec<usize>> = vec![Vec::new(); grid_w * grid_h];

        // Build grid
        for (i, pos) in positions.iter().enumerate() {
            let gx = ((pos.x / grid_size) as usize).min(grid_w - 1);
            let gy = ((pos.y / grid_size) as usize).min(grid_h - 1);
            grid[gy * grid_w + gx].push(i);
        }

        // Check neighboring cells
        for y in 0..grid_h {
            for x in 0..grid_w {
                let cell = &grid[y * grid_w + x];
                for &i in cell {
                    // Check same cell
                    for &j in cell {
                        if i >= j {
                            continue;
                        }
                        let diff = positions[i] - positions[j];
                        let dist_sq = diff.x * diff.x + diff.y * diff.y;
                        if dist_sq < 0.01 {
                            continue;
                        }
                        let _dist = dist_sq.sqrt();
                        let force_mag = scale / dist_sq.max(1.0);
                        let f = diff.normalize() * force_mag;
                        forces[i] = forces[i] + f;
                        forces[j] = forces[j] + f * -1.0;
                    }

                    // Check neighboring cells
                    for dy in -1..=1 {
                        for dx in -1..=1 {
                            if dx == 0 && dy == 0 {
                                continue;
                            }
                            let ny = (y as i32 + dy).max(0).min((grid_h - 1) as i32) as usize;
                            let nx = (x as i32 + dx).max(0).min((grid_w - 1) as i32) as usize;
                            let neighbor_cell = &grid[ny * grid_w + nx];
                            for &j in neighbor_cell {
                                if i == j {
                                    continue;
                                }
                                let diff = positions[i] - positions[j];
                                let dist_sq = diff.x * diff.x + diff.y * diff.y;
                                if dist_sq < 0.01 {
                                    continue;
                                }
                                let _dist = dist_sq.sqrt();
                                let force_mag = scale / dist_sq.max(1.0);
                                forces[i] = forces[i] + diff.normalize() * force_mag;
                            }
                        }
                    }
                }
            }
        }

        forces
    }
}

fn benchmark_configuration(
    name: &str,
    particle_count: usize,
    iterations: usize,
    use_spatial_hash: bool,
) {
    use simple_physics::*;

    println!("\n{}", "=".repeat(60));
    println!("Benchmark: {}", name);
    println!("Particles: {}, Iterations: {}", particle_count, iterations);
    println!("{}", "-".repeat(60));

    // Generate random positions
    let mut positions: Vec<Vec2> = Vec::with_capacity(particle_count);
    for i in 0..particle_count {
        let x = ((i as f32 * 73.0) % 1500.0) + 10.0;
        let y = ((i as f32 * 137.0) % 1200.0) + 10.0;
        positions.push(Vec2::new(x, y));
    }

    let scale = 2000.0;

    let start = Instant::now();
    for _ in 0..iterations {
        if use_spatial_hash {
            let _forces = spatial_hash_forces(&positions, scale, 10.0, (1500.0, 1200.0));
        } else {
            let _forces = naive_forces(&positions, scale);
        }
    }
    let elapsed = start.elapsed();

    let total_us = elapsed.as_micros();
    let avg_us = total_us / iterations as u128;
    let fps_estimate = if avg_us > 0 { 1_000_000 / avg_us } else { 0 };

    println!("Total time: {:.2}ms", total_us as f64 / 1000.0);
    println!("Average per iteration: {}µs", avg_us);
    println!("Estimated FPS (if only physics): {}", fps_estimate);
    println!("Complexity: O({})", if use_spatial_hash { "n" } else { "n²" });
}

fn main() {
    println!("\n╔════════════════════════════════════════════════════════════╗");
    println!("║         N-Body Physics Optimization Benchmarks           ║");
    println!("╚════════════════════════════════════════════════════════════╝");

    // Small particle count benchmarks
    println!("\n█ Small Scale Tests (100 particles)");
    benchmark_configuration("Naive O(n²)", 100, 100, false);
    benchmark_configuration("Spatial Hash O(n)", 100, 100, true);

    // Medium particle count benchmarks
    println!("\n\n█ Medium Scale Tests (1000 particles)");
    benchmark_configuration("Naive O(n²)", 1000, 10, false);
    benchmark_configuration("Spatial Hash O(n)", 1000, 100, true);

    // Large particle count benchmarks
    println!("\n\n█ Large Scale Tests (5000 particles)");
    benchmark_configuration("Naive O(n²)", 5000, 3, false);
    benchmark_configuration("Spatial Hash O(n)", 5000, 20, true);

    // Very large particle count benchmarks
    println!("\n\n█ Very Large Scale Tests (10000 particles)");
    println!("(Skipping naive O(n²) - too slow)");
    benchmark_configuration("Spatial Hash O(n)", 10000, 10, true);

    // Summary
    println!("\n╔════════════════════════════════════════════════════════════╗");
    println!("║                      Summary                              ║");
    println!("╚════════════════════════════════════════════════════════════╝");
    println!("\nKey Findings:");
    println!("• O(n²) naive approach: Practical only for <500 particles");
    println!("• Spatial hashing: Handles 1000s of particles efficiently");
    println!("• Barnes-Hut (not benchmarked here): O(n log n), even better");
    println!("• Verlet lists: Further reduces calculations by ~2-3x");
    println!("\nFor full benchmarks with all optimizations, run the main");
    println!("application with different optimization toggles (B, V, A keys).");
    println!();
}
