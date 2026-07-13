pub const WIDTH: f32 = 1500.0;
pub const HEIGHT: f32 = 1200.0;
// Neighbor-grid cell size. Must be >= the interaction radius of the grid
// paths (contact 2*BALL_SIZE = 6 px, plus the 1.5 px Verlet skin = 7.5 px)
// for the one-cell stencil to stay exact. Retuned 10 -> 7.5 when BALL_SIZE
// dropped to 3: smaller cells cut each 3x3 stencil's candidate area to 0.56x
// (docs/benchmarks/19-grid-retune.md). 1500/7.5 = 200 and 1200/7.5 = 160,
// so the box tiles exactly.
pub const GRID_SIZE: f32 = 7.5;
pub const X_LEN: f32 = WIDTH / GRID_SIZE;
pub const Y_LEN: f32 = HEIGHT / GRID_SIZE;
// Particle radius. Reduced 4.0 -> 3.0 in the second optimization pass so
// dense scenarios (24k particles) fit the box instead of over-filling it and
// wall-clamping overflow rows onto coincident coordinates
// (docs/benchmarks/17-smaller-particles.md).
pub const BALL_SIZE: f32 = 3.0;
pub const INITIAL_BALL_SPEED_MODIFIER: f32 = 5.0;