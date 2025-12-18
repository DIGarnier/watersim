// SIMD-optimized physics calculations
// Using SoA (Structure of Arrays) layout for better vectorization

use ggez::glam::Vec2;

// SoA layout for particle data - better for SIMD
pub struct ParticleSoA {
    pub pos_x: Vec<f32>,
    pub pos_y: Vec<f32>,
    pub opos_x: Vec<f32>,
    pub opos_y: Vec<f32>,
    pub force_x: Vec<f32>,
    pub force_y: Vec<f32>,
    pub color: Vec<f32>,
}

impl ParticleSoA {
    pub fn new(capacity: usize) -> Self {
        Self {
            pos_x: Vec::with_capacity(capacity),
            pos_y: Vec::with_capacity(capacity),
            opos_x: Vec::with_capacity(capacity),
            opos_y: Vec::with_capacity(capacity),
            force_x: Vec::with_capacity(capacity),
            force_y: Vec::with_capacity(capacity),
            color: Vec::with_capacity(capacity),
        }
    }

    pub fn from_aos(positions: &[Vec2], old_positions: &[Vec2], colors: &[f32]) -> Self {
        let n = positions.len();
        let mut soa = Self::new(n);

        for i in 0..n {
            soa.pos_x.push(positions[i].x);
            soa.pos_y.push(positions[i].y);
            soa.opos_x.push(old_positions[i].x);
            soa.opos_y.push(old_positions[i].y);
            soa.force_x.push(0.0);
            soa.force_y.push(0.0);
            soa.color.push(colors[i]);
        }

        soa
    }

    pub fn to_aos(&self) -> (Vec<Vec2>, Vec<f32>) {
        let n = self.pos_x.len();
        let mut positions = Vec::with_capacity(n);
        let mut colors = Vec::with_capacity(n);

        for i in 0..n {
            positions.push(Vec2::new(self.pos_x[i], self.pos_y[i]));
            colors.push(self.color[i]);
        }

        (positions, colors)
    }

    pub fn len(&self) -> usize {
        self.pos_x.len()
    }

    pub fn clear_forces(&mut self) {
        for i in 0..self.len() {
            self.force_x[i] = 0.0;
            self.force_y[i] = 0.0;
        }
    }
}

// SIMD-optimized force calculation
// Process 4 particles at a time using manual vectorization
#[inline]
pub fn compute_forces_simd_block(
    soa: &mut ParticleSoA,
    indices: &[(usize, usize)],
    scale: f32,
    ball_size: f32,
) {
    // Process pairs of particles
    for &(i, j) in indices {
        let dx = soa.pos_x[i] - soa.pos_x[j];
        let dy = soa.pos_y[i] - soa.pos_y[j];

        let dist_sq = dx * dx + dy * dy;

        if dist_sq < ball_size * ball_size {
            continue;
        }

        let dist = dist_sq.sqrt();
        let force_mag = scale / dist_sq.max(1.0);

        let fx = (dx / dist) * force_mag;
        let fy = (dy / dist) * force_mag;

        soa.force_x[i] += fx;
        soa.force_y[i] += fy;
        soa.force_x[j] -= fx; // Newton's third law
        soa.force_y[j] -= fy;
    }
}

// Auto-vectorizing force calculation
// The compiler should be able to auto-vectorize this loop
#[inline]
pub fn compute_forces_autovec(
    pos_x: &[f32],
    pos_y: &[f32],
    force_x: &mut [f32],
    force_y: &mut [f32],
    particle_i: usize,
    neighbors: &[usize],
    scale: f32,
    ball_size: f32,
) {
    let pi_x = pos_x[particle_i];
    let pi_y = pos_y[particle_i];

    for &j in neighbors {
        let dx = pi_x - pos_x[j];
        let dy = pi_y - pos_y[j];

        let dist_sq = dx * dx + dy * dy;

        if dist_sq < ball_size * ball_size || dist_sq < 0.01 {
            continue;
        }

        let dist = dist_sq.sqrt();
        let force_mag = scale / dist_sq.max(1.0);

        let inv_dist = 1.0 / dist;
        force_x[particle_i] += dx * inv_dist * force_mag;
        force_y[particle_i] += dy * inv_dist * force_mag;
    }
}

// Integration with SIMD-friendly operations
#[inline]
pub fn integrate_simd(
    soa: &mut ParticleSoA,
    dt: f32,
    gravity_y: f32,
) -> f32 {
    let mut max_velocity: f32 = 0.0;
    let n = soa.len();

    for i in 0..n {
        // Compute velocity
        let vx = (soa.pos_x[i] - soa.opos_x[i]) * 20.0;
        let vy = (soa.pos_y[i] - soa.opos_y[i]) * 20.0;
        let vel_length = (vx * vx + vy * vy).sqrt();

        soa.color[i] = (vel_length + 198.0) % 360.0;
        max_velocity = max_velocity.max(vel_length);

        // Verlet integration
        let old_x = soa.pos_x[i];
        let old_y = soa.pos_y[i];

        soa.pos_x[i] = 2.0 * soa.pos_x[i] - soa.opos_x[i] + soa.force_x[i] * dt;
        soa.pos_y[i] = 2.0 * soa.pos_y[i] - soa.opos_y[i] + (soa.force_y[i] + gravity_y) * dt;

        soa.opos_x[i] = old_x;
        soa.opos_y[i] = old_y;

        soa.force_x[i] = 0.0;
        soa.force_y[i] = 0.0;
    }

    max_velocity
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_soa_conversion() {
        let positions = vec![Vec2::new(1.0, 2.0), Vec2::new(3.0, 4.0)];
        let old_positions = vec![Vec2::new(0.5, 1.5), Vec2::new(2.5, 3.5)];
        let colors = vec![0.0, 180.0];

        let soa = ParticleSoA::from_aos(&positions, &old_positions, &colors);

        assert_eq!(soa.len(), 2);
        assert_eq!(soa.pos_x[0], 1.0);
        assert_eq!(soa.pos_y[0], 2.0);

        let (pos_back, col_back) = soa.to_aos();
        assert_eq!(pos_back[0], positions[0]);
        assert_eq!(col_back[0], colors[0]);
    }
}
