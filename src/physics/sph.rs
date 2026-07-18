//! Shared SPH plumbing for the pressure-projection solvers: a runtime-sized
//! counting-sort neighbor grid and the 2D smoothing kernels. Kept separate
//! from PBF's hand-rolled `PbfGrid` (which predates this module and stays as
//! it is, validated) so new SPH strategies don't each re-copy the same grid.

use glam::Vec2;

use crate::constants::{HEIGHT, WIDTH};

/// 2D SPH smoothing kernels with support radius `h`: poly6 for density,
/// spiky-gradient for forces (its gradient stays large as r→0, so the pressure
/// solve pushes near-coincident particles apart instead of collapsing them).
#[derive(Clone, Copy)]
pub struct SphKernel {
    h: f32,
    h2: f32,
    poly6: f32,
    spiky_grad: f32,
}

impl SphKernel {
    pub fn new(h: f32) -> Self {
        let pi = std::f32::consts::PI;
        let h2 = h * h;
        Self {
            h,
            h2,
            poly6: 4.0 / (pi * h2 * h2 * h2 * h2),  // 4/(π h^8)
            spiky_grad: -30.0 / (pi * h2 * h2 * h), // −30/(π h^5)
        }
    }

    /// W(r,h) = (4/π h^8)(h²−r²)³, evaluated from r². Zero beyond h.
    #[inline(always)]
    pub fn w(&self, r2: f32) -> f32 {
        if r2 >= self.h2 {
            0.0
        } else {
            let d = self.h2 - r2;
            self.poly6 * d * d * d
        }
    }

    /// ∇W wrt x_i for the offset d = x_i − x_j: (−30/π h^5)(h−r)² · d/r.
    /// Returns ZERO at r=0 (no axis) and r≥h.
    #[inline(always)]
    pub fn grad(&self, d: Vec2) -> Vec2 {
        let r2 = d.length_squared();
        if r2 >= self.h2 || r2 < 1e-12 {
            return Vec2::ZERO;
        }
        let r = r2.sqrt();
        let f = self.spiky_grad * (self.h - r) * (self.h - r) / r;
        d * f
    }

    /// Rest density of a square lattice at spacing `s` (poly6 sum incl. self).
    pub fn rest_density(&self, s: f32) -> f32 {
        let reach = (self.h / s).ceil() as i32 + 1;
        let mut rho = 0.0;
        for iy in -reach..=reach {
            for ix in -reach..=reach {
                let r2 = ((ix * ix + iy * iy) as f32) * s * s;
                rho += self.w(r2);
            }
        }
        rho
    }
}

/// Uniform grid at a runtime cell size, counting-sort CSR layout (Green 2010,
/// Hoetzlein 2014). Cell size is set to the SPH support radius h so the 3×3
/// block around a particle's cell contains every neighbour within h.
#[derive(Default)]
pub struct SphGrid {
    cell: f32,
    w: usize,
    h: usize,
    cell_start: Vec<u32>, // w*h + 1 offsets
    cursor: Vec<u32>,
    cell_of: Vec<u32>,
    indices: Vec<u32>,
}

impl SphGrid {
    pub fn new(cell_size: f32) -> Self {
        let w = (WIDTH / cell_size).ceil() as usize + 1;
        let h = (HEIGHT / cell_size).ceil() as usize + 1;
        Self {
            cell: cell_size,
            w,
            h,
            cell_start: vec![0; w * h + 1],
            cursor: vec![0; w * h],
            cell_of: Vec::new(),
            indices: Vec::new(),
        }
    }

    #[inline(always)]
    fn coord(&self, p: Vec2) -> (usize, usize) {
        let x = ((p.x / self.cell) as isize).clamp(0, self.w as isize - 1) as usize;
        let y = ((p.y / self.cell) as isize).clamp(0, self.h as isize - 1) as usize;
        (x, y)
    }

    pub fn build(&mut self, positions: &[Vec2]) {
        let n = positions.len();
        let ncells = self.w * self.h;
        self.cell_of.resize(n, 0);
        self.indices.resize(n, 0);
        self.cursor.fill(0);

        for (i, p) in positions.iter().enumerate() {
            let (x, y) = self.coord(*p);
            let c = (y * self.w + x) as u32;
            self.cell_of[i] = c;
            self.cursor[c as usize] += 1;
        }

        let mut sum = 0u32;
        for c in 0..ncells {
            let count = self.cursor[c];
            self.cell_start[c] = sum;
            self.cursor[c] = sum;
            sum += count;
        }
        self.cell_start[ncells] = sum;

        for i in 0..n {
            let c = self.cell_of[i] as usize;
            self.indices[self.cursor[c] as usize] = i as u32;
            self.cursor[c] += 1;
        }
    }

    /// Invoke `f(j)` for every particle in the 3×3 cell block around `pos`.
    #[inline(always)]
    pub fn for_neighbors(&self, pos: Vec2, mut f: impl FnMut(usize)) {
        let (cx, cy) = self.coord(pos);
        let y0 = cy.saturating_sub(1);
        let y1 = (cy + 1).min(self.h - 1);
        let x0 = cx.saturating_sub(1);
        let x1 = (cx + 1).min(self.w - 1);
        for yy in y0..=y1 {
            let base = yy * self.w;
            let s = self.cell_start[base + x0] as usize;
            let e = self.cell_start[base + x1 + 1] as usize;
            for &j in &self.indices[s..e] {
                f(j as usize);
            }
        }
    }
}
