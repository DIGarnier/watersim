use ggez::glam::Vec2;
use std::sync::mpsc::Receiver;

use crate::constants::{
    BALL_SIZE, GRID_SIZE, HEIGHT, INITIAL_BALL_SPEED_MODIFIER, WIDTH, X_LEN, Y_LEN,
};

const GRAVITY: Vec2 = Vec2::new(0.0, 9.8);
pub const PHYS_TIME_STEP: f32 = 1.0 / 480.0;

const LEFT_WALL: f32 = 0.;
const RIGHT_WALL: f32 = WIDTH;
const BOTTOM_WALL: f32 = 0.;
const TOP_WALL: f32 = HEIGHT;

pub enum EventToPthread {
    Cannon((Vec2, Vec2)),
    Scale(f32),
}

#[derive(Default, Clone)]
pub struct ShareData {
    pub c_pos: Vec<Vec2>,
    pub c_color: Vec<f32>,
    pub phys_time: f32,
}

fn arbitrary_vector_field(mut pos: Vec2) -> Vec2 {
    pos *= 0.01;
    let dx = pos.x.sin();
    let dy = pos.y.cos() * 10.0;
    Vec2::new(dx, dy).perp() * 0.0
}

pub struct Physics {
    c_opos: Vec<Vec2>,
    c_force: Vec<Vec2>,
    table: Vec<Vec<usize>>,
    others: Vec<usize>,
    pub rx: Receiver<EventToPthread>,
    pub scale: f32,
}

impl Physics {
    pub fn new(
        c_opos: Vec<Vec2>,
        c_force: Vec<Vec2>,
        table: Vec<Vec<usize>>,
        others: Vec<usize>,
        rx: Receiver<EventToPthread>,
        scale: f32,
    ) -> Self {
        Self {
            c_opos,
            c_force,
            table,
            others,
            rx,
            scale,
        }
    }

    pub fn step(&mut self, dt: f32, share: &mut ShareData) {
        self.integrate(dt, share);
        self.check_ball_collisions(&mut share.c_pos);
    }

    fn integrate(&mut self, dt: f32, share: &mut ShareData) {
        for i in 0..share.c_pos.len() {
            let c_pos = &mut share.c_pos[i];
            let c_opos = &mut self.c_opos[i];
            let c_color = &mut share.c_color[i];
            let c_force = &mut self.c_force[i];

            let oldnpos = *c_pos;
            let velocity = (*c_pos - *c_opos) * 20.;
            *c_color = (velocity.length() + 198.) % 360.;

            *c_pos =
                *c_pos * 2.0 - *c_opos + (arbitrary_vector_field(*c_pos) + GRAVITY + *c_force) * dt;
            *c_opos = oldnpos;
            *c_force = Vec2::ZERO;
        }
    }

    fn check_ball_collisions(&mut self, c_pos: &mut [Vec2]) {
        for _ in 0..8 {
            for i in 0..(X_LEN * Y_LEN) as usize {
                self.table[i].clear();
            }

            for (i, pos_a) in c_pos.iter().enumerate() {
                let y = (pos_a.y / GRID_SIZE).min(Y_LEN - 1.0) as usize;
                let x = (pos_a.x / GRID_SIZE).min(X_LEN - 1.0) as usize;
                self.table[y * X_LEN as usize + x].push(i);
            }

            for y in 0..(Y_LEN) as usize {
                for x in 0..(X_LEN) as usize {
                    self.others.clear();
                    let currents = &self.table[y * X_LEN as usize + x];
                    for dy in [-1, 0, 1] {
                        for dx in [-1, 0, 1] {
                            if dx == 0 && dy == 0 {
                                continue;
                            }
                            let ny = (y as i32 + dy).clamp(0, Y_LEN as i32) as usize;
                            let nx = (x as i32 + dx).clamp(0, X_LEN as i32) as usize;
                            self.others.extend(&self.table[ny * X_LEN as usize + nx]);
                        }
                    }

                    for i in 0..currents.len() {
                        let pos_a = c_pos[currents[i]];
                        for j in i + 1..currents.len() {
                            let pos_b = c_pos[currents[j]];

                            self.c_force[currents[i]] += force(pos_a, pos_b, self.scale) / 8.0;

                            if ball_collides(pos_a, BALL_SIZE, pos_b, BALL_SIZE) {
                                let mut col_axis = pos_a - pos_b;
                                let mvt = 0.75 * (col_axis.length() - (BALL_SIZE + BALL_SIZE));
                                col_axis = col_axis
                                    .try_normalize()
                                    .unwrap_or_else(|| Vec2::new(0., 0.));
                                c_pos[currents[i]] -= col_axis * mvt / 2.0;
                                c_pos[currents[j]] += col_axis * mvt / 2.0;
                            }
                        }
                    }

                    for i in 0..currents.len() {
                        let pos_a = c_pos[currents[i]];
                        for j in 0..self.others.len() {
                            let pos_b = c_pos[self.others[j]];

                            if pos_a == pos_b {
                                continue;
                            }

                            self.c_force[currents[i]] += force(pos_a, pos_b, self.scale) / 8.0;

                            if ball_collides(pos_a, BALL_SIZE, pos_b, BALL_SIZE) {
                                let mut col_axis = pos_a - pos_b;
                                let mvt = 0.75 * (col_axis.length() - (BALL_SIZE + BALL_SIZE));
                                col_axis = col_axis
                                    .try_normalize()
                                    .unwrap_or_else(|| Vec2::new(0., 0.));
                                c_pos[currents[i]] -= col_axis * mvt / 2.0;
                                c_pos[self.others[j]] += col_axis * mvt / 2.0;
                            }
                        }
                    }
                }
            }

            self.check_wall_collisions(c_pos);
        }
    }

    fn check_wall_collisions(&mut self, c_pos: &mut [Vec2]) {
        for y in 0..(Y_LEN) as usize {
            for &i in self.table[y * X_LEN as usize].iter() {
                resolve_wall_collision(&mut c_pos[i], &mut self.c_opos[i]);
            }
        }

        for y in 0..(Y_LEN) as usize {
            for &i in self.table[y * X_LEN as usize + (X_LEN - 1.0) as usize].iter() {
                resolve_wall_collision(&mut c_pos[i], &mut self.c_opos[i]);
            }
        }

        for x in 0..(X_LEN) as usize {
            for &i in self.table[x].iter() {
                resolve_wall_collision(&mut c_pos[i], &mut self.c_opos[i]);
            }
        }

        for x in 0..(X_LEN) as usize {
            for &i in self.table[((Y_LEN - 1.0) * X_LEN) as usize + x].iter() {
                resolve_wall_collision(&mut c_pos[i], &mut self.c_opos[i]);
            }
        }
    }

    pub fn do_cannon(&mut self, dt: f32, share: &mut ShareData, start: Vec2, cannon: Vec2) {
        for k in 0..20 {
            self.cannon(-k as f32 * (2.2 * BALL_SIZE), 0., dt, share, start, cannon);
        }
    }

    fn cannon(
        &mut self,
        shift: f32,
        color: f32,
        dt: f32,
        share: &mut ShareData,
        start: Vec2,
        cannon: Vec2,
    ) {
        let ball_pos = start + cannon.perp().normalize() * shift;
        let speed = cannon * INITIAL_BALL_SPEED_MODIFIER * dt;
        let ball_opos = ball_pos - speed;

        share.c_pos.push(ball_pos);
        self.c_opos.push(ball_opos);
        self.c_force.push(Vec2::ZERO);
        share.c_color.push(color);
    }
}

#[inline(always)]
fn ball_collides(pos_a: Vec2, scale_a: f32, pos_b: Vec2, scale_b: f32) -> bool {
    (pos_a - pos_b).length_squared() < ((scale_a + scale_b) * (scale_a + scale_b))
}

#[inline(always)]
fn force(pos_a: Vec2, pos_b: Vec2, scale: f32) -> Vec2 {
    let dir = pos_a - pos_b;
    let dist = dir.length();
    if dist < BALL_SIZE {
        return Vec2::ZERO;
    }

    (dir.normalize() * scale) / (dist * dist).max(1.0)
}

enum Collision {
    Top,
    Bottom,
    Left,
    Right,
}

fn collides_wall(pos_a: Vec2, scale_a: f32) -> (Option<Collision>, Option<Collision>) {
    let mut vert = None;
    if pos_a.y - scale_a <= BOTTOM_WALL {
        vert = Some(Collision::Bottom);
    } else if pos_a.y + scale_a >= TOP_WALL {
        vert = Some(Collision::Top);
    }

    let mut hor = None;
    if pos_a.x - scale_a <= LEFT_WALL {
        hor = Some(Collision::Left);
    } else if pos_a.x + scale_a >= RIGHT_WALL {
        hor = Some(Collision::Right);
    }

    (hor, vert)
}

fn resolve_wall_collision(c_pos: &mut Vec2, c_opos: &mut Vec2) {
    let (hor, ver) = collides_wall(*c_pos, BALL_SIZE);

    let curr_vel = (*c_pos - *c_opos) * 0.4;
    const EPS: f32 = 0.00001;
    use Collision::*;
    match hor {
        Some(Left) => {
            c_pos.x = LEFT_WALL + BALL_SIZE + EPS;
            c_opos.x = c_pos.x + curr_vel.x;
        }
        Some(Right) => {
            c_pos.x = RIGHT_WALL - BALL_SIZE - EPS;
            c_opos.x = c_pos.x + curr_vel.x;
        }
        _ => {}
    }

    match ver {
        Some(Bottom) => {
            c_pos.y = BOTTOM_WALL + BALL_SIZE + EPS;
            c_opos.y = c_pos.y + curr_vel.y;
        }
        Some(Top) => {
            c_pos.y = TOP_WALL - BALL_SIZE - EPS;
            c_opos.y = c_pos.y + curr_vel.y;
        }
        _ => {}
    }
}
