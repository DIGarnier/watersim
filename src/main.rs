// Defines the amount of time that should elapse between each physics step.
const TIME_STEP: f32 = 1.0 / 100.0;

const WIDTH: f32 = 1500.0;
const HEIGHT: f32 = 1200.0;
const GRID_SIZE: f32 = 10.0;
const X_LEN: f32 = WIDTH / GRID_SIZE;
const Y_LEN: f32 = HEIGHT / GRID_SIZE;

// x coordinates
const LEFT_WALL: f32 = 0.;
const RIGHT_WALL: f32 = WIDTH;
// y coordinates
const BOTTOM_WALL: f32 = 0.;
const TOP_WALL: f32 = HEIGHT;

// We set the z-value of the ball to 1 so it renders on top in the case of overlapping sprites.
const BALL_STARTING_POSITION: Vec2 = Vec2::new(LEFT_WALL + 40.0, TOP_WALL / 2.0);
const BALL_SIZE: f32 = 6.0;
const BALL_SPEED: f32 = 700.0;
const INITIAL_BALL_DIRECTION: Vec2 = Vec2::new(0.5, -0.5);

const BACKGROUND_COLOR: Color = Color::from_rgb(0., 0., 0.);

use std::ops::Mul;
use std::process::exit;
use std::thread;

use rand::prelude::*;
use speedy2d::color::Color;
use speedy2d::dimen::{Vec2, Vector2};
use speedy2d::font::{Font, TextLayout, TextOptions};
use speedy2d::shape::Rect;
use speedy2d::window::{KeyScancode, VirtualKeyCode, WindowHandler, WindowHelper};
use speedy2d::{Graphics2D, Window};

const GRAVITY: Vector2<f32> = Vec2::new(0.0, 9.8);

struct Physics {
    c_pos: Vec<Vec2>,
    c_opos: Vec<Vec2>,
    c_color: Vec<(f32, f32, f32)>,
    total_time: std::time::Instant,
    table: Vec<Vec<usize>>,
    others: Vec<usize>,
    currents: Vec<usize>,
}

impl Physics {
    
    fn transfer_data(&self) -> (Vec<Vector2<f32>>, Vec<(f32, f32, f32)>) {
        (self.c_pos.clone(), self.c_color.clone())
    }

    fn step(&mut self, dt: f32) {
        self.euler(dt);
        let nb_checks = self.check_ball_collisions();
        // let ball_time = t_euler.elapsed().as_micros() as f32 / 1000.0;
        self.check_wall_collisions();
    }

    fn euler(&mut self, dt: f32) {
        let fs = Vec2::new(0.0, 9.8);
        for (c_pos, c_opos) in self.c_pos.iter_mut().zip(&mut self.c_opos) {
            let oldnpos = *c_pos;
            *c_pos = c_pos.mul(2.0) - *c_opos + GRAVITY * dt;
            *c_opos = oldnpos;
        }
    }

    fn check_ball_collisions(&mut self) -> i32 {
        let mut nb = 0;

        let mut rng = rand::thread_rng();
        let mut ynum = (0..Y_LEN as usize).collect::<Vec<_>>();
        let mut xnum = (0..X_LEN as usize).collect::<Vec<_>>();
        ynum.shuffle(&mut rng);
        xnum.shuffle(&mut rng);

        for _ in 0..10 {
            for i in 0..(X_LEN * Y_LEN) as usize {
                self.table[i].clear();
            }

            for (i, pos_a) in self.c_pos.iter().enumerate() {
                let y = (pos_a.y / GRID_SIZE).min(Y_LEN - 1.0) as usize;
                let x = (pos_a.x / GRID_SIZE).min(X_LEN - 1.0) as usize;
                self.table[y * X_LEN as usize + x].push(i);
            }

            for y in 0..(Y_LEN) as usize {
                for x in 0..(X_LEN) as usize {
                    let x = xnum[x];
                    let y = ynum[y];
                    self.currents.clear();
                    self.others.clear();

                    self.currents.extend(&self.table[y * X_LEN as usize + x]);
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

                    for i in 0..self.currents.len() {
                        let pos_a = self.c_pos[self.currents[i]];
                        for j in i + 1..self.currents.len() {
                            let pos_b = self.c_pos[self.currents[j]];

                            if ball_collides(pos_a, BALL_SIZE, pos_b, BALL_SIZE) {
                                let mut col_axis = pos_a - pos_b;
                                let mvt = 0.75 * (col_axis.magnitude() - (BALL_SIZE + BALL_SIZE));
                                col_axis =
                                    col_axis.normalize().unwrap_or_else(|| Vec2::new(0., 0.));
                                self.c_pos[self.currents[i]] -= col_axis * mvt / 2.0;
                                self.c_pos[self.currents[j]] += col_axis * mvt / 2.0;
                            }
                            nb += 1;
                        }
                    }

                    for i in 0..self.currents.len() {
                        let pos_a = self.c_pos[self.currents[i]];
                        for j in 0..self.others.len() {
                            let pos_b = self.c_pos[self.others[j]];

                            if ball_collides(pos_a, BALL_SIZE, pos_b, BALL_SIZE) {
                                let mut col_axis = pos_a - pos_b;
                                let mvt = 0.75 * (col_axis.magnitude() - (BALL_SIZE + BALL_SIZE));
                                col_axis =
                                    col_axis.normalize().unwrap_or_else(|| Vec2::new(0., 0.));
                                self.c_pos[self.currents[i]] -= col_axis * mvt / 2.0;
                                self.c_pos[self.others[j]] += col_axis * mvt / 2.0;
                            }
                            nb += 1;
                        }
                    }
                }
            }
        }

        nb
    }

    fn check_wall_collisions(&mut self) {
        for (c_pos, c_opos) in self.c_pos.iter_mut().zip(self.c_opos.iter_mut()) {
            let (hor, ver) = wall_collides(*c_pos, BALL_SIZE);

            let curr_vel = (*c_pos - *c_opos).mul(0.4);
            use Collision::*;
            match hor {
                Some(Left) => {
                    c_pos.x = LEFT_WALL + BALL_SIZE;
                    c_opos.x = c_pos.x + curr_vel.x;
                }
                Some(Right) => {
                    c_pos.x = RIGHT_WALL - BALL_SIZE;
                    c_opos.x = c_pos.x + curr_vel.x;
                }
                _ => {}
            }

            match ver {
                Some(Bottom) => {
                    c_pos.y = BOTTOM_WALL + BALL_SIZE;
                    c_opos.y = c_pos.y + curr_vel.y;
                }
                Some(Top) => {
                    c_pos.y = TOP_WALL - BALL_SIZE;
                    c_opos.y = c_pos.y + curr_vel.y;
                }
                _ => {}
            }
        }
    }

    fn cannon(&mut self, shift: f32, color: (f32, f32, f32), dt: f32) {
        let ball_pos = BALL_STARTING_POSITION + Vec2::new(0.0, shift);
        let speed = INITIAL_BALL_DIRECTION.normalize().unwrap() * BALL_SPEED * dt;
        let ball_opos = ball_pos - speed;

        self.c_pos.push(ball_pos);
        self.c_opos.push(ball_opos);
        self.c_color.push(color);
    }
}

fn main() {
    let font = Font::new(include_bytes!("C:\\WINDOWS\\FONTS\\ARIAL.TTF")).unwrap();
    let window = Window::new_centered("Title", (WIDTH as u32, HEIGHT as u32)).unwrap();
    window.run_loop(MyWindowHandler {
        font,
        physics_thread: Physics {
            c_pos: Vec::default(),
            c_opos: Vec::default(),
            c_color: Vec::default(),
            total_time: std::time::Instant::now(),
            table: vec![Vec::new(); ((X_LEN + 1.0) * (Y_LEN + 1.0)) as usize],
            others: Vec::with_capacity(10000),
            currents: Vec::with_capacity(10000),
        },
    });
}

fn hsl_to_rgb(h: f32, s: f32, l: f32) -> (f32, f32, f32) {
    let c = (1.0 - (2.0 * l - 1.0).abs()) * s;
    let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
    let m = l - c / 2.0;

    let (r, g, b) = match h {
        h if h < 0.0 => (0.0, 0.0, 0.0),
        h if h < 60.0 => (c, x, 0.0),
        h if h < 120.0 => (x, c, 0.0),
        h if h < 180.0 => (0.0, c, x),
        h if h < 240.0 => (0.0, x, c),
        h if h < 300.0 => (x, 0.0, c),
        h if h < 360.0 => (c, 0.0, x),
        _ => (0.0, 0.0, 0.0),
    };

    let r = ((r + m) * 255.0).round();
    let b = ((b + m) * 255.0).round();
    let g = ((g + m) * 255.0).round();

    (r / 255.0, g / 255.0, b / 255.0)
}

struct MyWindowHandler {
    font: Font,
    physics_thread: Physics,
}


impl WindowHandler for MyWindowHandler {
    fn on_draw(&mut self, helper: &mut WindowHelper, graphics: &mut Graphics2D) {
        graphics.clear_screen(BACKGROUND_COLOR);
        self.physics_thread.step(TIME_STEP);
        for (pos, (h, s, l)) in self.physics_thread.c_pos.iter().zip(&self.physics_thread.c_color) {
            let (r, g, b) = hsl_to_rgb(*h, *s, *l);
            graphics.draw_circle(pos, BALL_SIZE, Color::from_rgb(r, g, b))
        }

        let total_time = self.physics_thread.total_time.elapsed().as_millis();

        let text = self.font.layout_text(
            &format!(
                "total: {:.3} ms\nnb obj: {}",
                // "total: {:.3} ms\nball: {:.3} ms\ncollision: {}\nnb obj: {}",
                total_time,
                // ball_time,
                // nb_checks,
                self.physics_thread.c_pos.len()
            ),
            28.0,
            TextOptions::new(),
        );
        self.physics_thread.total_time = std::time::Instant::now();
        let position = Vec2::new(10.0, 10.0);
        let crop_window = Rect::from_tuples((0.0, 0.0), (400.0, 400.0));
        graphics.draw_text_cropped(position, crop_window, Color::BLUE, &text);
        helper.request_redraw();
    }

    fn on_key_down(
        &mut self,
        _helper: &mut WindowHelper,
        virtual_key_code: Option<VirtualKeyCode>,
        _scancode: KeyScancode,
    ) {
        match virtual_key_code {
            Some(VirtualKeyCode::C) => {
                let (h, _, _) = self.physics_thread.c_color.last().unwrap_or_else(|| &(0., 0., 0.));
                let hsl = ((h + 1.) % 360., 0.75, 0.5);

                self.physics_thread.cannon(2.0 * (BALL_SIZE + 2.0), hsl, TIME_STEP);
                self.physics_thread.cannon(BALL_SIZE + 1.0, hsl, TIME_STEP);
                self.physics_thread.cannon(-(BALL_SIZE + 1.0), hsl, TIME_STEP);
                self.physics_thread.cannon(-2.0 * (BALL_SIZE + 2.0), hsl, TIME_STEP);
            }
            _ => {}
        }
    }
}

#[inline(always)]
fn ball_collides(pos_a: Vec2, scale_a: f32, pos_b: Vec2, scale_b: f32) -> bool {
    (pos_a - pos_b).magnitude_squared() < ((scale_a + scale_b) * (scale_a + scale_b))
}

enum Collision {
    Top,
    Bottom,
    Left,
    Right,
}

fn wall_collides(pos_a: Vec2, scale_a: f32) -> (Option<Collision>, Option<Collision>) {
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
