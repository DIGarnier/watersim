const PHYS_TIME_STEP: f32 = 1.0 / 480.0;

const WIDTH: f32 = 1500.0;
const HEIGHT: f32 = 1200.0;
const GRID_SIZE: f32 = 10.0;
const X_LEN: f32 = WIDTH / GRID_SIZE;
const Y_LEN: f32 = HEIGHT / GRID_SIZE;

const LEFT_WALL: f32 = 0.;
const RIGHT_WALL: f32 = WIDTH;
const BOTTOM_WALL: f32 = 0.;
const TOP_WALL: f32 = HEIGHT;

const BALL_SIZE: f32 = 4.0;
const INITIAL_BALL_SPEED: f32 = 5.0;

const BACKGROUND_COLOR: Color = Color::new(0., 0., 0., 0.0);

use std::path;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Mutex};

use ggez::conf::{WindowMode, WindowSetup};
use ggez::event::MouseButton;
use ggez::glam::Vec2;
use ggez::graphics::{Color, DrawMode, DrawParam, Image, InstanceArray, Mesh, Text};

use ggez::{event, graphics, Context, ContextBuilder, GameResult};

const GRAVITY: Vec2 = Vec2::new(0.0, 9.8);

#[derive(Default, Clone)]
struct ShareData {
    c_pos: Vec<Vec2>,
    c_color: Vec<f32>,
    phys_time: f32,
}

struct Physics {
    c_opos: Vec<Vec2>,
    phys_time: std::time::Instant,
    table: Vec<Vec<usize>>,
    others: Vec<usize>,
    currents: Vec<usize>,
    cannon_rx: Receiver<(Vec2, Vec2)>,
}

fn field(mut pos: Vec2) -> Vec2 {
    pos *= 0.01;
    let dx = (pos.y + pos.x).sin();
    let dy = (pos.x - pos.y).cos();
    Vec2::new(dx, dy) * 0.0
}

impl Physics {
    fn step(&mut self, dt: f32, share: &mut ShareData) {
        self.phys_time = std::time::Instant::now();
        self.euler(dt, share);
        let _nb_checks = self.check_ball_collisions(&mut share.c_pos);
    }

    fn euler(&mut self, dt: f32, share: &mut ShareData) {
        for ((c_pos, c_opos), c_color) in share
            .c_pos
            .iter_mut()
            .zip(&mut self.c_opos)
            .zip(share.c_color.iter_mut())
        {
            let oldnpos = *c_pos;
            let velocity = (*c_pos - *c_opos) * 20.;
            *c_color = (velocity.length() + 198.) % 360.;

            *c_pos = *c_pos * 2.0 - *c_opos + (field(*c_pos) + GRAVITY) * dt;
            *c_opos = oldnpos;
        }
    }

    fn check_ball_collisions(&mut self, c_pos: &mut [Vec2]) -> i32 {
        let mut nb = 0;

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
                        let pos_a = c_pos[self.currents[i]];
                        for j in i + 1..self.currents.len() {
                            let pos_b = c_pos[self.currents[j]];

                            if ball_collides(pos_a, BALL_SIZE, pos_b, BALL_SIZE) {
                                let mut col_axis = pos_a - pos_b;
                                let mvt = 0.75 * (col_axis.length() - (BALL_SIZE + BALL_SIZE));
                                col_axis = col_axis
                                    .try_normalize()
                                    .unwrap_or_else(|| Vec2::new(0., 0.));
                                c_pos[self.currents[i]] -= col_axis * mvt / 2.0;
                                c_pos[self.currents[j]] += col_axis * mvt / 2.0;
                            }
                            nb += 1;
                        }
                    }

                    for i in 0..self.currents.len() {
                        let pos_a = c_pos[self.currents[i]];
                        for j in 0..self.others.len() {
                            let pos_b = c_pos[self.others[j]];

                            if ball_collides(pos_a, BALL_SIZE, pos_b, BALL_SIZE) {
                                let mut col_axis = pos_a - pos_b;
                                let mvt = 0.75 * (col_axis.length() - (BALL_SIZE + BALL_SIZE));
                                col_axis = col_axis
                                    .try_normalize()
                                    .unwrap_or_else(|| Vec2::new(0., 0.));
                                c_pos[self.currents[i]] -= col_axis * mvt / 2.0;
                                c_pos[self.others[j]] += col_axis * mvt / 2.0;
                            }
                            nb += 1;
                        }
                    }
                }
            }

            self.check_wall_collisions(c_pos);
        }

        nb
    }

    fn check_wall_collisions(&mut self, c_pos: &mut [Vec2]) {
        for x in 0..(X_LEN) as usize {
            for &i in self.table[0 * X_LEN as usize + x].iter() {
                do_wall_collision(&mut c_pos[i], &mut self.c_opos[i]);
            }
        }

        for x in 0..(X_LEN) as usize {
            for &i in self.table[((Y_LEN - 1.0) * X_LEN) as usize + x].iter() {
                do_wall_collision(&mut c_pos[i], &mut self.c_opos[i]);
            }
        }

        for y in 0..(Y_LEN) as usize {
            for &i in self.table[y * X_LEN as usize].iter() {
                do_wall_collision(&mut c_pos[i], &mut self.c_opos[i]);
            }
        }

        for y in 0..(Y_LEN) as usize {
            for &i in self.table[y * X_LEN as usize + (X_LEN - 1.0) as usize].iter() {
                do_wall_collision(&mut c_pos[i], &mut self.c_opos[i]);
            }
        }
    }

    fn do_cannon(&mut self, dt: f32, share: &mut ShareData, start: Vec2, cannon: Vec2) {
        let c_pos = &mut share.c_pos;
        let c_color = &mut share.c_color;
        for k in 0..20 {
            self.cannon(
                -k as f32 * (2.2 * BALL_SIZE),
                0.,
                dt,
                c_pos,
                c_color,
                start,
                cannon,
            );
        }
    }

    fn cannon(
        &mut self,
        shift: f32,
        color: f32,
        dt: f32,
        c_pos: &mut Vec<Vec2>,
        c_color: &mut Vec<f32>,
        start: Vec2,
        cannon: Vec2,
    ) {
        let ball_pos = start + cannon.perp().normalize() * shift;
        let speed = cannon * INITIAL_BALL_SPEED * dt;
        let ball_opos = ball_pos - speed;

        c_pos.push(ball_pos);
        self.c_opos.push(ball_opos);
        c_color.push(color);
    }
}

fn main() -> GameResult {
    let (mut ctx, events_loop) = ContextBuilder::new("ballz", "ggez")
        .window_setup(
            WindowSetup::default()
                .title("Cached text example!")
                .vsync(false),
        )
        .window_mode(WindowMode::default().dimensions(WIDTH, HEIGHT))
        .add_resource_path(path::PathBuf::from("./resources"))
        .build()?;

    let share_data = Arc::default();

    let to_physics_thread: Arc<Mutex<ShareData>> = Arc::clone(&share_data);
    let to_draw_thread = Arc::clone(&share_data);

    let (cannon_tx, cannon_rx) = channel();

    std::thread::spawn(move || {
        let mut physics = Physics {
            c_opos: Vec::default(),
            phys_time: std::time::Instant::now(),
            table: vec![Vec::new(); ((X_LEN + 1.0) * (Y_LEN + 1.0)) as usize],
            others: Vec::with_capacity(30000),
            currents: Vec::with_capacity(30000),
            cannon_rx,
        };

        let clock = std::time::Instant::now();
        let mut phys_frame_start = clock.elapsed().as_secs_f32();
        loop {
            let dt = clock.elapsed().as_secs_f32() - phys_frame_start;
            
            if dt >= PHYS_TIME_STEP {
                let Ok(mut share) = to_physics_thread.lock() else {
                    continue;
                };

                physics.step(dt, &mut share);

                if let Ok((start, cannon)) = physics.cannon_rx.try_recv() {
                    physics.do_cannon(dt, &mut share, start, cannon);
                }

                share.phys_time = dt;
                phys_frame_start = clock.elapsed().as_secs_f32();
            }

        }
    });

    let state = MainState::new(&mut ctx, to_draw_thread, cannon_tx)?;
    event::run(ctx, events_loop, state)
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

struct MainState {
    share: Arc<Mutex<ShareData>>,
    shader: graphics::Shader,
    cannon_tx: Sender<(Vec2, Vec2)>,
    circles: InstanceArray,
    circle: Mesh,
    nb_obj: usize,
    image: Image,
    mouse_start_pos: Option<Vec2>,
    cannon: Option<Vec2>,
}

impl MainState {
    fn new(
        ctx: &mut Context,
        share: Arc<Mutex<ShareData>>,
        cannon_tx: Sender<(Vec2, Vec2)>,
    ) -> GameResult<MainState> {
        let shader = graphics::ShaderBuilder::new()
            .fragment_path("/blur.wgsl")
            .build(&ctx.gfx)?;

        let circle = graphics::Mesh::new_circle(
            ctx,
            DrawMode::fill(),
            Vec2::default(),
            BALL_SIZE + 4.0,
            0.0001,
            Color::WHITE,
        )?;

        let circles = InstanceArray::new(ctx, None);

        Ok(MainState {
            share,
            shader,
            cannon_tx,
            circles,
            circle,
            nb_obj: 0,
            image: Image::new_canvas_image(
                ctx,
                ctx.gfx.surface_format(),
                WIDTH as _,
                HEIGHT as _,
                1,
            ),
            mouse_start_pos: None,
            cannon: None,
        })
    }
}

impl event::EventHandler<ggez::GameError> for MainState {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        while ctx.time.check_update_time(60) {
            let Ok(share_data) = self.share.lock() else {
                return Ok(());
            };

            if share_data.c_pos.len() == self.nb_obj {
                for ((i, pos), h) in share_data.c_pos.iter().enumerate().zip(&share_data.c_color) {
                    let a = hsl_to_rgb(*h, 0.75, 0.5);
                    self.circles
                        .update(i as _, DrawParam::new().dest(*pos).color(a));
                }
            } else {
                self.circles.clear();
                for (pos, h) in share_data.c_pos.iter().zip(&share_data.c_color) {
                    let a = hsl_to_rgb(*h, 0.75, 0.5);
                    self.circles.push(DrawParam::new().dest(*pos).color(a));
                }
            }

            self.nb_obj = share_data.c_pos.len();

            match (self.mouse_start_pos, self.cannon) {
                (Some(start), Some(cannon)) => self.cannon_tx.send((start, cannon)).unwrap(),
                _ => {}
            }
        }

        Ok(())
    }

    fn mouse_button_down_event(
        &mut self,
        _ctx: &mut Context,
        button: MouseButton,
        x: f32,
        y: f32,
    ) -> GameResult {
        if MouseButton::Left == button {
            self.mouse_start_pos = Some((x, y).into());
        }

        Ok(())
    }

    fn mouse_button_up_event(
        &mut self,
        _ctx: &mut Context,
        button: MouseButton,
        _x: f32,
        _y: f32,
    ) -> GameResult {
        if MouseButton::Left == button {
            self.mouse_start_pos = None;
            self.cannon = None;
        }
        Ok(())
    }

    fn mouse_motion_event(
        &mut self,
        _ctx: &mut Context,
        x: f32,
        y: f32,
        _dx: f32,
        _dy: f32,
    ) -> GameResult {
        if let Some(start) = self.mouse_start_pos {
            self.cannon = Some(Vec2::new(x, y) - start);
        }

        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        let mut canvas = graphics::Canvas::from_image(ctx, self.image.clone(), BACKGROUND_COLOR);
        canvas.draw_instanced_mesh(self.circle.clone(), &self.circles, DrawParam::default());
        canvas.finish(ctx)?;

        let mut canvas = graphics::Canvas::from_frame(ctx, BACKGROUND_COLOR);

        canvas.set_shader(&self.shader);
        canvas.draw(&self.image, DrawParam::default());
        canvas.set_default_shader();

        let fps = ctx.time.fps();
        let nb_obj = self.nb_obj;
        let fps_display = Text::new(format!("FPS: {fps}\nnb obj: {nb_obj}"));

        canvas.draw(
            &fps_display,
            graphics::DrawParam::from(Vec2::new(10.0, 10.0)).color(Color::WHITE),
        );

        canvas.finish(ctx)?;
        Ok(())
    }
}

#[inline(always)]
fn ball_collides(pos_a: Vec2, scale_a: f32, pos_b: Vec2, scale_b: f32) -> bool {
    (pos_a - pos_b).length_squared() < ((scale_a + scale_b) * (scale_a + scale_b))
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

fn do_wall_collision(c_pos: &mut Vec2, c_opos: &mut Vec2) {
    let (hor, ver) = wall_collides(*c_pos, BALL_SIZE);

    let curr_vel = (*c_pos - *c_opos) * 0.4;
    use Collision::*;
    const EPS: f32 = 0.001;
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
