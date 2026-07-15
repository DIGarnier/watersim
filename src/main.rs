use lolballs::{constants, physics};

use std::path;
use std::sync::mpsc::{channel, Sender};
use std::sync::{Arc, Mutex};

use ggez::conf::{WindowMode, WindowSetup};
use ggez::event::MouseButton;
use ggez::glam::Vec2;
use ggez::graphics::{Color, DrawMode, DrawParam, Image, InstanceArray, Mesh, Text};

use constants::{BALL_SIZE, HEIGHT, WIDTH};
use ggez::winit::event::VirtualKeyCode;
use ggez::{event, graphics, Context, ContextBuilder, GameResult};
use physics::{EventToPthread, Physics, ShareData, Strategy, PHYS_TIME_STEP};

const BACKGROUND_COLOR: Color = Color::new(0., 0., 0., 0.0);

/// Parse `--sim granular|pbf` (default granular) from the command line.
fn parse_strategy() -> Strategy {
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let val = if let Some(v) = a.strip_prefix("--sim=") {
            Some(v.to_string())
        } else if a == "--sim" {
            args.next()
        } else {
            None
        };
        if let Some(v) = val {
            match Strategy::parse(&v) {
                Some(s) => return s,
                None => eprintln!("unknown --sim value '{v}', using granular"),
            }
        }
    }
    Strategy::Granular
}

fn main() -> GameResult {
    let strategy = parse_strategy();
    let choices = Strategy::all()
        .iter()
        .map(|s| s.token())
        .collect::<Vec<_>>()
        .join("|");
    println!("fluid model: {strategy:?}  (switch with --sim {choices})");
    let (mut ctx, events_loop) = ContextBuilder::new("ballz", "ggez")
        .window_setup(WindowSetup::default().vsync(false))
        .window_mode(WindowMode::default().dimensions(WIDTH, HEIGHT))
        .add_resource_path(path::PathBuf::from("./resources"))
        .build()?;

    let share_data = Arc::default();

    let to_physics_thread: Arc<Mutex<ShareData>> = Arc::clone(&share_data);

    let (tx, rx) = channel();

    std::thread::spawn(move || {
        let mut physics = Physics::new(
            Vec::with_capacity(15000),
            Vec::with_capacity(15000),
            rx,
            2000.0,
        );
        physics.set_strategy(strategy);
        // Drive a FIXED internal timestep. The old loop fed the raw, variable
        // wall-clock dt straight into the Størmer–Verlet / PBF integrators,
        // which assume a constant dt: a changing dt reinterprets the encoded
        // velocity and injects energy (the granular "breathing" wave), and the
        // adaptive-dt controller made it worse by oscillating dt. A standard
        // accumulator advances the sim in PHYS_TIME_STEP chunks to track real
        // time, exactly like the headless render tool that stays stable.
        physics.set_adaptive_dt(false);
        let debug = std::env::var("WATERSIM_DEBUG").is_ok();

        let clock = std::time::Instant::now();
        let mut last = clock.elapsed().as_secs_f32();
        let mut accumulator = 0.0f32;
        let mut step_count: u64 = 0;
        loop {
            let now = clock.elapsed().as_secs_f32();
            // Clamp the frame delta so a hitch (or a slow, particle-heavy step)
            // can't build an unbounded backlog ("spiral of death").
            accumulator += (now - last).min(0.05);
            last = now;

            if accumulator < PHYS_TIME_STEP {
                std::thread::sleep(std::time::Duration::from_micros(200));
                continue;
            }

            let Ok(mut share) = to_physics_thread.lock() else {
                continue;
            };

            let mut did = 0;
            while accumulator >= PHYS_TIME_STEP && did < 48 {
                physics.step(PHYS_TIME_STEP, &mut share);
                accumulator -= PHYS_TIME_STEP;
                did += 1;
                step_count += 1;
            }
            // If we hit the substep cap the machine can't keep up; drop the
            // backlog and run in slow motion rather than spiral.
            accumulator = accumulator.min(PHYS_TIME_STEP);

            while let Ok(event) = physics.rx.try_recv() {
                use EventToPthread::*;
                match event {
                    Cannon((start, cannon)) => {
                        physics.do_cannon(PHYS_TIME_STEP, &mut share, start, cannon)
                    }
                    Scale(scale) => {
                        physics.add_scale(scale);
                    }
                    ToggleVerletLists => physics.toggle_verlet_lists(),
                    ToggleAdaptiveDt => physics.toggle_adaptive_dt(),
                }
            }

            share.phys_time = PHYS_TIME_STEP;

            if debug && step_count % 60 == 0 {
                let ps = &share.perf_stats;
                let density = if strategy == Strategy::Pbf {
                    format!(" rho/rho0={:.2}", ps.pbf_density_ratio)
                } else {
                    String::new()
                };
                eprintln!(
                    "[{strategy:?}] n={} mean_speed={:.1} max_speed={:.1}{density} solve={}us",
                    ps.total_particles, ps.mean_speed, ps.max_speed, ps.collision_time_us,
                );
            }
        }
    });

    let to_draw_thread = Arc::clone(&share_data);
    let state = MainState::new(&mut ctx, to_draw_thread, tx)?;
    event::run(ctx, events_loop, state)
}

struct MainState {
    share: Arc<Mutex<ShareData>>,
    shader: graphics::Shader,
    tx: Sender<EventToPthread>,
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
        tx: Sender<EventToPthread>,
    ) -> GameResult<MainState> {
        let shader = graphics::ShaderBuilder::new()
            .fragment_path("/blur.wgsl")
            .build(&ctx.gfx)?;

        let circle = graphics::Mesh::new_circle(
            ctx,
            DrawMode::fill(),
            Vec2::default(),
            BALL_SIZE + 10.0,
            0.0001,
            Color::WHITE,
        )?;
        let circles = InstanceArray::new(ctx, None);

        Ok(MainState {
            share,
            shader,
            tx,
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
                    self.circles.update(
                        i as _,
                        DrawParam::new().dest(*pos).color(hsl_to_rgb(*h, 0.75, 0.5)),
                    );
                }
            } else {
                self.circles.clear();
                for (pos, h) in share_data.c_pos.iter().zip(&share_data.c_color) {
                    self.circles
                        .push(DrawParam::new().dest(*pos).color(hsl_to_rgb(*h, 0.75, 0.5)));
                }
            }

            self.nb_obj = share_data.c_pos.len();

            if let (Some(start), Some(cannon)) = (self.mouse_start_pos, self.cannon) {
                self.tx
                    .send(EventToPthread::Cannon((start, cannon)))
                    .unwrap()
            }
        }

        Ok(())
    }

    fn key_down_event(
        &mut self,
        _ctx: &mut Context,
        input: ggez::input::keyboard::KeyInput,
        _repeated: bool,
    ) -> GameResult {
        const SCALER: f32 = 100.0;
        match input.keycode {
            Some(VirtualKeyCode::W) => self.tx.send(EventToPthread::Scale(SCALER)).unwrap(),
            Some(VirtualKeyCode::S) => self.tx.send(EventToPthread::Scale(-SCALER)).unwrap(),
            // Toggle optimization techniques
            Some(VirtualKeyCode::V) => self.tx.send(EventToPthread::ToggleVerletLists).unwrap(),
            Some(VirtualKeyCode::A) => self.tx.send(EventToPthread::ToggleAdaptiveDt).unwrap(),
            _ => (),
        };

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

        // Get performance stats from share data
        let perf_stats = if let Ok(share) = self.share.lock() {
            share.perf_stats.clone()
        } else {
            Default::default()
        };

        let fps_display = Text::new(format!(
            "FPS: {fps} | Particles: {nb_obj}\n\
            \n\
            Optimizations:\n\
            [V] Verlet Lists: {}\n\
            [A] Adaptive dt: {}\n\
            \n\
            Performance:\n\
            Integration: {}µs\n\
            Collision: {}µs\n\
            Current dt: {:.6}s\n\
            \n\
            Diagnostics:\n\
            Mean speed: {:.1}\n\
            Max speed: {:.1}\n\
            {}\
            \n\
            Controls:\n\
            [W/S] Adjust force scale\n\
            Mouse drag: Add particles",
            if perf_stats.verlet_lists_enabled {
                "ON"
            } else {
                "OFF"
            },
            if perf_stats.adaptive_dt_enabled {
                "ON"
            } else {
                "OFF"
            },
            perf_stats.integration_time_us,
            perf_stats.collision_time_us,
            perf_stats.current_dt,
            perf_stats.mean_speed,
            perf_stats.max_speed,
            if perf_stats.pbf_density_ratio > 0.0 {
                format!("Density rho/rho0: {:.2}\n", perf_stats.pbf_density_ratio)
            } else {
                String::new()
            },
        ));

        canvas.draw(
            &fps_display,
            graphics::DrawParam::from(Vec2::new(10.0, 10.0)).color(Color::WHITE),
        );

        canvas.finish(ctx)?;
        Ok(())
    }
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
