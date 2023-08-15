pub mod constants;
pub mod physics;

use std::path;
use std::sync::mpsc::{channel, Sender};
use std::sync::{Arc, Mutex};

use ggez::conf::{WindowMode, WindowSetup};
use ggez::event::MouseButton;
use ggez::glam::Vec2;
use ggez::graphics::{Color, DrawMode, DrawParam, Image, InstanceArray, Mesh, Text};

use ggez::winit::event::VirtualKeyCode;
use ggez::{event, graphics, Context, ContextBuilder, GameResult};
use constants::{BALL_SIZE, HEIGHT, WIDTH, X_LEN, Y_LEN};
use physics::{EventToPthread, Physics, ShareData, PHYS_TIME_STEP};

const BACKGROUND_COLOR: Color = Color::new(0., 0., 0., 0.0);

fn main() -> GameResult {
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
            Vec::default(),
            Vec::default(),
            vec![Vec::new(); ((X_LEN + 1.0) * (Y_LEN + 1.0)) as usize],
            Vec::with_capacity(30000),
            Vec::with_capacity(30000),
            rx,
            2000.0,
        );

        let clock = std::time::Instant::now();
        let mut phys_frame_start = clock.elapsed().as_secs_f32();
        loop {
            let dt = clock.elapsed().as_secs_f32() - phys_frame_start;

            if dt >= PHYS_TIME_STEP {
                let Ok(mut share) = to_physics_thread.lock() else {
                    continue;
                };

                physics.step(dt, &mut share);

                if let Ok(event) = physics.rx.try_recv() {
                    use EventToPthread::*;
                    match event {
                        Cannon((start, cannon)) => physics.do_cannon(dt, &mut share, start, cannon),
                        Scale(scale) => {
                            physics.scale += scale;
                        }
                    }
                }

                share.phys_time = dt;
                phys_frame_start = clock.elapsed().as_secs_f32();
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
        let fps_display = Text::new(format!("FPS: {fps}\nnb obj: {nb_obj}"));

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
