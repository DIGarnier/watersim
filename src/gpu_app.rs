//! Single-device GPU simulation and direct particle renderer for the live app.
#![allow(dead_code)] // bytemuck 1.13 derive compatibility symbols on new Rust

use std::borrow::Cow;
use std::io::Write;
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::mpsc;
use std::time::Instant;

use bytemuck::{Pod, Zeroable};
use glam::Vec2;
use wgpu::util::DeviceExt;

use ggez::winit;
use winit::event::{ElementState, Event, MouseButton, VirtualKeyCode, WindowEvent};
use winit::event_loop::{ControlFlow, EventLoop};
use winit::window::WindowBuilder;

use lolballs::constants::{
    BALL_SIZE, GRID_SIZE, HEIGHT, INITIAL_BALL_SPEED_MODIFIER, WIDTH, X_LEN, Y_LEN,
};
use lolballs::physics::PHYS_TIME_STEP;

const GRID_W: u32 = X_LEN as u32;
const GRID_H: u32 = Y_LEN as u32;
const CELL_COUNT: u32 = GRID_W * GRID_H;
const MAX_PER_CELL: u32 = 64;
const PARTICLES_PER_SHARD: usize = 16_777_216;
const MAX_PARTICLES: usize = PARTICLES_PER_SHARD * 2;
const DEFAULT_PARTICLES: usize = 16_000_000;
const GRANULAR_LIMIT: usize = 131_072;
const ACTIVE_PARTICLES_PER_FRAME: usize = 4_000_000;
const RENDER_PARTICLES_PER_FRAME: usize = 16_000_000;
const FLOW_TIME_STEP: f32 = 1.0 / 120.0;
const WORKGROUP: u32 = 256;
const VIDEO_WIDTH: u32 = 960;
const VIDEO_HEIGHT: u32 = 720;
const VIDEO_FPS: u32 = 30;
const VIDEO_FRAMES: u32 = VIDEO_FPS * 5;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Particle {
    position: [f32; 2],
    old_position: [f32; 2],
    force: [f32; 2],
    scratch: [f32; 2],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct SimParams {
    particle_count: u32,
    grid_w: u32,
    grid_h: u32,
    max_per_cell: u32,
    scale_over_8: f32,
    relax: f32,
    ball_size: f32,
    grid_size: f32,
    accel_dt2: f32,
    gravity_x: f32,
    gravity_y: f32,
    _pad0: f32,
    width: f32,
    height: f32,
    _pad1: f32,
    _pad2: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct RenderParams {
    width: f32,
    height: f32,
    radius: f32,
    _pad: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct MassiveParams {
    particle_count: u32,
    substeps: u32,
    update_stride: u32,
    update_phase: u32,
    width: f32,
    height: f32,
    time: f32,
    flow_strength: f32,
    pointer_x: f32,
    pointer_y: f32,
    impulse_x: f32,
    impulse_y: f32,
    render_stride: u32,
    render_phase: u32,
    _pad0: u32,
    _pad1: u32,
}

struct VideoCapture {
    texture: wgpu::Texture,
    readback: wgpu::Buffer,
    child: Child,
    stdin: Option<ChildStdin>,
    frame: u32,
    path: String,
}

impl VideoCapture {
    fn new(device: &wgpu::Device, path: String) -> Self {
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("video capture target"),
            size: wgpu::Extent3d {
                width: VIDEO_WIDTH,
                height: VIDEO_HEIGHT,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Bgra8UnormSrgb,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let readback = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("video framebuffer readback"),
            size: (VIDEO_WIDTH * VIDEO_HEIGHT * 4) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut child = Command::new("ffmpeg")
            .args([
                "-y",
                "-loglevel",
                "error",
                "-f",
                "rawvideo",
                "-pixel_format",
                "bgra",
                "-video_size",
                "960x720",
                "-framerate",
                "30",
                "-i",
                "-",
                "-an",
                "-c:v",
                "libx264",
                "-preset",
                "fast",
                "-crf",
                "18",
                "-pix_fmt",
                "yuv420p",
                &path,
            ])
            .stdin(Stdio::piped())
            .spawn()
            .expect("start ffmpeg for WATERSIM_RECORD");
        let stdin = child.stdin.take().expect("open ffmpeg stdin");
        Self {
            texture,
            readback,
            child,
            stdin: Some(stdin),
            frame: 0,
            path,
        }
    }

    fn view(&self) -> wgpu::TextureView {
        self.texture
            .create_view(&wgpu::TextureViewDescriptor::default())
    }

    fn download_frame(&mut self, device: &wgpu::Device, queue: &wgpu::Queue) -> bool {
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("capture rendered frame"),
        });
        encoder.copy_texture_to_buffer(
            wgpu::ImageCopyTexture {
                texture: &self.texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::ImageCopyBuffer {
                buffer: &self.readback,
                layout: wgpu::ImageDataLayout {
                    offset: 0,
                    bytes_per_row: Some(VIDEO_WIDTH * 4),
                    rows_per_image: Some(VIDEO_HEIGHT),
                },
            },
            wgpu::Extent3d {
                width: VIDEO_WIDTH,
                height: VIDEO_HEIGHT,
                depth_or_array_layers: 1,
            },
        );
        queue.submit(Some(encoder.finish()));
        let slice = self.readback.slice(..);
        let (tx, rx) = mpsc::sync_channel(1);
        slice.map_async(wgpu::MapMode::Read, move |result| {
            let _ = tx.send(result);
        });
        device.poll(wgpu::Maintain::Wait);
        rx.recv()
            .expect("video readback callback")
            .expect("map video framebuffer");
        {
            let mapped = slice.get_mapped_range();
            self.stdin
                .as_mut()
                .expect("ffmpeg input is open")
                .write_all(&mapped)
                .expect("write video frame to ffmpeg");
        }
        self.readback.unmap();
        self.frame += 1;
        self.frame >= VIDEO_FRAMES
    }

    fn finish(mut self) -> String {
        drop(self.stdin.take());
        let status = self.child.wait().expect("wait for ffmpeg");
        assert!(status.success(), "ffmpeg failed with {status}");
        self.path
    }
}

pub struct GpuApp {
    particles: wgpu::Buffer,
    massive_particles_a: wgpu::Buffer,
    massive_particles_b: wgpu::Buffer,
    sim_params: wgpu::Buffer,
    massive_params: wgpu::Buffer,
    render_params: wgpu::Buffer,
    compute_bind: wgpu::BindGroup,
    render_bind: wgpu::BindGroup,
    massive_compute_bind: wgpu::BindGroup,
    massive_render_bind: wgpu::BindGroup,
    clear: wgpu::ComputePipeline,
    integrate: wgpu::ComputePipeline,
    collision_ab: wgpu::ComputePipeline,
    collision_ba: wgpu::ComputePipeline,
    finalize: wgpu::ComputePipeline,
    force: wgpu::ComputePipeline,
    render: wgpu::RenderPipeline,
    massive_init: wgpu::ComputePipeline,
    massive_flow: wgpu::ComputePipeline,
    massive_render: wgpu::RenderPipeline,
    particle_count: usize,
    scale: f32,
    sim_time: f32,
    frame_index: u32,
    needs_massive_init: bool,
    pointer: [f32; 2],
    impulse: [f32; 2],
    render_particles_per_frame: usize,
    pub submit_time_us: u64,
}

impl GpuApp {
    pub fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("direct GPU simulation/render shader"),
            source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(include_str!(
                "../resources/gpu_direct.wgsl"
            ))),
        });
        let massive_compute_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("packed massive-particle compute shader"),
            source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(include_str!(
                "../resources/gpu_massive_compute.wgsl"
            ))),
        });
        let massive_render_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("packed massive-particle point shader"),
            source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(include_str!(
                "../resources/gpu_massive_render.wgsl"
            ))),
        });
        let compute_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("direct GPU compute bindings"),
            entries: &[
                storage_entry(0, false, wgpu::ShaderStages::COMPUTE),
                storage_entry(1, false, wgpu::ShaderStages::COMPUTE),
                uniform_entry(2, wgpu::ShaderStages::COMPUTE),
            ],
        });
        let render_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("direct GPU render bindings"),
            entries: &[
                storage_entry(0, true, wgpu::ShaderStages::VERTEX),
                uniform_entry(1, wgpu::ShaderStages::VERTEX),
            ],
        });
        let massive_compute_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("massive GPU compute bindings"),
                entries: &[
                    storage_entry(0, false, wgpu::ShaderStages::COMPUTE),
                    storage_entry(1, false, wgpu::ShaderStages::COMPUTE),
                    uniform_entry(2, wgpu::ShaderStages::COMPUTE),
                ],
            });
        let massive_render_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("massive GPU render bindings"),
                entries: &[
                    storage_entry(0, true, wgpu::ShaderStages::VERTEX),
                    storage_entry(1, true, wgpu::ShaderStages::VERTEX),
                    uniform_entry(2, wgpu::ShaderStages::VERTEX),
                ],
            });
        let compute_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("direct GPU compute pipeline layout"),
                bind_group_layouts: &[&compute_layout],
                push_constant_ranges: &[],
            });
        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("direct GPU render pipeline layout"),
                bind_group_layouts: &[&render_layout],
                push_constant_ranges: &[],
            });
        let massive_compute_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("massive GPU compute pipeline layout"),
                bind_group_layouts: &[&massive_compute_layout],
                push_constant_ranges: &[],
            });
        let massive_render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("massive GPU render pipeline layout"),
                bind_group_layouts: &[&massive_render_layout],
                push_constant_ranges: &[],
            });
        let compute = |entry| {
            device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some(entry),
                layout: Some(&compute_pipeline_layout),
                module: &shader,
                entry_point: entry,
            })
        };
        let clear = compute("clear_grid");
        let integrate = compute("integrate_and_bin");
        let collision_ab = compute("collision_ab");
        let collision_ba = compute("collision_ba");
        let finalize = compute("finalize_collision");
        let force = compute("force_main");
        let render = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("direct particle render pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "particle_vertex",
                buffers: &[],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "particle_fragment",
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleStrip,
                strip_index_format: None,
                ..Default::default()
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
        });
        let massive_compute = |entry| {
            device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some(entry),
                layout: Some(&massive_compute_pipeline_layout),
                module: &massive_compute_shader,
                entry_point: entry,
            })
        };
        let massive_init = massive_compute("init_particles");
        let massive_flow = massive_compute("flow_main");
        let massive_render = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("one-vertex massive-particle renderer"),
            layout: Some(&massive_render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &massive_render_shader,
                entry_point: "particle_vertex",
                buffers: &[],
            },
            fragment: Some(wgpu::FragmentState {
                module: &massive_render_shader,
                entry_point: "particle_fragment",
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::PointList,
                ..Default::default()
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
        });

        let particles = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("detailed particle state"),
            size: (GRANULAR_LIMIT * std::mem::size_of::<Particle>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let massive_particles_a = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("packed 64-bit massive particle state shard A"),
            size: (PARTICLES_PER_SHARD * std::mem::size_of::<[u32; 2]>()) as u64,
            usage: wgpu::BufferUsages::STORAGE,
            mapped_at_creation: false,
        });
        let massive_particles_b = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("packed 64-bit massive particle state shard B"),
            size: (PARTICLES_PER_SHARD * std::mem::size_of::<[u32; 2]>()) as u64,
            usage: wgpu::BufferUsages::STORAGE,
            mapped_at_creation: false,
        });
        let grid = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("packed atomic particle grid"),
            size: ((CELL_COUNT + 1 + CELL_COUNT * MAX_PER_CELL) * 4) as u64,
            usage: wgpu::BufferUsages::STORAGE,
            mapped_at_creation: false,
        });
        let sim_params = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("direct simulation params"),
            contents: bytemuck::bytes_of(&SimParams::zeroed()),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let massive_params = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("massive simulation/render params"),
            contents: bytemuck::bytes_of(&MassiveParams::zeroed()),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let render_params = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("direct render params"),
            contents: bytemuck::bytes_of(&RenderParams {
                width: WIDTH,
                height: HEIGHT,
                radius: BALL_SIZE + 10.0,
                _pad: 0.0,
            }),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let compute_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("direct compute bind group"),
            layout: &compute_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: particles.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: grid.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: sim_params.as_entire_binding(),
                },
            ],
        });
        let render_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("direct render bind group"),
            layout: &render_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: particles.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: render_params.as_entire_binding(),
                },
            ],
        });
        let massive_compute_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("massive compute bind group"),
            layout: &massive_compute_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: massive_particles_a.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: massive_particles_b.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: massive_params.as_entire_binding(),
                },
            ],
        });
        let massive_render_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("massive render bind group"),
            layout: &massive_render_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: massive_particles_a.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: massive_particles_b.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: massive_params.as_entire_binding(),
                },
            ],
        });
        Self {
            particles,
            massive_particles_a,
            massive_particles_b,
            sim_params,
            massive_params,
            render_params,
            compute_bind,
            render_bind,
            massive_compute_bind,
            massive_render_bind,
            clear,
            integrate,
            collision_ab,
            collision_ba,
            finalize,
            force,
            render,
            massive_init,
            massive_flow,
            massive_render,
            particle_count: 0,
            scale: 2000.0,
            sim_time: 0.0,
            frame_index: 0,
            needs_massive_init: false,
            pointer: [0.5, 0.5],
            impulse: [0.0; 2],
            render_particles_per_frame: std::env::var("WATERSIM_RENDER_PARTICLES")
                .ok()
                .and_then(|value| value.parse().ok())
                .unwrap_or(RENDER_PARTICLES_PER_FRAME)
                .max(1),
            submit_time_us: 0,
        }
    }

    pub fn particle_count(&self) -> usize {
        self.particle_count
    }
    fn high_scale(&self) -> bool {
        self.particle_count > GRANULAR_LIMIT
    }
    fn time_step(&self) -> f32 {
        if self.high_scale() {
            FLOW_TIME_STEP
        } else {
            PHYS_TIME_STEP
        }
    }
    pub fn add_scale(&mut self, delta: f32) {
        self.scale += delta;
    }

    fn seed(&mut self, queue: &wgpu::Queue, count: usize) {
        let count = count.min(MAX_PARTICLES);
        if count > GRANULAR_LIMIT {
            // Initialization is itself a GPU kernel: no multi-hundred-MiB CPU
            // allocation or one-time upload when starting tens of millions.
            self.particle_count = count;
            self.needs_massive_init = true;
            return;
        }
        let columns = ((count as f32 * WIDTH / HEIGHT).sqrt().ceil() as usize).max(1);
        let rows = count.div_ceil(columns).max(1);
        let sx = (WIDTH - 24.0) / columns.saturating_sub(1).max(1) as f32;
        let sy = (HEIGHT - 24.0) / rows.saturating_sub(1).max(1) as f32;
        let particles: Vec<_> = (0..count)
            .map(|i| {
                let p = [
                    12.0 + (i % columns) as f32 * sx,
                    12.0 + (i / columns) as f32 * sy,
                ];
                Particle {
                    position: p,
                    old_position: p,
                    force: [0.0; 2],
                    scratch: [0.0; 2],
                }
            })
            .collect();
        queue.write_buffer(&self.particles, 0, bytemuck::cast_slice(&particles));
        self.particle_count = count;
        self.update_render_params(queue);
    }

    pub fn cannon(&mut self, queue: &wgpu::Queue, start: Vec2, cannon: Vec2) {
        if self.high_scale() {
            // At massive scale, dragging controls the flow field instead of
            // adding an imperceptible handful of particles.
            self.pointer = [start.x / WIDTH, start.y / HEIGHT];
            self.impulse = [
                (cannon.x / WIDTH).clamp(-1.0, 1.0),
                (cannon.y / HEIGHT).clamp(-1.0, 1.0),
            ];
            return;
        }
        if self.particle_count + 20 > GRANULAR_LIMIT || cannon.length_squared() < 1e-12 {
            return;
        }
        let mut new_particles = Vec::with_capacity(20);
        for k in 0..20 {
            let position = start + cannon.perp().normalize() * (-k as f32 * 2.2 * BALL_SIZE);
            let old = position - cannon * INITIAL_BALL_SPEED_MODIFIER * PHYS_TIME_STEP;
            new_particles.push(Particle {
                position: position.to_array(),
                old_position: old.to_array(),
                force: [0.0; 2],
                scratch: [0.0; 2],
            });
        }
        let offset = (self.particle_count * std::mem::size_of::<Particle>()) as u64;
        queue.write_buffer(
            &self.particles,
            offset,
            bytemuck::cast_slice(&new_particles),
        );
        self.particle_count += new_particles.len();
        self.update_render_params(queue);
    }

    fn update_render_params(&self, queue: &wgpu::Queue) {
        let radius = if self.high_scale() {
            (700.0 / (self.particle_count as f32).sqrt()).clamp(0.5, 2.0)
        } else {
            BALL_SIZE + 10.0
        };
        queue.write_buffer(
            &self.render_params,
            0,
            bytemuck::bytes_of(&RenderParams {
                width: WIDTH,
                height: HEIGHT,
                radius,
                _pad: 0.0,
            }),
        );
    }

    /// Encode simulation and rendering into one command buffer. This is the
    /// zero-copy hot path: the vertex shader consumes the exact storage buffer
    /// written by the compute passes, and the CPU never maps particle state.
    pub fn frame(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        view: &wgpu::TextureView,
        substeps: usize,
    ) {
        let started = Instant::now();
        let high_scale = self.high_scale();
        let mut update_stride = 1u32;
        let mut update_phase = 0u32;
        let mut render_stride = 1u32;
        let mut render_phase = 0u32;
        if self.particle_count > 0 && high_scale {
            update_stride = self.particle_count.div_ceil(ACTIVE_PARTICLES_PER_FRAME) as u32;
            update_phase = self.frame_index % update_stride;
            render_stride = self
                .particle_count
                .div_ceil(self.render_particles_per_frame) as u32;
            render_phase = self.frame_index % render_stride;
            let params = MassiveParams {
                particle_count: self.particle_count as u32,
                substeps: substeps as u32 * update_stride,
                update_stride,
                update_phase,
                width: WIDTH,
                height: HEIGHT,
                time: self.sim_time,
                flow_strength: self.scale / 2000.0,
                pointer_x: self.pointer[0],
                pointer_y: self.pointer[1],
                impulse_x: self.impulse[0],
                impulse_y: self.impulse[1],
                render_stride,
                render_phase,
                _pad0: 0,
                _pad1: 0,
            };
            queue.write_buffer(&self.massive_params, 0, bytemuck::bytes_of(&params));
            if substeps > 0 {
                self.frame_index = self.frame_index.wrapping_add(1);
                self.sim_time += substeps as f32 * self.time_step();
                self.impulse[0] *= 0.92;
                self.impulse[1] *= 0.92;
            }
        } else if self.particle_count > 0 && substeps > 0 {
            let params = SimParams {
                particle_count: self.particle_count as u32,
                grid_w: GRID_W,
                grid_h: GRID_H,
                max_per_cell: MAX_PER_CELL,
                scale_over_8: self.scale / 8.0,
                relax: 0.375,
                ball_size: BALL_SIZE,
                grid_size: GRID_SIZE,
                accel_dt2: PHYS_TIME_STEP,
                gravity_x: 0.0,
                gravity_y: 9.8,
                _pad0: 0.0,
                width: WIDTH,
                height: HEIGHT,
                _pad1: 0.0,
                _pad2: 0.0,
            };
            queue.write_buffer(&self.sim_params, 0, bytemuck::bytes_of(&params));
            self.sim_time += substeps as f32 * self.time_step();
        }
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("unified GPU simulation + render frame"),
        });
        if self.particle_count > 0 && high_scale && (self.needs_massive_init || substeps > 0) {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("packed massive-particle update"),
            });
            pass.set_bind_group(0, &self.massive_compute_bind, &[]);
            if self.needs_massive_init {
                let all = (self.particle_count as u32 + WORKGROUP - 1) / WORKGROUP;
                let groups_x = all.min(65_535);
                let groups_y = all.div_ceil(65_535);
                pass.set_pipeline(&self.massive_init);
                pass.dispatch_workgroups(groups_x, groups_y, 1);
                self.needs_massive_init = false;
            }
            if substeps > 0 {
                let active = (self.particle_count as u32 - update_phase).div_ceil(update_stride);
                let groups = (active + WORKGROUP - 1) / WORKGROUP;
                pass.set_pipeline(&self.massive_flow);
                pass.dispatch_workgroups(groups, 1, 1);
            }
        } else if self.particle_count > 0 && substeps > 0 {
            let particles = (self.particle_count as u32 + WORKGROUP - 1) / WORKGROUP;
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("GPU physics substeps"),
            });
            pass.set_bind_group(0, &self.compute_bind, &[]);
            let cells = (CELL_COUNT + 1 + WORKGROUP - 1) / WORKGROUP;
            for _ in 0..substeps {
                pass.set_pipeline(&self.clear);
                pass.dispatch_workgroups(cells, 1, 1);
                pass.set_pipeline(&self.integrate);
                pass.dispatch_workgroups(particles, 1, 1);
                pass.set_pipeline(&self.collision_ab);
                pass.dispatch_workgroups(particles, 1, 1);
                pass.set_pipeline(&self.collision_ba);
                pass.dispatch_workgroups(particles, 1, 1);
                pass.set_pipeline(&self.collision_ab);
                pass.dispatch_workgroups(particles, 1, 1);
                pass.set_pipeline(&self.finalize);
                pass.dispatch_workgroups(particles, 1, 1);
                pass.set_pipeline(&self.force);
                pass.dispatch_workgroups(particles, 1, 1);
            }
        }
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("direct particle pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.0,
                            g: 0.0,
                            b: 0.0,
                            a: 1.0,
                        }),
                        store: true,
                    },
                })],
                depth_stencil_attachment: None,
            });
            if high_scale {
                pass.set_pipeline(&self.massive_render);
                pass.set_bind_group(0, &self.massive_render_bind, &[]);
                let rendered = (self.particle_count as u32 - render_phase).div_ceil(render_stride);
                pass.draw(0..rendered, 0..1);
            } else {
                pass.set_pipeline(&self.render);
                pass.set_bind_group(0, &self.render_bind, &[]);
                pass.draw(0..4, 0..self.particle_count as u32);
            }
        }
        queue.submit(Some(encoder.finish()));
        self.submit_time_us = started.elapsed().as_micros() as u64;
    }
}

/// Native single-device loop for the default GPU path. ggez deliberately
/// requests WebGL2-downlevel limits (zero compute workgroups), so the GPU path
/// uses winit + wgpu directly while the legacy strategies keep their ggez UI.
pub fn run() -> ! {
    let record_path = std::env::var("WATERSIM_RECORD").ok();
    let event_loop = EventLoop::new();
    let window = WindowBuilder::new()
        .with_title("watersim — GPU direct")
        .with_inner_size(winit::dpi::LogicalSize::new(WIDTH as f64, HEIGHT as f64))
        .with_visible(record_path.is_none())
        .build(&event_loop)
        .expect("create GPU window");
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::PRIMARY,
        dx12_shader_compiler: Default::default(),
    });
    let surface = unsafe { instance.create_surface(&window) }.expect("create GPU surface");
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
        power_preference: wgpu::PowerPreference::HighPerformance,
        compatible_surface: Some(&surface),
        force_fallback_adapter: false,
    }))
    .expect("no graphics/compute GPU adapter");
    let info = adapter.get_info();
    let (device, queue) = pollster::block_on(adapter.request_device(
        &wgpu::DeviceDescriptor {
            label: Some("watersim unified compute/render device"),
            features: wgpu::Features::empty(),
            limits: wgpu::Limits::default().using_resolution(adapter.limits()),
        },
        None,
    ))
    .expect("request unified GPU device");
    let capabilities = surface.get_capabilities(&adapter);
    let format = capabilities
        .formats
        .iter()
        .copied()
        .find(|f| f.is_srgb())
        .unwrap_or(capabilities.formats[0]);
    let size = window.inner_size();
    let uncapped = std::env::var("WATERSIM_UNCAPPED").is_ok();
    let mut config = wgpu::SurfaceConfiguration {
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        format,
        width: size.width.max(1),
        height: size.height.max(1),
        present_mode: if uncapped {
            wgpu::PresentMode::AutoNoVsync
        } else {
            wgpu::PresentMode::Fifo
        },
        alpha_mode: capabilities.alpha_modes[0],
        view_formats: vec![],
    };
    surface.configure(&device, &config);
    let render_format = if record_path.is_some() {
        wgpu::TextureFormat::Bgra8UnormSrgb
    } else {
        format
    };
    let mut app = GpuApp::new(&device, render_format);
    let seed_count = std::env::var("WATERSIM_SEED_PARTICLES")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(DEFAULT_PARTICLES);
    app.seed(&queue, seed_count);
    let mut capture = record_path.map(|path| VideoCapture::new(&device, path));
    let mut mouse_start = None;
    let mut mouse = Vec2::ZERO;
    let mut accumulator = 0.0f32;
    let mut last = Instant::now();
    let mut title_started = Instant::now();
    let mut frames = 0u32;
    let debug = std::env::var("WATERSIM_DEBUG").is_ok();
    println!(
        "GPU direct compute + render: {} via {:?}",
        info.name, info.backend
    );

    event_loop.run(move |event, _, control_flow| {
        *control_flow = ControlFlow::Poll;
        match event {
            Event::WindowEvent { event, .. } => match event {
                WindowEvent::CloseRequested => *control_flow = ControlFlow::Exit,
                WindowEvent::Resized(size) => {
                    config.width = size.width.max(1);
                    config.height = size.height.max(1);
                    surface.configure(&device, &config);
                }
                WindowEvent::CursorMoved { position, .. } => {
                    mouse = Vec2::new(
                        position.x as f32 / config.width as f32 * WIDTH,
                        position.y as f32 / config.height as f32 * HEIGHT,
                    );
                }
                WindowEvent::MouseInput {
                    state,
                    button: MouseButton::Left,
                    ..
                } => {
                    if state == ElementState::Pressed {
                        mouse_start = Some(mouse);
                    } else {
                        mouse_start = None;
                    }
                }
                WindowEvent::KeyboardInput { input, .. }
                    if input.state == ElementState::Pressed =>
                {
                    match input.virtual_keycode {
                        Some(VirtualKeyCode::Escape) => *control_flow = ControlFlow::Exit,
                        Some(VirtualKeyCode::W) => app.add_scale(100.0),
                        Some(VirtualKeyCode::S) => app.add_scale(-100.0),
                        _ => {}
                    }
                }
                _ => {}
            },
            Event::MainEventsCleared => window.request_redraw(),
            Event::RedrawRequested(_) => {
                if capture.is_some() {
                    let view = capture.as_ref().unwrap().view();
                    // 120 Hz simulation sampled into a deterministic 30 FPS
                    // video, independent of readback/encoder wall time.
                    app.frame(&device, &queue, &view, 4);
                    let done = capture.as_mut().unwrap().download_frame(&device, &queue);
                    let recorded = capture.as_ref().unwrap().frame;
                    window.set_title(&format!(
                        "watersim — recording 33M | {recorded}/{VIDEO_FRAMES}"
                    ));
                    if done {
                        let path = capture.take().unwrap().finish();
                        println!("wrote {path} ({VIDEO_WIDTH}x{VIDEO_HEIGHT}, 5 seconds)");
                        *control_flow = ControlFlow::Exit;
                    }
                    return;
                }
                let now = Instant::now();
                accumulator += now.duration_since(last).as_secs_f32().min(0.05);
                last = now;
                let time_step = app.time_step();
                let max_steps = if app.high_scale() { 8.0 } else { 48.0 };
                let steps = (accumulator / time_step).floor().min(max_steps) as usize;
                accumulator -= steps as f32 * time_step;
                // Never let a slow frame create a self-sustaining catch-up
                // storm. High-scale flow degrades in time, not responsiveness.
                accumulator = accumulator.min(time_step);
                if let Some(start) = mouse_start {
                    app.cannon(&queue, start, mouse - start);
                }
                let frame = match surface.get_current_texture() {
                    Ok(frame) => frame,
                    Err(_) => {
                        surface.configure(&device, &config);
                        return;
                    }
                };
                let view = frame
                    .texture
                    .create_view(&wgpu::TextureViewDescriptor::default());
                app.frame(&device, &queue, &view, steps);
                frame.present();
                frames += 1;
                let elapsed = title_started.elapsed().as_secs_f32();
                if elapsed >= 0.5 {
                    let fps = frames as f32 / elapsed;
                    let status = format!(
                        "watersim — GPU direct | {:.0} FPS | {} particles | submit {} µs",
                        fps,
                        app.particle_count(),
                        app.submit_time_us
                    );
                    window.set_title(&status);
                    if debug {
                        eprintln!("{status}");
                    }
                    frames = 0;
                    title_started = Instant::now();
                }
            }
            _ => {}
        }
    })
}

fn storage_entry(
    binding: u32,
    read_only: bool,
    visibility: wgpu::ShaderStages,
) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}
fn uniform_entry(binding: u32, visibility: wgpu::ShaderStages) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Uniform,
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}
