//! Fully GPU-resident granular solver using wgpu compute (Metal on macOS).
#![allow(dead_code)] // suppresses compatibility-check symbols from old bytemuck_derive

use std::borrow::Cow;
use std::sync::mpsc;
use std::time::Instant;

use bytemuck::{Pod, Zeroable};
use glam::Vec2;
use wgpu::util::DeviceExt;

use super::{FluidSolver, GranularSolver, ShareData, PHYS_TIME_STEP};
use crate::constants::{BALL_SIZE, GRID_SIZE, HEIGHT, WIDTH, X_LEN, Y_LEN};

const GRID_W: u32 = X_LEN as u32;
const GRID_H: u32 = Y_LEN as u32;
const CELL_COUNT: u32 = GRID_W * GRID_H;
const MAX_PER_CELL: u32 = 64;
const MAX_PARTICLES: usize = 262_144;
const WORKGROUP_SIZE: u32 = 256;

#[repr(C)]
#[allow(dead_code)] // bytemuck 1.13's derive emits compatibility checks on new Rust
#[derive(Clone, Copy, Pod, Zeroable)]
struct GpuVec2 {
    x: f32,
    y: f32,
}

impl From<Vec2> for GpuVec2 {
    fn from(v: Vec2) -> Self {
        Self { x: v.x, y: v.y }
    }
}
impl From<GpuVec2> for Vec2 {
    fn from(v: GpuVec2) -> Self {
        Self::new(v.x, v.y)
    }
}

#[repr(C)]
#[allow(dead_code)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct GpuParams {
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

struct GpuState {
    device: wgpu::Device,
    queue: wgpu::Queue,
    positions: wgpu::Buffer,
    scratch: wgpu::Buffer,
    forces: wgpu::Buffer,
    old_positions: wgpu::Buffer,
    cell_counts: wgpu::Buffer,
    params: wgpu::Buffer,
    readback: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    clear: wgpu::ComputePipeline,
    integrate: wgpu::ComputePipeline,
    collide_ab: wgpu::ComputePipeline,
    collide_ba: wgpu::ComputePipeline,
    force: wgpu::ComputePipeline,
    particle_count: usize,
    adapter_name: String,
}

impl GpuState {
    fn new() -> Result<Self, String> {
        pollster::block_on(async {
            let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
                backends: wgpu::Backends::PRIMARY,
                dx12_shader_compiler: Default::default(),
            });
            let adapter = instance
                .request_adapter(&wgpu::RequestAdapterOptions {
                    power_preference: wgpu::PowerPreference::HighPerformance,
                    compatible_surface: None,
                    force_fallback_adapter: false,
                })
                .await
                .ok_or_else(|| "no compatible GPU adapter".to_owned())?;
            let info = adapter.get_info();
            let (device, queue) = adapter
                .request_device(
                    &wgpu::DeviceDescriptor {
                        label: Some("watersim GPU compute device"),
                        features: wgpu::Features::empty(),
                        // The shader uses six storage buffers. Request the WebGPU
                        // defaults (8/stage), well within Apple M5's adapter limits.
                        limits: wgpu::Limits::default(),
                    },
                    None,
                )
                .await
                .map_err(|e| format!("request_device failed: {e}"))?;

            let mut layout_entries = Vec::with_capacity(7);
            for binding in 0..7 {
                layout_entries.push(wgpu::BindGroupLayoutEntry {
                    binding,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: if binding == 6 {
                        wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Uniform,
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        }
                    } else {
                        wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        }
                    },
                    count: None,
                });
            }
            let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("GPU granular bindings"),
                entries: &layout_entries,
            });
            let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("GPU granular pipeline layout"),
                bind_group_layouts: &[&layout],
                push_constant_ranges: &[],
            });
            let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("GPU granular shader"),
                source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(include_str!(
                    "../../resources/granular_compute.wgsl"
                ))),
            });
            let make_pipeline = |entry_point| {
                device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                    label: Some(entry_point),
                    layout: Some(&pipeline_layout),
                    module: &shader,
                    entry_point,
                })
            };
            let clear = make_pipeline("clear_grid");
            let integrate = make_pipeline("integrate_and_bin");
            let collide_ab = make_pipeline("collision_a_to_b");
            let collide_ba = make_pipeline("collision_b_to_a");
            let force = make_pipeline("force_main");

            let vec_bytes = (MAX_PARTICLES * std::mem::size_of::<GpuVec2>()) as u64;
            let storage = |label, size, usage| {
                device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some(label),
                    size,
                    usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST | usage,
                    mapped_at_creation: false,
                })
            };
            let positions = storage(
                "particle positions",
                vec_bytes,
                wgpu::BufferUsages::COPY_SRC,
            );
            let scratch = storage(
                "position ping-pong",
                vec_bytes,
                wgpu::BufferUsages::COPY_SRC,
            );
            let forces = storage("particle forces", vec_bytes, wgpu::BufferUsages::empty());
            let old_positions = storage(
                "particle old positions",
                vec_bytes,
                wgpu::BufferUsages::COPY_SRC,
            );
            let cell_counts = storage(
                "atomic cell counts",
                ((CELL_COUNT + 1) * 4) as u64,
                wgpu::BufferUsages::COPY_SRC,
            );
            let cell_particles = storage(
                "cell particle buckets",
                (CELL_COUNT * MAX_PER_CELL * 4) as u64,
                wgpu::BufferUsages::empty(),
            );
            let params = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("GPU granular parameters"),
                contents: bytemuck::bytes_of(&GpuParams::zeroed()),
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            });
            // positions + old positions + one u32 overflow count.
            let readback = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("render snapshot readback"),
                size: vec_bytes * 2 + 4,
                usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            let buffers = [
                &positions,
                &scratch,
                &forces,
                &old_positions,
                &cell_counts,
                &cell_particles,
                &params,
            ];
            let entries: Vec<_> = buffers
                .iter()
                .enumerate()
                .map(|(binding, buffer)| wgpu::BindGroupEntry {
                    binding: binding as u32,
                    resource: buffer.as_entire_binding(),
                })
                .collect();
            let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("GPU granular bind group"),
                layout: &layout,
                entries: &entries,
            });
            Ok(Self {
                device,
                queue,
                positions,
                scratch,
                forces,
                old_positions,
                cell_counts,
                params,
                readback,
                bind_group,
                clear,
                integrate,
                collide_ab,
                collide_ba,
                force,
                particle_count: 0,
                adapter_name: format!("{} via {:?}", info.name, info.backend),
            })
        })
    }

    fn upload_new_particles(&mut self, positions: &[Vec2], old: &[Vec2]) -> Result<(), String> {
        if positions.len() > MAX_PARTICLES {
            return Err(format!(
                "particle limit exceeded: {} > {MAX_PARTICLES}",
                positions.len()
            ));
        }
        if positions.len() < self.particle_count {
            self.particle_count = 0; // scenario reset: replace the whole GPU state
        }
        if positions.len() == self.particle_count {
            return Ok(());
        }
        let start = self.particle_count;
        let gpu_positions: Vec<GpuVec2> =
            positions[start..].iter().copied().map(Into::into).collect();
        let gpu_old: Vec<GpuVec2> = old[start..].iter().copied().map(Into::into).collect();
        let zero_force = vec![GpuVec2 { x: 0.0, y: 0.0 }; positions.len() - start];
        let offset = (start * std::mem::size_of::<GpuVec2>()) as u64;
        self.queue.write_buffer(
            &self.positions,
            offset,
            bytemuck::cast_slice(&gpu_positions),
        );
        self.queue
            .write_buffer(&self.old_positions, offset, bytemuck::cast_slice(&gpu_old));
        self.queue
            .write_buffer(&self.forces, offset, bytemuck::cast_slice(&zero_force));
        self.particle_count = positions.len();
        Ok(())
    }

    fn submit(&mut self, dt: f32, gravity: Vec2, scale: f32, omega: f32, iterations: usize) {
        let n = self.particle_count as u32;
        let params = GpuParams {
            particle_count: n,
            grid_w: GRID_W,
            grid_h: GRID_H,
            max_per_cell: MAX_PER_CELL,
            scale_over_8: scale / 8.0,
            relax: 0.375 * omega,
            ball_size: BALL_SIZE,
            grid_size: GRID_SIZE,
            accel_dt2: dt * dt / PHYS_TIME_STEP,
            gravity_x: gravity.x,
            gravity_y: gravity.y,
            _pad0: 0.0,
            width: WIDTH,
            height: HEIGHT,
            _pad1: 0.0,
            _pad2: 0.0,
        };
        self.queue
            .write_buffer(&self.params, 0, bytemuck::bytes_of(&params));
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("GPU granular step"),
            });
        let particle_groups = (n + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE;
        let cell_groups = (CELL_COUNT + 1 + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE;
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("integrate, bin, collide"),
            });
            pass.set_bind_group(0, &self.bind_group, &[]);
            pass.set_pipeline(&self.clear);
            pass.dispatch_workgroups(cell_groups, 1, 1);
            pass.set_pipeline(&self.integrate);
            pass.dispatch_workgroups(particle_groups, 1, 1);
            for iteration in 0..iterations {
                pass.set_pipeline(if iteration % 2 == 0 {
                    &self.collide_ab
                } else {
                    &self.collide_ba
                });
                pass.dispatch_workgroups(particle_groups, 1, 1);
            }
        }
        if iterations % 2 == 1 {
            let bytes = self.particle_count as u64 * std::mem::size_of::<GpuVec2>() as u64;
            encoder.copy_buffer_to_buffer(&self.scratch, 0, &self.positions, 0, bytes);
        }
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("next-step forces"),
            });
            pass.set_bind_group(0, &self.bind_group, &[]);
            pass.set_pipeline(&self.force);
            pass.dispatch_workgroups(particle_groups, 1, 1);
        }
        self.queue.submit(Some(encoder.finish()));
    }

    fn snapshot(&mut self, positions: &mut [Vec2], old: &mut [Vec2]) -> Result<u32, String> {
        let n = self.particle_count;
        let bytes = (n * std::mem::size_of::<GpuVec2>()) as u64;
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("GPU render snapshot"),
            });
        encoder.copy_buffer_to_buffer(&self.positions, 0, &self.readback, 0, bytes);
        encoder.copy_buffer_to_buffer(&self.old_positions, 0, &self.readback, bytes, bytes);
        encoder.copy_buffer_to_buffer(
            &self.cell_counts,
            (CELL_COUNT * 4) as u64,
            &self.readback,
            bytes * 2,
            4,
        );
        self.queue.submit(Some(encoder.finish()));
        let slice = self.readback.slice(0..bytes * 2 + 4);
        let (tx, rx) = mpsc::sync_channel(1);
        slice.map_async(wgpu::MapMode::Read, move |result| {
            let _ = tx.send(result);
        });
        self.device.poll(wgpu::Maintain::Wait);
        rx.recv()
            .map_err(|_| "GPU snapshot callback dropped".to_owned())?
            .map_err(|e| format!("GPU snapshot mapping failed: {e}"))?;
        let overflow;
        {
            let mapped = slice.get_mapped_range();
            let vector_bytes = bytes as usize;
            let gpu_positions: &[GpuVec2] = bytemuck::cast_slice(&mapped[..vector_bytes]);
            let gpu_old: &[GpuVec2] = bytemuck::cast_slice(&mapped[vector_bytes..2 * vector_bytes]);
            for i in 0..n {
                positions[i] = gpu_positions[i].into();
                old[i] = gpu_old[i].into();
            }
            overflow = u32::from_ne_bytes(
                mapped[2 * vector_bytes..2 * vector_bytes + 4]
                    .try_into()
                    .unwrap(),
            );
        }
        self.readback.unmap();
        Ok(overflow)
    }
}

enum Backend {
    Gpu(GpuState),
    Cpu(GranularSolver),
}

/// Granular simulation whose entire numerical pipeline remains on the GPU.
/// CPU positions are snapshots for rendering, not the authoritative state.
pub struct GpuGranularSolver {
    backend: Backend,
    scale: f32,
    iterations: usize,
    omega: f32,
    step: usize,
    readback_interval: usize,
}

impl GpuGranularSolver {
    pub fn new(scale: f32) -> Self {
        let backend = match GpuState::new() {
            Ok(gpu) => {
                println!("GPU compute: {}", gpu.adapter_name);
                Backend::Gpu(gpu)
            }
            Err(e) => {
                eprintln!("GPU unavailable ({e}); falling back to CPU granular");
                Backend::Cpu(GranularSolver::new(scale, Vec::new()))
            }
        };
        let readback_interval = std::env::var("WATERSIM_GPU_READBACK_INTERVAL")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(8)
            .max(1);
        Self {
            backend,
            scale,
            iterations: 3,
            omega: 1.0,
            step: 0,
            readback_interval,
        }
    }
}

impl FluidSolver for GpuGranularSolver {
    fn name(&self) -> &'static str {
        "GPU Granular"
    }
    fn add_scale(&mut self, delta: f32) {
        self.scale += delta;
        if let Backend::Cpu(cpu) = &mut self.backend {
            cpu.add_scale(delta);
        }
    }
    fn set_solver_iterations(&mut self, n: usize) {
        self.iterations = n.max(1);
        if let Backend::Cpu(cpu) = &mut self.backend {
            cpu.set_solver_iterations(n);
        }
    }
    fn set_solver_omega(&mut self, omega: f32) {
        self.omega = omega;
        if let Backend::Cpu(cpu) = &mut self.backend {
            cpu.set_solver_omega(omega);
        }
    }
    fn forces_grid(&mut self, p: &[Vec2]) -> Vec<Vec2> {
        let mut cpu = GranularSolver::new(self.scale, Vec::new());
        cpu.forces_grid(p)
    }
    fn forces_direct(&self, p: &[Vec2]) -> Vec<Vec2> {
        GranularSolver::new(self.scale, Vec::new()).forces_direct(p)
    }
    fn substep(&mut self, dt: f32, gravity: Vec2, share: &mut ShareData, old: &mut Vec<Vec2>) {
        if let Backend::Cpu(cpu) = &mut self.backend {
            share.perf_stats.gpu_enabled = false;
            cpu.substep(dt, gravity, share, old);
            return;
        }
        old.resize(share.c_pos.len(), Vec2::ZERO);
        let started = Instant::now();
        let result = if let Backend::Gpu(gpu) = &mut self.backend {
            gpu.upload_new_particles(&share.c_pos, old).map(|_| {
                if gpu.particle_count > 0 {
                    gpu.submit(dt, gravity, self.scale, self.omega, self.iterations);
                }
            })
        } else {
            unreachable!()
        };
        if let Err(e) = result {
            eprintln!("GPU compute failed ({e}); switching to CPU fallback");
            self.backend = Backend::Cpu(GranularSolver::new(self.scale, Vec::new()));
            share.perf_stats.gpu_enabled = false;
            return;
        }
        self.step += 1;
        share.perf_stats.gpu_enabled = true;
        share.perf_stats.gpu_compute_time_us += started.elapsed().as_micros() as u64;

        if self.step % self.readback_interval == 0 {
            let sync_started = Instant::now();
            let snapshot = if let Backend::Gpu(gpu) = &mut self.backend {
                gpu.snapshot(&mut share.c_pos, old)
            } else {
                unreachable!()
            };
            share.perf_stats.gpu_compute_time_us += sync_started.elapsed().as_micros() as u64;
            match snapshot {
                Ok(overflow) => {
                    if overflow > 0 {
                        eprintln!("GPU grid overflow: {overflow} particles exceeded {MAX_PER_CELL} occupants/cell");
                    }
                    let mut sum = 0.0;
                    let mut max: f32 = 0.0;
                    for i in 0..share.c_pos.len() {
                        let speed = ((share.c_pos[i] - old[i]) * 20.0).length();
                        share.c_color[i] = (speed + 198.0) % 360.0;
                        sum += speed;
                        max = max.max(speed);
                    }
                    share.perf_stats.mean_speed = sum / share.c_pos.len().max(1) as f32;
                    share.perf_stats.max_speed = max;
                }
                Err(e) => {
                    eprintln!("GPU snapshot failed ({e}); switching to CPU fallback");
                    self.backend = Backend::Cpu(GranularSolver::new(self.scale, Vec::new()));
                    share.perf_stats.gpu_enabled = false;
                }
            }
        }
    }
}
