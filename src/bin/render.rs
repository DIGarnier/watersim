// Headless comparison renderer for the two fluid models.
//
// This tool exists because the machine that develops this code has no display,
// so the ggez front end can't run here. It drives the real physics engine
// (lolballs::physics::Physics) exactly like the bench does, rasterizes the
// particle state to frames itself, and writes an animated GIF plus a still PNG
// per scenario — encoders hand-rolled below so the crate stays dependency-free.
//
//   cargo run --release --no-default-features --features render --bin render
//
// Each scenario is run twice (Granular then Pbf) from the identical initial
// state and identical gravity, and the two runs are composited side by side so
// the behavioral difference is visible at a glance.

use std::sync::mpsc::channel;

use glam::Vec2;
use lolballs::constants::{BALL_SIZE, HEIGHT, WIDTH};
use lolballs::physics::{PbfParams, Physics, ShareData, Strategy, PHYS_TIME_STEP};

// ---------------------------------------------------------------------------
// Render configuration
// ---------------------------------------------------------------------------

const PANEL_W: usize = 460; // panel width in pixels
const PANEL_H: usize = (PANEL_W as f32 * HEIGHT / WIDTH) as usize; // keep aspect
const MARGIN: usize = 12;
const GAP: usize = 12; // gap between the two panels
const TITLE_H: usize = 26; // room for the panel title above each panel
const DISC_R: i32 = 2; // particle disc radius in render pixels

const CANVAS_W: usize = MARGIN * 2 + PANEL_W * 2 + GAP;
const CANVAS_H: usize = MARGIN + TITLE_H + PANEL_H + MARGIN;

const FPS: u32 = 25;
const FRAMES: usize = 110; // ~4.4 s of animation
const SUBSTEPS_PER_FRAME: usize = 480 / FPS as usize; // physics substeps between frames

// Palette layout: 0 = background, 1 = white text, 2 = divider, 3..=255 = a hue
// ring at the app's saturation/lightness (particles are colored by a speed hue,
// so a hue-indexed palette reproduces them almost exactly with no dithering).
const PAL_BG: u8 = 0;
const PAL_TEXT: u8 = 1;
const PAL_DIV: u8 = 2;
const HUE_BASE: usize = 3;
const HUE_COUNT: usize = 256 - HUE_BASE;

fn build_palette() -> [[u8; 3]; 256] {
    let mut p = [[0u8; 3]; 256];
    p[PAL_BG as usize] = [10, 14, 22];
    p[PAL_TEXT as usize] = [235, 240, 245];
    p[PAL_DIV as usize] = [70, 80, 100];
    for i in 0..HUE_COUNT {
        let h = i as f32 / (HUE_COUNT - 1) as f32 * 360.0;
        let (r, g, b) = hsl_to_rgb(h, 0.75, 0.5);
        p[HUE_BASE + i] = [(r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8];
    }
    p
}

// Quantize the speed hue into a small number of bands. Snapping neighboring
// particles to the same palette index turns the per-particle velocity speckle
// into flat runs — much cleaner to read *and* far more compressible (LZW needs
// runs), which keeps the GIFs small.
const HUE_BANDS: usize = 18;

#[inline]
fn hue_index(h: f32) -> u8 {
    let h = ((h % 360.0) + 360.0) % 360.0;
    let band = ((h / 360.0 * HUE_BANDS as f32) as usize).min(HUE_BANDS - 1);
    let i = band * (HUE_COUNT - 1) / (HUE_BANDS - 1);
    (HUE_BASE + i.min(HUE_COUNT - 1)) as u8
}

// Same HSL->RGB the ggez front end uses, so the offline frames match the app.
fn hsl_to_rgb(h: f32, s: f32, l: f32) -> (f32, f32, f32) {
    let c = (1.0 - (2.0 * l - 1.0).abs()) * s;
    let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
    let m = l - c / 2.0;
    let (r, g, b) = match h {
        h if h < 60.0 => (c, x, 0.0),
        h if h < 120.0 => (x, c, 0.0),
        h if h < 180.0 => (0.0, c, x),
        h if h < 240.0 => (0.0, x, c),
        h if h < 300.0 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };
    (r + m, g + m, b + m)
}

// ---------------------------------------------------------------------------
// Canvas + rasterization
// ---------------------------------------------------------------------------

struct Canvas {
    w: usize,
    h: usize,
    px: Vec<u8>, // palette indices
}

impl Canvas {
    fn new(w: usize, h: usize) -> Self {
        Self {
            w,
            h,
            px: vec![PAL_BG; w * h],
        }
    }

    #[inline]
    fn set(&mut self, x: i32, y: i32, idx: u8) {
        if x >= 0 && y >= 0 && (x as usize) < self.w && (y as usize) < self.h {
            self.px[y as usize * self.w + x as usize] = idx;
        }
    }

    fn disc(&mut self, cx: i32, cy: i32, r: i32, idx: u8) {
        let r2 = r * r;
        for dy in -r..=r {
            for dx in -r..=r {
                if dx * dx + dy * dy <= r2 {
                    self.set(cx + dx, cy + dy, idx);
                }
            }
        }
    }

    fn fill_rect(&mut self, x0: usize, y0: usize, w: usize, h: usize, idx: u8) {
        for y in y0..(y0 + h).min(self.h) {
            for x in x0..(x0 + w).min(self.w) {
                self.px[y * self.w + x] = idx;
            }
        }
    }
}

/// Draw one simulation state into the given panel origin of the canvas.
fn draw_panel(canvas: &mut Canvas, ox: usize, oy: usize, share: &ShareData) {
    let scale = PANEL_W as f32 / WIDTH;
    for (p, &hue) in share.c_pos.iter().zip(&share.c_color) {
        let x = ox as f32 + p.x * scale;
        let y = oy as f32 + p.y * scale;
        canvas.disc(x as i32, y as i32, DISC_R, hue_index(hue));
    }
}

// ---------------------------------------------------------------------------
// Bitmap font (5x7) for the two panel titles: only the letters in
// "GRANULAR" and "PBF" are defined.
// ---------------------------------------------------------------------------

fn glyph(c: char) -> Option<[u8; 7]> {
    // rows top->bottom, low 5 bits used (bit4 = leftmost pixel)
    let g = match c {
        'G' => [
            0b01110, 0b10001, 0b10000, 0b10111, 0b10001, 0b10001, 0b01110,
        ],
        'R' => [
            0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001,
        ],
        'A' => [
            0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001,
        ],
        'N' => [
            0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001,
        ],
        'U' => [
            0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110,
        ],
        'L' => [
            0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111,
        ],
        'P' => [
            0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000,
        ],
        'B' => [
            0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110,
        ],
        'F' => [
            0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000,
        ],
        ' ' => [0, 0, 0, 0, 0, 0, 0],
        _ => return None,
    };
    Some(g)
}

fn draw_text(canvas: &mut Canvas, x: usize, y: usize, text: &str, s: i32, idx: u8) {
    let mut cx = x as i32;
    for ch in text.chars() {
        if let Some(g) = glyph(ch) {
            for (row, bits) in g.iter().enumerate() {
                for col in 0..5 {
                    if bits & (1 << (4 - col)) != 0 {
                        for yy in 0..s {
                            for xx in 0..s {
                                canvas.set(cx + col * s + xx, y as i32 + row as i32 * s + yy, idx);
                            }
                        }
                    }
                }
            }
        }
        cx += 6 * s; // 5px glyph + 1px space, scaled
    }
}

// ---------------------------------------------------------------------------
// Scenarios
// ---------------------------------------------------------------------------

struct Scenario {
    name: &'static str,
    positions: Vec<Vec2>,
    /// Gravity as a function of frame index (constant for most; the slosh
    /// scenario reverses it to drive left/right waves).
    gravity: fn(usize) -> Vec2,
}

/// Regular lattice block filling [x0,x1]×[y0,y1] at the PBF/contact spacing.
fn block(x0: f32, y0: f32, x1: f32, y1: f32) -> Vec<Vec2> {
    let s = 2.0 * BALL_SIZE; // 6 px
    let mut v = Vec::new();
    let mut y = y0;
    while y <= y1 {
        let mut x = x0;
        while x <= x1 {
            v.push(Vec2::new(x, y));
            x += s;
        }
        y += s;
    }
    v
}

fn scenarios() -> Vec<Scenario> {
    // Dam break: a tall column against the left wall, released under gravity.
    let dam = Scenario {
        name: "dam_break",
        positions: block(
            BALL_SIZE + 4.0,
            0.30 * HEIGHT,
            0.34 * WIDTH,
            HEIGHT - BALL_SIZE - 4.0,
        ),
        gravity: |_| Vec2::new(0.0, 5.0),
    };

    // Drop + splash: a shallow resting pool plus a compact blob dropped above it.
    let mut splash_pos = block(
        BALL_SIZE + 4.0,
        0.80 * HEIGHT,
        WIDTH - BALL_SIZE - 4.0,
        HEIGHT - BALL_SIZE - 4.0,
    );
    splash_pos.extend(block(
        0.42 * WIDTH,
        0.06 * HEIGHT,
        0.58 * WIDTH,
        0.26 * HEIGHT,
    ));
    let splash = Scenario {
        name: "drop_splash",
        positions: splash_pos,
        gravity: |_| Vec2::new(0.0, 6.0),
    };

    // Slosh tank: a layer of liquid on the floor, gravity tilted left then right
    // on a ~1.3 s period so it sloshes back and forth.
    let slosh = Scenario {
        name: "slosh_tank",
        positions: block(
            BALL_SIZE + 4.0,
            0.62 * HEIGHT,
            WIDTH - BALL_SIZE - 4.0,
            HEIGHT - BALL_SIZE - 4.0,
        ),
        gravity: |frame| {
            let phase = (frame / 32) % 2; // flip every 32 frames (~1.3 s)
            let gx = if phase == 0 { 3.5 } else { -3.5 };
            Vec2::new(gx, 4.0)
        },
    };

    vec![dam, splash, slosh]
}

// ---------------------------------------------------------------------------
// Simulation capture
// ---------------------------------------------------------------------------

/// Run one strategy on a scenario, returning one panel per animation frame.
fn simulate(scenario: &Scenario, strategy: Strategy) -> Vec<Vec<u8>> {
    let n = scenario.positions.len();
    let (_tx, rx) = channel();
    let mut physics = Physics::new(scenario.positions.clone(), vec![Vec2::ZERO; n], rx, 2000.0);
    physics.toggle_adaptive_dt(); // deterministic fixed dt
    physics.set_strategy(strategy);

    let mut share = ShareData {
        c_pos: scenario.positions.clone(),
        c_color: vec![0.0; n],
        ..Default::default()
    };

    let mut panels = Vec::with_capacity(FRAMES);
    for frame in 0..FRAMES {
        physics.set_gravity((scenario.gravity)(frame));
        for _ in 0..SUBSTEPS_PER_FRAME {
            physics.step(PHYS_TIME_STEP, &mut share);
        }
        let mut panel = Canvas::new(PANEL_W, PANEL_H);
        draw_panel(&mut panel, 0, 0, &share);
        panels.push(panel.px);
    }
    panels
}

/// Composite the two runs into one labelled canvas per frame.
fn compose(granular: &[Vec<u8>], pbf: &[Vec<u8>]) -> Vec<Vec<u8>> {
    let lx = MARGIN;
    let rx = MARGIN + PANEL_W + GAP;
    let py = MARGIN + TITLE_H;
    (0..FRAMES)
        .map(|f| {
            let mut c = Canvas::new(CANVAS_W, CANVAS_H);
            // divider
            c.fill_rect(
                MARGIN + PANEL_W + GAP / 2 - 1,
                MARGIN,
                2,
                CANVAS_H - 2 * MARGIN,
                PAL_DIV,
            );
            // blit panels
            blit(&mut c, lx, py, &granular[f]);
            blit(&mut c, rx, py, &pbf[f]);
            // titles
            draw_text(&mut c, lx + 4, MARGIN + 4, "GRANULAR", 2, PAL_TEXT);
            draw_text(&mut c, rx + 4, MARGIN + 4, "PBF", 2, PAL_TEXT);
            c.px
        })
        .collect()
}

fn blit(canvas: &mut Canvas, ox: usize, oy: usize, panel: &[u8]) {
    for y in 0..PANEL_H {
        for x in 0..PANEL_W {
            let idx = panel[y * PANEL_W + x];
            if idx != PAL_BG {
                canvas.px[(oy + y) * canvas.w + (ox + x)] = idx;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// PNG encoder (uncompressed zlib "stored" blocks) — for still verification.
// ---------------------------------------------------------------------------

fn png_write(path: &str, w: usize, h: usize, indices: &[u8], pal: &[[u8; 3]; 256]) {
    // Expand indexed -> RGB with a filter byte (0) per scanline.
    let mut raw = Vec::with_capacity(h * (1 + w * 3));
    for y in 0..h {
        raw.push(0u8);
        for x in 0..w {
            let c = pal[indices[y * w + x] as usize];
            raw.extend_from_slice(&c);
        }
    }

    let mut png: Vec<u8> = vec![0x89, b'P', b'N', b'G', 0x0d, 0x0a, 0x1a, 0x0a];
    // IHDR
    let mut ihdr = Vec::new();
    ihdr.extend_from_slice(&(w as u32).to_be_bytes());
    ihdr.extend_from_slice(&(h as u32).to_be_bytes());
    ihdr.extend_from_slice(&[8, 2, 0, 0, 0]); // bit depth 8, color type 2 (RGB)
    write_chunk(&mut png, b"IHDR", &ihdr);
    // IDAT: zlib wrapper + stored deflate blocks
    let idat = zlib_stored(&raw);
    write_chunk(&mut png, b"IDAT", &idat);
    write_chunk(&mut png, b"IEND", &[]);
    std::fs::write(path, png).unwrap();
}

fn write_chunk(out: &mut Vec<u8>, kind: &[u8; 4], data: &[u8]) {
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    let start = out.len();
    out.extend_from_slice(kind);
    out.extend_from_slice(data);
    let crc = crc32(&out[start..]);
    out.extend_from_slice(&crc.to_be_bytes());
}

fn zlib_stored(data: &[u8]) -> Vec<u8> {
    let mut out = vec![0x78, 0x01]; // zlib header, no compression
    let mut i = 0;
    while i < data.len() {
        let chunk = (data.len() - i).min(0xFFFF);
        let final_block = i + chunk >= data.len();
        out.push(if final_block { 1 } else { 0 }); // BFINAL, BTYPE=00 (stored)
        out.extend_from_slice(&(chunk as u16).to_le_bytes());
        out.extend_from_slice(&(!(chunk as u16)).to_le_bytes());
        out.extend_from_slice(&data[i..i + chunk]);
        i += chunk;
    }
    out.extend_from_slice(&adler32(data).to_be_bytes());
    out
}

fn adler32(data: &[u8]) -> u32 {
    let (mut a, mut b) = (1u32, 0u32);
    for &byte in data {
        a = (a + byte as u32) % 65521;
        b = (b + a) % 65521;
    }
    (b << 16) | a
}

fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

// ---------------------------------------------------------------------------
// Animated GIF encoder (LZW, giflib-compatible code-size rule).
// ---------------------------------------------------------------------------

fn gif_write(path: &str, w: usize, h: usize, frames: &[Vec<u8>], pal: &[[u8; 3]; 256], delay: u16) {
    let mut out: Vec<u8> = Vec::new();
    out.extend_from_slice(b"GIF89a");
    // Logical Screen Descriptor
    out.extend_from_slice(&(w as u16).to_le_bytes());
    out.extend_from_slice(&(h as u16).to_le_bytes());
    out.push(0xF7); // GCT present, 256 entries (size code 7)
    out.push(0); // background color index
    out.push(0); // pixel aspect ratio
                 // Global Color Table
    for c in pal.iter() {
        out.extend_from_slice(c);
    }
    // NETSCAPE loop extension (loop forever)
    out.extend_from_slice(&[0x21, 0xFF, 0x0B]);
    out.extend_from_slice(b"NETSCAPE2.0");
    out.extend_from_slice(&[0x03, 0x01, 0x00, 0x00, 0x00]);

    for frame in frames {
        // Graphic Control Extension (delay only, no transparency)
        out.extend_from_slice(&[0x21, 0xF9, 0x04, 0x00]);
        out.extend_from_slice(&delay.to_le_bytes());
        out.extend_from_slice(&[0x00, 0x00]);
        // Image Descriptor
        out.push(0x2C);
        out.extend_from_slice(&0u16.to_le_bytes()); // left
        out.extend_from_slice(&0u16.to_le_bytes()); // top
        out.extend_from_slice(&(w as u16).to_le_bytes());
        out.extend_from_slice(&(h as u16).to_le_bytes());
        out.push(0x00); // no local color table
        out.push(8); // LZW minimum code size
        let data = lzw_encode(frame, 8);
        // Emit as sub-blocks of <= 255 bytes.
        for chunk in data.chunks(255) {
            out.push(chunk.len() as u8);
            out.extend_from_slice(chunk);
        }
        out.push(0x00); // block terminator
    }
    out.push(0x3B); // trailer
    std::fs::write(path, out).unwrap();
}

/// Variable-width LZW as used by GIF. `min_code_size` = 8 for a 256-color image.
///
/// The code-size bump rule matches giflib exactly: emit a code at the current
/// width, then — using the *pre-increment* next-code value — grow the width
/// once `next >= 2^width`. Doing the bump after `next++` (the naive reading)
/// promotes one code too early and yields a "broken data stream".
fn lzw_encode(indices: &[u8], min_code_size: u32) -> Vec<u8> {
    use std::collections::HashMap;
    let clear = 1u32 << min_code_size; // 256
    let eoi = clear + 1; // 257

    let mut bits = BitWriter::new();
    let mut code_size = min_code_size + 1;
    let mut next = eoi + 1; // 258

    // Emit a code at the current width, then apply the deferred size bump.
    let emit = |bits: &mut BitWriter, code_size: &mut u32, next: u32, code: u32| {
        bits.write(code, *code_size);
        if next >= (1 << *code_size) && *code_size < 12 {
            *code_size += 1;
        }
    };

    emit(&mut bits, &mut code_size, next, clear);
    if indices.is_empty() {
        emit(&mut bits, &mut code_size, next, eoi);
        return bits.finish();
    }

    let mut dict: HashMap<(u32, u8), u32> = HashMap::new();
    let mut prefix = indices[0] as u32;
    for &k in &indices[1..] {
        if let Some(&c) = dict.get(&(prefix, k)) {
            prefix = c;
        } else {
            emit(&mut bits, &mut code_size, next, prefix);
            if next >= 4095 {
                // Dictionary full: reset to match the decoder's reset.
                emit(&mut bits, &mut code_size, next, clear);
                dict.clear();
                code_size = min_code_size + 1;
                next = eoi + 1;
            } else {
                dict.insert((prefix, k), next);
                next += 1;
            }
            prefix = k as u32;
        }
    }
    emit(&mut bits, &mut code_size, next, prefix);
    emit(&mut bits, &mut code_size, next, eoi);
    bits.finish()
}

/// LSB-first bit accumulator into a byte stream.
struct BitWriter {
    out: Vec<u8>,
    acc: u32,
    nbits: u32,
}

impl BitWriter {
    fn new() -> Self {
        Self {
            out: Vec::new(),
            acc: 0,
            nbits: 0,
        }
    }
    fn write(&mut self, code: u32, size: u32) {
        self.acc |= code << self.nbits;
        self.nbits += size;
        while self.nbits >= 8 {
            self.out.push((self.acc & 0xFF) as u8);
            self.acc >>= 8;
            self.nbits -= 8;
        }
    }
    fn finish(mut self) -> Vec<u8> {
        if self.nbits > 0 {
            self.out.push((self.acc & 0xFF) as u8);
        }
        self.out
    }
}

// ---------------------------------------------------------------------------

/// Diagnostic: settle a PBF block and report how well it holds the rest
/// spacing and whether it stays finite. Sweeps the λ cap on both a dense block
/// (should stay incompressible, ρ/ρ0 ≈ 1) and a tiny blob (the sparse case that
/// exploded in the live app — must stay sane). `mean nn` should sit near the
/// rest spacing (6 px).
fn stats_mode() {
    for &lambda_max in &[f32::INFINITY, 100.0, 30.0, 10.0] {
        println!("=== lambda_max = {lambda_max} ===");
        let params = PbfParams {
            lambda_max,
            ..PbfParams::default()
        };
        print!("  dense(40x40): ");
        settle_and_report(params, 40);
        print!("  tiny (5x5):   ");
        settle_and_report(params, 5);
    }
}

fn settle_and_report(params: PbfParams, dim: usize) {
    let s = 2.0 * BALL_SIZE;
    let mut positions = Vec::new();
    for gy in 0..dim {
        for gx in 0..dim {
            positions.push(Vec2::new(400.0 + gx as f32 * s, 300.0 + gy as f32 * s));
        }
    }
    let n = positions.len();
    let (_tx, rx) = channel();
    let mut physics = Physics::new(positions.clone(), vec![Vec2::ZERO; n], rx, 2000.0);
    physics.toggle_adaptive_dt();
    physics.set_strategy(Strategy::Pbf);
    physics.set_pbf_params(params);
    physics.set_gravity(Vec2::new(0.0, 5.0));
    let mut share = ShareData {
        c_pos: positions,
        c_color: vec![0.0; n],
        ..Default::default()
    };
    for _ in 0..1200 {
        physics.step(PHYS_TIME_STEP, &mut share);
    }
    // brute-force nearest neighbor (n is small)
    let p = &share.c_pos;
    let mut nn_sum = 0.0f64;
    let mut clumped = 0usize;
    for i in 0..n {
        let mut best = f32::INFINITY;
        for j in 0..n {
            if i != j {
                best = best.min((p[i] - p[j]).length_squared());
            }
        }
        let d = best.sqrt();
        nn_sum += d as f64;
        if d < 0.5 * s {
            clumped += 1;
        }
    }
    let ymin = p.iter().map(|q| q.y).fold(f32::INFINITY, f32::min);
    let ymax = p.iter().map(|q| q.y).fold(f32::NEG_INFINITY, f32::max);

    // Replicate the PBF poly6 kernel (h=15) to measure achieved vs rest density.
    let h = 15.0f32;
    let h2 = h * h;
    let coef = 4.0 / (std::f32::consts::PI * h2 * h2 * h2 * h2);
    let w = |r2: f32| -> f32 {
        if r2 >= h2 {
            0.0
        } else {
            let d = h2 - r2;
            coef * d * d * d
        }
    };
    // rest density = square lattice at spacing s
    let reach = (h / s).ceil() as i32 + 1;
    let mut rho0 = 0.0f32;
    for iy in -reach..=reach {
        for ix in -reach..=reach {
            rho0 += w(((ix * ix + iy * iy) as f32) * s * s);
        }
    }
    // mean achieved density on a sample of interior particles
    let mut dens_sum = 0.0f64;
    let sample = n.min(400);
    for i in 0..sample {
        let mut rho = 0.0f32;
        for j in 0..n {
            rho += w((p[i] - p[j]).length_squared());
        }
        dens_sum += rho as f64;
    }
    let mean_rho = dens_sum / sample as f64;
    let nan = p
        .iter()
        .filter(|q| !q.x.is_finite() || !q.y.is_finite())
        .count();
    let escaped = p
        .iter()
        .filter(|q| q.x < -1.0 || q.x > WIDTH + 1.0 || q.y < -1.0 || q.y > HEIGHT + 1.0)
        .count();
    let sane = if nan == 0 && escaped == 0 {
        "ok".to_string()
    } else {
        format!("{nan} NaN, {escaped} escaped")
    };
    println!(
        "n={n:<4} mean_nn={:.2} clumped={clumped:<4} height={:.0} ratio={:.2} sane={sane}",
        nn_sum / n as f64,
        ymax - ymin,
        mean_rho / rho0 as f64,
    );
}

/// Write a couple of small GIFs with known pixels for external validation.
/// The palette here is the identity ramp so decoded RGB == index, letting the
/// checker confirm the LZW stream round-trips bit-for-bit.
fn gif_selftest() {
    let mut pal = [[0u8; 3]; 256];
    for (i, c) in pal.iter_mut().enumerate() {
        *c = [i as u8, i as u8, i as u8];
    }
    // Frame 0: gradient that exercises long runs and code growth.
    // Frame 1: pseudo-random to exercise dictionary churn and resets.
    let (w, h) = (64usize, 48usize);
    let mut f0 = vec![0u8; w * h];
    let mut f1 = vec![0u8; w * h];
    let mut seed = 12345u32;
    for i in 0..w * h {
        f0[i] = ((i / w) as u8).wrapping_mul(5);
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        f1[i] = (seed >> 24) as u8;
    }
    std::fs::create_dir_all("selftest").unwrap();
    gif_write("selftest/test.gif", w, h, &[f0, f1], &pal, 10);
    println!("wrote selftest/test.gif ({}x{}, 2 frames)", w, h);
}

/// Headless reproduction of the live app: drive each scenario under both models
/// with the app's fixed-timestep loop and emit the same per-second diagnostic
/// line `WATERSIM_DEBUG=1` prints, then a verdict. This is how we "run with
/// debug" without a display — the numbers are what matter.
fn debug_run() {
    const SECONDS: usize = 6;
    for scenario in scenarios() {
        for strategy in [Strategy::Granular, Strategy::Pbf] {
            let n = scenario.positions.len();
            let (_tx, rx) = channel();
            let mut physics =
                Physics::new(scenario.positions.clone(), vec![Vec2::ZERO; n], rx, 2000.0);
            physics.set_adaptive_dt(false); // fixed timestep, exactly like the app
            physics.set_strategy(strategy);
            let mut share = ShareData {
                c_pos: scenario.positions.clone(),
                c_color: vec![0.0; n],
                ..Default::default()
            };

            let mut worst_speed = 0.0f32;
            let mut worst_ratio = 0.0f32;
            println!("\n=== {} / {strategy:?} ({n} particles) ===", scenario.name);
            for sec in 0..SECONDS {
                for sub in 0..480 {
                    let frame = (sec * 480 + sub) / SUBSTEPS_PER_FRAME;
                    physics.set_gravity((scenario.gravity)(frame));
                    physics.step(PHYS_TIME_STEP, &mut share);
                }
                let ps = &share.perf_stats;
                worst_speed = worst_speed.max(ps.max_speed);
                worst_ratio = worst_ratio.max(ps.pbf_density_ratio);
                let density = if strategy == Strategy::Pbf {
                    format!(" rho/rho0={:.2}", ps.pbf_density_ratio)
                } else {
                    String::new()
                };
                println!(
                    "  t={}s  mean_speed={:6.1}  max_speed={:7.1}{density}",
                    sec + 1,
                    ps.mean_speed,
                    ps.max_speed,
                );
            }
            let nan = share
                .c_pos
                .iter()
                .filter(|p| !p.x.is_finite() || !p.y.is_finite())
                .count();
            let escaped = share
                .c_pos
                .iter()
                .filter(|p| p.x < -1.0 || p.x > WIDTH + 1.0 || p.y < -1.0 || p.y > HEIGHT + 1.0)
                .count();
            let verdict = if nan == 0 && escaped == 0 {
                "STABLE"
            } else {
                "UNSTABLE"
            };
            println!(
                "  verdict: {verdict} (nan={nan}, escaped={escaped}, worst max_speed={worst_speed:.0}, worst rho/rho0={worst_ratio:.2})"
            );
        }
    }
}

/// Tuning harness: drop a block of fluid onto the floor and let it settle,
/// then measure how *calm* it gets. Real water settles to still — residual
/// mean/max speed should decay toward the granular resting level while the
/// fluid stays incompressible (ρ/ρ0 ≈ 1) and doesn't clump. Sweeps the PBF
/// coefficients that inject/damp energy (viscosity, vorticity, s_corr,
/// per-iteration clamp) and prints a table so the calmest stable config wins.
fn tune_mode() {
    let s = 2.0 * BALL_SIZE;
    // A block released above the floor: transient splash, then it should settle.
    let mut positions = Vec::new();
    for gy in 0..44 {
        for gx in 0..70 {
            positions.push(Vec2::new(
                0.30 * WIDTH + gx as f32 * s,
                0.20 * HEIGHT + gy as f32 * s,
            ));
        }
    }
    let n = positions.len();

    // ε=1e-3 (softer density solve) did most of the calming; find the lowest
    // viscosity that still settles so splashes stay lively.
    let base = PbfParams {
        vorticity: 0.0,
        scorr_k: 3.0,
        max_corr: 0.12 * 15.0,
        eps_cfm: 1e-3,
        ..PbfParams::default()
    };
    let configs: &[(&str, PbfParams)] = &[
        (
            "xsph=.05",
            PbfParams {
                xsph_c: 0.05,
                ..base
            },
        ),
        (
            "xsph=.1",
            PbfParams {
                xsph_c: 0.1,
                ..base
            },
        ),
        (
            "xsph=.15",
            PbfParams {
                xsph_c: 0.15,
                ..base
            },
        ),
        (
            "xsph=.2",
            PbfParams {
                xsph_c: 0.2,
                ..base
            },
        ),
        (
            "xsph=.1 eps=2e-3",
            PbfParams {
                xsph_c: 0.1,
                eps_cfm: 2e-3,
                ..base
            },
        ),
        (
            "xsph=.1 k=1.5",
            PbfParams {
                xsph_c: 0.1,
                scorr_k: 1.5,
                ..base
            },
        ),
    ];

    println!("| config | resid mean_spd | resid max_spd | rho/rho0 | clumped% | sane |");
    println!("|---|---|---|---|---|---|");
    for (label, params) in configs {
        let (_tx, rx) = channel();
        let mut physics = Physics::new(positions.clone(), vec![Vec2::ZERO; n], rx, 2000.0);
        physics.set_adaptive_dt(false);
        physics.set_strategy(Strategy::Pbf);
        physics.set_pbf_params(*params);
        physics.set_gravity(Vec2::new(0.0, 5.0));
        let mut share = ShareData {
            c_pos: positions.clone(),
            c_color: vec![0.0; n],
            ..Default::default()
        };
        // 8 s total; average the residual energy over the final 1.5 s.
        let total = 8 * 480;
        let tail = total - 720;
        let mut mean_acc = 0.0f64;
        let mut max_acc = 0.0f64;
        let mut cnt = 0usize;
        for step in 0..total {
            physics.step(PHYS_TIME_STEP, &mut share);
            if step >= tail {
                mean_acc += share.perf_stats.mean_speed as f64;
                max_acc += share.perf_stats.max_speed as f64;
                cnt += 1;
            }
        }
        let p = &share.c_pos;
        let mut clumped = 0usize;
        for i in (0..n).step_by(7) {
            let mut best = f32::INFINITY;
            for j in 0..n {
                if i != j {
                    best = best.min((p[i] - p[j]).length_squared());
                }
            }
            if best.sqrt() < 0.5 * s {
                clumped += 1;
            }
        }
        let sampled = (0..n).step_by(7).count();
        let nan = p
            .iter()
            .filter(|q| !q.x.is_finite() || !q.y.is_finite())
            .count();
        let escaped = p
            .iter()
            .filter(|q| q.x < -1.0 || q.x > WIDTH + 1.0 || q.y < -1.0 || q.y > HEIGHT + 1.0)
            .count();
        println!(
            "| {label} | {:.1} | {:.0} | {:.2} | {:.0}% | {} |",
            mean_acc / cnt as f64,
            max_acc / cnt as f64,
            share.perf_stats.pbf_density_ratio,
            100.0 * clumped as f64 / sampled as f64,
            if nan == 0 && escaped == 0 {
                "ok"
            } else {
                "BAD"
            },
        );
    }
}

/// Settle a block under each of two PBF configs for `secs` seconds and write a
/// side-by-side still, so "residual mean_speed ≈ 20" can be judged by eye:
/// gentle slosh vs unphysical surface boiling.
fn settle_still(secs: usize) {
    let s = 2.0 * BALL_SIZE;
    let mut positions = Vec::new();
    for gy in 0..44 {
        for gx in 0..70 {
            positions.push(Vec2::new(
                0.30 * WIDTH + gx as f32 * s,
                0.20 * HEIGHT + gy as f32 * s,
            ));
        }
    }
    let n = positions.len();
    // Left: the old energetic tuning (stiff ε, weak XSPH, vorticity on).
    // Right: the new calm default.
    let configs: [(&str, PbfParams); 2] = [
        (
            "PAF",
            PbfParams {
                eps_cfm: 1.0e-4,
                scorr_k: 6.0,
                xsph_c: 0.08,
                vorticity: 0.0006,
                max_corr: 0.25 * 15.0,
                ..PbfParams::default()
            },
        ),
        ("PBF", PbfParams::default()),
    ];
    let pal = build_palette();
    let mut canvas = Canvas::new(CANVAS_W, CANVAS_H);
    canvas.fill_rect(
        MARGIN + PANEL_W + GAP / 2 - 1,
        MARGIN,
        2,
        CANVAS_H - 2 * MARGIN,
        PAL_DIV,
    );
    for (idx, (label, params)) in configs.iter().enumerate() {
        let (_tx, rx) = channel();
        let mut physics = Physics::new(positions.clone(), vec![Vec2::ZERO; n], rx, 2000.0);
        physics.set_adaptive_dt(false);
        physics.set_strategy(Strategy::Pbf);
        physics.set_pbf_params(*params);
        physics.set_gravity(Vec2::new(0.0, 5.0));
        let mut share = ShareData {
            c_pos: positions.clone(),
            c_color: vec![0.0; n],
            ..Default::default()
        };
        for _ in 0..secs * 480 {
            physics.step(PHYS_TIME_STEP, &mut share);
        }
        let ox = MARGIN + idx * (PANEL_W + GAP);
        draw_panel(&mut canvas, ox, MARGIN + TITLE_H, &share);
        draw_text(&mut canvas, ox + 4, MARGIN + 4, label, 2, PAL_TEXT);
        println!(
            "{label}: mean_speed={:.1} max_speed={:.0} rho/rho0={:.2}",
            share.perf_stats.mean_speed,
            share.perf_stats.max_speed,
            share.perf_stats.pbf_density_ratio
        );
    }
    std::fs::create_dir_all("renders").unwrap();
    png_write("renders/settle.png", CANVAS_W, CANVAS_H, &canvas.px, &pal);
    println!("wrote renders/settle.png (settled {secs}s)");
}

fn main() {
    if std::env::args().any(|a| a == "--settle-still") {
        settle_still(8);
        return;
    }
    if std::env::args().any(|a| a == "--tune") {
        tune_mode();
        return;
    }
    if std::env::args().any(|a| a == "--gif-selftest") {
        gif_selftest();
        return;
    }
    if std::env::args().any(|a| a == "--stats") {
        stats_mode();
        return;
    }
    if std::env::args().any(|a| a == "--debug-run") {
        debug_run();
        return;
    }
    let pal = build_palette();
    let out_dir = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "renders".to_string());
    std::fs::create_dir_all(&out_dir).unwrap();

    println!(
        "canvas {CANVAS_W}x{CANVAS_H}, {FRAMES} frames @ {FPS} fps, panels {PANEL_W}x{PANEL_H}"
    );

    for scenario in scenarios() {
        let n = scenario.positions.len();
        println!("\n[{}] {} particles", scenario.name, n);

        let t = std::time::Instant::now();
        let granular = simulate(&scenario, Strategy::Granular);
        println!("  granular simulated in {:.1}s", t.elapsed().as_secs_f32());

        let t = std::time::Instant::now();
        let pbf = simulate(&scenario, Strategy::Pbf);
        println!("  pbf simulated in {:.1}s", t.elapsed().as_secs_f32());

        let frames = compose(&granular, &pbf);

        let gif_path = format!("{out_dir}/{}.gif", scenario.name);
        let delay = (100 / FPS) as u16;
        gif_write(&gif_path, CANVAS_W, CANVAS_H, &frames, &pal, delay);
        let gif_kb = std::fs::metadata(&gif_path).unwrap().len() / 1024;
        println!("  wrote {gif_path} ({gif_kb} KB)");

        // A still from ~40% through, for quick visual verification.
        let still = &frames[FRAMES * 2 / 5];
        let png_path = format!("{out_dir}/{}_still.png", scenario.name);
        png_write(&png_path, CANVAS_W, CANVAS_H, still, &pal);
        println!("  wrote {png_path}");
    }
}
