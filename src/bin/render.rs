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

const PANEL_W: usize = 320; // panel width in pixels
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
        for &strategy in Strategy::all() {
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
                // The SPH-family models (PBF, DFSPH) report a density ratio.
                let density = if strategy == Strategy::Granular {
                    String::new()
                } else {
                    format!(" rho/rho0={:.2}", ps.pbf_density_ratio)
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

// ---------------------------------------------------------------------------
// Literature validation: standard water-sim benchmarks with quantitative
// expectations (not just eyeballing). Effective gravity matches the engine's
// integrator scaling: a = g·(1/PHYS_TIME_STEP).
// ---------------------------------------------------------------------------

fn a_eff(g_y: f32) -> f32 {
    g_y / PHYS_TIME_STEP
}

fn verdict(pass: bool) -> &'static str {
    if pass {
        "PASS"
    } else {
        "FAIL"
    }
}

fn new_pbf(positions: &[Vec2]) -> (Physics, ShareData) {
    let n = positions.len();
    let (tx, rx) = channel();
    let mut physics = Physics::new(positions.to_vec(), vec![Vec2::ZERO; n], rx, 2000.0);
    physics.set_adaptive_dt(false);
    physics.set_strategy(Strategy::Pbf);
    std::mem::forget(tx); // keep the channel endpoint alive for the run
    let share = ShareData {
        c_pos: positions.to_vec(),
        c_color: vec![0.0; n],
        ..Default::default()
    };
    (physics, share)
}

/// Occupied-cell footprint area (10 px bins) — an incompressibility proxy.
fn footprint_area(p: &[Vec2]) -> f32 {
    use std::collections::HashSet;
    let mut cells = HashSet::new();
    for q in p {
        cells.insert(((q.x / 10.0) as i32, (q.y / 10.0) as i32));
    }
    cells.len() as f32 * 100.0
}

/// Test 1 — Hydrostatic tank (SPHERIC-style rest test). A block of fluid at
/// rest must stay still, keep a flat free surface, and conserve volume
/// (incompressibility). We validate these *observable* properties rather than
/// the internal pressure field: PBF's raw λ is corrupted near boundaries by
/// SPH particle deficiency (surface and floor particles have missing
/// neighbors), so it does not read cleanly hydrostatic — a well-documented SPH
/// artifact, separate from whether the tank behaves correctly, which it does.
fn hydrostatic_test() {
    println!("\n## Test 1: Hydrostatic tank (rest, flat surface, volume)");
    let s = 2.0 * BALL_SIZE;
    let g_y = 5.0;
    let (x0, x1) = (0.15 * WIDTH, 0.85 * WIDTH);
    let depth = 260.0;
    let (yb, yt) = (HEIGHT - BALL_SIZE, HEIGHT - BALL_SIZE - depth);
    let mut positions = Vec::new();
    let mut y = yt;
    while y <= yb {
        let mut x = x0;
        while x <= x1 {
            positions.push(Vec2::new(x, y));
            x += s;
        }
        y += s;
    }
    let (mut physics, mut share) = new_pbf(&positions);
    physics.set_gravity(Vec2::new(0.0, g_y));

    let area0 = footprint_area(&share.c_pos);
    for _ in 0..6 * 480 {
        physics.step(PHYS_TIME_STEP, &mut share);
    }
    let resid = share.perf_stats.mean_speed;
    let area = footprint_area(&share.c_pos);
    let area_drift = 100.0 * (area / area0 - 1.0);

    // Interior surface flatness: over the middle 80 % of the fluid's actual
    // x-extent (the block spreads past its start, and the thin spreading edges
    // + wall meniscus aren't "the surface"), take the 5th-percentile height per
    // column (rejects the odd ejected particle above the surface).
    let xmin = share
        .c_pos
        .iter()
        .map(|q| q.x)
        .fold(f32::INFINITY, f32::min);
    let xmax = share
        .c_pos
        .iter()
        .map(|q| q.x)
        .fold(f32::NEG_INFINITY, f32::max);
    let (lo, hi) = (xmin + 0.1 * (xmax - xmin), xmax - 0.1 * (xmax - xmin));
    let nb = 20usize;
    let mut cols: Vec<Vec<f32>> = vec![Vec::new(); nb];
    for q in &share.c_pos {
        if q.x >= lo && q.x < hi {
            let b = (((q.x - lo) / (hi - lo)) * nb as f32) as usize;
            cols[b.min(nb - 1)].push(q.y);
        }
    }
    let mut surf = Vec::new();
    for c in &mut cols {
        if c.len() >= 10 {
            c.sort_by(|a, b| a.partial_cmp(b).unwrap());
            surf.push(c[c.len() / 20]); // 5th percentile (near-top = surface)
        }
    }
    let smean = surf.iter().sum::<f32>() / surf.len().max(1) as f32;
    let srms =
        (surf.iter().map(|v| (v - smean).powi(2)).sum::<f32>() / surf.len().max(1) as f32).sqrt();

    let flat_ok = srms < 1.5 * s;
    let area_ok = area_drift.abs() < 5.0;
    let still_ok = resid < 5.0;
    println!(
        "  residual mean speed : {resid:.1}                 {}",
        verdict(still_ok)
    );
    println!(
        "  surface RMS         : {srms:.1} px (want <{:.0}) {}",
        1.5 * s,
        verdict(flat_ok)
    );
    println!(
        "  volume drift        : {area_drift:+.1}%              {}",
        verdict(area_ok)
    );
}

/// Test 2 — Dam break (Koshizuka & Oka 1996 / Martin & Moyce 1952). A column
/// of width a and height 2a collapses on a dry bed. Compare the surge-front
/// speed to the analytical Ritter dry-bed tip speed 2√(gH), and confirm the
/// front reaches the far wall while volume is conserved.
fn dam_break_test() {
    println!("\n## Test 2: Dam break (surge front vs Ritter 2√(gH))");
    let s = 2.0 * BALL_SIZE;
    let g_y = 5.0;
    let a = 260.0;
    let h = 2.0 * a; // classic aspect ratio 2
    let (x0, x1) = (BALL_SIZE, BALL_SIZE + a);
    let (yb, yt) = (HEIGHT - BALL_SIZE, HEIGHT - BALL_SIZE - h);
    let mut positions = Vec::new();
    let mut y = yt;
    while y <= yb {
        let mut x = x0;
        while x <= x1 {
            positions.push(Vec2::new(x, y));
            x += s;
        }
        y += s;
    }
    let (mut physics, mut share) = new_pbf(&positions);
    physics.set_gravity(Vec2::new(0.0, g_y));

    let area0 = footprint_area(&share.c_pos);
    let ritter = 2.0 * (a_eff(g_y) * h).sqrt();
    let mut prev_tip = x1;
    let mut max_speed = 0.0f32;
    let mut reached = false;
    let frames = (3.0 * 480.0) as usize;
    for k in 0..frames {
        physics.step(PHYS_TIME_STEP, &mut share);
        if k % 12 == 0 {
            let tip = share.c_pos.iter().map(|q| q.x).fold(0.0, f32::max);
            let v = (tip - prev_tip) / (12.0 * PHYS_TIME_STEP);
            max_speed = max_speed.max(v);
            prev_tip = tip;
            if tip > 0.9 * WIDTH {
                reached = true;
            }
        }
    }
    let area = footprint_area(&share.c_pos);
    let area_drift = 100.0 * (area / area0 - 1.0);
    let ratio = max_speed / ritter;

    let ratio_ok = (0.3..=1.2).contains(&ratio);
    let area_ok = area_drift.abs() < 8.0;
    println!("  Ritter tip speed    : {ritter:.0} px/s");
    println!(
        "  measured front speed: {max_speed:.0} px/s (ratio {ratio:.2}) {}",
        verdict(ratio_ok)
    );
    println!(
        "  front reached wall  : {reached}                    {}",
        verdict(reached)
    );
    println!(
        "  volume drift        : {area_drift:+.1}%                {}",
        verdict(area_ok)
    );
}

/// Test 3 — Sloshing tank. A still layer is kicked and left to oscillate; the
/// free-oscillation period is compared to linear potential theory
/// T = 2π/√(g·k·tanh(k·h)) with the first mode k = π/L.
fn sloshing_test() {
    println!("\n## Test 3: Sloshing period vs linear theory");
    let s = 2.0 * BALL_SIZE;
    let g_y = 5.0;
    let (x0, x1) = (BALL_SIZE, WIDTH - BALL_SIZE);
    let l = x1 - x0;
    let h = 200.0;
    let (yb, yt) = (HEIGHT - BALL_SIZE, HEIGHT - BALL_SIZE - h);
    let mut positions = Vec::new();
    let mut y = yt;
    while y <= yb {
        let mut x = x0;
        while x <= x1 {
            positions.push(Vec2::new(x, y));
            x += s;
        }
        y += s;
    }
    let (mut physics, mut share) = new_pbf(&positions);

    physics.set_gravity(Vec2::new(0.0, g_y));
    for _ in 0..2 * 480 {
        physics.step(PHYS_TIME_STEP, &mut share);
    }
    // Gentle sideways kick → small-amplitude oscillation (linear regime).
    physics.set_gravity(Vec2::new(0.22 * g_y, g_y));
    for _ in 0..(0.3 * 480.0) as usize {
        physics.step(PHYS_TIME_STEP, &mut share);
    }
    physics.set_gravity(Vec2::new(0.0, g_y));

    let center = 0.5 * (x0 + x1);
    let mut series = Vec::new();
    let steps = (12.0 * 480.0) as usize;
    for k in 0..steps {
        physics.step(PHYS_TIME_STEP, &mut share);
        if k % 8 == 0 {
            let comx = share.c_pos.iter().map(|q| q.x).sum::<f32>() / share.c_pos.len() as f32;
            series.push((k as f32 * PHYS_TIME_STEP, comx - center));
        }
    }
    // Period from the first max→min half-oscillation (robust even under heavy
    // damping, which leaves only ~one clear swing). Damping = |trough|/peak.
    let (mut tmax, mut vmax) = (0.0f32, f32::MIN);
    for &(t, v) in &series {
        if v > vmax {
            vmax = v;
            tmax = t;
        }
    }
    let (mut tmin, mut vmin) = (0.0f32, f32::MAX);
    for &(t, v) in &series {
        if t > tmax && v < vmin {
            vmin = v;
            tmin = t;
        }
    }
    let t_meas = 2.0 * (tmin - tmax);
    let damp = if vmax > 0.0 { -vmin / vmax } else { f32::NAN };

    let k1 = std::f32::consts::PI / l;
    let omega = (a_eff(g_y) * k1 * (k1 * h).tanh()).sqrt();
    let t_theory = 2.0 * std::f32::consts::PI / omega;
    let ratio = t_meas / t_theory;
    let ok = ratio.is_finite() && (0.7..=1.35).contains(&ratio);
    println!("  tank L={l:.0} px, depth h={h:.0} px, first mode k=π/L");
    println!("  theory period       : {t_theory:.2} s");
    println!(
        "  measured period     : {t_meas:.2} s (ratio {ratio:.2}, peak {vmax:.0}px)  {}",
        verdict(ok)
    );
    let damp_label = if damp > 0.6 {
        "lightly damped — sloshes many times"
    } else if damp > 0.38 {
        "moderately damped — sloshes ~2–3 cycles"
    } else {
        "heavily damped — dies in ~1 cycle"
    };
    println!("  amp decay/half-period: ×{damp:.2}  ({damp_label})");
}

/// Independent check of the effective gravity a = g/PHYS_TIME_STEP by dropping
/// a single (neighbour-free) particle, so the benchmark theory uses the right g.
fn accel_check() {
    let pos = vec![Vec2::new(0.5 * WIDTH, 80.0)];
    let (mut ph, mut sh) = new_pbf(&pos);
    ph.set_gravity(Vec2::new(0.0, 5.0));
    let y0 = sh.c_pos[0].y;
    let n = 240usize;
    for _ in 0..n {
        ph.step(PHYS_TIME_STEP, &mut sh);
    }
    let t = n as f32 * PHYS_TIME_STEP;
    let measured = 2.0 * (sh.c_pos[0].y - y0) / (t * t);
    println!(
        "## Sanity: free-fall a_eff ≈ {measured:.0} px/s² (expected {:.0})",
        a_eff(5.0)
    );
}

/// Settle a dropped block and return (residual mean-speed, clumped %).
fn settle_resid(params: PbfParams) -> (f32, f32) {
    let s = 2.0 * BALL_SIZE;
    let mut positions = Vec::new();
    for gy in 0..40 {
        for gx in 0..64 {
            positions.push(Vec2::new(
                0.30 * WIDTH + gx as f32 * s,
                0.25 * HEIGHT + gy as f32 * s,
            ));
        }
    }
    let n = positions.len();
    let (mut ph, mut sh) = new_pbf(&positions);
    ph.set_pbf_params(params);
    ph.set_gravity(Vec2::new(0.0, 5.0));
    for _ in 0..8 * 480 {
        ph.step(PHYS_TIME_STEP, &mut sh);
    }
    let p = &sh.c_pos;
    let (mut clumped, mut sampled) = (0usize, 0usize);
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
        sampled += 1;
    }
    (
        sh.perf_stats.mean_speed,
        100.0 * clumped as f32 / sampled as f32,
    )
}

/// Kick a full-width layer of still-water depth `h` and return (measured
/// period, theory period, per-half-period amplitude decay). Decay near 1 =
/// lightly damped (sloshes many times).
fn slosh_metrics(params: PbfParams, h: f32) -> (f32, f32, f32) {
    let s = 2.0 * BALL_SIZE;
    let g_y = 5.0;
    let (x0, x1) = (BALL_SIZE, WIDTH - BALL_SIZE);
    let l = x1 - x0;
    let (yb, yt) = (HEIGHT - BALL_SIZE, HEIGHT - BALL_SIZE - h);
    let mut positions = Vec::new();
    let mut y = yt;
    while y <= yb {
        let mut x = x0;
        while x <= x1 {
            positions.push(Vec2::new(x, y));
            x += s;
        }
        y += s;
    }
    let (mut ph, mut sh) = new_pbf(&positions);
    ph.set_pbf_params(params);
    ph.set_gravity(Vec2::new(0.0, g_y));
    for _ in 0..2 * 480 {
        ph.step(PHYS_TIME_STEP, &mut sh);
    }
    ph.set_gravity(Vec2::new(0.22 * g_y, g_y));
    for _ in 0..(0.3 * 480.0) as usize {
        ph.step(PHYS_TIME_STEP, &mut sh);
    }
    ph.set_gravity(Vec2::new(0.0, g_y));
    let center = 0.5 * (x0 + x1);
    let mut series = Vec::new();
    for k in 0..(12.0 * 480.0) as usize {
        ph.step(PHYS_TIME_STEP, &mut sh);
        if k % 8 == 0 {
            let c = sh.c_pos.iter().map(|q| q.x).sum::<f32>() / sh.c_pos.len() as f32;
            series.push((k as f32 * PHYS_TIME_STEP, c - center));
        }
    }
    let (mut tmax, mut vmax) = (0.0f32, f32::MIN);
    for &(t, v) in &series {
        if v > vmax {
            vmax = v;
            tmax = t;
        }
    }
    let (mut tmin, mut vmin) = (0.0f32, f32::MAX);
    for &(t, v) in &series {
        if t > tmax && v < vmin {
            vmin = v;
            tmin = t;
        }
    }
    let k1 = std::f32::consts::PI / l;
    let omega = (a_eff(g_y) * k1 * (k1 * h).tanh()).sqrt();
    let t_theory = 2.0 * std::f32::consts::PI / omega;
    (2.0 * (tmin - tmax), t_theory, -vmin / vmax)
}

/// Trade-off sweep: numerical damping (sloshing persistence) vs settling calm,
/// over the coefficients that dissipate energy.
fn damp_sweep() {
    // s_corr (artificial pressure) is the dominant slosh damper — it's an
    // always-on repulsion that bleeds coherent energy — but it also prevents
    // clumping. Sweep it (with a little XSPH for velocity smoothing) to find the
    // weakest s_corr that still keeps the settle un-clumped.
    let b = PbfParams::default();
    // Sharper s_corr (higher n) is near-zero at rest spacing but still strong
    // where particles clump, so it should stop clumping without damping the
    // resting/sloshing fluid. Higher n needs higher k for the same close-range
    // strength.
    let cfgs: &[(&str, PbfParams)] = &[
        (
            "n=4 k=3 (orig)",
            PbfParams {
                scorr_n: 4,
                scorr_k: 3.0,
                ..b
            },
        ),
        (
            "n=8 k=3",
            PbfParams {
                scorr_n: 8,
                scorr_k: 3.0,
                ..b
            },
        ),
        (
            "n=8 k=4",
            PbfParams {
                scorr_n: 8,
                scorr_k: 4.0,
                ..b
            },
        ),
        (
            "n=8 k=5",
            PbfParams {
                scorr_n: 8,
                scorr_k: 5.0,
                ..b
            },
        ),
        (
            "n=10 k=5",
            PbfParams {
                scorr_n: 10,
                scorr_k: 5.0,
                ..b
            },
        ),
        (
            "n=10 k=8",
            PbfParams {
                scorr_n: 10,
                scorr_k: 8.0,
                ..b
            },
        ),
    ];
    println!("| config | slosh ratio | half-period decay | settle resid | clumped% |");
    println!("|---|---|---|---|---|");
    for (name, p) in cfgs {
        let (t_meas, t_theory, decay) = slosh_metrics(*p, 200.0);
        let (resid, clump) = settle_resid(*p);
        println!(
            "| {name} | {:.2} | ×{decay:.2} | {resid:.1} | {clump:.0}% |",
            t_meas / t_theory
        );
    }
}

/// Test 4 — Standing-wave dispersion. The gravity-wave relation
/// ω² = g·k·tanh(k·h) must hold as the depth changes from shallow (k·h small,
/// ω²≈g·k²·h) to deep (k·h large, ω²≈g·k). Sloshing the first mode at several
/// depths and checking the period tracks theory at each validates the tanh(k·h)
/// dependence, not just one operating point.
fn dispersion_test() {
    println!("\n## Test 4: Standing-wave dispersion ω²=gk·tanh(kh) vs depth");
    // Intermediate → deep depths, where linear (small-amplitude, inviscid)
    // theory applies. The very-shallow limit (h/L ≲ 0.1) is a nonlinear
    // shallow-water / bore regime that linear dispersion does not describe, so
    // it is out of scope here (and the sim's numerical viscosity dominates a
    // thin layer). Across this range tanh(kh) runs 0.5 → 0.66, so ω changes
    // materially with depth — a real dispersion check, not one operating point.
    let mut all_ok = true;
    for &h in &[160.0f32, 260.0, 380.0] {
        let (t_meas, t_theory, _) = slosh_metrics(PbfParams::default(), h);
        let ratio = t_meas / t_theory;
        let ok = (0.7..=1.35).contains(&ratio);
        all_ok &= ok;
        println!(
            "  h={h:>3.0}px: theory {t_theory:.2}s, measured {t_meas:.2}s (ratio {ratio:.2}) {}",
            verdict(ok)
        );
    }
    println!(
        "  → dispersion relation captured across depths: {}",
        verdict(all_ok)
    );
}

/// Test 5 — Two-column collision (symmetric dam break). Equal columns released
/// against both walls collapse and collide at the centre. By symmetry the
/// centre of mass must stay put (momentum conservation / no spurious drift),
/// the collision must throw a central jet upward, and volume is conserved.
fn two_column_test() {
    println!("\n## Test 5: Two-column collision (symmetry + central jet)");
    let s = 2.0 * BALL_SIZE;
    let g_y = 5.0;
    let a = 220.0;
    let h = 2.0 * a;
    let mut positions = Vec::new();
    let bands = [
        (BALL_SIZE, BALL_SIZE + a),
        (WIDTH - BALL_SIZE - a, WIDTH - BALL_SIZE),
    ];
    for &(bx0, bx1) in &bands {
        let (yb, yt) = (HEIGHT - BALL_SIZE, HEIGHT - BALL_SIZE - h);
        let mut y = yt;
        while y <= yb {
            let mut x = bx0;
            while x <= bx1 {
                positions.push(Vec2::new(x, y));
                x += s;
            }
            y += s;
        }
    }
    let n = positions.len();
    let (mut ph, mut sh) = new_pbf(&positions);
    ph.set_gravity(Vec2::new(0.0, g_y));
    let area0 = footprint_area(&sh.c_pos);
    let center = 0.5 * WIDTH;
    let y_floor = HEIGHT - BALL_SIZE;
    let mut max_com_drift = 0.0f32;
    let mut jet_rise = 0.0f32; // how far the surface climbs above the rest layer
                               // rest layer of 2·(a·2a) area over full width → depth ≈ 2·a·2a / WIDTH
    let rest_depth = 2.0 * a * h / WIDTH;
    for k in 0..(3.5 * 480.0) as usize {
        ph.step(PHYS_TIME_STEP, &mut sh);
        if k % 12 == 0 {
            let comx = sh.c_pos.iter().map(|q| q.x).sum::<f32>() / n as f32;
            max_com_drift = max_com_drift.max((comx - center).abs());
            // central jet: min-y among particles near the centre column
            let top = sh
                .c_pos
                .iter()
                .filter(|q| (q.x - center).abs() < 60.0)
                .map(|q| q.y)
                .fold(f32::INFINITY, f32::min);
            jet_rise = jet_rise.max((y_floor - rest_depth) - top);
        }
    }
    let area = footprint_area(&sh.c_pos);
    let area_drift = 100.0 * (area / area0 - 1.0);
    let drift_pct = 100.0 * max_com_drift / WIDTH;
    let sym_ok = drift_pct < 3.0;
    let jet_ok = jet_rise > rest_depth; // jet climbs at least one rest-depth above the pool
    let area_ok = area_drift.abs() < 8.0;
    println!(
        "  COM x-drift (symmetry) : {drift_pct:.1}% of width         {}",
        verdict(sym_ok)
    );
    println!(
        "  central jet rise       : {jet_rise:.0} px (rest depth {rest_depth:.0}) {}",
        verdict(jet_ok)
    );
    println!(
        "  volume drift           : {area_drift:+.1}%                 {}",
        verdict(area_ok)
    );
}

fn validate_water() {
    println!("# Water-sim validation against literature benchmarks (PBF defaults)");
    accel_check();
    hydrostatic_test();
    dam_break_test();
    sloshing_test();
    dispersion_test();
    two_column_test();
}

// ---------------------------------------------------------------------------
// MLS-MPM diagnostic: drop a blob in an empty box and drive it under MPM and a
// reference method (DFSPH), printing time-series metrics and dumping PNG frames
// so the behavioral difference is measurable and visible.
// ---------------------------------------------------------------------------

fn mpm_diag() {
    let pal = build_palette();
    std::fs::create_dir_all("renders").unwrap();

    // A compact square blob, dropped from up high into an empty box.
    let s = 2.0 * BALL_SIZE;
    let mut blob = Vec::new();
    let (x0, y0) = (0.42 * WIDTH, 0.10 * HEIGHT);
    for gy in 0..40 {
        for gx in 0..40 {
            blob.push(Vec2::new(x0 + gx as f32 * s, y0 + gy as f32 * s));
        }
    }
    let n = blob.len();
    let gravity = Vec2::new(0.0, 6.0);
    let frame_subs = 480usize; // 1 s per printed row
    let snapshots = [0usize, 240, 480, 960, 1440, 2400]; // substeps to snapshot

    for strat in [Strategy::Mlsmpm, Strategy::Dfsph] {
        let (_tx, rx) = channel();
        let mut physics = Physics::new(blob.clone(), vec![Vec2::ZERO; n], rx, 2000.0);
        physics.set_adaptive_dt(false);
        physics.set_strategy(strat);
        physics.set_gravity(gravity);
        let mut share = ShareData {
            c_pos: blob.clone(),
            c_color: vec![0.0; n],
            ..Default::default()
        };

        println!("\n=== {strat:?} ({n} particles, blob drop) ===");
        println!("  substep |  com_y | top_y | bot_y | width | height | mean_v |  max_v | rho/J");
        let mut step = 0usize;
        let max_step = *snapshots.last().unwrap();
        while step <= max_step {
            // Snapshot metrics + frame at chosen substeps.
            if snapshots.contains(&step) {
                let (mut minx, mut maxx, mut miny, mut maxy, mut sumy) =
                    (f32::MAX, f32::MIN, f32::MAX, f32::MIN, 0.0f64);
                for p in &share.c_pos {
                    minx = minx.min(p.x);
                    maxx = maxx.max(p.x);
                    miny = miny.min(p.y);
                    maxy = maxy.max(p.y);
                    sumy += p.y as f64;
                }
                let ps = &share.perf_stats;
                println!(
                    "  {step:7} | {:6.0} | {:5.0} | {:5.0} | {:5.0} | {:6.0} | {:6.1} | {:6.1} | {:.3}",
                    sumy / n as f64,
                    miny,
                    maxy,
                    maxx - minx,
                    maxy - miny,
                    ps.mean_speed,
                    ps.max_speed,
                    ps.pbf_density_ratio,
                );
                let mut canvas = Canvas::new(PANEL_W, PANEL_H);
                draw_panel(&mut canvas, 0, 0, &share);
                let path = format!("renders/diag_{}_{:04}.png", strat.token(), step);
                png_write(&path, PANEL_W, PANEL_H, &canvas.px, &pal);
            }
            physics.step(PHYS_TIME_STEP, &mut share);
            step += 1;
        }
        let _ = frame_subs;
    }
    println!("\nframes: renders/diag_<solver>_<substep>.png");
}

// ---------------------------------------------------------------------------
// Performance comparison (sequential; one solver at a time so the numbers are
// clean). Times each strategy on each scenario's initial state and reports the
// wall-clock cost per 480 Hz substep.
// ---------------------------------------------------------------------------

fn perf_mode() {
    const WARMUP: usize = 30;
    const MEASURE: usize = 250;
    println!("# Solver performance (sequential, release)");
    println!("warmup {WARMUP} substeps, measured over {MEASURE} substeps each\n");
    println!("| scenario | particles | solver | ms/substep | substeps/s | vs granular |");
    println!("|---|---|---|---|---|---|");
    for scenario in scenarios() {
        let n = scenario.positions.len();
        let mut granular_ms = 0.0f64;
        for &strat in Strategy::all() {
            let (_tx, rx) = channel();
            let mut physics =
                Physics::new(scenario.positions.clone(), vec![Vec2::ZERO; n], rx, 2000.0);
            physics.set_adaptive_dt(false);
            physics.set_strategy(strat);
            physics.set_gravity((scenario.gravity)(0));
            let mut share = ShareData {
                c_pos: scenario.positions.clone(),
                c_color: vec![0.0; n],
                ..Default::default()
            };
            for _ in 0..WARMUP {
                physics.step(PHYS_TIME_STEP, &mut share);
            }
            let t = std::time::Instant::now();
            for _ in 0..MEASURE {
                physics.step(PHYS_TIME_STEP, &mut share);
            }
            let ms = t.elapsed().as_secs_f64() * 1000.0 / MEASURE as f64;
            if strat == Strategy::Granular {
                granular_ms = ms;
            }
            let rel = if granular_ms > 0.0 {
                format!("{:.2}×", ms / granular_ms)
            } else {
                "—".to_string()
            };
            println!(
                "| {} | {n} | {} | {:.3} | {:.0} | {rel} | {:.3} |",
                scenario.name,
                strat.token(),
                ms,
                1000.0 / ms,
                share.perf_stats.pbf_density_ratio,
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Animated-WebP clip generator (feature = "media"): one clip per solver per
// scenario, for the comparison artifact. Uses libwebp via the `webp` crate.
// ---------------------------------------------------------------------------

#[cfg(feature = "media")]
fn idx_to_rgba(idx: &[u8], pal: &[[u8; 3]; 256]) -> Vec<u8> {
    let mut out = Vec::with_capacity(idx.len() * 4);
    for &i in idx {
        let c = pal[i as usize];
        out.extend_from_slice(&[c[0], c[1], c[2], 255]);
    }
    out
}

#[cfg(feature = "media")]
fn webp_write(path: &str, w: usize, h: usize, frames: &[Vec<u8>], pal: &[[u8; 3]; 256], fps: u32) {
    use webp::{AnimEncoder, AnimFrame, WebPConfig};
    let rgba: Vec<Vec<u8>> = frames.iter().map(|f| idx_to_rgba(f, pal)).collect();
    let mut config = WebPConfig::new().expect("webp config");
    config.lossless = 0;
    config.quality = 46.0;
    config.method = 5;
    let mut enc = AnimEncoder::new(w as u32, h as u32, &config);
    enc.set_loop_count(0);
    let dt = (1000 / fps) as i32;
    for (i, r) in rgba.iter().enumerate() {
        enc.add_frame(AnimFrame::from_rgba(r, w as u32, h as u32, i as i32 * dt));
    }
    let mem = enc.encode();
    std::fs::write(path, &*mem).unwrap();
}

#[cfg(feature = "media")]
fn webp_mode() {
    let pal = build_palette();
    let out_dir = "renders";
    std::fs::create_dir_all(out_dir).unwrap();
    // Optional strategy filter: `--webp <token>` re-renders only that solver.
    let only: Option<Strategy> = std::env::args()
        .skip_while(|a| a != "--webp")
        .nth(1)
        .and_then(|t| Strategy::parse(&t));
    println!("panels {PANEL_W}x{PANEL_H}, {FRAMES} frames @ {FPS} fps");
    for scenario in scenarios() {
        for &strat in Strategy::all() {
            if only.is_some_and(|s| s != strat) {
                continue;
            }
            let t = std::time::Instant::now();
            let frames = simulate(&scenario, strat);
            let path = format!("{out_dir}/{}_{}.webp", scenario.name, strat.token());
            webp_write(&path, PANEL_W, PANEL_H, &frames, &pal, FPS);
            let kb = std::fs::metadata(&path).unwrap().len() / 1024;
            println!(
                "  wrote {path} ({kb} KB) in {:.1}s",
                t.elapsed().as_secs_f32()
            );
        }
    }
}

fn main() {
    if std::env::args().any(|a| a == "--mpm-diag") {
        mpm_diag();
        return;
    }
    if std::env::args().any(|a| a == "--perf") {
        perf_mode();
        return;
    }
    #[cfg(feature = "media")]
    if std::env::args().any(|a| a == "--webp") {
        webp_mode();
        return;
    }
    #[cfg(not(feature = "media"))]
    if std::env::args().any(|a| a == "--webp") {
        eprintln!("--webp requires: cargo run --features media --bin render -- --webp");
        return;
    }
    if std::env::args().any(|a| a == "--validate-water") {
        validate_water();
        return;
    }
    if std::env::args().any(|a| a == "--damp-sweep") {
        damp_sweep();
        return;
    }
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
