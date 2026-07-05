# Benchmark 01 — Build configuration (thin LTO + target-cpu=native)

- Date: 2026-07-01
- Change: `[profile.release] lto = "thin"` in Cargo.toml; `-C target-cpu=native`
  via `.cargo/config.toml`. No code changes.
- Compare against: `docs/benchmarks/00-baseline.md`

## Result

| particles | force path | steps | mean µs/step | median | p95 | min | max | 480 Hz real-time? | sane? |
|---|---|---|---|---|---|---|---|---|---|
| 1000 | barnes-hut | 300 | 4464 | 4483 | 4709 | 3857 | 5059 | NO | ok |
| 1000 | verlet-lists | 300 | 2954 | 2931 | 3146 | 2793 | 3395 | NO | ok |
| 1000 | spatial-hash | 300 | 3467 | 3467 | 3691 | 3192 | 3916 | NO | ok |
| 3000 | barnes-hut | 200 | 9392 | 9493 | 9933 | 6987 | 10726 | NO | ok |
| 3000 | verlet-lists | 200 | 3308 | 3267 | 3504 | 3181 | 4103 | NO | ok |
| 3000 | spatial-hash | 200 | 3894 | 3841 | 4228 | 3564 | 5248 | NO | ok |
| 6000 | barnes-hut | 120 | 16245 | 17009 | 17396 | 12094 | 19538 | NO | ok |
| 6000 | verlet-lists | 120 | 3917 | 3882 | 4140 | 3824 | 4205 | NO | ok |
| 6000 | spatial-hash | 120 | 5120 | 4599 | 6664 | 4339 | 7268 | NO | ok |
| 12000 | barnes-hut | 60 | 29147 | 27562 | 35127 | 23676 | 43358 | NO | ok |
| 12000 | verlet-lists | 60 | 5182 | 5020 | 5639 | 4914 | 6821 | NO | ok |
| 12000 | spatial-hash | 60 | 5672 | 5649 | 6003 | 5327 | 6639 | NO | ok |

## Delta vs baseline (median µs/step)

Small but consistent win, ~2–5% on the grid paths (e.g. verlet 3000: 3437 → 3267;
verlet 12000: 5369 → 5020; barnes-hut 12000: 29098 → 27562).

## Negative result worth recording

`codegen-units = 1` — commonly recommended alongside LTO — was **~15–20% slower**
than the stock profile on this workload/toolchain (verlet 1000 median 3012 → 3768 µs
in A/B quick runs) and was rejected. `target-cpu=native` alone was neutral-to-slightly
positive; kept since this sim always runs on the machine that builds it. All flag
combinations were A/B tested with reruns to rule out machine drift.
