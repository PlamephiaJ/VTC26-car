# CPU performance report

Date: 2026-08-24  
Branch: `gpu_optimization`  
Scope: CPU profiling and optimization only; no CUDA code was added.

## Executive result

The production default is `squared near + SoA hot arrays + DDA collision`, with
the uniform spatial grid disabled. On the fixed Spielberg replay this reduced
planning total latency by 31.5% mean, 31.7% P95, and 31.8% P99. Planning
success remained 500/500 and path length was unchanged at the reported
precision.

| Selected configuration | Mean ms | P95 ms | P99 ms | Success |
|---|---:|---:|---:|---:|
| CPU baseline | 6.622 | 7.298 | 7.578 | 500/500 |
| Recommended | 4.539 | 4.982 | 5.168 | 500/500 |
| Reduction | 31.5% | 31.7% | 31.8% | no change |

The largest useful change was DDA collision traversal. The uniform spatial
grid is a no-go for the current approximately 545-node local trees: it speeds
up nearest-neighbor lookup but makes `near` and overall latency worse.

## Method

- Release build (`-DCMAKE_BUILD_TYPE=Release`), pinned to CPU 0 with
  `taskset -c 0`.
- Host: Intel Core i7-13700HX, 24 logical CPUs.
- Actual `Spielberg.png` map: 2000 x 2000, 0.05796 m/cell, with the configured
  0.20 m static inflation.
- Fixed start `(3.95767, 1.06192)` and goal `(0.568055, 0.150586)`, separated
  by 3.51 m along `filtered.csv`.
- Fixed dynamic input: six retained frames, 400 points/frame (2400 points),
  representing repeated observations of one obstacle away from the tested
  local segment. Rasterization/inflation changed 203 cells.
- 10 unmeasured warm-up plans per variant, followed by 500 measured plans with
  seeds 1000 through 1499. Every variant receives the same map, obstacle input,
  planner parameters, and seed sequence.
- Percentiles use linear interpolation over the sorted samples.

The temporary multi-variant benchmark executable and its libpng dependency
were removed after the production configuration was selected. The methodology
and measured results are retained here as the experiment record.

## Isolated optimization results

| Variant | Mean ms | P95 ms | P99 ms | P95 vs baseline | Decision |
|---|---:|---:|---:|---:|---|
| Baseline | 6.622 | 7.298 | 7.578 | - | reference only |
| Squared near | 6.447 | 7.119 | 7.398 | -2.5% | keep |
| SoA only | 6.605 | 7.318 | 7.763 | +0.3% | keep only in selected combination |
| Uniform grid only | 6.777 | 7.463 | 7.784 | +2.3% | disable |
| DDA only | 4.789 | 5.266 | 5.442 | -27.8% | keep |
| All four combined | 4.844 | 5.368 | 6.435 | -26.4% | reject as default |
| Squared + SoA + DDA | 4.539 | 4.982 | 5.168 | -31.7% | selected default |
| Squared + DDA | 4.597 | 5.061 | 5.252 | -30.7% | simpler fallback |

SoA's isolated result is inside run-to-run noise, but in the DDA/squared
combination it improved mean/P95/P99 by 1.3%/1.6%/1.6% over the simpler
fallback. This is a small positive result. If maintenance simplicity is more
important than roughly 0.08 ms at P95, `structure_of_arrays=false` is a
reasonable fallback.

The grid reduced nearest time from 0.333 ms mean to 0.245 ms, but raised `near`
from 0.800 ms to 1.026 ms and total from 6.622 ms to 6.777 ms. At the current
tree size, hash/bucket traversal costs more than it saves. Retest it only if
iteration counts or tree sizes increase substantially.

## Selected plan() phase profile

| Phase | Baseline mean | Baseline P95 | Baseline P99 | Selected mean | Selected P95 | Selected P99 |
|---|---:|---:|---:|---:|---:|---:|
| Sampling | 0.137 | 0.140 | 0.155 | 0.136 | 0.139 | 0.140 |
| Nearest + steer | 0.333 | 0.348 | 0.355 | 0.332 | 0.346 | 0.353 |
| Initial collision | 0.061 | 0.062 | 0.063 | 0.046 | 0.047 | 0.048 |
| Near | 0.800 | 0.883 | 0.905 | 0.601 | 0.664 | 0.688 |
| Parent collision | 4.624 | 5.137 | 5.374 | 2.805 | 3.130 | 3.249 |
| Rewiring | 0.580 | 0.644 | 0.673 | 0.530 | 0.587 | 0.614 |
| Total | 6.622 | 7.298 | 7.578 | 4.539 | 4.982 | 5.168 |

All values are milliseconds.

The sum of named phases is intentionally below total because total also
includes node insertion, goal selection, path tracing, result materialization,
and timer overhead.

## Success and path quality

| Variant group | Success | Mean path m | P95 path m | Mean tree nodes |
|---|---:|---:|---:|---:|
| Sampled collision variants | 500/500 | 3.510 | 3.512 | 545.774 |
| DDA collision variants | 500/500 | 3.510 | 3.512 | 544.450 |

DDA is cell-exact and more conservative than the old resolution-spaced point
sampling. A unit test covers a shallow segment where the old method misses an
occupied crossed cell and DDA detects it.

## Dynamic-map rebuild at 20 Hz

This is independently measured around `clear -> rasterize -> inflate`, not
included in planning total.

| Stage | Mean ms | P95 ms | P99 ms |
|---|---:|---:|---:|
| Clear | <0.001 | <0.001 | <0.001 |
| Rasterize + inflate | 0.224 | 0.230 | 0.232 |
| Total | 0.224 | 0.230 | 0.232 |

At 20 Hz this rebuild consumes about 4.48 ms of CPU time per second, or 0.45%
of one logical core in this replay. It is not currently a CPU bottleneck, but
the instrumentation is retained for the later CUDA rasterization experiment.

## Validation and limitations

- Release build completed for the core library, ROS node, temporary benchmark,
  and tests. The temporary benchmark target was then removed during cleanup.
- Functional GoogleTests after cleanup: 28/28 passed.
- Existing repository-wide lint remains red for pre-existing copyright/style
  debt and offline XML schema lookup; these failures are not functional test
  regressions.
- This is a deterministic offline planner replay, not a loaded two-car
  simulator run. Publishing, TF, controller work, callback contention, CPU
  frequency control, and vehicle-level outcomes are outside this report.
  Therefore the CPU changes are strong candidates to retain, but the final
  competition go/no-go still requires an end-to-end replay under representative
  simulator load.
