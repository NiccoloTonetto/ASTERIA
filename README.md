# ASTERIA — GNC for the 4v4 magnetic soft-docking bench

Guidance, navigation and control for Project ASTERIA's planar air-bearing bench: a CubeSat-class chaser docking magnetically with a fixed target, closed loop in axial position and yaw, from 60 mm to bare face contact at −20 ± 10 mm/s.

This repository contains the plant and force model, the estimator and controller, the Simulink model generator, the embedded C entry point for the flight computer, the replay and identification tools for hardware tuning, and the tests that verify all of it.

**Status:** validated in simulation (96% Monte Carlo pass rate after calibration), embedded C generated and verified on the host, awaiting hardware.

## Start here

| If you are… | Read |
| --- | --- |
| integrating sensors, drivers or firmware | [`docs/ICD.md`](docs/ICD.md) — numbered requirements, frames, coil order, signs, the `gnc_step` interface, log format |
| running the bench or the calibration campaign | [`docs/RUNBOOK.md`](docs/RUNBOOK.md) |
| tuning the filter on real data | [`docs/TOOLCHAIN.md`](docs/TOOLCHAIN.md) |

## Quick start

```matlab
startup          % from the repository root: sets the path
run_all_tests    % ~2 minutes; checks that this installation reproduces the reference results
```

Expected: `plant`, `replay`, `precision`, `sil`, `simulink` report **pass**; `ekf`, `closed_loop`, `envelope`, `update_lut` report **ran** (they print tables to inspect). A **FAIL** or **ERROR** means the installation does not reproduce the reference: stop there.

Typical session:

```matlab
bench_init_3dof; optimize_maneuver; build_schedule   % parameters, reference, gains
build_simulink_model; out = sim('asteria_model');     % closed-loop simulation
P.mc.fs_range = [0.8 1.25]; monte_carlo               % robustness, 300 runs
```

## Layout

| Folder | Contents |
| --- | --- |
| `src/model` | force model (`asteria_wrench`), plant, sensor model, 1v1 axial model |
| `src/gnc` | EKF, EKF sizing, controller |
| `src/embedded` | code-generation versions and the firmware entry point `gnc_step` |
| `src/simulink` | wrappers called by the generated Simulink model |
| `scripts` | parameter set-up, reference, gain schedule, model and code builders, `update_lut` |
| `analysis` | Monte Carlo |
| `tools` | log generator, replay, identification |
| `tests` | all tests; `run_all_tests.m` at the root runs them |
| `docs` | runbook, ICD, toolchain |
| `data` | **measured** data, tracked: 1v1 calibration CSVs, bench logs worth keeping. Name files with date and rig, e.g. `2026-11-05_1v1_T1.csv` |

Generated files (`asteria_model.slx`, results `*.mat`, simulated logs, `codegen/`, MEX files) are written to the repository root and ignored by git; every one of them is reproducible from the scripts. `asteria_calibration.mat` is generated too: commit the CSV it came from in `data/`, and regenerate it with `update_lut`.

## Requirements

MATLAB R2026a and:

| Toolbox | Needed for |
| --- | --- |
| Simulink, Stateflow | the generated model (MATLAB Function blocks are configured through the Stateflow API) |
| Control System Toolbox | LQR gain schedule |
| Signal Processing Toolbox | innovation whiteness test in `replay_ekf` (`xcorr`) |
| Parallel Computing Toolbox | Monte Carlo (optional; runs serially without it, much slower) |
| MATLAB Coder | MEX build and software-in-the-loop |
| Embedded Coder | target library and library SIL/PIL |

Host SIL of the generated library on macOS also needs the full Xcode app (not only the Command Line Tools).

## Conventions that matter

`x` is the gap (approach velocity is negative). Positive chaser current attracts. Coils 1–2 are on the +y side and give +yaw; coils 3–4 on −y give −yaw. Positive yaw moves the +y side towards the target. Full definitions in the ICD.

## Licence

Apache License 2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE).

## Maintainer

ASTERIA GNC lead. Report issues through this repository.
