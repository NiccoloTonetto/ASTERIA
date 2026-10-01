# Replay and identification toolchain

Companion to `RUNBOOK.md` (§6 Tuning on hardware) and `ICD.md` (§5.2 Log record). The runbook is the procedure; this document is the tooling, what it has been proven to do, and what the proofs showed was wrong.

**Goal:** extract the model from the data instead of tuning the filter until it stops complaining. Every parameter is measured or fitted from a log, with an independent cross-check where one exists. The filter's noise parameters are touched last.

## Tools

| Tool | Does |
| --- | --- |
| `make_log(P, scenario, c, file)` | Hardware-format log from the simulator with injected truth: force scale, softening, mass, tilt, gyro/accel bias, per-sensor ToF offset and scale, latency, coil current lag, noise. Scenarios `rest`, `drift` (`x_start`, `v0`), `pulse`, `approach` |
| `replay_ekf(file, P)` | Re-runs the EKF on a log with measured currents; per channel: innovation bias, NIS, whiteness; true error when truth is present |
| `identify_from_log(file, P, stage)` | `rest`: sensor offset/scale/noise, IMU biases. `drift`: tilt (parabola + EKF). `latency`: delay, **against truth only**. `pulse`: force scale jointly with initial state |
| `test_replay`, `test_precision`, `test_sil`, `test_sil_lib` | Validation ladder and code verification |

## What each estimator does

- **`rest` (S1)** — regress each ToF reading against `gap + recess`: slope is scale error, intercept is offset, residual σ is the noise. Gyro/accelerometer means and σ. A measurement, not a fit; it freezes R.
- **`drift` (S2)** — drives off. Tilt from a model-free parabola through the raw range (acceleration = 2 × quadratic term), cross-checked against the EKF's own disturbance state. If they disagree, suspect the timestamps.
- **`latency` (S2)** — needs truth columns. Sweeps the delay and minimises the replay's position error against the camera. Use a log with changing velocity.
- **`pulse` (S3)** — known open-loop currents; integrates the plant from the measured currents and fits force scale, initial gap and initial velocity to the measured trajectory. The filter is not involved, so a mismatch can only be the force model or the input path. Feeds `update_lut`.

## Validation results (truth the filter was never told)

| Test | Injected | Recovered |
| --- | --- | --- |
| V1 ToF offsets / scales | +2.0 / −3.0 mm, 1.010 / 0.990 | +1.98 / −3.15 mm, 1.0099 / 0.9899 |
| V1 noise, gyro bias, accel bias | 5.0 mm, 0.300 °/s, 0.050 m/s² | 4.93 mm, 0.3006 °/s, 0.0501 m/s² |
| V2 table tilt | 2.568e-4 m/s² | parabola 2.605e-4, EKF 2.658e-4 |
| V2 ToF latency | 25.0 ms | 25.4 ms, against camera truth |
| V3 force scale | 1.30 / 0.70 / 1.00 | 1.333 / 0.728 / 1.068 |
| V6 single-precision storage | — | 0.155 µm vs 1.107 mm estimation error: safe |

## V5: fault injection

| Injected fault | First predicted | Measured |
| --- | --- | --- |
| none | all pass | all pass |
| ToF offset +3 mm, both | ToF mean fails | whiteness fails, mean passes; estimate 3.6 mm wrong |
| ToF latency 30 ms | whiteness fails | **nothing fails** |
| force scale 1.5 | ToF mean fails | gyro NIS + whiteness fail, ToF passes |
| gyro bias 1 °/s | gyro mean fails | nothing fails — its state absorbs it, correctly |

**Rule:** innovation tests catch model errors that change the *shape* of the dynamics and miss anything the state vector can absorb. The validation camera is not optional.

## Corrections found by auditing the tools

| Finding | Effect | Fix |
| --- | --- | --- |
| The first tilt "fit" optimised a prior the EKF never read | cost flat in tilt; its apparent success was `fminsearch`'s default 2.5e-4 initial step landing on the injected 2.57e-4 | parabola + EKF cross-check; never fit a parameter with no path into the replayed model |
| `asteria_ekf` cached the mass matrix | a changed `P.m` ignored until restart | rebuilt every call |
| Simulink wrappers cached `P` | new start gap / calibration ignored | `StartFcn` / `InitFcn` flush and regenerate |
| Harnesses kept the EKF Jacobian between runs | first 19 steps used the previous run's Jacobian | `clear asteria_ekf` per run |
| One parameter for sensor delay and filter belief | "compensated vs not" silently removed the delay too | `P.test.tof_latency` vs `P.tof.latency` |
| Single-seed conclusions | two reported findings were noise | performance claims from ≥ 60 seeds or the 300-run Monte Carlo |
| `fminsearch` zero start | zero-valued parameters get a 0.00025 step in their own units; fits stall | scaled, non-zero starts |
| Force-scale fit from one noisy sample | a 5 mm range sample had more leverage than the effect | initial gap and velocity fitted jointly |

## Code verification

| Check | Result |
| --- | --- |
| MATLAB reference vs codegen source (5,173-step log) | 1.1e-11 state, 7.9e-11 A |
| Codegen source vs compiled C (MEX) | 8.5e-12 state, 6.4e-11 A |
| Validated vs embedded Simulink model | identical contact and arrival; state within 7e-9 m |
| Target library | 27 files, 47.5 kB of C; 36 kB constant parameters |
