# ASTERIA 4v4 bench — Interface Control Document (GNC ↔ electronics)

What the flight computer, sensors and drivers must provide so that the GNC code runs as it was designed and verified. Every number here is taken from `bench_init_3dof` and the verified simulation. If the hardware cannot meet a line, tell the GNC lead before building around it: most of these are assumptions the controller was tuned on.

Units are SI everywhere: metres, seconds, radians, amperes.

## 1. Frames and sign conventions

**Axial gap `x`.** Distance between the chaser and target interface planes along the docking axis. `x = 0` is bare face contact; the approach runs from `x = 0.060` m towards 0, so approach velocity is **negative**.

**Lateral `y`.** In the table plane, perpendicular to the docking axis. Positive `y` is the side where coils 1–2 and `tof1` sit.

**Yaw `θ`.** Rotation of the chaser in the table plane. **Positive yaw moves the +y side of the chaser towards the target.** Under positive yaw, `tof1` reads shorter than `tof2`. The gyro must read positive for this rotation; if it reads negative, invert it in the driver, not in the GNC.

**Vertical `z`.** Normal to the table. Only used to place the coils.

## 2. Coils

Four coils per platform on a 25 mm square pattern. Chaser coil *k* faces target coil *k*.

| Index | y [mm] | z [mm] | Positive current alone produces |
| --- | --- | --- | --- |
| 1 | +25 | +25 | attraction, +yaw torque |
| 2 | +25 | −25 | attraction, +yaw torque |
| 3 | −25 | +25 | attraction, −yaw torque |
| 4 | −25 | −25 | attraction, −yaw torque |

At 40 mm gap, +0.2 A on one chaser coil (target biased) gives −1.63 mN axial and ±51 µN·m yaw. All four at +0.2 A give −6.51 mN. Negative current reverses the force: that is how the controller brakes.

**Drive scheme (target bias).** All four target coils carry a constant **+0.427 A**. The four chaser coils are commanded individually in **±0.427 A**. Force is proportional to chaser current × target current, so:

- **Both drives must be current-controlled**, not voltage-driven. Winding resistance rises ~16% at +40 K; under voltage drive the force would fall ~25% during a run.
- **Chaser drivers must be bidirectional** (H-bridge). The loop brakes by reversing current.
- **The index order in firmware must match the table above.** Swapping one coil across the ±y columns cancels yaw authority; swapping the columns reverses it and makes yaw unstable. Within a column (1↔2, 3↔4) a swap is harmless.

Supply: 9 V bus. Measure the H-bridge drop at 0.4 A and report it: `i_max` is computed from it (currently assumes 0.30 V). Coil time constant L/R ≈ 2.3 ms.

## 3. Sensors

| Sensor | Quantity | Rate | Placement / requirement | Noise the GNC assumes |
| --- | --- | --- | --- | --- |
| `tof1`, `tof2` (VL53L1X) | range to the target face [m] | 30 Hz, 33 ms timing budget | at y = +35 mm (`tof1`) and −35 mm (`tof2`), **recessed 50 mm** behind the chaser interface plane, looking along the docking axis | 5 mm |
| lateral sensor | `y` [m] | 30 Hz | **part not chosen yet** — needs ±20 mm range around centre | 5 mm |
| gyro (BMI088) | yaw rate [rad/s] | 500 Hz | z axis normal to the table | 0.2 °/s |
| accelerometer (BMI088) | axial specific force [m/s²] | 500 Hz | x axis **aligned with the docking axis**; logged now, used by the planned force-scale state | datasheet |
| current sense (INA240) | all 8 coil currents [A] | every 1 ms tick | chaser **and** target coils | 2 mA |

Notes the electronics team needs:

- **The 50 mm recess is required.** The VL53L1X is unreliable below ~40 mm; the recess keeps it in range down to contact. With the recess, a 40 mm gap reads ~90 mm. If the sensor reads the gap itself, it is blind for the last 40 mm.
- **The target face the ToF sensors see must be flat and matte.** Specular or dark surfaces bias the range.
- **The gyro rate must be an integer divisor of the 1 kHz tick** (500 Hz is). Other rates are not handled.
- **Range latency.** The GNC assumes ~20 ms effective delay on the ToF (`P.tof.latency`). Timestamp each ToF sample at data-ready; if possible, measure the real delay on the data-ready line.

## 4. The GNC entry point

The firmware calls one function every 1 ms: `gnc_step`, generated as C by `build_target`.

```
[xh, Pk, Kp, k, i_cmd] = gnc_step(xh, Pk, Kp, k, i_meas, z, t, pc)
```

| Argument | Size | Meaning |
| --- | --- | --- |
| `xh` | 10 | filter state `[x y θ vx vy ω dx dy dθ bg]` — owned by the caller between ticks |
| `Pk` | 10×10 | filter covariance — owned by the caller |
| `Kp` | 3×3 | cached Jacobian — owned by the caller |
| `k` | scalar | tick counter — owned by the caller |
| `i_meas` | 4 | chaser coil currents **measured this tick** (INA240), coil order of §2 |
| `z` | 4 | `[tof1; tof2; lateral; gyro]` — **NaN in any channel without a fresh sample this tick** |
| `t` | scalar | seconds since the approach started |
| `pc` | struct | constant parameters from `make_cg_params` (36 kB; place in flash) |
| `i_cmd` | 4 | commanded chaser currents [A], coil order of §2 |

**Initialisation before the first tick:** `xh = pc.x_init`, `Pk = pc.P0`, `Kp = zeros(3)`, `k = 0`. Then overwrite `xh(1)` with the first valid range: `mean(tof1, tof2) − 0.050`. The float will never start exactly at the nominal 60 mm.

**Order inside each tick:**

1. Read all fresh samples and the eight INA240 currents; timestamp them.
2. Build `z`, with NaN for stale channels.
3. Call `gnc_step`.
4. Saturate `i_cmd` to ±0.427 A in the driver as well (defence in depth), then command the drivers.
5. Write the log record (§5).

**Timing.** Worst-case execution of `gnc_step` must stay under **500 µs**, half the tick, leaving the rest for drivers and logging. The code is double precision: use a core with a **double-precision FPU** (Cortex-M7, e.g. STM32H7 or Teensy 4.x). On a Cortex-M4F the double arithmetic runs in software and is unlikely to fit.

**Stop conditions the firmware should own:** any NaN or Inf in `xh`; a tick overrun; `xh(1)` below 0.5 mm (contact); a manual abort. On any of them, command zero current on the chaser.

## 5. Log format

One record per 1 ms tick, CSV with this header. The replay and identification tools read exactly these columns.

| Column | Content |
| --- | --- |
| `t_s` | timestamp [s], one clock for every sensor |
| `tof1_m`, `tof2_m`, `lat_m` | raw readings, uncorrected; repeat the last value when no new sample |
| `tof_valid` | 1 on the tick a new range sample arrived, else 0 |
| `gyro_z_rads` | yaw rate |
| `imu_valid` | 1 on the tick a new IMU sample arrived |
| `acc_x_ms2` | axial accelerometer |
| `i1_A` … `i4_A` | **measured** chaser coil currents |
| `ic1_A` … `ic4_A` | commanded chaser currents |
| `i_target_A` | mean measured target current |
| `truth_x`, `truth_y`, `truth_th`, `truth_vx`, `truth_vy`, `truth_om` | validation camera, when fitted; omit the columns otherwise |

Additional columns are ignored by the tools, so also log the four individual target currents (`it1_A` … `it4_A`), bus voltage, and the `gnc_step` execution time. **If the clocks are not shared, the logs cannot be replayed.**

## 6. Mechanical and environment requirements that affect the electronics

- Non-magnetic fasteners on both platforms; keep steel at least 20 cm from the coils.
- Non-conductive or slotted coil brackets (eddy currents lag the force during current reversal).
- Align the docking axis with magnetic north: Earth's field then exerts no yaw torque on the chaser.
- Coil heating: ~3.7 W per coil at 9 V, ~7 K per run, ~10 min time constant. Allow cooldown or monitor resistance from the INA240 and bus voltage.

## 7. Checks before the first powered run

- [ ] Coil order: +current on coil 1 alone gives +yaw; on coil 3 alone, −yaw.
- [ ] Polarity: at ~20 mm, negative chaser current pushes the float away.
- [ ] ToF recess: at a known gap, each ToF reads gap + 50 mm.
- [ ] Gyro sign: rotating the +y side towards the target reads positive.
- [ ] Timing: `gnc_step` worst case under 500 µs on the target, measured.
- [ ] Log replays: a 30 s free-drift log loads in `replay_ekf` without error.

## 8. Open items for the electronics team

- Choose the lateral sensor (requirement in §3).
- Choose the processor (Cortex-M7 class) and run processor-in-the-loop (procedure in the runbook).
- Measure and report: H-bridge drop at 0.4 A, ToF latency on the data-ready line, sensor noise at rest.
