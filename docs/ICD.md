# ASTERIA 4v4 Bench — Interface Control Document

**GNC ↔ Electronics & Integration** · ASTERIA-GNC-ICD-001 · Rev A (draft for review)

## 1. Purpose and document control

This document fixes what the electronics and integration team must build so the GNC software runs as it was designed and verified. Every requirement has an ID, a rationale and a verification method, so both teams sign off the same list. Numbers come from the verified simulation, not from estimates made for this document.

| Field | Value |
| --- | --- |
| Document | ASTERIA-GNC-ICD-001 |
| Revision | A — draft for review |
| Date | 2026-10-01 |
| Owner | GNC lead (Niccolò Tonetto) |
| Counterpart | Electronics and integration lead — TBD |
| Approval | GNC lead, electronics lead, PM — pending |
| Source of all numbers | `scripts/bench_init_3dof.m` in the asteria-gnc repository |
| Supersedes | the informal version of this file (git history, before 2026-10-01) |

**In scope:** the sensors, the coil drivers, the flight computer running `gnc_step`, the data log, and the bench items that constrain the electronics. **Out of scope:** the GNC algorithms and tuning (see `docs/RUNBOOK.md`), lateral control (excluded for the first test by PM decision), and the 1v1 calibration rig apart from its data format.

**Applicable documents:** `docs/RUNBOOK.md` (procedures and performance), `docs/TOOLCHAIN.md` (replay and identification tools), and the datasheets of the VL53L1X time-of-flight sensor, the BMI088 IMU and the INA240 current-sense amplifier.

## 2. Conventions

Sign and ordering errors, not algorithms, are the most likely integration failure, so these definitions are binding. Units are SI throughout: metres, seconds, radians, amperes.

**Frames and signs**

| Quantity | Definition | Sign |
| --- | --- | --- |
| Gap `x` | distance between the chaser and target interface planes along the docking axis; `x = 0` is bare face contact | the approach runs from 0.060 m towards 0, so approach velocity is **negative** |
| Lateral `y` | in the table plane, perpendicular to the docking axis | positive on the side of coils 1–2 and `tof1` |
| Yaw `θ` | chaser rotation in the table plane | positive moves the +y side **towards** the target; `tof1` then reads shorter than `tof2` |
| Vertical `z` | normal to the table | used only to place the coils |
| Coil current | chaser coil current, target coils biased positive | positive **attracts** its facing target coil; negative repels (this is how the loop brakes) |

**Coil numbering.** Four coils per platform on a 25 mm square pattern; chaser coil *k* faces target coil *k*.

| Index | y [mm] | z [mm] | Positive current alone, at 40 mm gap and +0.2 A |
| --- | --- | --- | --- |
| 1 | +25 | +25 | −1.63 mN axial (attract), +51 µN·m yaw |
| 2 | +25 | −25 | −1.63 mN axial (attract), +51 µN·m yaw |
| 3 | −25 | +25 | −1.63 mN axial (attract), −51 µN·m yaw |
| 4 | −25 | −25 | −1.63 mN axial (attract), −51 µN·m yaw |

Swapping coils within a column (1↔2, 3↔4) is harmless. Swapping one coil across the ±y columns cancels yaw authority; swapping the columns reverses it and makes yaw unstable.

**Requirement wording.** *Shall* is binding. *Should* is a recommendation whose rejection needs a reason recorded. *TBD* = not yet decided; *TBC* = proposed, to be confirmed.

**Verification methods.** **T** test on hardware · **A** analysis or simulation · **I** inspection of hardware, drawings or code · **D** demonstration.

## 3. Interface overview

Eight interfaces, one 1 kHz tick. Four sensor paths feed the flight computer; `gnc_step` turns them into chaser currents every millisecond. The target coils only need enable and abort, and the camera is for validation, never control.

| IF | From → to | Carries | Rate | Requirements |
| --- | --- | --- | --- | --- |
| IF-01 | 2× VL53L1X → computer | range [m] with data-ready timestamp | 30 Hz | SEN-01, 02, 03, 08, 09 |
| IF-02 | lateral sensor → computer | y [m] | ≥ 30 Hz | SEN-04 |
| IF-03 | BMI088 → computer | yaw rate [rad/s], axial acceleration [m/s²] | gyro every 2 ms, accel every tick | SEN-05, 06 |
| IF-04 | 8× INA240 → computer | coil currents [A] | 1 kHz | SEN-07 |
| IF-05 | computer → 4 chaser drivers | current commands, ±0.427 A | 1 kHz | ACT-02 to 06 |
| IF-06 | computer → 4 target drivers | enable and abort; bias fixed in hardware | on event | ACT-01, SAF-02 |
| IF-07 | computer → log storage | one record per tick | 1 kHz | LOG-01, 02 |
| IF-08 | camera → log | truth pose on the same clock | camera rate | LOG-03 |

## 4. Requirements

32 requirements in six groups. Items marked TBC are proposals from the GNC side that the electronics team may challenge; everything else is what the verified simulation assumed.

### 4.1 Actuation (coils and drivers)

| ID | Requirement | Why | Verify |
| --- | --- | --- | --- |
| ACT-01 | Each target coil shall be driven at a constant +0.427 A ±1%, current-controlled. | Force scales with chaser × target current; the calibration assumes a fixed bias. | T |
| ACT-02 | Each chaser coil shall have a bidirectional, current-controlled driver covering ±0.427 A. | The loop brakes by reversing current. Under voltage drive, force would fall ~25% as the coils warm. | T |
| ACT-03 | A chaser current step shall settle within 2% in 2 ms (TBC). | The plant model assumes the coil's own L/R lag of 2.3 ms; a slower driver makes the simulation optimistic. | T |
| ACT-04 | Current command resolution shall be 2 mA or finer (about 10 bits over ±0.43 A). | Matches the current-sense noise; coarser steps become a disturbance. | A |
| ACT-05 | Each driver shall limit current to ±0.427 A in hardware, independent of software. | A software fault must not overdrive a coil. | T |
| ACT-06 | Firmware coil indices shall match §2. | A swap across the ±y columns destabilises yaw. | T |
| ACT-07 | The supply shall be 9 V; the H-bridge drop at 0.4 A shall be measured and reported to GNC. | The maximum current `i_max` is computed from it (0.30 V assumed). | T |

### 4.2 Sensing

| ID | Requirement | Why | Verify |
| --- | --- | --- | --- |
| SEN-01 | Two VL53L1X shall sit at y = +35 mm (`tof1`) and −35 mm (`tof2`), recessed 50 mm behind the chaser interface plane, 33 ms timing budget (~30 Hz). | Below ~40 mm the sensor is unreliable; the recess keeps it in range down to contact. The pair gives yaw. | I, T |
| SEN-02 | Each VL53L1X shall have its own I²C address (both default to 0x29): reassign at boot via XSHUT, or use separate buses. | Two sensors on one bus at the same address cannot both be read. | T |
| SEN-03 | The target face seen by the ToF sensors shall be flat and matte. | Specular or dark surfaces bias the range. | I |
| SEN-04 | A lateral sensor shall measure y over ±20 mm with ≤ 5 mm noise at ≥ 30 Hz. Part TBD. | The filter monitors lateral drift; 5 mm is what it was verified with. | T |
| SEN-05 | The gyro shall run at its native 1 kHz output rate; firmware shall deliver one yaw-rate sample every 2 ms (mean of two). | The BMI088 gyro has no 500 Hz rate (100/200/400/1000/2000 Hz). The GNC was verified at 2 ms; the alternative is a GNC change to 1 ms plus re-verification (TBC which). | I, A |
| SEN-06 | The accelerometer x axis shall align with the docking axis within 1° (TBC), logged every tick. | Feeds the planned force-scale state; misalignment scales the reading. | I |
| SEN-07 | An INA240 channel shall measure each of the 8 coil currents (chaser and target) every 1 ms tick, noise ≤ 2 mA rms. | The filter uses measured, not commanded, current; target drift is a force-scale error. | T |
| SEN-08 | Every sample shall be timestamped on one clock at ≤ 1 µs resolution; ToF at its data-ready interrupt. | Without a shared clock the logs cannot be replayed. | I, T |
| SEN-09 | The ToF effective latency shall be measured and reported (GNC assumes 20 ms). | Latency is invisible to the filter's own consistency checks. | T |

### 4.3 Processing

| ID | Requirement | Why | Verify |
| --- | --- | --- | --- |
| CPU-01 | The processor shall have a double-precision FPU (Cortex-M7 class, e.g. STM32H7 or Teensy 4.x). | The code is double precision; a Cortex-M4F emulates it in software and is unlikely to fit the tick. | I |
| CPU-02 | `gnc_step` shall be called once per 1 ms tick; worst-case execution ≤ 500 µs, measured on the target. | Leaves half the tick for drivers and logging. | T |
| CPU-03 | The 36 kB parameter struct `pc` shall be stored as constants in flash. | Keeps RAM for state (about 1 kB) and buffers. | I |
| CPU-04 | The generated C shall be integrated unmodified; any change goes back through `test_sil`. | Equivalence to the reference (1e-11) holds only for the generated code. | I |
| TIM-01 | Tick jitter shall stay ≤ 50 µs (TBC). | The filter integrates with a fixed 1 ms step. | T |

### 4.4 Data logging

| ID | Requirement | Why | Verify |
| --- | --- | --- | --- |
| LOG-01 | One record per tick in the §5 format. On-board storage may be binary if a converter produces exactly that CSV. | CSV text at 1 kHz is about 250 kB/s, heavy on a microcontroller; the tools read the CSV. | D |
| LOG-02 | The log shall also carry the four target currents, bus voltage and `gnc_step` execution time. | Force-scale diagnosis, coil heating, timing margin. | I |
| LOG-03 | Validation-camera frames shall be timestamped on the same clock, or tied to it by a common event (TBC). | Latency and offsets can only be measured against external truth. | T |

### 4.5 Safety and fault handling

| ID | Requirement | Why | Verify |
| --- | --- | --- | --- |
| SAF-01 | Firmware shall command zero chaser current on: NaN or Inf in the state, a tick overrun, gap estimate < 0.5 mm, or a manual abort. | Stops the loop acting on a corrupted estimate or at contact. | T, D |
| SAF-02 | A manual abort shall de-energise all coils, chaser and target, within 10 ms (TBC). | Bias current alone still attracts the float. | T |
| SAF-03 | Coil temperature shall be monitored (resistance from INA240 and bus voltage) with a stop threshold TBD. | ~3.7 W per coil at 9 V, ~7 K per run, ~10 min time constant. | T |

### 4.6 Mechanical and environment

| ID | Requirement | Why | Verify |
| --- | --- | --- | --- |
| ENV-01 | Fasteners on both platforms shall be non-magnetic; no steel within 20 cm of the coils. | Stray iron adds forces the model does not contain. | I |
| ENV-02 | Coil brackets shall be non-conductive or slotted. | Eddy currents lag the force during current reversal. | I |
| ENV-03 | The docking axis shall be aligned with magnetic north within ±5° (TBC). | Earth's field then exerts no yaw torque (up to ~10% of authority otherwise). | I |
| ENV-04 | The IMU should be thermally isolated from the coils. | Heating drifts the gyro bias. | I |
| ENV-05 | The floating chaser shall be weighed complete (batteries and electronics) and the mass reported. | Every gain scales with mass (0.950 kg assumed). | T |

## 5. Software interface and log format

### 5.1 `gnc_step`

The firmware calls one function per 1 ms tick, generated as C by `build_target` (Embedded Coder, ARM Cortex-M, 27 files, 47.5 kB of source).

```
[xh, Pk, Kp, k, i_cmd] = gnc_step(xh, Pk, Kp, k, i_meas, z, t, pc)
```

| Argument | Dir | Size | Content |
| --- | --- | --- | --- |
| `xh` | in/out | 10 | filter state `[x y θ vx vy ω dx dy dθ bg]`, owned by the caller between ticks |
| `Pk` | in/out | 10×10 | filter covariance, owned by the caller |
| `Kp` | in/out | 3×3 | cached Jacobian, owned by the caller |
| `k` | in/out | 1 | tick counter, owned by the caller |
| `i_meas` | in | 4 | chaser coil currents measured this tick [A], coil order of §2 |
| `z` | in | 4 | `[tof1; tof2; lateral; gyro]` in m, m, m, rad/s; **NaN in any channel without a fresh sample this tick** |
| `t` | in | 1 | seconds since the approach started |
| `pc` | in | struct | constant parameters from `make_cg_params` (36 kB) |
| `i_cmd` | out | 4 | commanded chaser currents [A], coil order of §2 |

**Initialisation**, before the first tick: `xh = pc.x_init`, `Pk = pc.P0`, `Kp` = zeros, `k = 0`. Then set `xh(1)` from the first valid range, mean(`tof1`, `tof2`) − 0.050 m: the float never starts exactly at the nominal 60 mm.

**Each tick, in this order:**

1. Read all fresh samples and the eight coil currents; timestamp them.
2. Build `z`, NaN for stale channels.
3. Call `gnc_step`.
4. Check the stop conditions (SAF-01); otherwise saturate `i_cmd` to ±0.427 A and command the drivers.
5. Write the log record.

**Regeneration.** `pc` must be regenerated with `make_cg_params` after any change to the GNC parameters: a new calibration, start gap, mass or gains. The code itself only changes when the algorithms do.

### 5.2 Log record

One record per tick. The replay and identification tools read exactly these column names; extra columns are ignored.

| Column | Unit | Content |
| --- | --- | --- |
| `t_s` | s | timestamp, one clock for every sensor |
| `tof1_m`, `tof2_m`, `lat_m` | m | raw readings, uncorrected; last value repeated between samples |
| `tof_valid` | — | 1 on the tick a new range sample arrived, else 0 |
| `gyro_z_rads` | rad/s | yaw rate |
| `imu_valid` | — | 1 on the tick a new IMU sample arrived |
| `acc_x_ms2` | m/s² | axial accelerometer |
| `i1_A` … `i4_A` | A | measured chaser coil currents |
| `ic1_A` … `ic4_A` | A | commanded chaser currents |
| `i_target_A` | A | mean measured target current |
| `it1_A` … `it4_A` | A | individual target currents (LOG-02) |
| `v_bus_V`, `exec_us` | V, µs | bus voltage, `gnc_step` execution time (LOG-02) |
| `truth_x` … `truth_om` | m, rad, m/s, rad/s | validation camera, when fitted; columns omitted otherwise |

## 6. Verification, open items and change log

### 6.1 Before the first powered run

Every item must pass before the float is flown under closed-loop control.

- [ ] Coil order: positive current on coil 1 alone gives +yaw, on coil 3 alone −yaw (ACT-06).
- [ ] Polarity: at ~20 mm gap, negative chaser current pushes the float away (ACT-02).
- [ ] Each driver's hardware limit holds at 0.427 A (ACT-05).
- [ ] Both ToF sensors answer on their own addresses and read gap + 50 mm at a known gap (SEN-01, SEN-02).
- [ ] Rotating the +y side towards the target reads positive yaw rate, one sample every 2 ms (SEN-05).
- [ ] All 8 current channels read their offset at rest with ≤ 2 mA rms noise (SEN-07).
- [ ] `gnc_step` worst case ≤ 500 µs and tick jitter ≤ 50 µs, measured on the target (CPU-02, TIM-01).
- [ ] Each stop condition drives the chaser to zero current; the manual abort de-energises all coils (SAF-01, SAF-02).
- [ ] A 30 s free-drift log loads in `replay_ekf` without error (LOG-01).

### 6.2 Open items

Connectors, pinouts and board layout belong to the electronics team and are not specified here.

| Item | Requirement | Owner | Needed before | Status |
| --- | --- | --- | --- | --- |
| Lateral sensor part | SEN-04 | Electronics | chaser board design | TBD |
| Processor and dev board | CPU-01 | Electronics | firmware work | TBD |
| Gyro: 2 ms decimation in firmware, or GNC re-verified at 1 ms | SEN-05 | GNC + electronics | firmware work | TBC |
| Current-loop settling time | ACT-03 | Electronics | driver design | TBC |
| Tick jitter limit | TIM-01 | Electronics | firmware work | TBC |
| Abort de-energise time | SAF-02 | Electronics | first powered run | TBC |
| Coil temperature stop threshold | SAF-03 | Electronics + GNC | first powered run | TBD |
| Camera synchronisation | LOG-03 | Integration | tuning stage S2 | TBC |
| Accelerometer and magnetic-north alignment tolerances | SEN-06, ENV-03 | Integration | first powered run | TBC |
| Report measured values: H-bridge drop, ToF latency, sensor noise at rest, chaser mass | ACT-07, SEN-09, ENV-05 | Electronics | first powered run | open |

### 6.3 Change log

| Rev | Date | Change |
| --- | --- | --- |
| A | 2026-10-01 | First formal issue. Supersedes the informal version of this file; adds requirement IDs and verification methods, the BMI088 output-rate correction (SEN-05) and VL53L1X addressing (SEN-02). |
