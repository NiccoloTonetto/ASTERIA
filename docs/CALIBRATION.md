# 1v1 calibration procedure

Calibration of the force model on the 1v1 rig: why it is needed, what it changes, the test procedure, and what follows from the result. Related: `RUNBOOK.md` §4 (force model and trust), `BOM_INPUT.md` (equipment E1–E9), `TOOLCHAIN.md`.

## 1. Purpose, files and parameters

**Why.** The 4v4 force model computes every force and torque from the coil geometry, but its absolute strength is a physics estimate good to roughly a factor of two (finite coil length, core shape and the winding's own contribution are approximated). The loop needs the force within about ±12–15%. In the Monte Carlo, with the force anywhere in 0.3–3× of the model, 49% of approaches pass; after calibration to ±20%, 96% do. A single coaxial pair is where the model's form is exact, so measuring it fixes the constants and the 4v4 geometry does the rest.

**The two fitted parameters**

| Parameter | Meaning | Geometry estimate | Controls |
| --- | --- | --- | --- |
| `q` (stored as `P.cal.scale = q/q0`) | pole strength per amp [A·m/A] | 25.05 | the strength of every force |
| `x0` (stored as `P.cal.x0`) | pole softening length [m] | 3.9 mm (core radius) | how the force steepens in the last millimetres, where braking happens |

> **Force goes as q².** `P.cal.scale = 1.20` means 44% more force, not 20%. The Monte Carlo's "force 0.8–1.25×" refers to force, not to `scale`.

**Files**

- `scripts/update_lut.m`: `update_lut('data/<file>.csv')` reads the CSV and fits both parameters with `asteria_axial_1v1`, the 1v1 form of the same pole model (near faces, two cross terms, far faces). The fit minimises relative error, so far-field points count as much as near-field ones.
- `asteria_calibration.mat`: written by `update_lut` to the repository root. It holds `scale`, `x0`, `q_perA`, fit quality (`rms_rel`, `max_rel`, `n_pts`) and `source` (CSV name and fit date). It is git-ignored because it is regenerable; the CSV in `data/` is the record that gets committed.
- `scripts/bench_init_3dof.m`: loads the .mat automatically if present, overriding the geometry values, and computes the pole strength used everywhere from it. Its first printed line says which model is loaded. Delete the .mat to return to the geometry model.
- No `.m` file is edited by the calibration.

**Limitation.** The 1v1 fit sees only the product of the two coils' strengths. If they differ, it returns the geometric mean, and the code then applies one `q` to all eight 4v4 coils. T0 checks that this is legitimate.

## 2. Procedure

Run `bench_init_3dof` first: `update_lut` takes its starting values from `P` in the workspace.

### T0 — coil characterisation (before T1)

- [ ] Resistance (with temperature) and inductance of every coil with the LCR meter (E7): the 1v1 pair and all eight 4v4 coils.
- [ ] On-axis field of each coil with the magnetometer (E6) on the non-magnetic micrometer stage (E2), 6–8 distances from 30 to 150 mm, each at +i, −i and 0. Use ½(B(+i) − B(−i)): this cancels Earth's field, the sensor offset and core remanence.
- [ ] Fit each coil's strength together with a distance offset.
- [ ] **Pass:** all coils within ±5%, and the 1v1 pair representative of the 4v4 set. Otherwise the single-`q` calibration does not transfer and the model needs per-coil strengths: stop and raise it with GNC.

### T1 — axial force map (the calibration)

**Setup**

- [ ] Chaser coil on the balance pan (E1), on a 10–15 cm non-magnetic riser (E4). The balance weighs by electromagnetic force compensation, and a coil field near its mechanism corrupts the reading.
- [ ] Balance capacity checked: each coil is ~200 g plus riser and mount, and a typical 0.1 mg analytical balance tops out near 220 g. If it does not fit, a 1 mg balance with more capacity is acceptable (2% resolution at 100 mm, ~0.5 mN).
- [ ] Target coil above, on the non-magnetic micrometer stage (aluminium or brass); coils coaxial within 0.5 mm (printed alignment jig).
- [ ] Leads thin, slack, twisted and strain-relieved to a fixed point off the pan.
- [ ] Both supplies current-controlled; currents measured to ±1%.
- [ ] Draft shield on; no vibration sources nearby.

**Sign convention.** Attraction pulls the pan up, so the reading drops:

`force_N = (zero reading − energised reading) × 9.81`, attraction positive.

**Background checks**

- [ ] Both coils energised, ≥ 200 mm apart: the reading must not change. If it does, raise the riser.
- [ ] At a close gap, target energised and chaser at zero current: record the small attraction (the target field magnetising the chaser core). It is real physics and is needed for T2.

**Gap zero**

- [ ] Lower the target onto a feeler gauge (E3) of known thickness (e.g. 0.50 mm) on the chaser face, record the micrometer reading, remove the gauge, and offset the zero.
- [ ] Never press the faces together on the balance (overload). A 0.5 mm zero error biases every near-contact point and is silently absorbed into `x0`.

**Sweep**

- Gaps (mm): 2, 3, 4, 6, 8, 10, 15, 20, 30, 40, 60, 80, 100.
- Target current fixed at +0.427 A; chaser currents ±0.1, ±0.2, ±0.4 A.
- At 5, 20 and 60 mm, repeat with the target at 0.2 and 0.3 A (checks product scaling).
- Each point: zero → energise a few seconds until settled → read → de-energise → zero. Use the mean of the two zeros (brackets drift).
- Short energisation, and let the coils cool: a warm coil drives convection that the balance reads as force.
- Approach every gap from the same direction (micrometer backlash). One full sweep inward and one outward, three repeats in total, to reveal hysteresis and drift.
- About 250 points, roughly one day.

**Data file:** `data/YYYY-MM-DD_1v1_T1.csv`

```csv
gap_m,i_chaser_A,i_target_A,force_N
0.040,0.200,0.427,0.00051
0.040,-0.200,0.427,-0.00050
```

SI units, signed currents, force attraction-positive. (The numbers only show the format.)

**Acceptance**

- [ ] RMS misfit < 5% over 5–100 mm (`update_lut` warns above 15%)
- [ ] Force linear in current within 2%
- [ ] Negative chaser current gives repulsion (`update_lut` warns if it never does: rig not in target-bias mode)
- [ ] Force scales with target current

### T2 — near contact (same rig)

- [ ] Every 0.5 mm from 0.5 to 10 mm.
- [ ] Current up/down sweeps at 2 and 5 mm (hysteresis).
- [ ] Force at zero chaser current with the target biased.
- [ ] Separate CSV (`data/YYYY-MM-DD_1v1_T2.csv`). Do not fit it: compare it against the calibrated model and record the gap below which the misfit exceeds 15%. That gap is the documented validity limit of the force model. Sub-2 mm points, where the cores interact and saturate locally, would distort the far-field fit the controller relies on.

## 3. After the results

**Automatic, inside `update_lut`**

1. New `scale` and `x0` change every force the model computes, including the 4v4 field.
2. `bench_init_3dof` reruns: the design-point linearisation and LQR change.
3. `optimize_maneuver` reruns: the authority changes, so the reference's duration and peak current change.
4. `build_schedule` reruns: gains rebuilt at all 40 gaps.
5. The new closed-loop eigenvalues and reference summary are printed. Check that all eigenvalues have negative real parts and that the reference still arrives at about −20.7 mm/s.

**Then, by hand**

- [ ] `build_simulink_model` (the embedded parameter struct `pc` is regenerated at simulation start).
- [ ] `run_all_tests`: must stay green with the calibrated model.
- [ ] Monte Carlo with the real uncertainty:
  - Combine the fit residual, the measurement budget (~±2.5% far-field, ±4% near contact) and the T0 coil spread into a force uncertainty, and set `P.mc.fs_range`.
  - Narrow the softening range in `monte_carlo.m` (now 2–8 mm) around the fitted `x0`.
  - Run `monte_carlo`. That pass rate, not the 96% simulation figure, is the one to quote.
- [ ] Revisit the start-gap/tilt table (`RUNBOOK.md` §5): the authority scales with force, so the tilt tolerance does too.
- [ ] Firmware: regenerate `pc` with `make_cg_params` and re-flash the constant table.
- [ ] Commit the CSV. Update the illustrative force numbers in `ICD.md` §2, which come from the uncalibrated model.

**Reading the result**

| Result | Meaning | Action |
| --- | --- | --- |
| RMS < 5%, force scale ~0.8–1.25 | model form confirmed; gains rebuilt | proceed to the hardware stages |
| RMS < 5%, force far from 1 (e.g. 0.5 or 2) | geometry estimate off, calibration fixed it | proceed; re-check the tilt table and the reference authority margin |
| RMS 5–15% | model-form error, usually near contact | check gap zero and near-contact points; refit with ≥ 5 mm only; document the validity limit |
| RMS > 15% | model does not describe the rig | stop: check gap zero, bias current, alignment, iron saturation |
| no repulsion at negative current | rig wired or driven wrong | fix the rig; data unusable |
| force not linear in current | saturation or measurement problem | check the currents and the balance setup |

## Lab-day checklist

- [ ] `bench_init_3dof` run; the first line reads "geometry model, UNCALIBRATED" (no stale .mat)
- [ ] Balance levelled, warmed up, draft shield on, capacity checked
- [ ] Riser, stage, jig and fasteners non-magnetic; no steel within 20 cm
- [ ] Both supplies in current mode; current meter connected
- [ ] Background checks done and recorded
- [ ] Gap zero set with the feeler gauge and written down
- [ ] Data template ready (CSV header above); file named with date and test
- [ ] After T1: `update_lut`, read the warnings, check the eigenvalues and the reference
