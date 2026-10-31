# BOM input from GNC — 4v4 bench

What the GNC needs bought, borrowed or built, and why each item matters, for merging into the project BOM. Each line traces to a requirement in `ICD.md`. Regulators, connectors, passives and board layout belong to the electronics team.

Status: **Chosen**, **Proposed** (to be confirmed with electronics), **TBD**, **On order**. Open items are tracked in `ICD.md` §6.2.

## Choices behind the list

**Lateral sensor: a VL53L4CD looking sideways at a fixed reference rail.** It is the same time-of-flight family as the two range sensors, so the bus, driver style and mounting carry over. It ranges from 1 mm (no ~40 mm dead zone like the VL53L1X), has an 18° field of view (a ~10 mm spot on the rail at 30 mm) and runs up to 100 Hz.

| Option rejected | Why |
| --- | --- |
| Third VL53L1X + rail | works, but needs ≥ 40 mm standoff and a taller rail; second choice |
| VL6180 | end of life; ST recommends the VL53L4CD instead |
| Laser displacement sensor on the table | data must reach the float wirelessly: latency and a second clock |
| Validation camera | must stay independent to be able to check the filter |
| Hall-effect or inductive sensors | corrupted by the coils' own fields |
| Optical-flow sensor | velocity only, drifts |

Consequences: a reference rail on the bench (item 3), and a GNC change. The side-looking reading also sees yaw, so the filter's measurement model must be updated in the MATLAB and embedded versions and re-verified (`OPEN_POINTS.md`). The 5 mm noise figure in the ICD should be confirmed against the VL53L4CD datasheet.

**Flight computer: STM32H7, on an in-house carrier PCB.** The microcontroller is the riskiest part of a board to design (fine-pitch package, power configuration, decoupling, crystal, boot and debug), and a respin costs two to three weeks. The GNC needs nothing a module lacks. The STM32H7 is chosen because MathWorks' STM32 support does processor-in-the-loop with execution profiling on Nucleo and custom boards alike, so the PIL procedure in `RUNBOOK.md` §8 works now and later on the own board. From R2026a this support comes through the STM32 Microcontroller Blockset (licence to check). A Teensy 4.1 was considered and rejected: a good Cortex-M7, but no official MathWorks PIL support.

Path:
1. Nucleo-H743ZI2 as the development board: firmware, drivers, logging and PIL start before any PCB exists.
2. In-house chaser carrier PCB with a compact STM32H7 module (or the Nucleo) plugged in; module pins chosen to map onto the bare chip.
3. Optional, later: a fully integrated board, laid out from the proven carrier and ST's open Nucleo schematics.

## Electronics and bench items

| # | Item | Qty | Where | Key specification | Req. | Status |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | Time-of-flight sensor, VL53L1X | 2 + 1 spare | chaser | 33 ms budget (~30 Hz); individually addressable (XSHUT) | SEN-01, 02 | Chosen |
| 2 | Lateral sensor, VL53L4CD, side-looking | 1 + 1 spare | chaser | 1–1200 mm, 18° FoV, up to 100 Hz; ~30 mm from the rail; address planned with item 1 | SEN-04 | Proposed |
| 3 | Lateral reference rail | 1 | table or target platform | ≥ 150 mm long; parallel to the docking axis within 0.5°; matte; non-magnetic and non-conductive; outside the docking envelope | SEN-04 | Proposed |
| 4 | IMU, BMI088 | 1 + 1 spare | chaser | gyro at 1 kHz, decimated to 2 ms in firmware; accelerometer x on the docking axis | SEN-05, 06 | Chosen |
| 5 | Current-sense amplifier INA240 + shunt | 4 | chaser | bidirectional, ≤ 2 mA rms over ±0.5 A at 1 kHz | SEN-07 | Chosen |
| 6 | Current-sense amplifier INA240 + shunt | 4 | target | ≤ 2 mA rms at 0.427 A | SEN-07 | Chosen |
| 7 | ADC for current sense, TLV2544 (4 channels, 12-bit, SPI) | 2 (one per platform) | both | all 8 currents sampled every 1 ms | SEN-07 | Proposed |
| 8 | Bidirectional current-mode coil driver | 4 | chaser | ±0.427 A; settles in 2 ms; hardware limit; ≤ 2 mA resolution | ACT-02…05 | TBD |
| 9 | Constant-current coil driver | 4 | target | +0.427 A ±1%; enable input; hardware limit | ACT-01, SAF-02 | TBD |
| 10 | Flight computer: compact STM32H7 module | 1 | chaser | Cortex-M7 with double-precision FPU; ≥ 36 kB flash for parameters; µs timer | CPU-01…03 | Proposed |
| 11 | Chaser carrier PCB (in-house) | 1 + 1 spare | chaser | battery power; items 5, 7 and 8; ToF and IMU connectors with XSHUT lines; microSD socket; abort input; SWD; test points on ToF data-ready and a tick GPIO | CPU-01, SEN-08, TIM-01 | TBD |
| 12 | Target driver PCB (in-house) | 1 | target | items 6, 7 and 9; enable line from the abort chain | ACT-01, SAF-02 | TBD |
| 13 | Development board, Nucleo-H743ZI2 | 1 | lab | firmware and PIL before the carrier exists | CPU-02 | Proposed |
| 14 | Log storage (microSD on the carrier) | 1 | chaser | ~250 kB/s for CSV, less with binary logging | LOG-01 | Proposed |
| 15 | Chaser battery | 1 | chaser | 9 V; ~15 W for 4 coils at full current plus electronics | ACT-07 | TBD |
| 16 | Target bench supply | 1 | target | 9 V, ≥ 2 A | ACT-01 | Proposed |
| 17 | Manual abort switch | 1 | bench | de-energises all 8 coils within 10 ms (TBC) | SAF-02 | Proposed |
| 18 | Solenoid coils, same type as the 1v1 pair | 8 + spares | both | matched within ±5% in pole strength (checked in T0) | — | On order |
| 19 | Non-magnetic fasteners | lot | both | brass or nylon; no steel, including stainless, within 20 cm of the coils | ENV-01 | Proposed |
| 20 | Coil brackets | 8 | both | non-conductive (e.g. printed) or slotted metal | ENV-02 | Proposed |
| 21 | Validation camera with markers | 1 | overhead | ≥ 60 fps over ~200 mm; timestamps on the bench clock | LOG-03 | TBD |

## Why each component matters

1. **Time-of-flight sensors (VL53L1X ×2).** The only absolute measurement of the gap: everything the filter knows about position comes from them. Mounted 70 mm apart, their difference also gives yaw. The 50 mm recess keeps them above their ~40 mm minimum range all the way to contact; without it the filter is blind for the last 40 mm, exactly where the braking happens. The spare is there because losing one stops testing.
2. **Lateral sensor (VL53L4CD).** Lateral motion is not controlled in this phase, but the float drifts sideways in a slow, undamped ~0.07 Hz oscillation, and braking makes the lateral direction unstable. The sensor lets the filter track the drift and lets the operator abort a run heading for an off-centre contact. It works from 1 mm, so it can sit 30 mm from its rail and keep the whole ±20 mm lateral range between about 10 and 50 mm of reading.
3. **Lateral reference rail.** A side-looking sensor needs a fixed surface to range against along the whole approach. Its straightness is part of the measurement: a rail 0.5° off the docking axis fakes under 1 mm of lateral motion over 100 mm of travel, well below the 5 mm noise the filter assumes, and the remainder is measured in stage S1.
4. **IMU (BMI088).** The gyro measures yaw rate directly; in simulation it cut the yaw error from 1.24° to 0.38° RMS and the yaw-rate error from 0.43 to 0.12 °/s, because the two range sensors alone give yaw only at 30 Hz and noisily. The accelerometer is not used yet, but it is the basis of the planned force-scale estimate; logging it now gives the data to design it. The BMI088 gyro has no 500 Hz rate, so it runs at 1 kHz and the firmware decimates.
5. **Current sensing on the chaser coils (INA240 ×4).** The filter predicts motion from the current actually flowing, not the command: the coil lags the command by its ~2.3 ms time constant and the driver saturates. Feeding the command instead makes the filter's disturbance estimate absorb the mismatch and drift. The same signal closes the drivers' current loops.
6. **Current sensing on the target coils (INA240 ×4).** The force scales with chaser current × target current. If the target current drifts (coil heating, supply sag), the actuator is weaker or stronger than modelled, and nothing else would reveal it. The measured current and bus voltage also give coil resistance, hence temperature (SAF-03).
7. **ADC (TLV2544 ×2).** All eight currents must be fresh in every 1 ms tick; a slower or staggered sample feeds the filter a stale input. 12 bits keep the converter's step below the 2 mA noise the filter was tuned with. One four-channel converter per platform matches the four coils on each side.
8. **Chaser drivers: bidirectional, current-mode (×4).** Braking means reversing the force, which means reversing the current. Current mode keeps the force independent of coil heating: under voltage drive the force would fall ~25% as the windings warm during a run. The hardware limit protects the coils from a software fault, and fine command resolution keeps quantisation from becoming a disturbance.
9. **Target drivers: constant current (×4).** The target bias is what makes the force linear in the chaser current, and the force calibration assumes it constant within 1%. The enable input matters for safety: the bias alone still attracts the float, so an abort must be able to cut it.
10. **Flight computer (STM32H7 module).** `gnc_step` is double-precision code, about 15 thousand floating-point operations per 1 ms tick. A Cortex-M7 with a double-precision FPU runs that comfortably; a Cortex-M4F emulates double arithmetic in software and is unlikely to fit the tick.
11. **Chaser carrier PCB.** Everything specific to ASTERIA lives here, while the hard-to-route processor stays on a proven module. The test points matter as much as the circuits: the timing requirements (tick jitter, ToF latency, execution time) can only be verified by probing them with the logic analyser.
12. **Target driver PCB.** The target platform is fixed and needs no processor: four constant-current drivers, their current sensing, and an enable line tied into the abort chain.
13. **Development board (Nucleo-H743ZI2).** Firmware, drivers, logging and processor-in-the-loop can start before the carrier PCB exists. It is the item most likely to delay the first lab day if it arrives late.
14. **Log storage.** Every hardware tuning step works offline on recorded logs: the filter is replayed and tuned in minutes instead of re-flown. The Nucleo has no card slot, hence the microSD socket on the carrier.
15. **Chaser battery.** The float cannot carry cables: on an air bearing that measures forces of micronewtons to millinewtons, a cable would be the largest force in the system. The battery's mass is part of the float's mass, which every controller gain scales with, so the complete float must be weighed (ENV-05).
16. **Target bench supply.** The target is fixed, so it can use a bench supply: four coils at 0.427 A from 9 V, with margin.
17. **Manual abort switch.** A stop that does not depend on the software. It must cut the target coils too, because their bias alone keeps pulling the float in.
18. **Solenoid coils.** The force model assumes identical coils; a coil 10% weaker than its neighbours turns into an unwanted yaw torque. Coil-to-coil spread is measured in T0.
19. **Non-magnetic fasteners.** Steel near a coil becomes magnetised and adds forces and torques that are in no model, at the same scale the loop works with. Many stainless steels become magnetic after machining, hence brass or nylon.
20. **Coil brackets.** A conductive bracket forms a closed loop around the coil, and changing current induces eddy currents in it that delay the force. The delay is worst during current reversal, which is the braking manoeuvre. Non-conductive or slotted brackets break the loop.
21. **Validation camera.** The filter's own consistency checks cannot detect sensor offsets or latency: in fault-injection tests, a 3 mm range offset and a 30 ms sensor delay both passed every self-check. Only an external measurement catches them. The camera is for validation only and never feeds the controller.

## Test and calibration equipment

| # | Item | Used for | Key specification | Status |
| --- | --- | --- | --- | --- |
| E1 | Analytical balance | 1v1 force calibration (T1, T2) | 0.1 mg resolution; capacity for coil + riser (~220 g or more) | TBD |
| E2 | Micrometer stage, non-magnetic (aluminium or brass) | setting the 1v1 gap (T1), field mapping (T0) | 0.01 mm, ≥ 100 mm travel | TBD |
| E3 | Feeler gauge set | gap zero at contact (T1) | 0.05 mm steps | TBD |
| E4 | Non-magnetic riser | keeping the coil field off the balance (T1) | 10–15 cm, printed or wood | TBD |
| E5 | Gauge blocks or precision spacers | ToF calibration at rest (S1) | 10, 20, 40, 60 mm | TBD |
| E6 | Magnetometer (RM3100) or Hall probe | per-coil field (T0) | resolves ~10 µT | Proposed |
| E7 | LCR meter | coil resistance and inductance (T0) | — | TBD |
| E8 | Logic analyser | ToF latency, tick timing, jitter (SEN-09, CPU-02, TIM-01) | ≥ 4 channels, ≥ 10 MHz | TBD |
| E9 | Machinist's level | table levelling | 0.02 mm/m | TBD |

Most of these can be borrowed from a university lab.

## Why each piece of equipment matters

- **E1. Analytical balance.** The force model is only trustworthy once calibrated. The calibration measures the force between one coil pair against gap, down to about 0.5 mN at 100 mm; a 0.1 mg balance resolves 1 µN, a 0.2% error there.
- **E2. Micrometer stage.** Sets each calibration gap. Near contact the force is steep: a 0.1 mm gap error is a 2–3% force error. Non-magnetic, because a steel stage next to the coil adds force.
- **E3. Feeler gauge set.** Sets the zero of the gap at contact. A 0.5 mm error in that zero biases every near-contact point, and the fit absorbs it into the model.
- **E4. Non-magnetic riser.** An analytical balance weighs by electromagnetic force compensation, so a strong coil field near its mechanism corrupts the reading.
- **E5. Gauge blocks or spacers.** The float is clamped at known gaps to measure each ToF sensor's offset, scale and noise. That fixes the sensor model once, so it never has to be tuned.
- **E6. Magnetometer.** Measures each coil's strength from its field, independently of any force test. A force test on a pair only gives the product of two coils; the field measurement separates them and shows whether the eight coils are matched. At 100 mm the field is about 40 µT, the size of Earth's field, so each point is taken at +i and −i.
- **E7. LCR meter.** Resistance and inductance per coil give the electrical time constant (~2.3 ms), which the plant model and the driver settling requirement both depend on.
- **E8. Logic analyser.** The only way to verify the timing requirements: the ToF sensor's real latency on its data-ready line, the 1 ms tick and its jitter, and `gnc_step`'s execution time on the target.
- **E9. Machinist's level.** Table tilt is the dominant disturbance on an air bearing. At the 60 mm start gap the loop comfortably tolerates ~0.011°; a 0.02 mm/m level resolves about 0.001°. The residual is then measured with the drift test.
