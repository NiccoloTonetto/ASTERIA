# A7: uncalibrated Monte Carlo with path-following

Ana, November 2026. Open point A7 in `OPEN_POINTS.md`. This replaces the 44% uncalibrated pass rate, which was measured with the earlier time-indexed controller.

## Setup

- Uncalibrated geometry model (no `asteria_calibration.mat`), 60 mm start, 300 runs, seed 2026.
- Truth plant drawn per run: force 0.3–3× the model (log-uniform), softening length x0 2–8 mm, mass ±20%, tilt ±0.002°, gyro bias ±0.5 °/s.
- As in the post-calibration Monte Carlo (`RUNBOOK.md` §7): 20 ms ToF delay, coil current lag, measured-current input.
- Pass: contact with an arrival speed of −20 ± 10 mm/s. Lateral offset is not part of the criterion.
- A run stops at contact or at 12.9 s, 2.5× the 5.14 s reference duration.

## Summary

Over the full uncalibrated range (0.3–3×), the pass rate with path-following is **49.0%** (147 of 300). At least 90% of approaches pass when the real force is between 0.77× and 1.27× the model; in the force bins, 92–93% pass between 0.65× and 1.39×.

| Outcome | Runs | Share |
| --- | --- | --- |
| Pass | 147 | 49.0% |
| Too fast (faster than −30 mm/s) | 86 | 28.7% |
| No contact within 12.9 s | 60 | 20.0% |
| Too slow (slower than −10 mm/s) | 7 | 2.3% |

## Results by force scale

| Force | n | Pass | No contact | Too fast | Too slow | Arrival, median | Saturation, median |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.30–0.44 | 49 | 8 (16%) | 32 | 7 | 2 | −22.5 mm/s | 91% |
| 0.44–0.65 | 57 | 21 (37%) | 15 | 19 | 2 | −28.1 mm/s | 78% |
| 0.65–0.95 | 44 | 41 (93%) | 0 | 2 | 1 | −20.7 mm/s | 63% |
| 0.95–1.39 | 48 | 44 (92%) | 0 | 3 | 1 | −24.4 mm/s | 5% |
| 1.39–2.04 | 46 | 29 (63%) | 1 | 16 | 0 | −27.7 mm/s | 13% |
| 2.04–3.00 | 56 | 4 (7%) | 12 | 39 | 1 | −69.2 mm/s | 68% |

- Bins are log-spaced, as printed by `monte_carlo`.
- Arrival is the median over the runs that reach contact.
- Saturation is the median fraction of 1 ms ticks with a coil command at the current limit, over all runs in the bin.

| Softening x0 | n | Pass |
| --- | --- | --- |
| 2–4 mm | 83 | 51% |
| 4–6 mm | 110 | 50% |
| 6–8 mm | 107 | 47% |

## Findings

**Too fast.** Arriving too fast is the most common failure: 28.7% of runs (86) touched the target faster than −30 mm/s.
- 26 of these runs had a real force below 0.65× the model force, where the actuator is too weak to brake enough.
- 55 had a force above 1.39×. They fall into two different failures:
  - **1.39–2.04×:** the current is saturated only 13% of the time (median), so the controller is not short of authority. The likely cause is the mismatch between the force model that the estimator and the feedforward use and the stronger real actuator. This is the fast bias already visible at 0.95–1.39×, grown from −24.4 to −27.7 mm/s median. Force-scale estimation (B2) targets it.
  - **Above 2.04×:** the current is saturated 68% of the time and only 4 of 56 runs pass. The median arrival speed reaches −69 mm/s. What drives the saturation there was not isolated.

**No contact.** In 20.0% of runs (60) the float had not reached contact when the simulation stopped at 12.9 s. All 60 ended at this time limit. `mc_run` does not record the final gap, so how far short they stopped is not known.
- 47 had a force below 0.65×. There the current is saturated most of the time (median 78–91% in the two bins) and there isn't enough authority to complete the approach.
- The other 13 had a force above 1.39×, 12 of them above 2.04×, and they drifted sideways. The median final lateral offset is 293 mm for the 12 runs above 2.04×; the single run in 1.39–2.04× ended 68 mm off-axis. This is consistent with the lateral instability while braking with repulsion, which grows with the force (C4).

**Too slow.** Only 2.3% of runs (7) arrived too slowly, slower than −10 mm/s.

**Inside the band.** When the real force is between 0.65× and 1.39× the model force, the pass rate is 92–93%: 85 of 92 runs, about ±4% standard error per bin. There is a slight fast bias when the actuator is stronger than modelled: at 0.95–1.39× the median arrival speed is −24.4 mm/s instead of the −20 mm/s target.

**Band and bins.** The 0.77–1.27× band comes from the sliding window in `monte_carlo`, and the window's width scales with the force range.
- Here each point of the band averages the runs within a factor of 1.28 in force. Near the edges this mixes in the failing regions, so the band comes out narrower than the bins.
- The two statements are consistent; the band is the conservative one.
- The post-calibration band (0.80–1.24×) uses a window of a factor of 1.05, so the edges of the two bands are not comparable.

**Softening length.** The pass rate stays at 47–51% across the x0 range. That does not show x0 has little effect:
- each x0 bin holds 83–110 runs, about ±5% standard error;
- the force scale dominates the failures, which dilutes any x0 effect.

The x0 sensitivity is better judged within the calibrated force range.

**Lateral offset (not scored).** Passing runs end with a median lateral offset of 2.2 mm (90th percentile 7.2 mm).
- 8 of the 147 end more than 10 mm off-axis; the largest ends 137 mm off.
- With a 10 mm limit, the pass rate would be 46.3% (139 of 300). The 10 mm is only a placeholder until C4 sets a requirement.
- Longer approaches drift more: over the runs that reach contact, the correlation between |y_end| and t_end is 0.38.

## For the proposal

- Uncalibrated, force 0.3–3×: 49.0% pass. Calibration is required.
- At least 90% pass with the force within 0.77–1.27× the model (conservative); 92% across 0.65–1.39× in the bins.
- The pass criterion scores the axial arrival only; lateral offset is reported separately.

## Follow-ups

- `mc_run`: return the final gap, so that a run stopped by the time limit can be told apart from one that stalled.
- C4: add a lateral criterion to the pass definition once a requirement exists.
- x0: check the sensitivity in the post-calibration Monte Carlo.

## Reproduce

From the repository root, with no `asteria_calibration.mat` (the first line printed by `bench_init_3dof` reads "geometry model, UNCALIBRATED"):

```matlab
bench_init_3dof; optimize_maneuver; build_schedule;
P.mc.fs_range = [0.3 3];
monte_carlo                         % summary, per-bin and softening tables, force band
```

To compute the outcome breakdown and the lateral figures from `mc_results.mat`, run the following. `prctile` and `corr` need the Statistics and Machine Learning Toolbox.

```matlab
load mc_results.mat                 % T, P
vf = P.traj.vf;  tol = P.mc.v_tol;
fast = T.contact & T.v_end < vf - tol;
slow = T.contact & T.v_end > vf + tol;
edges = exp(linspace(log(0.3), log(3), 7));
fprintf('%11s %4s %10s %8s %8s %5s %10s\n', 'force', 'n', 'no contact', 'too fast', 'too slow', 'pass', '|y|>10 mm');
for k = 1:numel(edges)-1
    s = T.fscale >= edges(k) & T.fscale < edges(k+1);
    fprintf('%4.2f-%4.2f %4d %10d %8d %8d %5d %10d\n', edges(k), edges(k+1), sum(s), sum(s & ~T.contact), ...
            sum(s & fast), sum(s & slow), sum(s & T.pass), sum(s & abs(T.y_end) > 0.010));
end
t_max = 2.5*P.ref.T;
fprintf('no contact, stopped at the %.2f s limit: %d of %d\n', t_max, ...
        sum(~T.contact & T.t_end >= t_max - P.Ts/2), sum(~T.contact));
yp = 1e3*abs(T.y_end(T.pass));
fprintf('pass runs, |y_end|: median %.1f mm, 90th pct %.1f mm, max %.1f mm; %d of %d above 10 mm\n', ...
        median(yp), prctile(yp,90), max(yp), sum(yp > 10), numel(yp));
fprintf('corr(|y_end|, t_end), contact runs: %.2f\n', corr(abs(T.y_end(T.contact)), T.t_end(T.contact)));
```
