function est = identify_from_log(logfile, P, stage)
% IDENTIFY_FROM_LOG  Extract model parameters from a recorded log.
%   est = identify_from_log('log.csv', P, stage)
%     'rest'    (S1) sensor offsets, scales, noise, gyro and accelerometer bias
%     'drift'   (S2) table tilt: parabola through the range, cross-checked by the EKF
%     'latency' (S2) ToF measurement delay -- REQUIRES truth columns (the camera)
%     'pulse'   (S3) force scale, by fitting the open-loop trajectory
%
%   Each stage estimates only what it owns. Nothing is applied automatically.
%
%   Why latency is separate: a constant delay during motion at constant velocity puts the
%   estimate a fixed v*tau behind the truth while leaving the innovations zero-mean and white.
%   The filter is then perfectly self-consistent and wrong, which is exactly the failure the
%   innovations cannot see. Identifying it needs either an external truth (camera) or a direct
%   electrical measurement of the sensor timing. Do not try to fit it from innovations.
T = readtable(logfile);
n_tof = numel(P.tof.b);
est = struct('stage', stage);

switch lower(stage)
case 'rest'
    % Clamped at known gaps: the residual against truth gives offset, scale and noise per
    % sensor. This is a measurement, not a fit; it is what freezes R for good.
    v = T.tof_valid == 1;
    gap_true = T.truth_x(v);
    for j = 1:n_tof
        r = T.(sprintf('tof%d_m', j))(v);
        A = [gap_true + P.tof.recess, ones(sum(v),1)];   % modelled range = gap + recess
        p = A\r;                                          % [scale; offset]
        est.tof_scale(j)  = p(1);
        est.tof_offset(j) = p(2);
        est.tof_sigma(j)  = std(r - A*p);
    end
    g = T.gyro_z_rads(T.imu_valid == 1);
    est.gyro_bias  = mean(g);       est.gyro_sigma = std(g);
    a = T.acc_x_ms2(T.imu_valid == 1);
    est.acc_bias   = mean(a);       est.acc_sigma  = std(a);
    est.note = 'set the ToF offsets/scales, P.tof.sigma, the gyro bias prior; freeze R after this';

case 'drift'
    % Drives off: the only force is the disturbance (table tilt). Two independent estimates,
    % which must agree:
    %   1. model-free: a parabola through the raw range; acceleration = 2 x quadratic term
    %   2. the EKF's own disturbance state, averaged over the second half of a replay
    % WARNING, learned the hard way: an earlier version "fitted" the tilt by likelihood through
    % a prior the EKF never read, so the cost was flat. Its apparent success was fminsearch's
    % default initial step (2.5e-4) landing on the injected value (2.57e-4). Never fit a
    % parameter that has no path into the model being replayed.
    v = T.tof_valid == 1;  t = T.t_s(v);  r = mean([T.tof1_m(v), T.tof2_m(v)], 2);
    pf = polyfit(t, r, 2);
    est.tilt_acc_parabola = 2*pf(1);
    R  = replay_ekf(logfile, P, false);
    h2 = round(size(R.est,1)/2):size(R.est,1);
    est.tilt_acc_ekf = mean(R.est(h2,7));
    est.tilt_acc = est.tilt_acc_parabola;            % the model-free one is the reference
    est.tilt_deg = asind(min(abs(est.tilt_acc)/9.81,1))*sign(est.tilt_acc);
    est.agree = abs(est.tilt_acc_ekf - est.tilt_acc_parabola) < ...
                0.15*abs(est.tilt_acc_parabola) + 2e-5;
    est.note = 'tilt is a property of the table; if the two estimates disagree, suspect timestamps';

case 'latency'
    % Needs truth. Sweep the delay and take the one that minimises the replay's position error
    % against the camera. Use a log with CHANGING velocity: at constant velocity every delay
    % gives the same innovations and only the truth separates them.
    assert(ismember('truth_x', T.Properties.VariableNames), ...
           'latency identification needs truth columns (camera) in the log');
    f = @(tau) truth_rms(logfile, set_latency(P, tau));
    taus = linspace(0, 0.06, 13);
    J = arrayfun(f, taus);
    [~, k] = min(J);
    lo = taus(max(k-1,1));  hi = taus(min(k+1,numel(taus)));
    est.latency   = fminbnd(f, lo, hi, optimset('TolX',1e-4,'Display','off'));
    est.rms_mm    = 1e3*f(est.latency);
    est.rms_mm_0  = 1e3*f(0);
    est.sweep_tau = taus;   est.sweep_rms_mm = 1e3*J;
    est.note = 'set P.tof.latency; also worth measuring electrically on the data-ready line';

case 'pulse'
    % Open loop with known currents: the filter is not involved. Fit the force scale by
    % replaying the PLANT against the measured trajectory.
    % Fit the force scale TOGETHER with the initial state. The trajectory is started from a
    % single range sample, whose 5 mm noise is the same size as the effect being measured;
    % fitting x0 and v0 alongside the scale removes that leverage.
    f  = @(q) pulse_cost(T, P, q);
    q0 = [1; 1; 1];                       % [scale; x0 offset in mm; v0 in mm/s]
    % non-zero start: fminsearch seeds its simplex with 5% of each value, and a zero
    % entry gets 0.00025 in ITS units -- a quarter of a micrometre here.
    q  = fminsearch(f, q0, optimset('TolX',1e-4,'TolFun',1e-10, ...
                                    'MaxFunEvals',600,'Display','off'));
    est.force_scale = q(1);
    est.x0_offset_mm = q(2);   est.v0_mm_s = q(3);
    est.rms_mm      = 1e3*sqrt(f(q));
    est.note        = 'force scale multiplies the calibrated model; refit q with update_lut';

otherwise
    error('stage must be rest, drift, latency or pulse');
end
end

% ---- helpers
function P = set_latency(P, tau), P.tof.latency = abs(tau); end

function e = truth_rms(logfile, P)
R = replay_ekf(logfile, P, false);
e = R.rms_truth(1);
end

function J = pulse_cost(T, P, q)
% q = [force scale; initial gap offset (mm); initial velocity (mm/s)]
Pp = P;  Pp.coil.q_perA = P.coil.q_perA*sqrt(abs(q(1)));
dt = P.Ts;
v   = T.tof_valid == 1;
k0  = find(v, 1, 'first');            % rows before the first range sample are NaN
x00 = mean([T.tof1_m(k0), T.tof2_m(k0)]) - P.tof.recess + 1e-3*q(2);
s   = [x00; 0; 0; 1e-3*q(3); 0; 0];
xm = mean([T.tof1_m(v), T.tof2_m(v)], 2) - P.tof.recess;
tm = T.t_s(v);  xs = nan(height(T),1);
for k = k0:height(T)
    i_meas = [T.i1_A(k); T.i2_A(k); T.i3_A(k); T.i4_A(k)];
    f = @(ss) asteria_plant(0, ss, i_meas, Pp, [0;0;0]);
    k1=f(s); k2=f(s+dt/2*k1); k3=f(s+dt/2*k2); k4=f(s+dt*k3);
    s = s + dt/6*(k1+2*k2+2*k3+k4);  s(1) = max(s(1), 1e-4);
    xs(k) = s(1);
end
keep = tm >= T.t_s(k0);
J = mean((interp1(T.t_s(k0:end), xs(k0:end), tm(keep)) - xm(keep)).^2);
end
