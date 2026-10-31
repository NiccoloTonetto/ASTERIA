%% bench_init_3dof.m  --  4v4 planar bench: parameters, wrench model, LQR, EKF
% Builds the parameter struct P used by every other script, the Simulink model and the tests.
%
% Conventions
%   World frame: x along the docking axis, y lateral, z up. Target grounded, interface plane at
%   x = 0, its coils in x in [-L, 0].
%   Chaser pose xi = [x; y; th]: x = gap between the interface planes (> 0, closes to 0),
%   y = lateral offset, th = yaw about the chaser interface centre (right-handed about z).
%   Fx > 0 pushes the platforms apart (attraction is Fx < 0). Plant: m*xdd = Fx + m*dx.
%   Drive: target coils at a fixed bias, chaser coil k at i_k; i_k > 0 attracts, i_k < 0 repels.
%
% Control structure
%   With cant = 0, dW/di has rank 2: the currents command Fx and Mz, and Fy is tied to Mz.
%   x and th are closed loop (LQR on [x th vx om]); y is passively restoring under attraction,
%   estimated and monitored only. Section 5 sweeps the coil cant (lateral authority).
%
% Force model: softened Gilbert pole model, scaled by the 1v1 calibration (P.cal).

clear P;      % no clc: called by update_lut and the tests
P = struct();
mu0 = 4*pi*1e-7;

%% 0. CONFIGURATION
P.m       = 0.950;        % [kg] floating chaser, estimate (4 x 0.200 coils + ~0.15 float);
                          % replace with the weighed value
P.J       = 1.6e-3;       % [kg m^2] yaw inertia about the interface centre, estimate
                          % (uniform 0.1 x 0.1 m puck: m*(w^2+d^2)/12)
P.n_coil  = 4;            % coils per platform (4v4)
P.layout.r     = 25e-3;   % [m] coil offset from interface centre, square pattern (+-r, +-r)
P.layout.cant  = 0;       % [deg] coil axis cant; 0 = axes parallel to x, no lateral authority.
                          % First test: lateral out of scope (rank 2 is intended).
P.layout.cant_dir = [1 0; -1 0; 1 0; -1 0];   % per-coil tilt direction (y,z), rows match layout.pos;
                          % this pattern gives rank 3 when cant > 0, radial or uniform ones stay
                          % rank 2 (Section 5)

% Coil (same type as the 1v1 pair)
P.coil.r_core = 3.9e-3;   P.coil.L    = 30e-3;   P.coil.N  = 1289;
P.coil.R20    = 17.62;    P.coil.mu_r = 1000;    P.coil.cH = 0.85;
P.coil.dT     = 40;       % [K] assumed winding rise, used for hot resistance
P.drv.V_bus   = 9.0;      % [V] coil supply; F scales as (V/R)^2
P.drv.V_drop  = 0.3;      % [V] H-bridge drop, placeholder until measured
P.drv.n_bridge = 4;       % independent H-bridges per chaser coil
P.drive.mode   = 'target_bias';   % target coils at a fixed bias, chaser coils modulated:
                          % wrench linear in i and reversible ('symmetric' is attract-only)
P.drive.i_bias = [];      % [A] target-side bias current ([] -> i_max)

% Calibration, set by the 1v1 campaign (update_lut)
P.cal.scale = 1.00;       % multiplies the modelled pole strength q; 1.00 = uncalibrated
P.cal.x0    = 3.9e-3;     % [m] pole softening length (~core radius), fitted by the calibration
% asteria_calibration.mat (written by update_lut) overrides both; delete it to return to the
% geometry model
if exist('asteria_calibration.mat','file')
    C_ = load('asteria_calibration.mat');
    P.cal.scale = C_.cal.scale;  P.cal.x0 = C_.cal.x0;  P.cal.source = C_.cal.source;
    clear C_
else
    P.cal.source = 'geometry model, UNCALIBRATED';
end

% Rates and sensors
P.Ts        = 1e-3;       % [s] controller / EKF predict rate
P.tof.Ts    = 0.033;      % [s] VL53L1X timing budget
P.tof.sigma = 5e-3;       % [m] 1-sigma range noise
P.tof.latency = 0.020;    % [s] ToF delay assumed by the EKF (about half the timing budget
                          % plus transport); measured in stage S2
P.tof.r_min = 0.040;      % [m] minimum range; the recess must exceed it
P.tof.b     = [+0.035; -0.035];   % [m] lateral positions of the two face-looking ToF sensors
P.tof.recess = 0.050;     % [m] sensor recess behind the interface plane
P.lat.sigma = 5e-3;       % [m] lateral sensor noise, placeholder until measured
P.lat.have  = true;       % false: y unobservable
P.imu.nd_acc  = 175e-6*9.81;  % [m/s^2/sqrt(Hz)] BMI088 accel noise density
P.imu.bw      = 100;          % [Hz] accelerometer bandwidth (logged; not used by the EKF)
P.gyro.have   = true;     % BMI088 gyro, the only direct yaw-rate sensor
P.gyro.Ts     = 0.002;    % [s] 500 Hz; must be an integer multiple of P.Ts
P.gyro.sigma  = deg2rad(0.2);   % [rad/s] 1-sigma per sample (~0.014 deg/s/sqrt(Hz) at 200 Hz BW)
P.gyro.sig_b0 = deg2rad(0.5);   % [rad/s] initial bias uncertainty
P.gyro.sig_bw = deg2rad(0.01);  % [rad/s per sqrt(s)] bias random walk
P.gyro.bias   = deg2rad(0.3);   % [rad/s] true bias, used by the simulations only
P.ina.sigma = 2e-3;       % [A] current sense noise

% Simulink scenario settings, read by the blocks at run time (never stored in the .slx)
P.test.d     = [0;0;0];   % [m/s^2, rad/s^2] disturbance: table tilt, air flow, umbilical
P.test.noise = 1;         % 1 = sensor noise on
P.test.tof_latency = 0.020;  % [s] delay applied by the simulated sensor; P.tof.latency is the
                          % filter's assumption, kept separate

% LQR design point and Bryson bounds
P.des.x      = 0.040;     % [m] linearisation gap
P.des.i_frac = 0.5;       % trim current as a fraction of i_max
P.lqr.dx_max  = 0.010;    % [m]     axial error bound
P.lqr.dth_max = deg2rad(2);   % [rad]
P.lqr.dv_max  = 0.010;    % [m/s]
P.lqr.dom_max = deg2rad(5);   % [rad/s]

% Approach endpoints; the profile is built by optimize_maneuver (P.ref)
P.traj.x0 = 0.060;  P.traj.xf = 0.000;  P.traj.vf = -0.020;
% Start gap 60 mm for the first hardware test, then 80 and 100 mm after each passes; tilt
% tolerance shrinks with the gap (RUNBOOK section 5)

% EKF
P.ekf.x_min = 5e-4;       % [m] lower clamp on the estimated gap; keep it well below the
                          % terminal phase (5 mm biased the estimate at contact)
P.ekf.sig_a = 1e-3;       % [m/s^2]   unmodelled linear acceleration
P.ekf.sig_al= 1e-3;       % [rad/s^2] unmodelled angular acceleration
P.ekf.sig_d = 1e-4;       % disturbance random-walk rate

%% 1. ELECTRICAL LIMITS
P.coil.R_hot = P.coil.R20*(1 + 0.00393*P.coil.dT);
P.i_max = (P.drv.V_bus - P.drv.V_drop)/P.coil.R_hot;
P.i_sat = P.i_max;
P.des.i = P.des.i_frac*P.i_max;
if isempty(P.drive.i_bias), P.drive.i_bias = P.i_max; end
P.pwr.per_coil = P.i_max^2*P.coil.R_hot;
P.pwr.float    = P.n_coil*P.pwr.per_coil;     % dissipated on the moving platform
P.pwr.total    = 2*P.pwr.float;

%% 2. ACTUATOR MODEL (pole strength from the coil geometry, scaled by P.cal.scale)
c = P.coil;
rho_cu   = 1.72e-8;
fR       = @(d) rho_cu*c.N*2*pi*(c.r_core + c.N*d^2/(2*c.L))/(pi*d^2/4) - c.R20;
c.d_wire = fzero(fR, [0.1e-3 0.5e-3]);
c.r_mean = c.r_core + c.N*c.d_wire^2/(2*c.L);
k_ar     = c.L/(2*c.r_core);  e_ar = sqrt(k_ar^2 - 1);
c.Nd     = (k_ar/e_ar*log(k_ar + e_ar) - 1)/(k_ar^2 - 1);
c.chi    = (c.mu_r - 1)/(1 + c.Nd*(c.mu_r - 1));
c.H_perA = c.cH*c.N/c.L;
c.m_perA = c.chi*c.H_perA*pi*c.r_core^2*c.L + c.N*pi*c.r_mean^2;
c.q_perA = P.cal.scale*c.m_perA/c.L;          % pole strength per amp, the calibrated quantity
c.B_core_max = mu0*c.chi*c.H_perA*P.i_max;
c.Lind   = mu0*c.N^2*(c.chi*pi*c.r_core^2 + pi*c.r_mean^2)/c.L;
c.tau    = c.Lind/c.R_hot;
P.coil   = c;

% Coil positions (y,z) on both platforms and cant directions
r = P.layout.r;
P.layout.pos = [ r  r;  r -r; -r  r; -r -r];
P.layout.u   = P.layout.cant_dir;    % tilt direction per coil, unit rows

%% 3. WRENCH MODEL AND ITS JACOBIANS AT THE DESIGN POSE
xi0 = [P.des.x; 0; 0];
i0  = P.des.i*ones(P.n_coil,1);

W0  = asteria_wrench(P, xi0, i0);
Bi  = zeros(3, P.n_coil);  h = 1e-5;
for k = 1:P.n_coil
    ip = i0; ip(k) = ip(k)+h;  im = i0; im(k) = im(k)-h;
    Bi(:,k) = (asteria_wrench(P,xi0,ip) - asteria_wrench(P,xi0,im))/(2*h);
end
Kpose = zeros(3,3);  hp = [1e-6; 1e-6; 1e-6];
for k = 1:3
    xp = xi0; xp(k) = xp(k)+hp(k);  xm = xi0; xm(k) = xm(k)-hp(k);
    Kpose(:,k) = (asteria_wrench(P,xp,i0) - asteria_wrench(P,xm,i0))/(2*hp(k));
end
P.des.W0 = W0;  P.des.Bi = Bi;  P.des.Kpose = Kpose;
P.des.sv = svd(Bi);
P.des.rank_ctrl = sum(P.des.sv > 1e-6*max(P.des.sv));

%% 4. PLANT, CONTROLLABLE SUBSYSTEM, LQR
Minv = diag([1/P.m, 1/P.m, 1/P.J]);
P.A6 = [zeros(3) eye(3); Minv*Kpose zeros(3)];        % states [x y th vx vy om]
P.B6 = [zeros(3,P.n_coil); Minv*Bi];
P.C6 = eye(3,6);

% y-coupling of the stiffness at the aligned pose (checked, not assumed)
P.chk.y_coupling = max(abs([Kpose(1,2) Kpose(3,2) Kpose(2,1) Kpose(2,3)]));
sel  = [1 3 4 6];                                      % x, th, vx, om
P.A4 = P.A6(sel,sel);  P.B4 = P.B6(sel,:);
assert(rank(ctrb(P.A4,P.B4)) == 4, 'axial+yaw subsystem not controllable -- check cant/layout');

P.lqr.Q = diag([1/P.lqr.dx_max^2, 1/P.lqr.dth_max^2, 1/P.lqr.dv_max^2, 1/P.lqr.dom_max^2]);
P.lqr.R = eye(P.n_coil)/P.i_max^2;
[P.K4, ~, P.cl_eig] = lqr(P.A4, P.B4, P.lqr.Q, P.lqr.R);   % di = -K4*[dx; dth; dvx; dom]
P.ol_eig6 = eig(P.A6);

% Lateral and yaw open-loop modes
P.mode.ky     = -Kpose(2,2);                  % [N/m]   > 0 = restoring
P.mode.kth    = -Kpose(3,3);                  % [N m/rad] < 0 = unstable
P.mode.y_freq = sqrt(max(P.mode.ky,0)/P.m)/(2*pi);
P.mode.th_pole= sqrt(max(-P.mode.kth,0)/P.J);

% Trim allocation: minimum-norm currents for a demanded (Fx, Mz)
P.alloc.pinv = pinv(Bi([1 3],:));
P.alloc.null = null(Bi);

%% 5. CANT SWEEP (lateral authority from canted coils)
cants = 0:2:12;
P.cant.deg = cants(:);
P.cant.dFx = zeros(numel(cants),1);
P.cant.dFy = zeros(numel(cants),1);
P.cant.dMz = zeros(numel(cants),1);
for n = 1:numel(cants)
    Pn = P;  Pn.layout.cant = cants(n);
    Bn = zeros(3,P.n_coil);
    for k = 1:P.n_coil
        ip = i0; ip(k) = ip(k)+h;  im = i0; im(k) = im(k)-h;
        Bn(:,k) = (asteria_wrench(Pn,xi0,ip) - asteria_wrench(Pn,xi0,im))/(2*h);
    end
    P.cant.dFx(n) = abs(sum(Bn(1,:)));                 % axial authority, all coils together
    P.cant.dFy(n) = max(abs(Bn(2,:)))*2;               % lateral authority, differential
    P.cant.dMz(n) = max(abs(Bn(3,:)))*2;
    P.cant.rank(n) = rank(Bn, 1e-9);
end

%% 6. SENSORS AND EKF (10 states: [x y th vx vy om dx dy dth bg])
P = asteria_ekf_sizes(P);
% observability of the pose at the design point
sref = [xi0; 0; 0; 0; 0; 0; 0; 0];
Hn   = zeros(numel(asteria_meas(P,sref)), 10);
for k = [1 2 3 6 10]
    xp = sref; xp(k)=xp(k)+1e-6;  xm = sref; xm(k)=xm(k)-1e-6;
    Hn(:,k) = (asteria_meas(P,xp) - asteria_meas(P,xm))/2e-6;
end
P.ekf.H_design = Hn;
P.ekf.obs_rank = rank(Hn(:,1:3));
% yaw resolution of the ToF pair alone [rad per sample]
P.ekf.th_per_sample = P.tof.sigma*sqrt(2)/(max(P.tof.b)-min(P.tof.b));

%% 7. REPORT
fprintf('=== ASTERIA bench_init_3dof ===  force model: %s (scale %.3f, x0 %.2f mm)\n', P.cal.source, P.cal.scale, 1e3*P.cal.x0);
fprintf('m = %.3f kg   J = %.2e kg m^2   %dv%d   cant = %.1f deg   V_bus = %.1f V\n', ...
        P.m, P.J, P.n_coil, P.n_coil, P.layout.cant, P.drv.V_bus);
fprintf('i_max = %.3f A   P_coil = %.2f W   P_float = %.1f W   P_total = %.1f W   tau = %.2f ms\n', ...
        P.i_max, P.pwr.per_coil, P.pwr.float, P.pwr.total, 1e3*c.tau);
fprintf('q = %.3f A*m/A   B_core = %.2f T\n', c.q_perA, c.B_core_max);

fprintf('\n-- Wrench at design pose (x = %.0f mm, aligned, i = %.3f A all coils) --\n', 1e3*P.des.x, P.des.i);
fprintf('Fx = %+.3f mN   Fy = %+.2e mN   Mz = %+.2e uN*m\n', 1e3*W0(1), 1e3*W0(2), 1e6*W0(3));
fprintf('dFx/di = %+.3f mN/A (per coil)   dFy/di = %.1e mN/A   dMz/di = %+.1f uN*m/A\n', ...
        1e3*Bi(1,1), 1e3*max(abs(Bi(2,:))), 1e6*Bi(3,1));
fprintf('singular values of dW/di: [%.3e %.3e %.3e] -> RANK %d of 3\n', P.des.sv, P.des.rank_ctrl);
fprintf('y/x and y/th cross-coupling in Kpose: %.1e (0 = y decouples)\n', P.chk.y_coupling);

fprintf('\n-- Open-loop modes --\n');
fprintf('axial:   K_x = %+.3e N/m   -> pole %+.3f rad/s (unstable)\n', Kpose(1,1), sqrt(max(Kpose(1,1)/P.m,0)));
fprintf('lateral: k_y = %+.3e N/m   -> %.4f Hz undamped, NOT commandable\n', P.mode.ky, P.mode.y_freq);
fprintf('yaw:     k_th= %+.3e N m/rad -> pole %+.3f rad/s (unstable, commandable)\n', P.mode.kth, P.mode.th_pole);

fprintf('\n-- LQR on [x th vx om] --\n');
disp(P.K4);
fprintf('closed-loop eigenvalues: %s\n', mat2str(P.cl_eig.', 4));

fprintf('\n-- Cant sweep (differential authority per coil pair, at i = %.3f A) --\n', P.des.i);
fprintf('%6s %12s %12s %12s %6s\n', 'deg', 'dFx/di[mN/A]', 'dFy/di[mN/A]', 'dMz/di[uNm/A]', 'rank');
for n = 1:numel(cants)
    fprintf('%6.1f %12.3f %12.4f %12.1f %6d\n', P.cant.deg(n), 1e3*P.cant.dFx(n), ...
            1e3*P.cant.dFy(n), 1e6*P.cant.dMz(n), P.cant.rank(n));
end

fprintf('\n-- Approach --\n');
if isfield(P,'ref')
    fprintf('reference in P.ref: T = %.1f s, arrival %.1f mm/s, peak %.0f%% of i_max\n', ...
            P.ref.T, 1e3*P.ref.v(end), 100*max(abs(P.ref.i))/P.i_max);
else
    fprintf('endpoints %.0f -> %.0f mm at %.0f mm/s; run optimize_maneuver to build P.ref\n', ...
            1e3*P.traj.x0, 1e3*P.traj.xf, 1e3*P.traj.vf);
end

fprintf('\n-- Estimator --\n');
fprintf('%d ToF (recessed %.0f mm) + lateral sensor: pose observability rank %d of 3\n', ...
        numel(P.tof.b), 1e3*P.tof.recess, P.ekf.obs_rank);

clear c e_ar k_ar fR rho_cu r xi0 i0 W0 Bi Kpose Minv sel h hp k n ip im xp xm ...
      cants Pn Bn Hn mu0
