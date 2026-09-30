function make_log(P, scenario, c, outfile)
% MAKE_LOG  Write a hardware-format log from the simulator, with truth injected.
%   make_log(P, scenario, c, outfile)
%   scenario: 'rest' | 'drift' | 'pulse' | 'approach'   (stages S1, S2, S3, S5)
%   c: TRUTH deviations the replay is supposed to discover, none of which the filter is told:
%      c.fscale  force scale        c.x0      softening length [m]
%      c.m       mass [kg]          c.tilt    [3x1] disturbance (tilt etc.)
%      c.gbias   gyro bias [rad/s]  c.abias   accelerometer bias [m/s^2]
%      c.tof_off [n x 1] per-sensor range offsets [m]
%      c.tof_sc  [n x 1] per-sensor range scale factors
%      c.latency ToF measurement delay [s]
%      c.seed    noise seed         c.gaps    (rest only) clamped gaps [m]
%      c.i_pulse (pulse only) [amplitude A, start s, stop s]
%   The CSV has exactly the channels a flight-standard log would have, plus truth_* columns
%   that on hardware come from the validation camera (or not at all).
rng(c.seed);
dt = P.Ts;  n_tof = numel(P.tof.b);
Pp = P;                                          % TRUTH plant
Pp.coil.q_perA = P.coil.q_perA*sqrt(c.fscale);
Pp.cal.x0 = c.x0;  Pp.m = c.m;  Pp.J = P.J*c.m/P.m;

switch lower(scenario)
    case 'rest',     T_end = numel(c.gaps)*20;   % 20 s clamped at each gap
    case 'drift',    T_end = 25;
    case 'pulse',    T_end = 8;
    case 'approach', T_end = 2.2*P.ref.T;
    otherwise, error('unknown scenario %s', scenario);
end
N = round(T_end/dt);
nd = round(c.latency/dt);                        % delay in loop ticks
buf = zeros(6, nd+1);                            % ring of past true states

s  = [P.ref.x(1); 0; 0; 0; 0; 0];
% 'drift' accepts a start gap and an initial velocity: a slow release identifies the tilt,
% a fast coast identifies the ToF latency. A drift from rest cannot do the latter -- the
% float never moves fast enough for a delay to show above the range noise.
if isfield(c,'x_start'), s(1) = c.x_start; end
if isfield(c,'v0'),      s(4) = c.v0;      end
if strcmpi(scenario,'rest'), s(1) = c.gaps(1); end
xh = [P.ref.x(1); zeros(9,1)];  Pk = P.ekf.P0;   % onboard filter (closed loop only)
i_meas = zeros(P.n_coil,1);
rows = zeros(N, 8 + 2*P.n_coil + 7);
tof_every = round(P.tof.Ts/dt);  gyro_every = round(P.gyro.Ts/dt);

for k = 1:N
    t = (k-1)*dt;
    % ---- command
    switch lower(scenario)
        case 'rest'
            idx = min(floor(t/20)+1, numel(c.gaps));
            s = [c.gaps(idx); 0; 0; 0; 0; 0];    % clamped: no dynamics
            i_cmd = zeros(P.n_coil,1);
        case 'drift'
            i_cmd = zeros(P.n_coil,1);
        case 'pulse'
            on = t >= c.i_pulse(2) && t < c.i_pulse(3);
            i_cmd = c.i_pulse(1)*on*ones(P.n_coil,1);
        case 'approach'
            [~, i_cmd] = asteria_ctrl(xh, t, P);
    end
    % ---- coil current lags the command through L/R
    a_lag  = exp(-dt/P.coil.tau);
    i_meas = a_lag*i_meas + (1-a_lag)*i_cmd;
    i_log  = i_meas + P.ina.sigma*randn(P.n_coil,1);
    % ---- truth propagation (RK4), except when clamped
    if ~strcmpi(scenario,'rest')
        f = @(ss) asteria_plant(t, ss, i_meas, Pp, c.tilt);
        k1=f(s); k2=f(s+dt/2*k1); k3=f(s+dt/2*k2); k4=f(s+dt*k3);
        s = s + dt/6*(k1+2*k2+2*k3+k4);
        if s(1) <= 0, rows = rows(1:k-1,:); N = k-1; break; end
    end
    buf = [s, buf(:,1:end-1)];                   %#ok<AGROW>  newest first
    s_del = buf(:, min(nd+1, size(buf,2)));      % what the ToF was looking at
    % ---- sensors
    z = nan(n_tof + P.lat.have + P.gyro.have, 1);
    tof_valid = 0;  imu_valid = 0;
    if mod(k, tof_every) == 0
        zt = asteria_meas(P, [s_del; zeros(3,1); c.gbias]);
        z(1:n_tof) = c.tof_sc(:).*zt(1:n_tof) + c.tof_off(:) + P.tof.sigma*randn(n_tof,1);
        if P.lat.have, z(n_tof+1) = zt(n_tof+1) + P.lat.sigma*randn; end
        tof_valid = 1;
    end
    if mod(k, gyro_every) == 0
        z(end) = s(6) + c.gbias + P.gyro.sigma*randn;
        imu_valid = 1;
    end
    w = asteria_wrench(Pp, s(1:3), i_meas);      % specific force: magnetic only, no tilt
    acc = w(1)/Pp.m + c.abias + P.imu.nd_acc*sqrt(P.imu.bw)*randn;
    % ---- onboard filter, closed loop only
    if strcmpi(scenario,'approach')
        [xh, Pk] = asteria_ekf(xh, Pk, i_log, z, P, dt, mod(k,20)==0);
    end
    rows(k,:) = [t, z(1:n_tof).', z(n_tof+1), tof_valid, z(end), imu_valid, acc, ...
                 i_log.', i_cmd.', P.drive.i_bias, s(1:6).'];
end
rows = rows(1:N,:);

hdr = ['t_s,tof1_m,tof2_m,lat_m,tof_valid,gyro_z_rads,imu_valid,acc_x_ms2,' ...
       'i1_A,i2_A,i3_A,i4_A,ic1_A,ic2_A,ic3_A,ic4_A,i_target_A,' ...
       'truth_x,truth_y,truth_th,truth_vx,truth_vy,truth_om'];
fid = fopen(outfile,'w');  fprintf(fid, '%s\n', hdr);
fprintf(fid, [repmat('%.6e,',1,size(rows,2)-1) '%.6e\n'], rows.');
fclose(fid);
fprintf('make_log: %s -> %s (%d rows, %.1f s)\n', scenario, outfile, N, N*dt);
end
