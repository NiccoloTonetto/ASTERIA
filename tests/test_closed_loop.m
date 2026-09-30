%% test_closed_loop.m  --  full loop: plant + sensors + EKF + scheduled LQR
% Progression: perfect state first (isolates the controller), then through the estimator, then
% with everything wrong at once -- noise, table tilt, and a plant whose force is 1.5x the model.
% Run after bench_init_3dof, optimize_maneuver, build_schedule.

assert(isfield(P,'sched'), 'run build_schedule first');
dt = P.Ts;  tof_every = round(P.tof.Ts/dt);  gyro_every = round(P.gyro.Ts/dt);
n_tof = numel(P.tof.b);
runs = {'perfect state        ', false, [0;0;0],            1.0, false;
        'EKF, clean           ', false, [0;0;0],            1.0, true;
        'EKF + noise          ', true,  [0;0;0],            1.0, true;
        'EKF + noise + tilt   ', true,  [1.5e-3;5e-4;1e-5], 1.0, true;
        'EKF + all + force x1.5', true, [1.5e-3;5e-4;1e-5], 1.5, true};
fprintf('=== closed loop, approach to contact ===\n');
fprintf('%s %10s %11s %10s %9s %9s\n','run','v_end[mm/s]','RMS track[mm]','y_end[mm]','th_end[d]','sat[%]');
for r = 1:size(runs,1)
    [nm, noisy, dtrue, fscale, use_ekf] = runs{r,:};
    clear asteria_ekf
    rng(1);
    Pp = P;  Pp.coil.q_perA = P.coil.q_perA*sqrt(fscale);     % TRUTH plant (controller keeps P)
    s  = [P.ref.x(1); 0; 0; 0; 0; 0];
    xh = [P.ref.x(1); 0; 0; 0; 0; 0; 0; 0; 0; 0];
    Pk = P.ekf.P0;  k = 0;  trk = [];  nsat = 0;
    while s(1) > 1e-4 && k*dt < 2.5*P.ref.T
        k = k + 1;  t = (k-1)*dt;
        est = xh;  if ~use_ekf, est = [s; 0;0;0;0]; end
        [i_cmd, i_app] = asteria_ctrl(est, t, P);
        nsat = nsat + any(abs(i_cmd) > P.i_sat + 1e-12);
        f  = @(ss) asteria_plant(t, ss, i_app, Pp, dtrue);
        k1=f(s); k2=f(s+dt/2*k1); k3=f(s+dt/2*k2); k4=f(s+dt*k3);
        s  = s + dt/6*(k1+2*k2+2*k3+k4);
        if use_ekf
            ztrue = asteria_meas(P, [s; 0;0;0; P.gyro.bias]);
            z = nan(size(ztrue));
            if mod(k,tof_every)==0
                idx = 1:(n_tof+P.lat.have);  z(idx) = ztrue(idx);
                if noisy
                    z(1:n_tof) = z(1:n_tof) + P.tof.sigma*randn(n_tof,1);
                    z(n_tof+1) = z(n_tof+1) + P.lat.sigma*randn;
                end
            end
            if mod(k,gyro_every)==0, z(end) = ztrue(end) + noisy*P.gyro.sigma*randn; end
            [xh, Pk] = asteria_ekf(xh, Pk, i_app, z, P, dt, mod(k,20)==0);
        end
        trk(end+1) = s(1) - interp1(P.ref.t, P.ref.x, min(t,P.ref.T), 'linear','extrap'); %#ok<SAGROW>
    end
    fprintf('%s %10.1f %11.3f %10.2f %9.3f %9.1f\n', nm, 1e3*s(4), 1e3*sqrt(mean(trk.^2)), ...
            1e3*s(2), rad2deg(s(3)), 100*nsat/k);
end
fprintf('\nTarget arrival velocity: %.0f mm/s. Lateral is uncommanded (cant = 0), so y_end is\nwhatever the passive lateral spring does with the initial condition.\n', 1e3*P.traj.vf);
