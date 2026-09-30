%% test_envelope.m  --  closed-loop envelope: how much tilt and how much force error it takes
% Produces the two numbers the bench needs as requirements: the levelling spec, and the force
% calibration error the loop can absorb. Run after build_schedule.
assert(isfield(P,'sched'), 'run build_schedule first');
dt = P.Ts; tof_every = round(P.tof.Ts/dt); gyro_every = round(P.gyro.Ts/dt);
n_tof = numel(P.tof.b);
a100 = abs(subsref(asteria_wrench(P,[P.traj.x0;0;0],P.i_max*ones(P.n_coil,1)), ...
                   struct('type','()','subs',{{1}})))/P.m;
fprintf('axial authority at %.0f mm: %.2e m/s^2 = tilt of %.4f deg (ALL of it, at i_max)\n', ...
        1e3*P.traj.x0, a100, asind(a100/9.81));
fprintf('\n-- tilt sweep (noise on, nominal force) --\n%12s %10s %10s %10s %8s\n', ...
        'tilt[deg]','frac auth','v_end','RMS trk','sat[%]');
for dx = [0 1e-4 3e-4 6e-4 1e-3 1.5e-3]
    [ve, sat, tr] = loop_run(P, [dx;0;0], 1.0, true, dt, tof_every, gyro_every, n_tof);
    fprintf('%12.4f %10.2f %10.1f %10.2f %8.1f\n', asind(dx/9.81), dx/a100, 1e3*ve, 1e3*tr, sat);
end
fprintf('\n-- force calibration sweep (noise on, tilt 3e-4 m/s^2) --\n%12s %10s %10s %8s\n', ...
        'force x','v_end','RMS trk','sat[%]');
for fs = [0.5 0.7 1.0 1.5 2.0]
    [ve, sat, tr] = loop_run(P, [3e-4;0;0], fs, true, dt, tof_every, gyro_every, n_tof);
    fprintf('%12.2f %10.1f %10.2f %8.1f\n', fs, 1e3*ve, 1e3*tr, sat);
end

function [v_end, satpct, rmstrk] = loop_run(P, dtrue, fscale, noisy, dt, tof_every, gyro_every, n_tof)
clear asteria_ekf
rng(1);
Pp = P;  Pp.coil.q_perA = P.coil.q_perA*sqrt(fscale);      % truth plant; controller keeps P
s  = [P.ref.x(1);0;0;0;0;0];  xh = [P.ref.x(1);zeros(9,1)];  Pk = P.ekf.P0;
k = 0;  nsat = 0;  trk = [];
while s(1) > 1e-4 && k*dt < 2.5*P.ref.T
    k = k+1;  t = (k-1)*dt;
    [ic, ia] = asteria_ctrl(xh, t, P);
    nsat = nsat + any(abs(ic) > P.i_sat + 1e-12);
    f = @(ss) asteria_plant(t, ss, ia, Pp, dtrue);
    k1=f(s); k2=f(s+dt/2*k1); k3=f(s+dt/2*k2); k4=f(s+dt*k3);
    s = s + dt/6*(k1+2*k2+2*k3+k4);
    zt = asteria_meas(P, [s;0;0;0;P.gyro.bias]);  z = nan(size(zt));
    if mod(k,tof_every)==0
        idx = 1:(n_tof+P.lat.have);  z(idx) = zt(idx);
        if noisy
            z(1:n_tof) = z(1:n_tof) + P.tof.sigma*randn(n_tof,1);
            z(n_tof+1) = z(n_tof+1) + P.lat.sigma*randn;
        end
    end
    if mod(k,gyro_every)==0, z(end) = zt(end) + noisy*P.gyro.sigma*randn; end
    [xh, Pk] = asteria_ekf(xh, Pk, ia, z, P, dt, mod(k,20)==0);
    trk(end+1) = s(1) - interp1(P.ref.t, P.ref.x, min(t,P.ref.T), 'linear','extrap'); %#ok<AGROW>
end
v_end = s(4);  satpct = 100*nsat/k;  rmstrk = sqrt(mean(trk.^2));
end
