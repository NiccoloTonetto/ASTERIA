%% test_ekf.m  --  EKF against PERFECT STATE, open loop (no feedback yet)
% Truth is propagated with the same plant, driven by the optimised reference current profile;
% the EKF sees only sensors. Four cases: clean, noise, disturbance, both. Then a with/without
% gyro comparison, which is the point of the 10th state.
% Run after bench_init_3dof and optimize_maneuver.

assert(isfield(P,'ref'), 'run optimize_maneuver first');
P_save = P;
for use_gyro = [true false]
    P = P_save;  P.gyro.have = use_gyro;  P = asteria_ekf_sizes(P);
    ekf_cases(P);
end
P = P_save;

function ekf_cases(P)
rng(0);
dt = P.Ts;  T_sim = min(6, P.ref.T);  n_step = round(T_sim/dt);
tof_every  = round(P.tof.Ts/dt);
gyro_every = round(P.gyro.Ts/dt);
n_tof = numel(P.tof.b);
i_of_t = @(t) max(min(interp1(P.ref.t, P.ref.i, min(t,P.ref.T), 'linear', 0), P.i_sat), -P.i_sat);
cases = {'clean          ', false, [0;0;0];
         'noise          ', true,  [0;0;0];
         'disturbance    ', false, [1.5e-3; 5e-4; 1e-5];
         'noise + dist.  ', true,  [1.5e-3; 5e-4; 1e-5]};
fprintf('\n=== EKF vs perfect state, gyro %s (%.0f s, %d Hz) ===\n', ...
        string(missing_str(P.gyro.have)), T_sim, 1/dt);
fprintf('%s %10s %10s %11s %11s %10s\n','case','RMS x[mm]','RMS y[mm]','RMS th[deg]','RMS om[d/s]','d_x err[%]');
for c = 1:size(cases,1)
    [nm, noisy, dtrue] = cases{c,:};
    clear asteria_ekf
    s  = [P.ref.x(1); 0; 0; 0; 0; 0];
    xh = [P.ref.x(1)+5e-3; 3e-3; deg2rad(0.5); 0;0;0; 0;0;0; 0];
    Pk = P.ekf.P0;
    err = zeros(n_step,4);  dh = zeros(n_step,1);
    for k = 1:n_step
        t  = (k-1)*dt;
        ic = i_of_t(t)*ones(P.n_coil,1);
        f  = @(ss) asteria_plant(t, ss, ic, P, dtrue);
        k1=f(s); k2=f(s+dt/2*k1); k3=f(s+dt/2*k2); k4=f(s+dt*k3);
        s  = s + dt/6*(k1+2*k2+2*k3+k4);  s(1) = max(s(1), 1e-4);
        ztrue = asteria_meas(P, [s; 0;0;0; P.gyro.bias]);
        z = nan(size(ztrue));
        if mod(k,tof_every)==0
            idx = 1:(n_tof + P.lat.have);
            z(idx) = ztrue(idx);
            if noisy
                z(1:n_tof) = z(1:n_tof) + P.tof.sigma*randn(n_tof,1);
                if P.lat.have, z(n_tof+1) = z(n_tof+1) + P.lat.sigma*randn; end
            end
        end
        if P.gyro.have && mod(k,gyro_every)==0
            z(end) = ztrue(end) + noisy*P.gyro.sigma*randn;
        end
        [xh, Pk] = asteria_ekf(xh, Pk, ic, z, P, dt, mod(k,20)==0);
        err(k,:) = [(xh(1:3) - s(1:3)).', xh(6)-s(6)];
        dh(k) = xh(7);
    end
    h2 = round(n_step/2):n_step;
    r = sqrt(mean(err(h2,:).^2, 1));
    if dtrue(1)~=0, de = 100*abs(mean(dh(h2))-dtrue(1))/abs(dtrue(1)); else, de = NaN; end
    fprintf('%s %10.3f %10.3f %11.4f %11.4f %10.1f\n', nm, 1e3*r(1), 1e3*r(2), ...
            rad2deg(r(3)), rad2deg(r(4)), de);
end
end
function s = missing_str(b), if b, s = "ON "; else, s = "OFF"; end, end
