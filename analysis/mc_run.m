function r = mc_run(P, c)
% MC_RUN  One closed-loop approach with a TRUTH plant that differs from the model.
%   The controller, estimator and reference always use the nominal P -- that is the point.
%   c.fscale  force multiplier of the truth plant (pole strength scales as sqrt)
%   c.x0      truth softening length [m] (near-field gradient steepness)
%   c.m       truth float mass [kg] (inertia scales with it)
%   c.d       [3x1] truth disturbance (table tilt etc.)
%   c.gbias   truth gyro bias [rad/s]
%   c.seed    noise seed
%   c.latency truth ToF delay [s] (default P.test.tof_latency). The filter's own belief is
%             P.tof.latency; keeping the two separate is what lets the compensation be tested.
% Realism matched to make_log: the ranges are delayed, the coil current lags the command
% through L/R, and the EKF is fed the MEASURED (lagged, noisy) current, as on hardware.
clear asteria_ekf                                  % no Jacobian carried over from a previous run
rng(c.seed);
if ~isfield(c,'latency'), c.latency = P.test.tof_latency; end
Pp = P;                                            % truth plant
Pp.coil.q_perA = P.coil.q_perA*sqrt(c.fscale);
Pp.cal.x0 = c.x0;
Pp.m = c.m;  Pp.J = P.J*c.m/P.m;
dt = P.Ts;  tof_every = round(P.tof.Ts/dt);  gyro_every = round(P.gyro.Ts/dt);
n_tof = numel(P.tof.b);
nd = max(round(c.latency/dt), 0);
a_lag = exp(-dt/P.coil.tau);
s  = [P.ref.x(1);0;0;0;0;0];
buf = repmat(s, 1, nd+1);
i_coil = zeros(P.n_coil,1);
xh = [P.ref.x(1);zeros(9,1)];  Pk = P.ekf.P0;
k = 0;  nsat = 0;  e2 = 0;  t_max = 2.5*P.ref.T;
while s(1) > 1e-4 && k*dt < t_max
    k = k+1;  t = (k-1)*dt;
    [ic, ia] = asteria_ctrl(xh, t, P);
    nsat = nsat + any(abs(ic) > P.i_sat + 1e-12);
    i_coil = a_lag*i_coil + (1-a_lag)*ia;          % what actually flows in the coils
    f = @(ss) asteria_plant(t, ss, i_coil, Pp, c.d);
    k1=f(s); k2=f(s+dt/2*k1); k3=f(s+dt/2*k2); k4=f(s+dt*k3);
    s = s + dt/6*(k1+2*k2+2*k3+k4);
    buf = [s, buf(:,1:end-1)];                      % newest first
    zt = asteria_meas(P, [buf(:,end);0;0;0;c.gbias]);  z = nan(size(zt));
    if mod(k,tof_every)==0
        idx = 1:(n_tof+P.lat.have);  z(idx) = zt(idx);
        z(1:n_tof) = z(1:n_tof) + P.tof.sigma*randn(n_tof,1);
        z(n_tof+1) = z(n_tof+1) + P.lat.sigma*randn;
    end
    if mod(k,gyro_every)==0, z(end) = s(6) + c.gbias + P.gyro.sigma*randn; end
    i_meas = i_coil + P.ina.sigma*randn(P.n_coil,1);
    [xh, Pk] = asteria_ekf(xh, Pk, i_meas, z, P, dt, mod(k,20)==0);
    xr = interp1(P.ref.t, P.ref.x, min(t,P.ref.T), 'linear','extrap');
    e2 = e2 + (s(1)-xr)^2;
end
r.contact = s(1) <= 1e-4;
r.v_end   = s(4);
r.t_end   = k*dt;
r.sat     = nsat/k;
r.rms_trk = sqrt(e2/k);
r.y_end   = s(2);
r.th_end  = s(3);
end
