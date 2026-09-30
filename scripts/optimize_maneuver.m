%% optimize_maneuver.m  --  reference profile for the axial approach
% The minimum-time profile (accelerate hard, reverse at a switch point) put the whole braking
% phase into the last ~6 mm and ~80 ms. At a 30 Hz range update that window holds two or three
% measurements, so any force-model error lands at contact uncorrected. The reference is
% therefore shaped deliberately slower than time-optimal:
%
%   v_des(x) = -min( v_cap, sqrt(vf^2 + 2*beta*W(x)) ),   W(x) = integral of a_avail from 0 to x
%
% i.e. the speed at every gap is set by the braking energy actually available from there to
% contact, using only a fraction beta of it. Braking then starts far out and lasts of order a
% second instead of 80 ms, which is what gives the estimator time to work. alpha caps the
% acceleration side the same way, leaving (1-alpha) of the current for feedback.
% Run after bench_init_3dof. Writes P.ref.

assert(exist('P','var')==1, 'run bench_init_3dof first');
P.ref.alpha = 0.5;      % fraction of authority the reference may use to accelerate
P.ref.x_gate = 0.030;   % [m] gap at which the terminal speed ramp starts
P.ref.v_cap  = 0.040;   % [m/s] speed cap / speed at the gate
P.ref.kv     = 20;      % [1/s] how hard the reference vehicle chases v_des
vf = P.traj.vf;  x0 = P.traj.x0;

% axial authority per unit mass, and the per-amp axial gain, on a grid
xg = [linspace(0, 0.02, 400), linspace(0.0205, x0, 600)].';
g1 = zeros(size(xg));                      % [N/A] axial force per amp of common-mode current
for k = 1:numel(xg)
    w = asteria_wrench(P, [max(xg(k),1e-5);0;0], ones(P.n_coil,1));
    g1(k) = w(1);
end
a_avail = abs(g1)*P.i_max/P.m;             % [m/s^2]
W  = cumtrapz(xg, a_avail);                % braking energy per unit mass available from 0 to x
v_des = @(x) -min(P.ref.v_cap, abs(vf) + (P.ref.v_cap-abs(vf))*min(max(x,0),x0)/P.ref.x_gate);
aav   = @(x) interp1(xg, a_avail, min(max(x,0),x0), 'linear','extrap');
gg    = @(x) interp1(xg, g1,      min(max(x,0),x0), 'linear','extrap');
acmd  = @(x,v) max(min(P.ref.kv*(v_des(x) - v), P.ref.alpha*aav(x)), -P.ref.alpha*aav(x));

sol = ode45(@(t,s)[s(2); acmd(s(1),s(2))], [0 400], [x0; 0], ...
            odeset('Events', @ctc, 'RelTol',1e-9, 'AbsTol',1e-12));
tt = linspace(0, sol.x(end), 4001).';  ss = deval(sol, tt).';
P.ref.T = sol.x(end);
P.ref.t = tt;  P.ref.x = ss(:,1);  P.ref.v = ss(:,2);
P.ref.a = arrayfun(@(k) acmd(ss(k,1), ss(k,2)), (1:numel(tt)).');
P.ref.i = P.m*P.ref.a./gg(P.ref.x);
% The per-amp axial gain vanishes AT contact (softened poles: the facing-pole separation goes
% to zero), so inverting it blows the feedforward up in the last fraction of a millimetre.
% The acceleration there is negligible anyway; clamp to the authority the reference is allowed.
P.ref.i = max(min(P.ref.i, P.ref.alpha*P.i_max), -P.ref.alpha*P.i_max);
P.ref.t_term = P.ref.T - interp1(P.ref.x(end:-1:1), tt(end:-1:1), 0.020);   % time inside 20 mm
[~, ipk] = max(abs(P.ref.v));

fprintf('=== reshaped approach (gate %.0f mm, v_cap %.0f mm/s, alpha %.2f) ===\n', 1e3*P.ref.x_gate, 1e3*P.ref.v_cap, P.ref.alpha);
fprintf('T = %.1f s   peak speed %.1f mm/s at x = %.1f mm   arrival %.1f mm/s\n', ...
        P.ref.T, 1e3*abs(P.ref.v(ipk)), 1e3*P.ref.x(ipk), 1e3*P.ref.v(end));
fprintf('last 20 mm take %.2f s  (=> %.0f ToF samples, %.0f gyro samples)\n', ...
        P.ref.t_term, P.ref.t_term/P.tof.Ts, P.ref.t_term/P.gyro.Ts);
fprintf('peak reference current %.3f A = %.0f%% of i_max\n', max(abs(P.ref.i)), 100*max(abs(P.ref.i))/P.i_max);
% The reference is feasible BY CONSTRUCTION (a_cmd is clamped to alpha*a_avail), so what is
% worth reporting is how much of the run sits ON that clamp: that is the authority-limited
% phase, where the profile is doing all it can rather than tracking v_des.
on_clamp = abs(abs(P.ref.a) - P.ref.alpha*aav(P.ref.x)) < 1e-9*max(aav(P.ref.x));
fprintf('authority-limited for %.0f%% of the approach; clamp releases at x = %.0f mm\n', ...
        100*mean(on_clamp), 1e3*P.ref.x(find(~on_clamp,1,'first')));

% Gap-indexed copy of the reference, for path-following inside P.ref.path_gate. The arrival spec
% is a function of GAP, so near contact the controller tracks the reference at the estimated gap,
% not at the current time. With a time-indexed reference, a run that falls behind catches up and
% arrives fast: arrival velocity correlated +0.81 with timing error over 60 noisy runs.
% Store the reference at 401 points, not the integration grid. The embedded code carries these
% tables as constants: at 4001 points they were 233 kB, at 401 about 23 kB. Linear interpolation
% between 13 ms samples is far below anything the loop can resolve (checked by Monte Carlo).
P.ref.n_table = 401;
tt_ = linspace(0, P.ref.T, P.ref.n_table).';
P.ref.x = interp1(P.ref.t, P.ref.x, tt_);  P.ref.v = interp1(P.ref.t, P.ref.v, tt_);
P.ref.i = interp1(P.ref.t, P.ref.i, tt_);
if isfield(P.ref,'a'), P.ref.a = interp1(P.ref.t, P.ref.a, tt_); end
P.ref.t = tt_;  clear tt_
[xp_, ia_] = unique(P.ref.x);
P.ref.xp = xp_;  P.ref.vp = P.ref.v(ia_);  P.ref.ip = P.ref.i(ia_);
P.ref.path_gate = P.ref.x_gate;
clear xp_ ia_
fprintf('\n%8s %10s %10s %12s\n','x[mm]','v_ref[mm/s]','i_ref[A]','a_avail[m/s2]');
for xq = [100 60 40 20 10 5 2 1]
    k = find(P.ref.x <= xq*1e-3, 1, 'first');
    if ~isempty(k)
        fprintf('%8.0f %10.1f %10.3f %12.2e\n', xq, 1e3*P.ref.v(k), P.ref.i(k), aav(P.ref.x(k)));
    end
end

%% Sensitivity of the arrival velocity to the force calibration, OPEN LOOP
fprintf('\n-- open loop, reference currents replayed into a mis-calibrated plant --\n');
fprintf('%10s %12s\n','force x','v_end[mm/s]');
i_of_t = @(t) interp1(P.ref.t, P.ref.i, min(t,P.ref.T), 'linear', 'extrap');
for fs = [0.5 1 1.5 2]
    Pp = P;  Pp.coil.q_perA = P.coil.q_perA*sqrt(fs);
    s2 = ode45(@(t,s) asteria_plant(t, s, i_of_t(t)*ones(P.n_coil,1), Pp, [0;0;0]), ...
               [0 3*P.ref.T], [x0;0;0;0;0;0], odeset('Events',@ctc,'RelTol',1e-9));
    fprintf('%10.1f %12.1f\n', fs, 1e3*s2.y(4,end));
end

function [val, ter, dir] = ctc(~, s)
val = s(1);  ter = 1;  dir = -1;
end
