%% optimize_maneuver.m  --  reference profile for the axial approach
% Run after bench_init_3dof. Writes P.ref (time-indexed and gap-indexed reference, feedforward).
%
% A reference vehicle chases a desired speed profile
%   v_des(x) = -min(v_cap, |vf| + (v_cap - |vf|)*x/x_gate)
% with acceleration limited to +-alpha*a_avail(x), where a_avail is the axial authority at
% full current. The speed ramps down to vf over the last x_gate, so braking spans ~0.75 s
% and ~20 range samples (a minimum-time profile brakes in ~0.1 s, see RUNBOOK section 10),
% and (1 - alpha) of the current is left for feedback.
% The feedforward current is i_ff = m*a_ref/g1(x), g1 the axial force per amp of common-mode
% current.

assert(exist('P','var')==1, 'run bench_init_3dof first');
P.ref.alpha = 0.5;      % fraction of the authority the reference may use
P.ref.x_gate = 0.030;   % [m] gap at which the terminal speed ramp starts
P.ref.v_cap  = 0.040;   % [m/s] speed cap / speed at the gate
P.ref.kv     = 20;      % [1/s] speed-tracking gain of the reference vehicle
vf = P.traj.vf;  x0 = P.traj.x0;

% axial force per amp and authority per unit mass on a gap grid
xg = [linspace(0, 0.02, 400), linspace(0.0205, x0, 600)].';
g1 = zeros(size(xg));                      % [N/A] axial force per amp of common-mode current
for k = 1:numel(xg)
    w = asteria_wrench(P, [max(xg(k),1e-5);0;0], ones(P.n_coil,1));
    g1(k) = w(1);
end
a_avail = abs(g1)*P.i_max/P.m;             % [m/s^2]
W  = cumtrapz(xg, a_avail);                % braking work per unit mass available from 0 to x
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
% g1 goes to zero at contact (softened poles), so the inverse is clamped to +-alpha*i_max
P.ref.i = max(min(P.ref.i, P.ref.alpha*P.i_max), -P.ref.alpha*P.i_max);
P.ref.t_term = P.ref.T - interp1(P.ref.x(end:-1:1), tt(end:-1:1), 0.020);   % time inside 20 mm
[~, ipk] = max(abs(P.ref.v));

fprintf('=== reshaped approach (gate %.0f mm, v_cap %.0f mm/s, alpha %.2f) ===\n', 1e3*P.ref.x_gate, 1e3*P.ref.v_cap, P.ref.alpha);
fprintf('T = %.1f s   peak speed %.1f mm/s at x = %.1f mm   arrival %.1f mm/s\n', ...
        P.ref.T, 1e3*abs(P.ref.v(ipk)), 1e3*P.ref.x(ipk), 1e3*P.ref.v(end));
fprintf('last 20 mm take %.2f s  (=> %.0f ToF samples, %.0f gyro samples)\n', ...
        P.ref.t_term, P.ref.t_term/P.tof.Ts, P.ref.t_term/P.gyro.Ts);
fprintf('peak reference current %.3f A = %.0f%% of i_max\n', max(abs(P.ref.i)), 100*max(abs(P.ref.i))/P.i_max);
% fraction of the approach spent on the authority limit
on_clamp = abs(abs(P.ref.a) - P.ref.alpha*aav(P.ref.x)) < 1e-9*max(aav(P.ref.x));
fprintf('authority-limited for %.0f%% of the approach; clamp releases at x = %.0f mm\n', ...
        100*mean(on_clamp), 1e3*P.ref.x(find(~on_clamp,1,'first')));

% Tables stored at 401 points (embedded constants ~23 kB; 4001 points gave the same Monte Carlo
% result). xp/vp/ip: gap-indexed copy used for path-following inside P.ref.path_gate.
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

%% Open-loop sensitivity of the arrival velocity to the force scale
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
