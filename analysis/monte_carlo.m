%% monte_carlo.m  --  closed-loop robustness to plant uncertainty
% Run after bench_init_3dof, optimize_maneuver, build_schedule. Uses parfor if available.
% Each run draws a truth plant and flies the nominal controller, estimator and reference
% against it (nothing retuned per run). Fixed seed: every call draws the same scenarios.
%
% Uncertainties (independent, uniform unless noted):
%   force scale     log-uniform over P.mc.fs_range
%   softening x0    2 .. 8 mm
%   float mass      +-20 % (inertia scaled with it)
%   table tilt      +-P.mc.tilt_deg on x and y
%   gyro bias       +-0.5 deg/s
%   sensor noise    seed per run
% Pass: contact reached with arrival velocity within P.mc.v_tol of P.traj.vf.
% Output: mc_results.mat (table T) and a printed summary.

assert(isfield(P,'sched'), 'run build_schedule first');
% defaults, overridden by fields already set in P.mc
if ~isfield(P,'mc'), P.mc = struct(); end
if ~isfield(P.mc,'N'),        P.mc.N        = 300;       end
if ~isfield(P.mc,'tilt_deg'), P.mc.tilt_deg = 0.002;     end   % [deg] levelling spec
if ~isfield(P.mc,'v_tol'),    P.mc.v_tol    = 0.010;     end   % [m/s] arrival spec: -20 +- 10 mm/s
if ~isfield(P.mc,'fs_range'), P.mc.fs_range = [0.3 3];   end   % force scale: [0.3 3] uncalibrated,
                                   % [0.8 1.25] after the 1v1 calibration
N = P.mc.N;  lo = P.mc.fs_range(1);  hi = P.mc.fs_range(2);
rng(2026);
fs  = exp(log(lo) + (log(hi)-log(lo))*rand(N,1));
x0  = 2e-3 + 6e-3*rand(N,1);
mm  = P.m*(0.8 + 0.4*rand(N,1));
tx  = P.mc.tilt_deg*(2*rand(N,1)-1);
ty  = P.mc.tilt_deg*(2*rand(N,1)-1);
gb  = deg2rad(0.5)*(2*rand(N,1)-1);

R = repmat(struct('contact',false,'v_end',NaN,'t_end',NaN,'sat',NaN, ...
                  'rms_trk',NaN,'y_end',NaN,'th_end',NaN), N, 1);
Pb = P;                                           % broadcast copy for the workers
use_par = license('test','Distrib_Computing_Toolbox');
if use_par && isempty(gcp('nocreate')), parpool('Processes'); end
tic;
if use_par
    parfor n = 1:N
        c = struct('fscale',fs(n),'x0',x0(n),'m',mm(n), ...
                   'd',[9.81*sind(tx(n)); 9.81*sind(ty(n)); 0],'gbias',gb(n),'seed',n);
        R(n) = mc_run(Pb, c);
    end
else
    for n = 1:N
        c = struct('fscale',fs(n),'x0',x0(n),'m',mm(n), ...
                   'd',[9.81*sind(tx(n)); 9.81*sind(ty(n)); 0],'gbias',gb(n),'seed',n);
        R(n) = mc_run(Pb, c);
    end
end
t_wall = toc;

T = struct2table(R);
T.fscale = fs;  T.x0_mm = 1e3*x0;  T.mass = mm;  T.tilt_x = tx;  T.tilt_y = ty;  T.gbias = gb;
T.pass = T.contact & abs(T.v_end - P.traj.vf) <= P.mc.v_tol;
save('mc_results.mat', 'T', 'P');

fprintf('=== Monte Carlo: %d runs in %.0f s, start gap %.0f mm, force %.2f..%.2fx, tilt %.4f deg ===\n', ...
        N, t_wall, 1e3*P.traj.x0, lo, hi, P.mc.tilt_deg);
fprintf('PASS (contact, arrival %.0f +- %.0f mm/s): %.1f%%\n', 1e3*P.traj.vf, 1e3*P.mc.v_tol, 100*mean(T.pass));
fprintf('  no contact:        %4.1f%%\n', 100*mean(~T.contact));
fprintf('  contact, too fast: %4.1f%%\n', 100*mean(T.contact & T.v_end < P.traj.vf - P.mc.v_tol));
fprintf('  contact, too slow: %4.1f%%\n', 100*mean(T.contact & T.v_end > P.traj.vf + P.mc.v_tol));

edges = exp(linspace(log(lo), log(hi), 7));
fprintf('\n%14s %6s %8s %12s %10s\n','force scale','n','pass','v_end med','sat med');
for k = 1:numel(edges)-1
    sel = T.fscale >= edges(k) & T.fscale < edges(k+1);
    fprintf('  %4.2f .. %4.2f %6d %7.0f%% %9.1f mm/s %8.0f%%\n', edges(k), edges(k+1), sum(sel), ...
            100*mean(T.pass(sel)), 1e3*median(T.v_end(sel & T.contact)), 100*median(T.sat(sel)));
end
xe = [2 4 6 8];
fprintf('\n%14s %6s %8s\n','softening','n','pass');
for k = 1:numel(xe)-1
    sel = T.x0_mm >= xe(k) & T.x0_mm < xe(k+1);
    fprintf('  %2.0f .. %2.0f mm %8d %7.0f%%\n', xe(k), xe(k+1), sum(sel), 100*mean(T.pass(sel)));
end

% force band with >= 90 % pass rate
fgrid = exp(linspace(log(lo), log(hi), 60));
bw = 0.25*log(hi/lo)/log(10);                   % sliding-window log-width, scaled to the range
pr = arrayfun(@(f) mean(T.pass(abs(log(T.fscale) - log(f)) < bw)), fgrid);
ok = pr >= 0.9;
if any(ok)
    fprintf('\nforce band with >= 90%% pass rate: %.2fx .. %.2fx the model\n', ...
            min(fgrid(ok)), max(fgrid(ok)));
else
    fprintf('\nno force band reaches a 90%% pass rate\n');
end
