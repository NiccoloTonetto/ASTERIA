function update_lut(csv_file)
% UPDATE_LUT  Calibrate the force model from a measured 1v1 table and rebuild everything.
%   update_lut('my_measurement.csv')
%
% The 4v4 model generates the full 3-D force/torque field from two scalars -- the pole strength
% per amp and the pole softening length. The 1v1 rig measures one coaxial pair, where the model
% is exact in form, so those two scalars are fitted there and the geometry does the rest. There
% is deliberately no tabulated 4v4 LUT: a table would only cover the aligned axis, and the
% controller needs lateral and yaw derivatives too.
%
% CSV FORMAT (header row required, any row order, SI units):
%   gap_m, i_chaser_A, i_target_A, force_N
%   gap_m      face-to-face gap
%   force_N    ATTRACTIVE-positive axial force (what a balance or pendulum reads)
% Include several currents and both signs of i_chaser if you can: a sign change that does not
% flip the force means the rig is not in target-bias mode.
%
% Writes asteria_calibration.mat, which bench_init_3dof picks up automatically, then re-runs
% bench_init_3dof, optimize_maneuver and build_schedule in the base workspace and prints the
% new closed-loop eigenvalues. Delete asteria_calibration.mat to return to the geometry model.

T = readtable(csv_file);
need = {'gap_m','i_chaser_A','i_target_A','force_N'};
assert(all(ismember(need, T.Properties.VariableNames)), ...
       'CSV must have columns: %s', strjoin(need, ', '));
g = T.gap_m;  ic = T.i_chaser_A;  it = T.i_target_A;  Fm = T.force_N;
assert(all(g >= 0) && all(isfinite(Fm)), 'gaps must be >= 0 and forces finite');

Pb = evalin('base','P');                      % geometry-model starting point
L  = Pb.coil.L;
q0 = Pb.coil.q_perA/Pb.cal.scale;             % uncalibrated geometry value
floorF = max(1e-6, 0.02*median(abs(Fm)));     % stops tiny forces dominating the relative fit
res = @(p) (asteria_axial_1v1(g, ic, it, exp(p(1)), exp(p(2)), L) - Fm)./max(abs(Fm), floorF);
cost = @(p) sum(res(p).^2);
p0 = [log(q0), log(Pb.cal.x0)];
opt = optimset('TolX',1e-10,'TolFun',1e-12,'MaxFunEvals',2e4,'MaxIter',1e4,'Display','off');
p  = fminsearch(cost, p0, opt);
q  = exp(p(1));  x0 = exp(p(2));
r  = res(p);
[rmax, imax] = max(abs(r));

cal = struct();
cal.scale  = q/q0;
cal.x0     = x0;
cal.q_perA = q;
cal.rms_rel = sqrt(mean(r.^2));
cal.max_rel = rmax;
cal.n_pts  = numel(g);
cal.source = sprintf('%s, fitted %s', csv_file, char(datetime('now','Format','yyyy-MM-dd HH:mm')));
save('asteria_calibration.mat', 'cal');

fprintf('=== update_lut: %s (%d points) ===\n', csv_file, cal.n_pts);
fprintf('pole strength q = %.3f A*m/A  (geometry model %.3f -> scale %.3f)\n', q, q0, cal.scale);
fprintf('softening    x0 = %.2f mm     (geometry model %.2f mm)\n', 1e3*x0, 1e3*Pb.cal.x0);
fprintf('fit residual: RMS %.1f%%, worst %.1f%% at gap %.1f mm, i_c %.3f A\n', ...
        100*cal.rms_rel, 100*rmax, 1e3*g(imax), ic(imax));
if cal.rms_rel > 0.15
    warning(['RMS misfit above 15%%: the pole model does not describe this rig well. ' ...
             'Check the gap zero, the bias current, and for iron saturation near contact ' ...
             'before trusting the rebuilt gains.']);
end
sgn = sign(ic.*it);  flip = sign(Fm);
if any(sgn < 0) && ~any(flip < 0)
    warning('Negative chaser current never produced repulsion: the rig is NOT in target-bias mode.');
end

evalin('base', ['run(''bench_init_3dof.m''); ', ...
                'evalc(''run(''''optimize_maneuver.m'''')''); ', ...
                'evalc(''run(''''build_schedule.m'''')'');']);
P = evalin('base','P');
fprintf('\nrebuilt with the calibrated model:\n');
fprintf('  design-point CL eigenvalues: %s\n', mat2str(P.cl_eig.', 4));
fprintf('  reference: T = %.1f s, arrival %.1f mm/s, peak %.0f%% of i_max\n', ...
        P.ref.T, 1e3*P.ref.v(end), 100*max(abs(P.ref.i))/P.i_max);
fprintf('  schedule rebuilt at %d gaps. Rebuild the Simulink model to pick it up.\n', numel(P.sched.x));
end
