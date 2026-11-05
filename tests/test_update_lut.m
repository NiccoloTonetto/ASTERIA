%% test_update_lut.m  --  update_lut recovers known calibration parameters
% Synthetic 1v1 data from known q and x0 (different from the model's) with 3% noise; checks
% the fit, then deletes asteria_calibration.mat and restores the geometry model.
assert(exist('P','var')==1, 'run bench_init_3dof first');
cal_file = 'asteria_calibration.mat';
assert(~exist(cal_file,'file'), 'a real calibration file exists -- move it before running this test');

q_true  = 1.30*P.coil.q_perA/P.cal.scale;     % 30% stronger than the geometry model
x0_true = 5.0e-3;
it      = P.drive.i_bias;
[G, IC] = ndgrid([2 4 6 8 10 15 20 30 40 60 80 100]*1e-3, [-0.3 -0.15 0.15 0.3]);
rng(7);
F = asteria_axial_1v1(G(:), IC(:), it, q_true, x0_true, P.coil.L);
F = F.*(1 + 0.03*randn(size(F)));             % 3% measurement noise
csv = 'fake_1v1.csv';
writetable(table(G(:), IC(:), it*ones(numel(G),1), F, ...
    'VariableNames', {'gap_m','i_chaser_A','i_target_A','force_N'}), csv);

update_lut(csv);
C = load(cal_file);
e_q  = C.cal.q_perA/q_true - 1;
e_x0 = C.cal.x0/x0_true - 1;
fprintf('\nrecovered q  %.3f vs true %.3f  (%+.1f%%)\n', C.cal.q_perA, q_true, 100*e_q);
fprintf('recovered x0 %.2f vs true %.2f mm (%+.1f%%)\n', 1e3*C.cal.x0, 1e3*x0_true, 100*e_x0);
if abs(e_q) < 0.03 && abs(e_x0) < 0.15, fprintf(' ok   calibration recovered\n');
else, fprintf('FAIL  calibration not recovered\n'); end

delete(cal_file);  delete(csv);
run('bench_init_3dof.m');                     % back to the uncalibrated geometry model
evalc('run(''optimize_maneuver.m'')');  evalc('run(''build_schedule.m'')');
fprintf('\ncleaned up: calibration file removed, workspace rebuilt from the geometry model\n');
