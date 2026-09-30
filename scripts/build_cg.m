%% build_cg.m  --  compile the embedded EKF and controller to C (MEX for SIL)
% The example inputs fix every size at compile time; the parameter struct is an INPUT, not a
% compiled-in constant, so recalibrating means regenerating pc, not recompiling.
assert(isfield(P,'sched'), 'run build_schedule first');
pc  = make_cg_params(P);
xh  = zeros(10,1);  Pk = zeros(10);  Kp = zeros(3);  ii = zeros(4,1);  z = zeros(4,1);
cfg = coder.config('mex');
cfg.IntegrityChecks = true;  cfg.ResponsivenessChecks = false;
tic; codegen -config cfg cg_ekf_step -args {xh, Pk, Kp, ii, z, pc, true} -o cg_ekf_step_mex; t1 = toc;
tic; codegen -config cfg cg_ctrl     -args {xh, 0, pc}                    -o cg_ctrl_mex;     t2 = toc;
fprintf('generated cg_ekf_step_mex (%.0f s) and cg_ctrl_mex (%.0f s)\n', t1, t2);
