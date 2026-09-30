%% test_simulink.m  --  the generated Simulink model: embedded blocks vs the validated extrinsic blocks
% Builds asteria_model twice -- once with the EKF and Controller blocks calling the code-generation
% functions directly (what gets compiled for the target), once with the original extrinsic wrappers
% -- runs both on the same noise seed and requires them to agree.
assert(isfield(P,'sched'), 'run bench_init_3dof, optimize_maneuver, build_schedule first');
P.sim.embedded = false;  evalc('run(''build_simulink_model.m'')');  rng(1);  oX = sim('asteria_model');
P.sim.embedded = true;   evalc('run(''build_simulink_model.m'')');  rng(1);  oE = sim('asteria_model');
n  = min(size(oX.s_log,1), size(oE.s_log,1));
ds = max(max(abs(oX.s_log(1:n,:) - oE.s_log(1:n,:))));
vX = 1e3*oX.s_log(end,4);  vE = 1e3*oE.s_log(end,4);
fprintf('extrinsic: contact %.3f s, %.2f mm/s | embedded: contact %.3f s, %.2f mm/s | max state difference %.1e\n', ...
        oX.tout(end), vX, oE.tout(end), vE, ds);
ok = ds < 1e-6 && oE.s_log(end,1) <= 1e-4 && abs(vE - 1e3*P.traj.vf) <= 10;
if ok, fprintf('PASS\n'); else, fprintf('FAIL\n'); end
