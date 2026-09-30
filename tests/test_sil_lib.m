%% test_sil_lib.m  --  SIL on the generated LIBRARY, not a MEX wrapper
% gnc_step_sil executes the exact C that build_target generated, compiled for the host and driven
% through Embedded Coder's SIL harness. Compared tick by tick against gnc_step in MATLAB on the
% same log. Run after build_target.
assert(exist('gnc_step_sil','file') > 0, 'run build_target first');
logfile = 'v5_1.csv';  T = readtable(logfile);  N = height(T);
pc = make_cg_params(P);
xa = pc.x_init;  Pa = pc.P0;  Ka = zeros(3);  ka = 0;      % MATLAB
xb = pc.x_init;  Pb = pc.P0;  Kb = zeros(3);  kb = 0;      % generated C, host SIL
dx = zeros(N,1);  di = zeros(N,1);  n_path = 0;
for k = 1:N
    z = nan(4,1);
    if T.tof_valid(k), z(1:3) = [T.tof1_m(k); T.tof2_m(k); T.lat_m(k)]; end
    if T.imu_valid(k), z(4) = T.gyro_z_rads(k); end
    ii = [T.i1_A(k); T.i2_A(k); T.i3_A(k); T.i4_A(k)];
    [xa, Pa, Ka, ka, ia] = gnc_step(xa, Pa, Ka, ka, ii, z, T.t_s(k), pc);
    [xb, Pb, Kb, kb, ib] = gnc_step_sil(xb, Pb, Kb, kb, ii, z, T.t_s(k), pc);
    dx(k) = max(abs(xa - xb));  di(k) = max(abs(ia - ib));
    n_path = n_path + (xa(1) < pc.path_gate);
end
clear gnc_step_sil                                            % shuts the SIL process down
fprintf('=== SIL on the generated library, %s (%d ticks, %d in the path-following phase) ===\n', ...
        logfile, N, n_path);
fprintf('max |state| %.2e   max |current| %.2e A\n', max(dx), max(di));
if max(dx) < 1e-9 && max(di) < 1e-9, fprintf('PASS\n'); else, fprintf('FAIL\n'); end
