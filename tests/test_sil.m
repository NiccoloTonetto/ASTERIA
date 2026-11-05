%% test_sil.m  --  generated code against the MATLAB reference on a recorded log
% Same sensor data and measured currents into three implementations:
%   REF  asteria_ekf / asteria_ctrl       MATLAB reference
%   CG   cg_ekf_step / cg_ctrl            code-generation versions, interpreted
%   MEX  cg_ekf_step_mex / cg_ctrl_mex    the same compiled by MATLAB Coder (built if missing)
% REF vs CG checks the rewrite, CG vs MEX checks the generated C.
% Run after bench_init_3dof, optimize_maneuver, build_schedule; needs the logs from test_replay.
assert(isfield(P,'sched'), 'run build_schedule first');
logfile = 'v5_1.csv';
assert(exist(logfile,'file')==2, 'run test_replay first to create the logs');
pc = make_cg_params(P);
T  = readtable(logfile);  N = height(T);
if exist('cg_ekf_step_mex','file') ~= 3 || exist('cg_ctrl_mex','file') ~= 3
    fprintf('MEX files not found: running build_cg\n');
    try
        build_cg
    catch ME
        fprintf('build_cg failed: %s\n', ME.message);
    end
end
use_mex = exist('cg_ekf_step_mex','file') == 3 && exist('cg_ctrl_mex','file') == 3;
if ~use_mex, warning('No MEX files: comparing REF vs CG only.'); end

clear asteria_ekf
x0 = [T.tof1_m(find(T.tof_valid,1)) - P.tof.recess; zeros(9,1)];
xr = x0;  Pr = P.ekf.P0;                         % reference
xc = x0;  Pc = P.ekf.P0;  Kc = zeros(3);          % generated code, interpreted
xm = x0;  Pm = P.ekf.P0;  Km = zeros(3);          % generated code, compiled
d_rc = zeros(N,2);  d_cm = zeros(N,2);
for k = 1:N
    t = T.t_s(k);
    z = nan(4,1);
    if T.tof_valid(k), z(1:3) = [T.tof1_m(k); T.tof2_m(k); T.lat_m(k)]; end
    if T.imu_valid(k), z(4) = T.gyro_z_rads(k); end
    ii  = [T.i1_A(k); T.i2_A(k); T.i3_A(k); T.i4_A(k)];
    rec = (k == 1) || mod(k,20) == 0;
    [xr, Pr]     = asteria_ekf(xr, Pr, ii, z, P, P.Ts, mod(k,20)==0);
    [ir, ~]      = asteria_ctrl(xr, t, P);
    [xc, Pc, Kc] = cg_ekf_step(xc, Pc, Kc, ii, z, pc, rec);
    [ic, ~]      = cg_ctrl(xc, t, pc);
    d_rc(k,:) = [max(abs(xr - xc)), max(abs(ir - ic))];
    if use_mex
        [xm, Pm, Km] = cg_ekf_step_mex(xm, Pm, Km, ii, z, pc, rec);
        [im, ~]      = cg_ctrl_mex(xm, t, pc);
        d_cm(k,:) = [max(abs(xc - xm)), max(abs(ic - im))];
    end
end
fprintf('=== SIL on %s (%d steps, %.1f s) ===\n', logfile, N, N*P.Ts);
fprintf('REF vs CG : max |state| %.2e   max |current| %.2e A\n', max(d_rc(:,1)), max(d_rc(:,2)));
if use_mex
    fprintf('CG  vs MEX: max |state| %.2e   max |current| %.2e A\n', max(d_cm(:,1)), max(d_cm(:,2)));
end
tol_state = 1e-9;  tol_i = 1e-9;
ok = max(d_rc(:,1)) < tol_state && max(d_rc(:,2)) < tol_i;
if use_mex, ok = ok && max(d_cm(:,1)) < tol_state && max(d_cm(:,2)) < tol_i; end
if ~ok
    fprintf('FAIL: implementations diverge; compare step by step before trusting the C\n');
elseif use_mex
    fprintf('PASS: all implementations agree to below 1e-9\n');
else
    fprintf('REF and CG agree to below 1e-9; compiled code not checked (no MEX)\n');
end
