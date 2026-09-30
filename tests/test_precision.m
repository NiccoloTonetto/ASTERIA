%% test_precision.m  --  V6: does the filter survive single-precision storage?
% The covariance spans roughly seven orders of magnitude (position variance ~1e-10 m^2 against
% disturbance terms ~1e-3) and single precision keeps about seven significant digits. This
% replays a log twice, once keeping the state and covariance in double and once rounding both
% to single after every step, and compares. It tests STORAGE precision, not single-precision
% arithmetic: it will reveal a dynamic-range problem but not accumulated rounding inside the
% arithmetic, which only a genuine single-precision build or SIL run can show.
assert(isfield(P,'ref'), 'run bench_init_3dof, optimize_maneuver, build_schedule first');
logfile = 'v5_1.csv';
assert(exist(logfile,'file')==2, 'run test_replay first to create the logs');
T = readtable(logfile);
n_tof = numel(P.tof.b);  nz = n_tof + P.lat.have + P.gyro.have;  N = height(T);

est = cell(1,2);
for mode = 1:2                                   % 1 = double, 2 = single storage
    clear asteria_ekf
    xh = [T.tof1_m(find(T.tof_valid,1)) - P.tof.recess; zeros(9,1)];
    Pk = P.ekf.P0;  E = nan(N,10);
    for k = 1:N
        z = nan(nz,1);
        if T.tof_valid(k)
            z(1:n_tof) = [T.tof1_m(k); T.tof2_m(k)];
            if P.lat.have, z(n_tof+1) = T.lat_m(k); end
        end
        if T.imu_valid(k) && P.gyro.have, z(end) = T.gyro_z_rads(k); end
        i_meas = [T.i1_A(k); T.i2_A(k); T.i3_A(k); T.i4_A(k)];
        [xh, Pk] = asteria_ekf(xh, Pk, i_meas, z, P, P.Ts, mod(k,20)==0);
        if mode == 2
            xh = double(single(xh));  Pk = double(single(Pk));
        end
        E(k,:) = xh.';
    end
    est{mode} = E;
end

d  = est{1} - est{2};
e1 = est{1}(:,1) - T.truth_x;                    % double-precision estimation error
fprintf('=== V6: single-precision storage, %s ===\n', logfile);
fprintf('max |difference| : x %.2e m, v %.2e m/s, dist %.2e m/s^2\n', ...
        max(abs(d(:,1))), max(abs(d(:,4))), max(abs(d(:,7))));
fprintf('RMS difference   : x %.3f um   vs estimation error %.3f mm\n', ...
        1e6*sqrt(mean(d(:,1).^2)), 1e3*sqrt(mean(e1.^2)));
ratio = sqrt(mean(d(:,1).^2))/sqrt(mean(e1.^2));
if ratio < 0.05
    fprintf('ratio %.2e -> ok: single-precision storage is safe\n', ratio);
else
    fprintf('ratio %.2e -> FAIL: scale the embedded states to millimetres\n', ratio);
end