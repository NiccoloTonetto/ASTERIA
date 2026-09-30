function R = replay_ekf(logfile, P, verbose)
% REPLAY_EKF  Re-run the EKF over a recorded log and report whether it is consistent.
%   R = replay_ekf('log.csv', P)
%   Runs asteria_ekf on the logged sensors and MEASURED currents, then applies the three
%   acceptance tests that decide whether a discrepancy is a tuning problem or a model problem:
%     1. innovation mean    -> a bias means a calibration or force error, not noise
%     2. NIS                -> the size of the innovations against the filter's own prediction
%     3. whiteness          -> structure left in the innovations means an unmodelled effect
%   With truth_* columns present it also reports the true error, which innovations alone
%   cannot give. Nothing here is tuned: it only says what is wrong.
if nargin < 3, verbose = true; end
clear asteria_ekf                      % no Jacobian carried over from another run
T = readtable(logfile);
dt = P.Ts;  n_tof = numel(P.tof.b);  nz = n_tof + P.lat.have + P.gyro.have;
N  = height(T);
xh = [T.tof1_m(find(T.tof_valid,1)) - P.tof.recess; zeros(9,1)];
Pk = P.ekf.P0;
innov = nan(N, nz);  S = nan(N, nz);  est = nan(N, 10);
has_truth = ismember('truth_x', T.Properties.VariableNames);
for k = 1:N
    z = nan(nz,1);
    if T.tof_valid(k)
        z(1:n_tof) = [T.tof1_m(k); T.tof2_m(k)];
        if P.lat.have, z(n_tof+1) = T.lat_m(k); end
    end
    if T.imu_valid(k) && P.gyro.have, z(end) = T.gyro_z_rads(k); end
    i_meas = [T.i1_A(k); T.i2_A(k); T.i3_A(k); T.i4_A(k)];
    [xh, Pk, info] = asteria_ekf(xh, Pk, i_meas, z, P, dt, mod(k,20)==0);
    est(k,:) = xh.';
    if ~isempty(info.innov)
        innov(k, info.mask) = info.innov.';
        S(k, info.mask)     = diag(info.S).';
    end
end

names = [compose("tof%d", 1:n_tof), "lat", "gyro"];
R = struct('names', {names}, 'innov', innov, 'S', S, 'est', est, 'pass', true);
if verbose, fprintf('=== replay: %s (%d samples, %.1f s) ===\n', logfile, N, N*dt); end
for j = 1:nz
    v = innov(~isnan(innov(:,j)), j);  sj = S(~isnan(S(:,j)), j);
    n = numel(v);
    mu    = mean(v);              sd_mu = sqrt(mean(sj)/n);     % expected spread of the mean
    nis   = mean(v.^2./sj);       nis_tol = 2*sqrt(2/n);
    ac    = xcorr(v - mu, 20, 'coeff');  ac = ac(22:end);       % lags 1..20
    % the bound is on the MAX over 20 lags, so it needs a multiple-comparison
    % correction; 2/sqrt(n) is the single-lag bound and would flag noise as structure
    ac_tol = norminv(1 - 0.025/20)/sqrt(n);
    ok_mu = abs(mu) < 3*sd_mu;  ok_nis = abs(nis-1) < nis_tol;  ok_ac = max(abs(ac)) < ac_tol;
    R.stats(j) = struct('name',names(j),'n',n,'mean',mu,'mean_tol',3*sd_mu, ...
                        'nis',nis,'nis_tol',nis_tol,'ac_max',max(abs(ac)),'ac_tol',ac_tol, ...
                        'ok_mean',ok_mu,'ok_nis',ok_nis,'ok_white',ok_ac);
    R.pass = R.pass && ok_mu && ok_nis && ok_ac;
    if verbose
        fprintf('%-5s n=%5d | mean %+9.2e (tol %.1e) %s | NIS %6.2f (1 +- %.2f) %s | |acf| %.3f (tol %.3f) %s\n', ...
            names(j), n, mu, 3*sd_mu, tick(ok_mu), nis, nis_tol, tick(ok_nis), ...
            max(abs(ac)), ac_tol, tick(ok_ac));
    end
end
if has_truth
    e = est(:,1:3) - [T.truth_x, T.truth_y, T.truth_th];
    R.rms_truth = sqrt(mean(e.^2, 1));
    if verbose
        fprintf('vs truth: RMS x %.3f mm, y %.3f mm, th %.4f deg\n', ...
                1e3*R.rms_truth(1), 1e3*R.rms_truth(2), rad2deg(R.rms_truth(3)));
    end
end
if verbose
    if R.pass, vd = 'consistent'; else, vd = 'NOT consistent'; end
    fprintf('VERDICT: %s\n', vd);
end
end

function s = tick(ok), if ok, s = 'ok  '; else, s = 'FAIL'; end, end
