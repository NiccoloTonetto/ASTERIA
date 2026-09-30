function z = sl_sensors(s, t, noise_on)
% Sensor model with multirate sampling and noise. Returns NaN in any channel that has no new
% sample at this step -- that is the contract asteria_ekf expects.
persistent P
if isempty(P), P = evalin('base','P'); end
% The range sensors see the pose as it was one latency ago, so keep a short history of the
% true state and sample it delayed. Without this the model is optimistic in exactly the
% way the EKF now compensates for, and the two cancel into a false result.
persistent buf
nd = max(round(P.test.tof_latency/P.Ts), 0);
if isempty(buf) || t <= 0, buf = repmat(s(:), 1, nd+1); end
buf = [s(:), buf(:, 1:end-1)];
s_del = buf(:, end);
zt = asteria_meas(P, [s_del; 0; 0; 0; P.gyro.bias]);
z  = nan(size(zt));
n  = numel(P.tof.b);
tol = P.Ts/2;
if abs(mod(t, P.tof.Ts)) < tol || abs(mod(t, P.tof.Ts) - P.tof.Ts) < tol
    z(1:n+P.lat.have) = zt(1:n+P.lat.have);
    if noise_on
        z(1:n) = z(1:n) + P.tof.sigma*randn(n,1);
        if P.lat.have, z(n+1) = z(n+1) + P.lat.sigma*randn; end
    end
end
if P.gyro.have && (abs(mod(t, P.gyro.Ts)) < tol || abs(mod(t, P.gyro.Ts) - P.gyro.Ts) < tol)
    z(end) = zt(end) + noise_on*P.gyro.sigma*randn;
end
end
