function z = sl_sensors(s, t, noise_on)
% SL_SENSORS  Multirate sensor model with noise and ToF latency. NaN where no new sample.
persistent P
if isempty(P), P = evalin('base','P'); end
% ToF ranges see the pose P.test.tof_latency ago: delay line on the true state
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
