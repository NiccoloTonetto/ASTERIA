function [xh, Pk, info] = asteria_ekf(xh, Pk, i, z, P, dt, recompute_jac)
% ASTERIA_EKF  One step of the 10-state planar EKF.
%   [xh, Pk, info] = asteria_ekf(xh, Pk, i, z, P, dt, recompute_jac)
%   xh    [x y th vx vy om dx dy dth bg]: pose, rates, disturbance accelerations, gyro bias
%   i     measured (post-saturation) coil currents [A]; the commanded value would let the
%         disturbance states absorb saturation
%   z     measurements as in asteria_meas, NaN where no new sample this step
%   recompute_jac  refresh the cached pose Jacobian (callers use every 20 steps)
%   info  innovations, their covariance and the channel mask, for replay_ekf
%   Kpose is persistent: run "clear asteria_ekf" before each new simulation.
persistent Kpose
Minv = diag([1/P.m, 1/P.m, 1/P.J]);
if isempty(Kpose) || recompute_jac
    Kpose = zeros(3);  hp = 1e-6;
    for k = 1:3
        xp = xh(1:3); xp(k) = xp(k)+hp;  xm = xh(1:3); xm(k) = xm(k)-hp;
        Kpose(:,k) = (asteria_wrench(P,xp,i) - asteria_wrench(P,xm,i))/(2*hp);
    end
end

% ---- predict (disturbances and gyro bias: random walks, no change)
ds = asteria_plant(0, xh(1:6), i, P, xh(7:9));
xh(1:6) = xh(1:6) + ds*dt;
xh(1)   = max(xh(1), P.ekf.x_min);
A = zeros(10);
A(1:3,4:6) = eye(3);
A(4:6,1:3) = Minv*Kpose;
A(4:6,7:9) = eye(3);
F  = eye(10) + A*dt;
Pk = F*Pk*F.' + P.ekf.Qd;

% ---- update on the channels with a fresh sample
info = struct('innov',[],'S',[],'mask',[]);
m = ~isnan(z);
if any(m)
    % ranges are predicted from the pose one sensor latency ago
    tau  = 0;  if isfield(P.tof,'latency'), tau = P.tof.latency; end
    hfun = @(s) asteria_meas(P, [s(1:3) - tau*s(4:6); s(4:6); s(7:10)]);
    h  = hfun(xh);
    H  = zeros(numel(h), 10);  hp = 1e-6;
    for k = [1 2 3 4 5 6 10]              % pose, rates (via the latency shift), gyro bias
        xp = xh; xp(k) = xp(k)+hp;  xm = xh; xm(k) = xm(k)-hp;
        H(:,k) = (hfun(xp) - hfun(xm))/(2*hp);
    end
    H = H(m,:);  Rd = P.ekf.Rd(m,m);
    innov = z(m) - h(m);
    S  = H*Pk*H.' + Rd;
    K  = (Pk*H.')/S;
    info.innov = innov;  info.S = S;  info.mask = m;
    xh = xh + K*innov;
    IKH = eye(10) - K*H;
    Pk = IKH*Pk*IKH.' + K*Rd*K.';             % Joseph form
    xh(1) = max(xh(1), P.ekf.x_min);
end
end
