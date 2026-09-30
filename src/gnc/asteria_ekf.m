function [xh, Pk, info] = asteria_ekf(xh, Pk, i, z, P, dt, recompute_jac)
% ASTERIA_EKF  One step of the 10-state planar EKF.
%   xh = [x; y; th; vx; vy; om; dx; dy; dth; bg]
%        pose, rates, three disturbance accelerations (random walk), gyro bias (random walk).
%   i  = POST-SATURATION currents actually sent to the coils. Feeding the commanded value
%        instead is what makes the disturbance states absorb the saturation and drift.
%   z  = measurement vector from asteria_meas, with NaN in any channel that has no new
%        sample this step (the ToF runs ~30x slower than the loop, the gyro ~4x).
%   recompute_jac = refresh the pose Jacobian. It varies on the timescale of the approach,
%        not of the 1 ms step, so every ~20 steps is plenty.
% Only the pose Jacobian is cached (refreshed every ~20 steps). The mass matrix is rebuilt every
% call: caching it meant a changed P.m was ignored until MATLAB restarted.
persistent Kpose
Minv = diag([1/P.m, 1/P.m, 1/P.J]);
if isempty(Kpose) || recompute_jac
    Kpose = zeros(3);  hp = 1e-6;
    for k = 1:3
        xp = xh(1:3); xp(k) = xp(k)+hp;  xm = xh(1:3); xm(k) = xm(k)-hp;
        Kpose(:,k) = (asteria_wrench(P,xp,i) - asteria_wrench(P,xm,i))/(2*hp);
    end
end

% ---- predict (disturbances and gyro bias are constant between updates)
ds = asteria_plant(0, xh(1:6), i, P, xh(7:9));
xh(1:6) = xh(1:6) + ds*dt;
xh(1)   = max(xh(1), P.ekf.x_min);
A = zeros(10);
A(1:3,4:6) = eye(3);
A(4:6,1:3) = Minv*Kpose;
A(4:6,7:9) = eye(3);
F  = eye(10) + A*dt;
Pk = F*Pk*F.' + P.ekf.Qd;

% ---- update, only on the channels that have a fresh sample
info = struct('innov',[],'S',[],'mask',[]);
m = ~isnan(z);
if any(m)
    % The range sensors report the pose as it was one latency ago, so the predicted
    % measurement is evaluated on the back-propagated pose. Without this the innovations
    % come out correlated with velocity and look like process noise.
    tau  = 0;  if isfield(P.tof,'latency'), tau = P.tof.latency; end
    hfun = @(s) asteria_meas(P, [s(1:3) - tau*s(4:6); s(4:6); s(7:10)]);
    h  = hfun(xh);
    H  = zeros(numel(h), 10);  hp = 1e-6;
    for k = [1 2 3 4 5 6 10]              % pose, rates (through the delay), gyro bias
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
