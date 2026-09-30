function [xh, Pk, Kp] = cg_ekf_step(xh, Pk, Kp, i, z, pc, recompute) %#codegen
% CG_EKF_STEP  Embedded EKF step, code-generation compatible.
%   All state is explicit (xh, Pk and the cached pose Jacobian Kp in and out): nothing hidden in
%   persistent variables, so the step is a pure function and can be tested against a log.
%   Measurements are applied as SCALAR updates, one channel at a time, all linearised at the
%   prior. With a diagonal R this is algebraically identical to asteria_ekf's batch update, but
%   needs no matrix inverse and no variable-size arrays. z uses NaN for "no new sample".
Minv = diag([1/pc.m, 1/pc.m, 1/pc.J]);
hp = 1e-6;
if recompute
    for k = 1:3
        xp = xh(1:3);  xp(k) = xp(k) + hp;  xm = xh(1:3);  xm(k) = xm(k) - hp;
        Kp(:,k) = (cg_wrench(pc, xp, i) - cg_wrench(pc, xm, i))/(2*hp);
    end
end
% ---- predict
W  = cg_wrench(pc, xh(1:3), i);
ds = [xh(4:6); W(1)/pc.m + xh(7); W(2)/pc.m + xh(8); W(3)/pc.J + xh(9)];
xh(1:6) = xh(1:6) + ds*pc.Ts;
xh(1) = max(xh(1), pc.x_min);
A = zeros(10);
A(1:3,4:6) = eye(3);  A(4:6,1:3) = Minv*Kp;  A(4:6,7:9) = eye(3);
F  = eye(10) + A*pc.Ts;
Pk = F*Pk*F.' + pc.Qd;
% ---- update: sequential scalar channels, one linearisation
if any(~isnan(z))
    x0 = xh;
    h0 = hdel(pc, x0);
    H  = zeros(4,10);
    for k = [1 2 3 4 5 6 10]
        xp = x0;  xp(k) = xp(k) + hp;  xm = x0;  xm(k) = xm(k) - hp;
        H(:,k) = (hdel(pc, xp) - hdel(pc, xm))/(2*hp);
    end
    for j = 1:4
        if ~isnan(z(j))
            Hj  = H(j,:);
            nu  = z(j) - h0(j) - Hj*(xh - x0);     % innovation against the SAME linearisation
            Sj  = Hj*Pk*Hj.' + pc.Rd(j);
            Kj  = (Pk*Hj.')/Sj;
            xh  = xh + Kj*nu;
            IKH = eye(10) - Kj*Hj;
            Pk  = IKH*Pk*IKH.' + (Kj*pc.Rd(j))*Kj.';   % Joseph form
        end
    end
    xh(1) = max(xh(1), pc.x_min);
end
end

function h = hdel(pc, s)
% measurement predicted from the pose one sensor latency ago
tau = pc.tof_latency;
sd  = [s(1:3) - tau*s(4:6); s(4:10)];
h   = cg_meas(pc, sd);
end
