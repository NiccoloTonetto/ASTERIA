function [i_cmd, i_app, e] = asteria_ctrl(xh, t, P)
% ASTERIA_CTRL  Gain-scheduled LQR on the axial and yaw channels with disturbance trim.
%   [i_cmd, i_app, e] = asteria_ctrl(xh, t, P)
%   xh     EKF state [x y th vx vy om dx dy dth bg]
%   t      time along the reference [s]
%   i_cmd  commanded currents; i_app saturated currents (the ones the EKF is fed)
%   e      error [x - x_ref; th; vx - v_ref; om]
%   i = i_ff - K4(x_hat)*e + A_alloc(x_hat)*(-[m*dx_hat; J*dth_hat])
%   Lateral is not controlled (rank-2 input at cant = 0).
x_ref = interp1(P.ref.t, P.ref.x, min(t,P.ref.T), 'linear', 'extrap');
v_ref = interp1(P.ref.t, P.ref.v, min(t,P.ref.T), 'linear', 'extrap');
i_ff  = interp1(P.ref.t, P.ref.i, min(t,P.ref.T), 'linear', 0);

% inside path_gate the reference is indexed by the estimated gap, not by time
% (position error zero by construction; v_ref(x) still gives position feedback)
if isfield(P.ref,'path_gate') && xh(1) < P.ref.path_gate
    xg    = min(max(xh(1), P.ref.xp(1)), P.ref.xp(end));
    v_ref = interp1(P.ref.xp, P.ref.vp, xg, 'linear');
    i_ff  = interp1(P.ref.xp, P.ref.ip, xg, 'linear');
    x_ref = xh(1);
end
xs = P.sched.x;  xq = min(max(xh(1), xs(1)), xs(end));
K4 = interp_slice(P.sched.K4, xs, xq);
Al = interp_slice(P.sched.A,  xs, xq);

e     = [xh(1)-x_ref; xh(3); xh(4)-v_ref; xh(6)];
di    = -K4*e;
dtrim = Al*(-[P.m*xh(7); P.J*xh(9)]);
i_cmd = i_ff*ones(P.n_coil,1) + di + dtrim;
i_app = max(min(i_cmd, P.i_sat), -P.i_sat);
end

function M = interp_slice(S, xs, xq)
k = find(xs <= xq, 1, 'last');
if k >= numel(xs), M = S(:,:,end); return; end
w = (xq - xs(k))/(xs(k+1) - xs(k));
M = (1-w)*S(:,:,k) + w*S(:,:,k+1);
end
