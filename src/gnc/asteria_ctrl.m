function [i_cmd, i_app, e] = asteria_ctrl(xh, t, P)
% ASTERIA_CTRL  Gain-scheduled LQR on the axial and yaw channels, plus disturbance trim.
%   xh    = EKF state [x y th vx vy om dx dy dth bg]
%   t     = time along the reference
%   i_cmd = commanded currents, i_app = after saturation (THIS is what the EKF must be fed).
%   The lateral channel is not commandable with cant = 0 and is deliberately left alone.
%
%   i = i_ff(t)            feedforward from the optimised reference
%     - K4(x_hat)*e        gain scheduled on the estimated gap
%     + alloc(x_hat)*(-[m*dx_hat; J*dth_hat])   disturbance trim, min-norm over the rank-2 image
x_ref = interp1(P.ref.t, P.ref.x, min(t,P.ref.T), 'linear', 'extrap');
v_ref = interp1(P.ref.t, P.ref.v, min(t,P.ref.T), 'linear', 'extrap');
i_ff  = interp1(P.ref.t, P.ref.i, min(t,P.ref.T), 'linear', 0);

% Terminal phase: path-following. Reference taken at the ESTIMATED GAP, so timing errors no
% longer turn into arrival-velocity errors. The position error is zero by construction; the slope
% of v_ref(x) still supplies position feedback through the velocity gain.
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
