function [i_cmd, i_app] = cg_ctrl(xh, t, pc) %#codegen
% CG_CTRL  Code-generation version of asteria_ctrl: scheduled LQR + feedforward + trim.
tt    = min(t, pc.ref_T);
x_ref = interp1(pc.ref_t, pc.ref_x, tt, 'linear', 'extrap');
v_ref = interp1(pc.ref_t, pc.ref_v, tt, 'linear', 'extrap');
i_ff  = interp1(pc.ref_t, pc.ref_i, tt, 'linear', 0);
% Terminal phase: path-following at the estimated gap (see asteria_ctrl).
if xh(1) < pc.path_gate
    xg    = min(max(xh(1), pc.ref_xp(1)), pc.ref_xp(end));
    v_ref = interp1(pc.ref_xp, pc.ref_vp, xg, 'linear');
    i_ff  = interp1(pc.ref_xp, pc.ref_ip, xg, 'linear');
    x_ref = xh(1);
end
xs = pc.sched_x;
xq = min(max(xh(1), xs(1)), xs(end));
K4 = islice(pc.sched_K4, xs, xq);
Al = islice(pc.sched_A,  xs, xq);
e  = [xh(1) - x_ref; xh(3); xh(4) - v_ref; xh(6)];
i_cmd = i_ff*ones(4,1) - K4*e + Al*(-[pc.m*xh(7); pc.J*xh(9)]);
i_app = min(max(i_cmd, -pc.i_sat), pc.i_sat);
end

function M = islice(S, xs, xq)
n = numel(xs);  k = 1;
for j = 1:n
    if xs(j) <= xq, k = j; end
end
if k >= n
    M = S(:,:,n);
else
    w = (xq - xs(k))/(xs(k+1) - xs(k));
    M = (1-w)*S(:,:,k) + w*S(:,:,k+1);
end
end
