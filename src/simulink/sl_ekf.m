function xh = sl_ekf(i_app, z, t)
% SL_EKF  Simulink wrapper for asteria_ekf. Filter state is persistent and reset at t = 0;
%   the model's StartFcn also clears it.
persistent P xh_ Pk_ k
if isempty(P), P = evalin('base','P'); end
if t <= 0 || isempty(xh_)
    xh_ = [P.traj.x0; zeros(9,1)];  Pk_ = P.ekf.P0;  k = 0;
end
k = k + 1;
[xh_, Pk_] = asteria_ekf(xh_, Pk_, i_app, z, P, P.Ts, mod(k,20) == 0);
xh = xh_;
end
