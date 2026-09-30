function i_cmd = sl_ctrl(xh, t)
% Simulink wrapper for the gain-scheduled controller. Saturation is a separate block, so what
% this returns is the COMMANDED current; the EKF must be fed the saturated one.
persistent P
if isempty(P), P = evalin('base','P'); end
i_cmd = asteria_ctrl(xh, t, P);
end
