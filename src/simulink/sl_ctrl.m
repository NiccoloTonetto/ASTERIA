function i_cmd = sl_ctrl(xh, t)
% SL_CTRL  Simulink wrapper for asteria_ctrl. Returns the commanded current; saturation is a
%   separate block and the EKF is fed its output.
persistent P
if isempty(P), P = evalin('base','P'); end
i_cmd = asteria_ctrl(xh, t, P);
end
