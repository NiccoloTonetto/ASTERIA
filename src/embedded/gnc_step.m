function [xh, Pk, Kp, k, i_cmd] = gnc_step(xh, Pk, Kp, k, i_meas, z, t, pc) %#codegen
% GNC_STEP  One 1 kHz tick of the embedded GNC -- the single entry point the firmware calls.
%   i_meas  the four chaser coil currents measured THIS tick (INA240): that is the current applied
%           over the last step, which is what the EKF must be fed.
%   z       [tof1; tof2; lateral; gyro], NaN in any channel without a fresh sample this tick
%   t       time since the approach started [s]
%   xh, Pk, Kp, k  filter state, owned by the caller between ticks (initialise from pc.x_init,
%           pc.P0, zeros(3), 0)
%   i_cmd   commanded coil currents; saturate in the driver as well as here
k = k + 1;
[xh, Pk, Kp] = cg_ekf_step(xh, Pk, Kp, i_meas, z, pc, (k == 1) || (mod(k,20) == 0));
[i_cmd, ~]   = cg_ctrl(xh, t, pc);
end
