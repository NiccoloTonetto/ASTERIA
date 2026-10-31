function [xh, Pk, Kp, k, i_cmd] = gnc_step(xh, Pk, Kp, k, i_meas, z, t, pc) %#codegen
% GNC_STEP  One 1 kHz tick of the embedded GNC; the firmware entry point (ICD 5.1).
%   i_meas         chaser coil currents measured this tick [A] (INA240)
%   z              [tof1; tof2; lateral; gyro], NaN where no fresh sample
%   t              time since the approach started [s]
%   xh, Pk, Kp, k  filter state kept by the caller; initialise to pc.x_init, pc.P0, zeros(3), 0
%   pc             constant parameters from make_cg_params
%   i_cmd          commanded coil currents [A], also limited in the drivers
k = k + 1;
[xh, Pk, Kp] = cg_ekf_step(xh, Pk, Kp, i_meas, z, pc, (k == 1) || (mod(k,20) == 0));
[i_cmd, ~]   = cg_ctrl(xh, t, pc);
end
