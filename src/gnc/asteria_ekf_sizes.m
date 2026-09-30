function P = asteria_ekf_sizes(P)
% ASTERIA_EKF_SIZES  Build the EKF dimensions and covariances from the sensor configuration.
% Separate from bench_init so a sensor flag can be flipped and the filter rebuilt in one call.
% State: [x y th vx vy om dx dy dth bg] -- 10, with the gyro bias last.
n_tof = numel(P.tof.b);
P.ekf.n  = 10;
P.ekf.x0 = [P.traj.x0; zeros(9,1)];
P.ekf.P0 = diag([P.tof.sigma^2, P.lat.sigma^2, deg2rad(2)^2, ...
                 (5e-3)^2, (5e-3)^2, deg2rad(2)^2, ...
                 (1e-3)^2, (1e-3)^2, (1e-4)^2, (2*P.gyro.sig_b0)^2]);
P.ekf.Qd = diag([0, 0, 0, ...
                 (P.ekf.sig_a*P.Ts)^2, (P.ekf.sig_a*P.Ts)^2, (P.ekf.sig_al*P.Ts)^2, ...
                 (P.ekf.sig_d*P.Ts)^2*[1 1 1], (P.gyro.sig_bw*P.Ts)^2]);
r = P.tof.sigma^2*ones(1,n_tof);
if P.lat.have,  r = [r, P.lat.sigma^2];   end
if P.gyro.have, r = [r, P.gyro.sigma^2];  end
P.ekf.Rd = diag(r);
end
