function pc = make_cg_params(P)
% MAKE_CG_PARAMS  Flatten P into the purely numeric, fixed-size struct the embedded code uses.
%   No strings, no nested structs, no optional fields: everything code generation needs to fix
%   at compile time. Regenerate after any change to P (calibration, start gap, gains).
assert(P.n_coil == 4 && numel(P.tof.b) == 2 && P.lat.have && P.gyro.have, ...
       'the embedded code is sized for 4 coils, 2 ToF, a lateral sensor and a gyro');
pc = struct();
pc.m = P.m;  pc.J = P.J;  pc.L = P.coil.L;  pc.x0 = P.cal.x0;
pc.qA = P.coil.q_perA;  pc.i_bias = P.drive.i_bias;
pc.target_bias = double(strcmpi(P.drive.mode, 'target_bias'));
pc.g = cosd(P.layout.cant);  pc.s = sind(P.layout.cant);
pc.pos = P.layout.pos;  pc.u = P.layout.u;
pc.Ts = P.Ts;
pc.tof_b = P.tof.b(:);  pc.tof_recess = P.tof.recess;  pc.tof_latency = P.tof.latency;
pc.Qd = P.ekf.Qd;  pc.Rd = diag(P.ekf.Rd);  pc.x_min = P.ekf.x_min;
pc.i_sat = P.i_sat;
pc.ref_t = P.ref.t(:);  pc.ref_x = P.ref.x(:);  pc.ref_v = P.ref.v(:);  pc.ref_i = P.ref.i(:);
pc.ref_T = P.ref.T;
pc.x_init = [P.traj.x0; zeros(9,1)];  pc.P0 = P.ekf.P0;   % filter initial state for the block
pc.ref_xp = P.ref.xp(:);  pc.ref_vp = P.ref.vp(:);  pc.ref_ip = P.ref.ip(:);  pc.path_gate = P.ref.path_gate;
pc.sched_x = P.sched.x(:);  pc.sched_K4 = P.sched.K4;  pc.sched_A = P.sched.A;
end
