function z = cg_meas(pc, s) %#codegen
% CG_MEAS  Code-generation version of asteria_meas: [tof1; tof2; lateral; gyro].
cth = cos(s(3));  sth = sin(s(3));
z = zeros(4,1);
for k = 1:2
    z(k) = (s(1) + cth*pc.tof_recess - sth*pc.tof_b(k))/cth;
end
z(3) = s(2);
z(4) = s(6) + s(10);
end
