function z = asteria_meas(P, s)
% ASTERIA_MEAS  Sensor model. s = state vector [x y th vx vy om (dx dy dth bg)].
%   Rows, in order:
%     numel(P.tof.b)  face-looking ToF ranges to the target plane x = 0. The sensors sit
%                     P.tof.recess behind the chaser interface plane, which is what keeps the
%                     40 mm dead zone behind the contact plane.
%     1 (optional)    lateral sensor measuring y directly            (P.lat.have)
%     1 (optional)    rate gyro measuring om + gyro bias             (P.gyro.have)
%   The ToF pair sees yaw only through the difference of two noisy ranges over a short
%   baseline, so the gyro is what actually makes the yaw channel usable. Its bias is a filter
%   state (s(10)); absolute yaw from the ToF pair is what makes that bias observable.
cth = cos(s(3));  sth = sin(s(3));
n   = numel(P.tof.b);
z   = zeros(n + P.lat.have + P.gyro.have, 1);
for k = 1:n
    p    = [s(1); s(2)] + [cth -sth; sth cth]*[P.tof.recess; P.tof.b(k)];
    z(k) = p(1)/cth;                       % slant range to the plane x = 0
end
r = n;
if P.lat.have,  r = r+1;  z(r) = s(2);  end
if P.gyro.have
    r = r+1;
    if numel(s) >= 10, z(r) = s(6) + s(10); else, z(r) = s(6); end
end
end
