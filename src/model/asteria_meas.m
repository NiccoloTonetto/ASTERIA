function z = asteria_meas(P, s)
% ASTERIA_MEAS  Sensor model.
%   z = asteria_meas(P, s),  s = [x y th vx vy om] or the 10-state [.. dx dy dth bg]
%   Rows of z:
%     numel(P.tof.b)  ToF slant ranges to the target plane x = 0; sensors recessed
%                     P.tof.recess behind the interface plane (minimum range ~40 mm)
%     P.lat.have      lateral offset y
%     P.gyro.have     yaw rate + gyro bias s(10)
%   Absolute yaw from the ToF pair is what makes the gyro bias observable.
cth = cos(s(3));  sth = sin(s(3));
n   = numel(P.tof.b);
z   = zeros(n + P.lat.have + P.gyro.have, 1);
for k = 1:n
    p    = [s(1); s(2)] + [cth -sth; sth cth]*[P.tof.recess; P.tof.b(k)];
    z(k) = p(1)/cth;                       % slant range to x = 0
end
r = n;
if P.lat.have,  r = r+1;  z(r) = s(2);  end
if P.gyro.have
    r = r+1;
    if numel(s) >= 10, z(r) = s(6) + s(10); else, z(r) = s(6); end
end
end
