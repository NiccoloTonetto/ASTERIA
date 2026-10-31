function ds = asteria_plant(~, s, i, P, d)
% ASTERIA_PLANT  Planar rigid-body dynamics of the floating chaser.
%   ds = asteria_plant(t, s, i, P, d)
%   s   [x; y; th; vx; vy; om], conventions in bench_init_3dof
%   i   applied (post-saturation) chaser currents [A], 4 x 1
%   d   disturbance accelerations [dx; dy; dth] (table tilt, air flow, umbilical)
%   No contact model: the caller stops the integration at x = 0.
W  = asteria_wrench(P, s(1:3), i);
ds = [ s(4); s(5); s(6);
       W(1)/P.m + d(1);
       W(2)/P.m + d(2);
       W(3)/P.J + d(3) ];
end
