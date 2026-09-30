function ds = asteria_plant(~, s, i, P, d)
% ASTERIA_PLANT  Continuous planar dynamics of the floating chaser.
%   ds = asteria_plant(t, s, i, P, d)
%     s = [x; y; th; vx; vy; om]   pose and rates (see bench_init_3dof.m for conventions)
%     i = 4x1 POST-SATURATION currents [A], one per actuator pair
%     P = parameter struct from bench_init_3dof
%     d = [dx; dy; dth] disturbance accelerations (table tilt, air flow, umbilical)
%   The wrench is a world-frame vector: Fx > 0 pushes the platforms apart.
%   No contact model -- the caller stops the integration at x = 0 (bare face contact).
W  = asteria_wrench(P, s(1:3), i);
ds = [ s(4); s(5); s(6);
       W(1)/P.m + d(1);
       W(2)/P.m + d(2);
       W(3)/P.J + d(3) ];
end
