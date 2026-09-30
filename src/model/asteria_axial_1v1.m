function F = asteria_axial_1v1(g, ic, it, q, x0, L)
% ASTERIA_AXIAL_1V1  Axial ATTRACTIVE force [N] of ONE coaxial solenoid pair (the 1v1 rig).
%   g  = face-to-face gap [m] (vector ok), ic / it = chaser / target currents [A],
%   q  = pole strength per amp [A*m/A], x0 = softening length [m], L = coil length [m].
%   Same softened Gilbert pole model as asteria_wrench, specialised to a single aligned pair so it
%   can be fitted directly against a force-vs-gap table. Positive = pulling together.
mu0 = 4*pi*1e-7;
k   = @(d) d./(d.^2 + x0^2).^1.5;
Fx  = mu0*(q*ic).*(q*it)/(4*pi) .* ( -k(g) + 2*k(g+L) - k(g+2*L) );
F   = -Fx;
end
