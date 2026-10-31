function F = asteria_axial_1v1(g, ic, it, q, x0, L)
% ASTERIA_AXIAL_1V1  Axial force [N] of one coaxial solenoid pair (1v1 rig), attraction positive.
%   F = asteria_axial_1v1(g, ic, it, q, x0, L)
%   g       face-to-face gap [m], vector allowed
%   ic, it  chaser and target currents [A]
%   q       pole strength per amp [A*m/A]
%   x0      softening length [m]
%   L       coil length [m]
%   Same pole model as asteria_wrench for a single aligned pair; used by update_lut for the fit.
mu0 = 4*pi*1e-7;
k   = @(d) d./(d.^2 + x0^2).^1.5;
Fx  = mu0*(q*ic).*(q*it)/(4*pi) .* ( -k(g) + 2*k(g+L) - k(g+2*L) );
F   = -Fx;
end
