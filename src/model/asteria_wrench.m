function W = asteria_wrench(P, xi, i)
% ASTERIA_WRENCH  Planar wrench [Fx; Fy; Mz] on the chaser, world frame (Gilbert pole model).
%   Positive Fx pushes the platforms APART. Mz is about the chaser interface centre.
%   xi = [x; y; th], i = n_coil x 1 chaser-side currents [A].
%
%   Drive modes (P.drive.mode):
%     'target_bias'  target coils held at +P.drive.i_bias, chaser coils carry i.
%                    Charges have independent signs -> W is LINEAR in i and REVERSIBLE:
%                    i < 0 repels. This is the bench configuration.
%     'symmetric'    both coils of a pair carry i. The facing faces then always present
%                    opposite poles, so W ~ i^2 and every pair is ATTRACT-ONLY, whatever
%                    the sign of i. Kept only to reproduce the earlier model.
mu0 = 4*pi*1e-7;
n   = P.n_coil;  L = P.coil.L;  x0 = P.cal.x0;
i   = i(:);
if isfield(P,'drive') && strcmpi(P.drive.mode,'symmetric')
    it = i;                                   % target follows the chaser
else
    it = P.drive.i_bias*ones(n,1);            % target held at a fixed bias
end
qT = P.coil.q_perA*it;
qC = P.coil.q_perA*i;

g = cos(deg2rad(P.layout.cant));  s = sin(deg2rad(P.layout.cant));
pos = P.layout.pos;  u = P.layout.u;

% target poles: front (+q) on the plane x = 0, back (-q) into the target
T  = zeros(2*n,3);  QT = zeros(2*n,1);
T(1:2:end,:) = [zeros(n,1), pos];
T(2:2:end,:) = [-L*g*ones(n,1), pos + L*s*u];
QT(1:2:end)  = +qT;  QT(2:2:end) = -qT;

% chaser poles: front (-q) on its interface plane, back (+q) into the chaser body,
% yawed by th about the chaser interface centre and translated by [x, y].
cth = cos(xi(3));  sth = sin(xi(3));
Rz  = [cth -sth; sth cth];
fr_l = [zeros(n,1), pos(:,1)];                    % local (x,y) of the front poles
bk_l = [L*g*ones(n,1), pos(:,1) - L*s*u(:,1)];    % local (x,y) of the back poles
fr_w = fr_l*Rz.' + [xi(1), xi(2)];
bk_w = bk_l*Rz.' + [xi(1), xi(2)];
C  = zeros(2*n,3);  QC = zeros(2*n,1);
C(1:2:end,:) = [fr_w, pos(:,2)];
C(2:2:end,:) = [bk_w, pos(:,2) - L*s*u(:,2)];
QC(1:2:end)  = -qC;  QC(2:2:end) = +qC;

% all chaser-pole / target-pole interactions at once
rv  = reshape(C,[2*n 1 3]) - reshape(T,[1 2*n 3]);        % (a,b,xyz)
d2  = sum(rv.^2, 3) + x0^2;
cf  = mu0*(QC*QT.')./(4*pi*d2.^1.5);                      % scalar coefficient per pole pair
fa  = squeeze(sum(cf.*rv, 2));                            % force on each chaser pole (2n x 3)
F   = sum(fa, 1);
rr  = C(:,1:2) - [xi(1), xi(2)];                          % arms from the chaser interface centre
Mz  = sum(rr(:,1).*fa(:,2) - rr(:,2).*fa(:,1));
W   = [F(1); F(2); Mz];
end
