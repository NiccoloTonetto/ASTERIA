function W = cg_wrench(pc, xi, i) %#codegen
% CG_WRENCH  Code-generation version of asteria_wrench: same pole model, plain scalar loops,
%   fixed sizes, no implicit expansion. Must agree with asteria_wrench to rounding.
mu0 = 4*pi*1e-7;  n = 4;  L = pc.L;  g = pc.g;  s = pc.s;
qC = pc.qA*i(:);
if pc.target_bias > 0, qT = pc.qA*pc.i_bias*ones(n,1); else, qT = qC; end
T = zeros(2*n,3);  QT = zeros(2*n,1);  C = zeros(2*n,3);  QC = zeros(2*n,1);
cth = cos(xi(3));  sth = sin(xi(3));
for k = 1:n
    y = pc.pos(k,1);  z = pc.pos(k,2);
    T(2*k-1,:) = [0, y, z];
    T(2*k,:)   = [-L*g, y + L*s*pc.u(k,1), z + L*s*pc.u(k,2)];
    QT(2*k-1)  = qT(k);   QT(2*k) = -qT(k);
    by = y - L*s*pc.u(k,1);
    C(2*k-1,:) = [-sth*y + xi(1),          cth*y + xi(2),          z];
    C(2*k,:)   = [cth*L*g - sth*by + xi(1), sth*L*g + cth*by + xi(2), z - L*s*pc.u(k,2)];
    QC(2*k-1)  = -qC(k);  QC(2*k) = qC(k);
end
F = zeros(1,3);  Mz = 0;
for a = 1:2*n
    fa = zeros(1,3);
    for b = 1:2*n
        r  = C(a,:) - T(b,:);
        d2 = r*r.' + pc.x0^2;
        fa = fa + (mu0*QC(a)*QT(b)/(4*pi*d2^1.5))*r;
    end
    F  = F + fa;
    Mz = Mz + (C(a,1) - xi(1))*fa(2) - (C(a,2) - xi(2))*fa(1);
end
W = [F(1); F(2); Mz];
end
