%% test_plant.m  --  validate asteria_plant/asteria_wrench before anything is built on top
% Every check is a physical invariant, not a regression against stored numbers. Run after
% bench_init_3dof. A FAIL here means the plant is wrong; do not proceed to the estimator.

assert(exist('P','var')==1, 'run bench_init_3dof first');
res  = true(0);
i4   = @(a) a*ones(4,1);
zero = [0;0;0];
fprintf('=== plant checks ===\n');

%% 1. No current, no force
W = asteria_wrench(P, [0.04;0;0], i4(0));
ok = norm(W) < 1e-15;  res(end+1) = ok;
fprintf('%s  zero current -> zero wrench (|W| = %.1e)\n', pass(ok), norm(W));

%% 2. Target-bias drive: linear in current and reversible
Wp = asteria_wrench(P, [0.04;0;0], i4(+0.2));
Wm = asteria_wrench(P, [0.04;0;0], i4(-0.2));
ok = Wp(1) < 0 && Wm(1) > 0 && abs(Wp(1)+Wm(1))/abs(Wp(1)) < 1e-12;  res(end+1) = ok;
fprintf('%s  current sign reverses the force (Fx = %+.3f / %+.3f mN)\n', ...
        pass(ok), 1e3*Wp(1), 1e3*Wm(1));
% With the target held at a fixed bias the wrench is LINEAR in the chaser currents. Under the
% old symmetric drive it was quadratic and attract-only; these two checks are what tells the
% two configurations apart, so they are the first thing to look at if the rig misbehaves.
i1 = [0.1;0.2;-0.05;0.15];
ok = norm(asteria_wrench(P,[0.04;0;0],2*i1) - 2*asteria_wrench(P,[0.04;0;0],i1)) ...
     / norm(asteria_wrench(P,[0.04;0;0],i1)) < 1e-12;  res(end+1) = ok;
fprintf('%s  W(2i) = 2 W(i) exactly (linear in current)\n', pass(ok));
Wr = asteria_wrench(P, [0.04;0;0], [0.2;0.2;-0.2;-0.2]);
ok = abs(Wr(1)) < 1e-3*abs(Wp(1)) && abs(Wr(3)) > 1e-7;  res(end+1) = ok;
fprintf('%s  half the pairs reversed -> axial cancels (%.1e mN), yaw remains (%.1f uN*m)\n', ...
        pass(ok), 1e3*Wr(1), 1e6*Wr(3));

%% 3. Mirror symmetry about y = 0
Wa = asteria_wrench(P, [0.04;+0.003;+0.02], i4(0.2));
Wb = asteria_wrench(P, [0.04;-0.003;-0.02], i4(0.2));
err = norm([Wa(1)-Wb(1); Wa(2)+Wb(2); Wa(3)+Wb(3)])/norm(Wa);
ok = err < 1e-9;  res(end+1) = ok;
fprintf('%s  mirror symmetry in (y,th) (rel. error %.1e)\n', pass(ok), err);

%% 5. Energy: drop test at fixed current vs the work integral
i_d = i4(P.i_max);
opt = odeset('Events', @(t,s) contact(t,s), 'RelTol',1e-10, 'AbsTol',1e-12);
sol = ode45(@(t,s) asteria_plant(t,s,i_d,P,zero), [0 500], [0.10;0;0;0;0;0], opt);
v_end = sol.y(4,end);
xg = linspace(1e-6, 0.10, 4001).';
Fx = zeros(size(xg));
for n = 1:numel(xg), w = asteria_wrench(P,[xg(n);0;0],i_d); Fx(n) = w(1); end
Wk = -trapz(xg, Fx);                       % work done closing from 100 mm to contact
err = abs(0.5*P.m*v_end^2 - Wk)/Wk;
ok = err < 2e-3;  res(end+1) = ok;
fprintf('%s  energy closes on a drop test (KE vs work, rel. error %.2e)\n', pass(ok), err);
fprintf('       contact after %.1f s at %.1f mm/s\n', sol.x(end), 1e3*abs(v_end));

%% 6. Lateral mode frequency matches the linearised stiffness
i_l = i4(P.des.i);
s0  = [P.des.x; 2e-3; 0; 0; 0; 0];
mask_y = [0;1;0;0;1;0];   % free the lateral DoF only: yaw is unstable and would swamp the run
sol = ode45(@(t,s) asteria_plant_masked(t,s,i_l,P,mask_y), [0 60], s0, odeset('RelTol',1e-10,'AbsTol',1e-12));
tv  = linspace(0,60,20001);
yy  = deval(sol, tv); yy = yy(2,:);
zc  = find(yy(1:end-1).*yy(2:end) < 0);                  % zero crossings
Tm  = 2*mean(diff(tv(zc)));                              % measured period
Tp  = 2*pi/sqrt(P.mode.ky/P.m);                          % predicted
err = abs(Tm-Tp)/Tp;  ok = err < 5e-2;  res(end+1) = ok;
fprintf('%s  lateral period %.2f s measured vs %.2f s predicted (%.1f%%)\n', ...
        pass(ok), Tm, Tp, 100*err);

%% 7. Yaw divergence rate matches the linearised stiffness
mask_th = [0;0;1;0;0;1];
th0 = 1e-4;
s0  = [P.des.x; 0; th0; 0; 0; P.mode.th_pole*th0];   % start ON the unstable eigenvector, else the
sol = ode45(@(t,s) asteria_plant_masked(t,s,i_l,P,mask_th), [0 6], s0, ...   % cosh transient biases the fit
            odeset('RelTol',1e-11,'AbsTol',1e-14));
tt  = linspace(2,6,500);  th = deval(sol,tt); th = th(3,:);
lam = polyfit(tt, log(th), 1);  lam = lam(1);
ok  = abs(lam - P.mode.th_pole)/P.mode.th_pole < 5e-2;  res(end+1) = ok;
fprintf('%s  yaw divergence %.3f rad/s measured vs %.3f predicted\n', pass(ok), lam, P.mode.th_pole);

%% 8. The control Jacobian is rank 2: axial force and yaw torque, lateral tied to them
Bi = zeros(3,P.n_coil);  hh = 1e-5;  i0 = P.des.i*ones(P.n_coil,1);
for k = 1:P.n_coil
    ip = i0; ip(k) = ip(k)+hh;  im = i0; im(k) = im(k)-hh;
    Bi(:,k) = (asteria_wrench(P,[P.des.x;0;0],ip) - asteria_wrench(P,[P.des.x;0;0],im))/(2*hh);
end
sv = svd(Bi);
ok = rank(Bi,1e-9) == 2 && sv(3) < 1e-6*sv(1);  res(end+1) = ok;
fprintf('%s  dW/di rank %d of 3, singular values %s (lateral is not independent)\n', ...
        pass(ok), rank(Bi,1e-9), mat2str(sv.',3));

fprintf('\n%d/%d checks passed\n', sum(res), numel(res));

%% helpers
function s = pass(ok)
s = ' ok '; if ~ok, s = 'FAIL'; end
end
function [val, ter, dir] = contact(~, s)
val = s(1);  ter = 1;  dir = -1;
end
function ds = asteria_plant_masked(t, s, i, P, mask)
% integrate only the DoF selected by mask, to isolate one mode at a time
ds = asteria_plant(t, s, i, P, [0;0;0]) .* mask;
end
