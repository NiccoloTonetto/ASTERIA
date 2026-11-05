%% build_schedule.m  --  LQR gain schedule along the reference
% Run after bench_init_3dof and optimize_maneuver. Writes P.sched.
% At each grid gap: linearise at the reference current, solve the LQR with the same Q and R,
% store K4 and the trim allocation. asteria_ctrl interpolates both on the estimated gap.

assert(isfield(P,'ref'), 'run optimize_maneuver first');
P.sched.x  = logspace(log10(5e-4), log10(P.traj.x0), 40).';   % log-spaced from 0.5 mm: the
                                   % plant scales like a power of the gap and changes fastest
                                   % in the last millimetre
n  = numel(P.sched.x);
P.sched.K4 = zeros(4,4,n);
P.sched.A  = zeros(P.n_coil,2,n);   % (Fx, Mz) demand -> four currents
Minv = diag([1/P.m, 1/P.m, 1/P.J]);
sel  = [1 3 4 6];
for k = 1:n
    xk = P.sched.x(k);
    ik = interp1(P.ref.x(end:-1:1), P.ref.i(end:-1:1), xk, 'linear', 'extrap');
    i0 = ik*ones(P.n_coil,1);
    Bi = zeros(3,P.n_coil);  h = 1e-5;
    for j = 1:P.n_coil
        ip = i0; ip(j) = ip(j)+h;  im = i0; im(j) = im(j)-h;
        Bi(:,j) = (asteria_wrench(P,[xk;0;0],ip) - asteria_wrench(P,[xk;0;0],im))/(2*h);
    end
    Kp = zeros(3);  hp = 1e-6;
    for j = 1:3
        xp = [xk;0;0]; xp(j) = xp(j)+hp;  xm = [xk;0;0]; xm(j) = xm(j)-hp;
        Kp(:,j) = (asteria_wrench(P,xp,i0) - asteria_wrench(P,xm,i0))/(2*hp);
    end
    A6 = [zeros(3) eye(3); Minv*Kp zeros(3)];
    B6 = [zeros(3,P.n_coil); Minv*Bi];
    P.sched.K4(:,:,k) = lqr(A6(sel,sel), B6(sel,:), P.lqr.Q, P.lqr.R);
    P.sched.A(:,:,k)  = pinv(Bi([1 3],:));
end
fprintf('gain schedule: %d gaps from %.0f to %.0f mm\n', n, 1e3*P.sched.x(1), 1e3*P.sched.x(end));
fprintf('%8s %12s %12s %12s\n','x[mm]','K(x,x)','K(th,th)','max Re CL');
for k = [1 round(n/2) n]
    xk = P.sched.x(k);
    ik = interp1(P.ref.x(end:-1:1), P.ref.i(end:-1:1), xk, 'linear','extrap');
    i0 = ik*ones(P.n_coil,1);  h = 1e-5;  Bi = zeros(3,P.n_coil);
    for j = 1:P.n_coil
        ip=i0; ip(j)=ip(j)+h; im=i0; im(j)=im(j)-h;
        Bi(:,j) = (asteria_wrench(P,[xk;0;0],ip)-asteria_wrench(P,[xk;0;0],im))/(2*h);
    end
    Kp = zeros(3); hp = 1e-6;
    for j = 1:3
        xp=[xk;0;0]; xp(j)=xp(j)+hp; xm=[xk;0;0]; xm(j)=xm(j)-hp;
        Kp(:,j) = (asteria_wrench(P,xp,i0)-asteria_wrench(P,xm,i0))/(2*hp);
    end
    A4 = [zeros(3) eye(3); Minv*Kp zeros(3)];  A4 = A4(sel,sel);
    B4 = [zeros(3,P.n_coil); Minv*Bi];  B4 = B4(sel,:);
    K  = P.sched.K4(:,:,k);
    fprintf('%8.1f %12.1f %12.1f %12.3f\n', 1e3*xk, K(1,1), K(1,2), max(real(eig(A4-B4*K))));
end
