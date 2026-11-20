%% new_build_schedule_TVLQR.m  --  time-varying LQR gain schedule along the reference

% the Riccati differential equation
% -dX/dt = A4'X + X A4 - X B4 R^-1 B4' X + Q,    K(t) = R^-1 B4(t)' X(t)
% is integrated backwards along the reference (RK4, step dt_ric), starting from the frozen
% (algebraic) solution at the smallest gap, and the gains are stored on the same gap grid.
% Same Q and R as build_schedule.


%get ready like in build_schedule
assert(isfield(P,'ref'), 'run optimize_maneuver first');
P.sched.x  = logspace(log10(5e-4), log10(P.traj.x0), 40).';   % same grid as build_schedule
n  = numel(P.sched.x);
P.sched.K4 = zeros(4,4,n);
P.sched.A  = zeros(P.n_coil,2,n);
Minv = diag([1/P.m, 1/P.m, 1/P.J]);
sel  = [1 3 4 6];
Q = P.lqr.Q;  R = P.lqr.R;


% !!!!!!!
% design model: optionally a smaller softening length than nominal, so the gains
% anticipate a sharper force near contact 
x0d = 3.0e-3; %3.9e-3 normal

if isfield(P.lqr,'tv_x0'), x0d = P.lqr.tv_x0; end
Pd = P;  Pd.cal.x0 = x0d;


[xu, iu] = unique(P.ref.x);      
t_end  = interp1(xu, P.ref.t(iu), P.sched.x(1));
dt_ric = 1e-3; % Riccati integration step
tg = linspace(0, t_end, max(2, ceil(t_end/dt_ric)+1)).';
m  = numel(tg); % number of time points we linearise in

% linearise at every time point of the reference
Ag = zeros(4,4,m);  Bg = zeros(4,P.n_coil,m);
for q = 1:m %through all time points
    xk = interp1(P.ref.t, P.ref.x, tg(q), 'linear', 'extrap'); %the gap at time tg(q)
    ik = interp1(P.ref.t, P.ref.i, tg(q), 'linear', 'extrap'); %the current at that time
    i0 = ik*ones(P.n_coil,1);
    Bi = zeros(3,P.n_coil);  h = 1e-5;  %the step of differential
    for j = 1:P.n_coil  %through each coil on magnet
        ip = i0; ip(j) = ip(j)+h;  im = i0; im(j) = im(j)-h;
        Bi(:,j) = (asteria_wrench(Pd,[xk;0;0],ip) - asteria_wrench(Pd,[xk;0;0],im))/(2*h);
    end
    Kp = zeros(3);  hp = 1e-6;  %stiffness
    for j = 1:3   %through x,y,theta
        xp = [xk;0;0]; xp(j) = xp(j)+hp;  xm = [xk;0;0]; xm(j) = xm(j)-hp;
        Kp(:,j) = (asteria_wrench(Pd,xp,i0) - asteria_wrench(Pd,xm,i0))/(2*hp);
    end
    A6 = [zeros(3) eye(3); Minv*Kp zeros(3)];
    B6 = [zeros(3,P.n_coil); Minv*Bi];
    Ag(:,:,q) = A6(sel,sel);  Bg(:,:,q) = B6(sel,:);
    % Aq and Bq take with sel what we need and q is for that moment q
end

[~, X] = lqr(Ag(:,:,m), Bg(:,:,m), Q, R);  %for the last moment normal LQR
%cuz future after it is just infinite being on that position
Xg = zeros(4,4,m);  Xg(:,:,m) = X;
ric = @(X,A,B) A.'*X + X*A - X*B*(R\B.')*X + Q;     % = -dX/dt

% RK4 (4th-order Runge-Kutta): each backward step combines the Riccati slope 
% at both ends and twice at the midpoint (A, B averaged),withh a fixed step
for q = m:-1:2 % RK4, backwards in time
    hs = tg(q) - tg(q-1);
    Am = (Ag(:,:,q) + Ag(:,:,q-1))/2;  Bm = (Bg(:,:,q) + Bg(:,:,q-1))/2;
    k1 = ric(X,           Ag(:,:,q),   Bg(:,:,q));
    k2 = ric(X + hs/2*k1, Am,          Bm);
    k3 = ric(X + hs/2*k2, Am,          Bm);
    k4 = ric(X + hs*k3,   Ag(:,:,q-1), Bg(:,:,q-1));
    X  = X + hs/6*(k1 + 2*k2 + 2*k3 + k4);
    X  = (X + X.')/2;                              
    Xg(:,:,q-1) = X;
end

% gains and allocation on the gap grid
Xv = reshape(Xg, 16, m).'; %making it a mx16 matrix
for k = 1:n
    xk = P.sched.x(k);
    tk = interp1(xu, P.ref.t(iu), xk);
    Xk = reshape(interp1(tg, Xv, tk), 4, 4);
    ik = interp1(P.ref.x(end:-1:1), P.ref.i(end:-1:1), xk, 'linear', 'extrap');
    i0 = ik*ones(P.n_coil,1);
    Bi = zeros(3,P.n_coil);  h = 1e-5;
    for j = 1:P.n_coil
        ip = i0; ip(j) = ip(j)+h;  im = i0; im(j) = im(j)-h;
        Bi(:,j) = (asteria_wrench(Pd,[xk;0;0],ip) - asteria_wrench(Pd,[xk;0;0],im))/(2*h);
    end
    B6 = [zeros(3,P.n_coil); Minv*Bi];
    P.sched.K4(:,:,k) = R \ (B6(sel,:).' * Xk);  %TVLQR equation
    P.sched.A(:,:,k)  = pinv(Bi([1 3],:));
end


%end of code, beneath just print

fprintf('time-varying gain schedule: %d gaps from %.1f to %.0f mm, Riccati over %.2f s (%d steps)\n', ...
        n, 1e3*P.sched.x(1), 1e3*P.sched.x(end), t_end, m-1);
fprintf('%8s %12s %12s %12s %12s\n','x[mm]','K(x,x) tv','K(x,x) frz','K(1,th) tv','max Re CL');
for k = [1 5 10 round(n/2) 30 n]
    xk = P.sched.x(k);
    ik = interp1(P.ref.x(end:-1:1), P.ref.i(end:-1:1), xk, 'linear','extrap');
    i0 = ik*ones(P.n_coil,1);  h = 1e-5;  Bi = zeros(3,P.n_coil);
    for j = 1:P.n_coil
        ip=i0; ip(j)=ip(j)+h; im=i0; im(j)=im(j)-h;
        Bi(:,j) = (asteria_wrench(Pd,[xk;0;0],ip)-asteria_wrench(Pd,[xk;0;0],im))/(2*h);
    end
    Kp = zeros(3); hp = 1e-6;
    for j = 1:3
        xp=[xk;0;0]; xp(j)=xp(j)+hp; xm=[xk;0;0]; xm(j)=xm(j)-hp;
        Kp(:,j) = (asteria_wrench(Pd,xp,i0)-asteria_wrench(Pd,xm,i0))/(2*hp);
    end
    A4 = [zeros(3) eye(3); Minv*Kp zeros(3)];  A4 = A4(sel,sel);
    B4 = [zeros(3,P.n_coil); Minv*Bi];  B4 = B4(sel,:);
    K  = P.sched.K4(:,:,k);
    Kf = lqr(A4, B4, Q, R);                        % frozen gain at this gap, for comparison
    fprintf('%8.1f %12.1f %12.1f %12.1f %12.3f\n', 1e3*xk, K(1,1), Kf(1,1), K(1,2), ...
            max(real(eig(A4-B4*K))));
end