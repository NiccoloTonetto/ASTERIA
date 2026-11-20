%% new_test_replay_with_margins.m -- detection margins of the replay checks over many seeds

% The prints detection rates and margins over many seeds, not from
% a single pass/FLAGGED verdict.

% For each check the ratio is recorded ( statistic / threshold )
% (below 1 passes, above 1 flags) and took the worst of the 12 ratios 
% as the margin of a run. 
 
startup;
bench_init_3dof;
optimize_maneuver;
build_schedule;

Ns = 50; %number of seeds per error

c0 = struct('fscale',1,'x0',P.cal.x0,'m',P.m,'tilt',[0;0;0],'gbias',0,'abias',0, ...
    'tof_off',[0;0],'tof_sc',[1;1],'latency',0,'seed',1);
faults = { 'none',               c0;
    'ToF offset +3 mm',   setfield(c0,'tof_off',[3e-3;3e-3]);
    'ToF latency 30 ms',  setfield(c0,'latency',0.030);
    'force scale 1.5',    setfield(c0,'fscale',1.5);
    'gyro bias 1 deg/s',  setfield(c0,'gbias',deg2rad(1)) };


margin = nan(size(faults,1), Ns);
fn = 'margin_log.csv';      
for f = 1:size(faults,1)  %goes through all faults
    for s = 1:Ns         %goes through all different seeds
        cc = faults{f,2};  cc.seed = s; 
        clear asteria_ekf   % fresh cached Jacobian for every run!
        evalc('make_log(P,''approach'',cc,fn)');
        R = replay_ekf(fn, P, false);
        st = R.stats;
        margin(f,s) = max([abs([st.mean])./[st.mean_tol], ...
            abs([st.nis]-1)./[st.nis_tol], ...
            [st.ac_max]./[st.ac_tol]]);
    end
end

fprintf('%-20s %8s %8s %8s %8s\n','fault','flagged','min','median','max');
for f = 1:size(faults,1)
    m = margin(f,:);
    fprintf('%-20s %7.0f%% %8.2f %8.2f %8.2f\n', faults{f,1}, 100*mean(m>1), min(m), median(m), max(m));
end


%The clean log, zero faults had 28% false alarms
%The ToF, latency and gyro bias has 32%,26%,26% 
%similar as the clean case, the test cant detect them
%the force scale is detected much more often(92%)

% fault                 flagged      min   median      max
% none                      28%     0.66     0.84     1.49
% ToF offset +3 mm          32%     0.69     0.93     1.41
% ToF latency 30 ms         26%     0.62     0.86     1.40
% force scale 1.5           92%     0.81     1.54  3090.70
% gyro bias 1 deg/s         26%     0.66     0.85     1.43