%% test_replay.m  --  prove the replay toolchain against known truth (V1..V5)
% Each case injects a fault the filter is never told about, then checks that the replay's three
% acceptance tests fail in the predicted way. If a fault does not produce its signature, the
% diagnostics table in the runbook is decorative and must be fixed before hardware.
assert(isfield(P,'ref'), 'run bench_init_3dof, optimize_maneuver, build_schedule first');
c0 = struct('fscale',1,'x0',P.cal.x0,'m',P.m,'tilt',[0;0;0],'gbias',0,'abias',0, ...
            'tof_off',[0;0],'tof_sc',[1;1],'latency',0,'seed',1);
faults = { 'none',                c0;
           'ToF offset +3 mm',    setfield(c0,'tof_off',[3e-3;3e-3]);
           'ToF latency 30 ms',   setfield(c0,'latency',0.030);
           'force scale 1.5',     setfield(c0,'fscale',1.5);
           'gyro bias 1 deg/s',   setfield(c0,'gbias',deg2rad(1)) };
expect = { 'all pass'; 'tof mean fails'; 'tof whiteness or mean fails'; ...
           'tof mean fails, NIS high'; 'gyro mean fails' };

fprintf('%-20s | %-22s | %-22s | %-9s | %s\n', ...
        'injected fault','tof1  mean/NIS/white','gyro  mean/NIS/white','RMS x mm','verdict');
for k = 1:size(faults,1)
    [nm, cc] = faults{k,:};
    fn = sprintf('v5_%d.csv', k);
    evalc(sprintf('make_log(P,''approach'',cc,''%s'')', fn));
    R  = replay_ekf(fn, P, false);
    a  = R.stats(1);  g = R.stats(4);
    fprintf('%-20s | %-22s | %-22s | %8.2f | %s\n', nm, ...
        sprintf('%s/%s/%s', yn(a.ok_mean), yn(a.ok_nis), yn(a.ok_white)), ...
        sprintf('%s/%s/%s', yn(g.ok_mean), yn(g.ok_nis), yn(g.ok_white)), ...
        1e3*R.rms_truth(1), verdict(R.pass));
end
fprintf('\nexpected signatures:\n');
for k = 1:numel(expect), fprintf('  %-20s -> %s\n', faults{k,1}, expect{k}); end

function s = yn(b),  if b, s = 'ok'; else, s = 'FAIL'; end, end
function s = verdict(p), if p, s = 'consistent'; else, s = 'FLAGGED'; end, end
