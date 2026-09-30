%% run_all_tests.m  --  does this installation reproduce the reference results?
% Run from the repository root after startup. About two minutes (plus ~10 s to build the MEX files if absent). To run a subset, set
% tests_to_run first, e.g.  tests_to_run = {'plant','replay'};  run_all_tests
%
% Verdicts: pass  -- the test printed its own PASS / checks passed
%           ran   -- ran without error; it prints tables to inspect rather than a verdict
%           FAIL / ERROR -- the installation does not reproduce the reference. Stop and look.
% State is kept in root appdata because several test scripts clear the workspace.
if ~exist('tests_to_run','var') || isempty(tests_to_run), tests_to_run = 'all'; end
rat_suite = { 'plant',       'test_plant';
              'ekf',         'test_ekf';
              'closed_loop', 'test_closed_loop';
              'envelope',    'test_envelope';
              'update_lut',  'test_update_lut';
              'replay',      'test_replay';
              'precision',   'test_precision';
              'sil',         'test_sil';
              'simulink',    'test_simulink' };
if ~(ischar(tests_to_run) && strcmp(tests_to_run,'all'))
    rat_suite = rat_suite(ismember(rat_suite(:,1), cellstr(tests_to_run)), :);
end
setappdata(0, 'rat_suite', rat_suite);
setappdata(0, 'rat_res', cell(0,3));
clear tests_to_run
fprintf('%-12s %-6s %s\n', 'test', 'result', 'time');
for rat_k = 1:size(rat_suite,1)
    rat_s = getappdata(0,'rat_suite');
    setappdata(0, 'rat_name', rat_s{rat_k,1});  setappdata(0, 'rat_t0', tic);
    try
        clear asteria_ekf
        evalc('run(''bench_init_3dof.m''); run(''optimize_maneuver.m''); run(''build_schedule.m'')');
        rat_out = evalc(['run(''' rat_s{rat_k,2} '.m'')']);
        rat_v = rat_verdict(getappdata(0,'rat_name'), rat_out);
    catch rat_ME
        rat_v = 'ERROR';  rat_out = rat_ME.message;
    end
    rat_r = getappdata(0,'rat_res');
    rat_r(end+1,:) = {getappdata(0,'rat_name'), rat_v, toc(getappdata(0,'rat_t0'))};
    setappdata(0, 'rat_res', rat_r);
    fprintf('%-12s %-6s %4.0f s\n', rat_r{end,1}, rat_r{end,2}, rat_r{end,3});
    if strcmp(rat_v,'ERROR') || strcmp(rat_v,'FAIL')
        fprintf('   %s\n', strtrim(extractBefore([rat_out newline], newline)));
    end
end
rat_r = getappdata(0,'rat_res');
fid = fopen('test_report.txt','w');
fprintf(fid, 'ASTERIA GNC test report, %s, MATLAB %s\n', char(datetime('now')), version);
for rat_k = 1:size(rat_r,1), fprintf(fid, '%-12s %-6s %4.0f s\n', rat_r{rat_k,:}); end
fclose(fid);
rat_bad = sum(ismember(rat_r(:,2), {'FAIL','ERROR'}));
fprintf('\n%d tests: %d pass, %d ran, %d failed. Report in test_report.txt\n', size(rat_r,1), ...
        sum(strcmp(rat_r(:,2),'pass')), sum(strcmp(rat_r(:,2),'ran')), rat_bad);
rmappdata(0,'rat_suite'); rmappdata(0,'rat_res'); rmappdata(0,'rat_name'); rmappdata(0,'rat_t0');
clear rat_*

function v = rat_verdict(name, out)
if strcmp(name, 'replay')
    % fault injection is supposed to print FAIL for the faulty cases; check the signatures instead
    L = splitlines(string(out));
    ok = any(startsWith(strtrim(L),'none') & contains(L,'consistent')) && ...
         any(contains(L,'ToF offset') & contains(L,'FLAGGED')) && ...
         any(contains(L,'force scale') & contains(L,'FLAGGED'));
    if ok, v = 'pass'; else, v = 'FAIL'; end
elseif contains(out, 'FAIL')
    v = 'FAIL';
elseif contains(out, {'PASS','checks passed','ok:'})
    v = 'pass';
else
    v = 'ran';
end
end
