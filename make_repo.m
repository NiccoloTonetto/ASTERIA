%% make_repo.m  --  assemble the asteria-gnc repository from the working folder (run once)
% Put this folder (the unzipped asteria-gnc) where you want the repository, then run this script
% from inside it. It copies the verified code from ASTERIA_bench into the repository layout, checks
% that nothing is missing, makes sure the old folder cannot shadow the new one, and sets the path.
% ASTERIA_bench itself is left untouched.
src = fullfile(getenv('HOME'), 'Documents', 'ASTERIA_bench');
dst = fileparts(mfilename('fullpath'));
assert(isfolder(src), 'working folder not found: %s', src);
layout = {
  fullfile('src','model'),    {'asteria_wrench','asteria_plant','asteria_meas','asteria_axial_1v1'}
  fullfile('src','gnc'),      {'asteria_ekf','asteria_ekf_sizes','asteria_ctrl'}
  fullfile('src','embedded'), {'cg_wrench','cg_meas','cg_ekf_step','cg_ctrl','gnc_step','make_cg_params'}
  fullfile('src','simulink'), {'sl_plant','sl_sensors','sl_ekf','sl_ctrl'}
  'scripts',                  {'bench_init_3dof','optimize_maneuver','build_schedule','build_simulink_model', ...
                               'build_cg','build_target','update_lut'}
  'analysis',                 {'monte_carlo','mc_run'}
  'tools',                    {'make_log','replay_ekf','identify_from_log'}
  'tests',                    {'test_plant','test_ekf','test_closed_loop','test_envelope','test_update_lut', ...
                               'test_replay','test_precision','test_sil','test_sil_lib'} };
missing = {};  n = 0;
for r = 1:size(layout,1)
    if ~isfolder(fullfile(dst, layout{r,1})), mkdir(fullfile(dst, layout{r,1})); end
    for f = layout{r,2}
        s = fullfile(src, [f{1} '.m']);
        if isfile(s), copyfile(s, fullfile(dst, layout{r,1}, [f{1} '.m'])); n = n + 1;
        else, missing{end+1} = f{1}; end %#ok<AGROW>
    end
end
fprintf('copied %d files into %s\n', n, dst);
if ~isempty(missing), warning('missing from %s: %s', src, strjoin(missing, ', ')); end
% files in the working folder that the repository deliberately leaves out
here = dir(fullfile(src, '*.m'));  known = [layout{:,2}];
extra = setdiff(erase({here.name}, '.m'), known);
if ~isempty(extra), fprintf('left out (legacy or scratch): %s\n', strjoin(extra, ', ')); end
% the old folder must not shadow the repository
if contains(path, src), rmpath(src); fprintf('removed %s from the path\n', src); end
run(fullfile(dst, 'startup.m'));
w = which('asteria_ekf');
assert(startsWith(w, dst), 'asteria_ekf resolves outside the repository: %s', w);
fprintf('all functions resolve inside the repository. Next: run_all_tests\n');
