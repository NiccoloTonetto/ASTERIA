%% build_target.m  --  C library of gnc_step for the flight computer (Embedded Coder)
% Generates ARM Cortex-M C code (not compiled here) and a host SIL build of the same library
% for test_sil_lib. Host SIL on macOS needs the full Xcode app.
assert(isfield(P,'sched'), 'run bench_init_3dof, optimize_maneuver, build_schedule first');
pc = make_cg_params(P);
ex = {zeros(10,1), zeros(10), zeros(3), 0, zeros(4,1), zeros(4,1), 0, pc};

cfg = coder.config('lib', 'ecoder', true);
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-M';
cfg.GenCodeOnly = true;
cfg.GenerateReport = true;
cfg.TargetLang = 'C';
tic; codegen -config cfg gnc_step -args ex -d codegen/gnc_target; t1 = toc;

cfs = coder.config('lib', 'ecoder', true);
cfs.VerificationMode = 'SIL';
try
    tic; codegen -config cfs gnc_step -args ex -d codegen/gnc_sil; t2 = toc;
catch ME
    t2 = NaN;
    fprintf('host SIL build skipped: %s\n  -> install Xcode (App Store), then: sudo xcode-select -s /Applications/Xcode.app ; mex -setup C\n', ...
            strtok(ME.message, newline));
end

f = [dir(fullfile('codegen','gnc_target','*.c')); dir(fullfile('codegen','gnc_target','*.h'))];
fprintf('target C: %d files, %.1f kB of source (%.0f s); host SIL build (%.0f s)\n', ...
        numel(f), sum([f.bytes])/1024, t1, t2);
w = whos('pc');
fprintf('parameter struct pc: %.1f kB -- place it in flash as const; the reference tables dominate\n', w.bytes/1024);
