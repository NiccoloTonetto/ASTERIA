%% build_target.m  --  standalone C library for the flight computer (Embedded Coder)
% Generates portable C for ARM Cortex-M from gnc_step (EKF + controller), without compiling it for
% the target -- that is the firmware toolchain's job. Also builds a host SIL version of the SAME
% library so test_sil_lib can prove the generated code against MATLAB before it goes near a board.
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
