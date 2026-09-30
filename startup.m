% STARTUP  Put the ASTERIA GNC folders on the MATLAB path and work from the repository root.
% Generated files (model, results, logs, generated code) are written to the root and git-ignored.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src','model'), fullfile(root,'src','gnc'), fullfile(root,'src','embedded'), ...
        fullfile(root,'src','simulink'), fullfile(root,'scripts'), fullfile(root,'analysis'), ...
        fullfile(root,'tools'), fullfile(root,'tests'));
cd(root);
fprintf('ASTERIA GNC: path set, working folder %s\n', root);
clear root
