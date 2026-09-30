function ds = sl_plant(s, i, d)
% Simulink wrapper for the plant. P is pulled once from the base workspace so the block
% needs no struct parameter. Called extrinsically from the MATLAB Function block.
persistent P
if isempty(P), P = evalin('base','P'); end
ds = asteria_plant(0, s, i, P, d);
end
