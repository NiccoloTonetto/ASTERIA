function ds = sl_plant(s, i, d)
% SL_PLANT  Simulink wrapper for asteria_plant, called extrinsically; P read from the base
%   workspace.
persistent P
if isempty(P), P = evalin('base','P'); end
ds = asteria_plant(0, s, i, P, d);
end
