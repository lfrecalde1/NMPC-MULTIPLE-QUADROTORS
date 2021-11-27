function [v] = system_dynamic(x, v, vc, ts)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
vp = dynamic_func(x, v, vc);
v = v +vp*ts;
end

