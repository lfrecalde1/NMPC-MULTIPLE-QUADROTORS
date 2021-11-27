function [hxd, hyd, hzd, hphid] = point_trajectory(t,x,y,z, phi)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
hxd = x*ones(1,length(t));
hyd = y*ones(1,length(t));
hzd = z*ones(1,length(t));
hphid = phi*(pi/180)*ones(1,length(t));
end

