function [h, hp] = open_loop_drone(h0, vc, v0, ts, N, L, x)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
h(:,1) = h0;
v(:,1) = v0;
for k=1:1:N
    v(:,k+1) = system_dynamic(x, v(:,k), vc(:,k), ts);
    [h(:,k+1), hp(:,k)]= system_drone(h(:,k), v(:,k+1), ts, L); 
end
end

