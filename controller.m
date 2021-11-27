function [control] = controller(h, hd, hdp, k1, k2, L)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
%% GENERALIZED VEECTOR OF ERROR
he = hd - h;

%% GAIN MATRICES OF THE SYSTEM
K1 = k1*eye(size(h,1));
K2 = k2*eye(size(h,1));

J = drone_jacobian(h, L);

control = pinv(J)*(hdp+K2*tanh(pinv(K2)*K1*he));

end

