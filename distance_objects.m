function [d] = distance_objects(obs, h)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
d = [];
for k =1:size(obs,2)
    aux = h(1:3)- obs(1:3,k);
    d = [d;norm(aux,2)];
end
end

