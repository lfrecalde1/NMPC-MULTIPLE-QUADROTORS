function [d] = ditancia(h, ob)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
y =[1,0,0,0;...
    0,1,0,0;...
    0,0,1,0];

obaux = y*h-ob;
d = [];
for k=1:1:size(ob,2)
    d = [d;norm(obaux(:,k),2)];
end
end

