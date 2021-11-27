function [c, ceq] = Nonlinear_constraint(h0, hd, vc, v0, ts, N, L, x, obj, r)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here

[h, hp] = open_loop_drone(h0, vc, v0, ts, N, L, x);

%% Generacion de vector de estados de restricciones
c = [];
ceq = [];

for k=1:N
    [desigualdad, igualdad] = constraints(h(:,k), obj, r);
    c = [c desigualdad];
    ceq = [ceq igualdad];
    
end
end