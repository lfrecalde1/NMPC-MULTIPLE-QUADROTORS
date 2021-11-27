function [costo] = Cost_Function_drone(h0, hd, vc, v0, ts, N, L, x, i)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here

[h, hp] = open_loop_drone(h0, vc, v0, ts, N, L, x);

%% generacion del vector de estados deseados
hdaux = hd(:,i:i+(N));

%% Definimos el valor del costo inicial
[he, u ] = general(h, hdaux, vc, N);

du = delta_u(vc, N);

Q = 1*eye(length(he));
R1 = 1*eye(length(du));

R2 = 0.03*eye(length(u));

%costo = he'*Q*he + du'*R1*du;
costo = he'*Q*he + u'*R2*u;
end