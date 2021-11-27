function [c,ceq] = constraints(h, ob, r)
%% Inequality constrain  <=
A = [1,0,0;...
    -1,0,0;...
     0,1,0;...
     0,-1,0;...
     0,0,1;...
     0,0,-1];
%% Generacion de las contantes de restriccion
z = [5;5;5;5;5;5];

haux = h(1:3,1);

%% Restricciones  de estados del sistema
c = (A*haux-z);

d = ditancia(h, ob);
R = r*ones(size(ob,2),1);

c1 = -d+R;
%% crecion del vector final de restriccionos
c = [c1];


ceq  = [0];
end