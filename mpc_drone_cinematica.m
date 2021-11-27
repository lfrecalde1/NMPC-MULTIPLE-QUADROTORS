function [f,solver,args] = mpc_drone_cinematica(bounded, N, L1, L2, L3, x, obs, ts)

addpath('/home/fer/casadi-linux-matlabR2014b-v3.4.5');
import casadi.*;

%% CONSTANTES DINAMICA SISTEMA
%% Dinamic Parameters
chi1 = x(1);
chi2 = x(2);
chi3 = x(3);
x4 = x(4);
x5 = x(5);
x6 = x(6);
x7 = x(7);
x8 = x(8);
x9 = x(9);
x10 = x(10);
x11 = x(11);
x12 = x(12);
x13 = x(13);
x14 = x(14);
x15 = x(15);
x16 = x(16);
x17 = x(17);
x18 = x(18);
x19 = x(19);
x20 = x(20);
x21 = x(21);
x22 = x(22);
x23 = x(23);
x24 = x(24);
x25 = x(25);
x26 = x(26);
x27 = x(27);

%% Definicion de las restricciones en las acciones de control
%% RESTRICCION ROBOT 1
ul1_max = bounded(1); 
ul1_min = bounded(2);

um1_max = bounded(3);
um1_min = bounded(4);

un1_max = bounded(5);
un1_min = bounded(6);

w1_max = bounded(7); 
w1_min = bounded(8);

%% RESTRICION ROBOT 2
ul2_max = bounded(9); 
ul2_min = bounded(10);

um2_max = bounded(11);
um2_min = bounded(12);

un2_max = bounded(13);
un2_min = bounded(14);

w2_max = bounded(15); 
w2_min = bounded(16);

%% RESTRICTION ROBOT 3
ul3_max = bounded(17); 
ul3_min = bounded(18);

um3_max = bounded(19);
um3_min = bounded(20);

un3_max = bounded(21);
un3_min = bounded(22);

w3_max = bounded(23); 
w3_min = bounded(24);

%% Generacion de las variables simbolicas de los estados del sistema
x1 = SX.sym('x1'); 
y1 = SX.sym('y1');
z1 = SX.sym('z1');
th1 = SX.sym('th1');

ul1 = SX.sym('ul1');
um1 = SX.sym('um1');
un1 = SX.sym('un1');
w1 = SX.sym('w1');

x2 = SX.sym('x2'); 
y2 = SX.sym('y2');
z2 = SX.sym('z2');
th2 = SX.sym('th2');

ul2 = SX.sym('ul2');
um2 = SX.sym('um2');
un2 = SX.sym('un2');
w2 = SX.sym('w2');

x3 = SX.sym('x3'); 
y3 = SX.sym('y3');
z3 = SX.sym('z3');
th3 = SX.sym('th3');

ul3 = SX.sym('ul3');
um3 = SX.sym('um3');
un3 = SX.sym('un3');
w3 = SX.sym('w3');



%% Definicion de cuantos estados en el sistema
states = [x1;y1;z1;th1;x2;y2;z2;th2;x3;y3;z3;th3;ul1;um1;un1;w1;ul2;um2;un2;w2;ul3;um3;un3;w3];
n_states = length(states);

%% Generacion de las variables simbolicas de las acciones del control del sistema
ul1_ref = SX.sym('ul1_ref');
um1_ref = SX.sym('um1_ref');
un1_ref = SX.sym('un1_ref');
w1_ref = SX.sym('w1_ref');

ul2_ref = SX.sym('ul2_ref');
um2_ref = SX.sym('um2_ref');
un2_ref = SX.sym('un2_ref');
w2_ref = SX.sym('w2_ref');

ul3_ref = SX.sym('ul3_ref');
um3_ref = SX.sym('um3_ref');
un3_ref = SX.sym('un3_ref');
w3_ref = SX.sym('w3_ref');

%% Defincion de cuantas acciones del control tiene el sistema
controls = [ul1_ref;um1_ref;un1_ref;w1_ref;ul2_ref;um2_ref;un2_ref;w2_ref;ul3_ref;um3_ref;un3_ref;w3_ref]; 
n_control = length(controls);

n_objects = size(obs,2)*size(obs,1);

%% Definicion de los las constantes dl sistema
a1 = L1(1);
b1 = L1(2);
r1 = L1(4);

a2 = L2(1);
b2 = L2(2);
r2 = L2(4);

a3 = L3(1);
b3 = L3(2);
r3 = L3(4);

%% DINAMICA SISTEMA 1
%% INERTIAL MATRIX
M_11 = [x6/(chi1*x6 - chi2*x5), 0, 0, -chi2/(chi1*x6 - chi2*x5);...
       0, 1/chi3, 0, 0;...
       0, 0, 1/x4, 0;...
      -x5/(chi1*x6 - chi2*x5), 0, 0, chi1/(chi1*x6 - chi2*x5)];

%% CENTRIFUGAL FORCES
C1 = [x7, x8 + w1*x9, x10, x11;...
    x12 + w1*x13, x14, x15, x16 + w1*x17;...
    x18, x19, x20, x21;...
    x22, x23 + w1*x24, x25, x26];

G1 = [0; 0; x27; 0];

%% DINAMICA SISTEMA 2
%% INERTIAL MATRIX
M_12 = [x6/(chi1*x6 - chi2*x5), 0, 0, -chi2/(chi1*x6 - chi2*x5);...
       0, 1/chi3, 0, 0;...
       0, 0, 1/x4, 0;...
      -x5/(chi1*x6 - chi2*x5), 0, 0, chi1/(chi1*x6 - chi2*x5)];

%% CENTRIFUGAL FORCES
C2 = [x7, x8 + w2*x9, x10, x11;...
    x12 + w2*x13, x14, x15, x16 + w2*x17;...
    x18, x19, x20, x21;...
    x22, x23 + w2*x24, x25, x26];

G2 = [0; 0; x27; 0];

%% DINAMICA SISTEMA 3
%% INERTIAL MATRIX
M_13 = [x6/(chi1*x6 - chi2*x5), 0, 0, -chi2/(chi1*x6 - chi2*x5);...
       0, 1/chi3, 0, 0;...
       0, 0, 1/x4, 0;...
      -x5/(chi1*x6 - chi2*x5), 0, 0, chi1/(chi1*x6 - chi2*x5)];

%% CENTRIFUGAL FORCES
C3 = [x7, x8 + w3*x9, x10, x11;...
    x12 + w3*x13, x14, x15, x16 + w3*x17;...
    x18, x19, x20, x21;...
    x22, x23 + w3*x24, x25, x26];

G3 = [0; 0; x27; 0];

%% Defincion del sistema pero usando espacios de estados todo el sistema de ecuaciones
J1 = [cos(th1), -sin(th1), 0, -(a1*sin(th1)+b1*cos(th1));...
     sin(th1), cos(th1), 0,  (a1*cos(th1)-b1*sin(th1));...
     0, 0, 1, 0;...
     0, 0, 0, 1]; 
 
J2 = [cos(th2), -sin(th2), 0, -(a2*sin(th2)+b2*cos(th2));...
     sin(th2), cos(th2), 0,  (a2*cos(th2)-b2*sin(th2));...
     0, 0, 1, 0;...
     0, 0, 0, 1];
 
J3 = [cos(th3), -sin(th3), 0, -(a3*sin(th3)+b3*cos(th3));...
 sin(th3), cos(th3), 0,  (a3*cos(th3)-b3*sin(th3));...
 0, 0, 1, 0;...
 0, 0, 0, 1];
 

A = [zeros(4,4), zeros(4,4), zeros(4,4), J1, zeros(4,4), zeros(4,4);...
     zeros(4,4), zeros(4,4), zeros(4,4), zeros(4,4), J2, zeros(4,4);...
     zeros(4,4), zeros(4,4), zeros(4,4), zeros(4,4), zeros(4,4), J3;...
     zeros(4,4), zeros(4,4), zeros(4,4), -M_11*C1, zeros(4,4), zeros(4,4);...
     zeros(4,4), zeros(4,4), zeros(4,4), zeros(4,4), -M_12*C2, zeros(4,4);...
     zeros(4,4), zeros(4,4), zeros(4,4), zeros(4,4), zeros(4,4), -M_13*C3;];

 aux = [zeros(4,1);...
        zeros(4,1);...
        zeros(4,1);...
        -M_11*G1;...
        -M_12*G2;...
        -M_13*G3]; 
    
B = [zeros(4,4), zeros(4,4), zeros(4,4);...
     zeros(4,4), zeros(4,4), zeros(4,4);...
     zeros(4,4), zeros(4,4), zeros(4,4);...
     M_11, zeros(4,4), zeros(4,4);...
     zeros(4,4), M_12, zeros(4,4);...
     zeros(4,4), zeros(4,4), M_13];   

rhs=(A*states+aux+B*controls);

%% Definicion de kas funciones del sistema
f = Function('f',{states,controls},{rhs}); 
U = SX.sym('U',n_control,N);
P = SX.sym('P',n_states + N*(n_states)+n_objects);
%% vector que incluye el vector de estados y la referencia
X = SX.sym('X',n_states,(N+1));

%% Vector que representa el problema de optimizacion
g = [];  % restricciones de estados del problema  de optimizacion

%%EMPY VECTOR ERRORS
he = [];

%% EMPY VECTOR CONTROL VALUES
u = [];

%% INITIAL CONDITION ERROR
st  = X(:,1); % initial state
g = [g;X(:,1)-P(1:24)]; % initial condition constraints

%% Definicon del bucle para optimizacion a los largo del tiempo
for k = 1:N
    st = X(:,k);  con = U(:,k);
    
    hd = P(24*k+1:24*k+12);
    
    h = X(1:12,k);

    %% Funcion costo a minimizar 
    he = [he;h-hd];
    u = [u;con];
    
    %% Actualizacion del sistema usando Euler runge kutta
    st_next = X(:,k+1);
    k1 = f(st, con);   % new 
    k2 = f(st + ts/2*k1, con); % new
    k3 = f(st + ts/2*k2, con); % new
    k4 = f(st + ts*k3, con); % new
    st_next_RK4=st +ts/6*(k1 +2*k2 +2*k3 +k4); % new 
    
    %% Restricciones del sistema se =basan en el modelo del sistema
    g = [g;st_next-st_next_RK4]; 
end

%% Cost final 
Q = 1*eye(size(he,1));
R = 0.05*eye(size(u,1));

%% FINAL COST
obj = he'*Q*he+u'*R*u;

for k = 1:N+1   % box constraints due to the 
    %% SELF COLLISION SYSTEM
    g = [g ; -sqrt((X(1,k)-X(5,k))^2 +(X(2,k)-X(6,k))^2 +(X(3,k)-X(7,k))^2) + r1 + r2];
    g = [g ; -sqrt((X(1,k)-X(9,k))^2 +(X(2,k)-X(10,k))^2+(X(3,k)-X(11,k))^2) + r1 + r3];
    g = [g ; -sqrt((X(5,k)-X(9,k))^2 +(X(6,k)-X(10,k))^2+(X(7,k)-X(11,k))^2) + r2 + r3];
    
    %% EXTERNAL OBJECTS
    for i=1:3:(size(obs,2)*size(obs,1))
        g = [g ; -sqrt((X(1,k)-P(n_states + N*(n_states)+(i)))^2 +(X(2,k)-P(n_states + N*(n_states)+(i+1)))^2+(X(3,k)-P(n_states + N*(n_states)+(i+2)))^2) + r1];
        g = [g ; -sqrt((X(5,k)-P(n_states + N*(n_states)+(i)))^2 +(X(6,k)-P(n_states + N*(n_states)+(i+1)))^2+(X(7,k)-P(n_states + N*(n_states)+(i+2)))^2) + r2];
        g = [g ; -sqrt((X(9,k)-P(n_states + N*(n_states)+(i)))^2 +(X(10,k)-P(n_states + N*(n_states)+(i+1)))^2+(X(11,k)-P(n_states + N*(n_states)+(i+2)))^2) + r3];    
    end
    
end

for k =1:N-1
    g = [g; U(4,k)-U(4,k+1) - 0.02];
    g = [g; U(4,k+1)-U(4,k) - 0.02];
    g = [g; U(8,k)-U(8,k+1) - 0.05];
    g = [g; U(8,k+1)-U(8,k) - 0.05];
    g = [g; U(12,k)-U(12,k+1) - 0.05];
    g = [g; U(12,k+1)-U(12,k) - 0.05];
    
end
% se crea el vector de desiscion solo de una columna
OPT_variables = [reshape(X,24*(N+1),1);reshape(U,12*N,1)];

nlprob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlprob,opts);

args = struct;

args.lbg(1:24*(N+1)) = 0;  %-1e-20  %Equality constraints
args.ubg(1:24*(N+1)) = 0;  %1e-20   %Equality constraints

args.lbg(24*(N+1)+1 : 24*(N+1)+ (3+3*size(obs,2))*(N+1)) = -inf; % inequality constraints
args.ubg(24*(N+1)+1 : 24*(N+1)+ (3+3*size(obs,2))*(N+1)) = 0; % inequality constraints


args.lbg(24*(N+1)+ (3+3*size(obs,2))*(N+1)+1:24*(N+1)+ (3+3*size(obs,2))*(N+1)+6*(N-1)) = -inf; % inequality constraints
args.ubg(24*(N+1)+ (3+3*size(obs,2))*(N+1)+1:24*(N+1)+ (3+3*size(obs,2))*(N+1)+6*(N-1)) = 0; % inequality constraints

%% DRONE 1
args.lbx(1:24:24*(N+1),1) = -inf; %state x1 lower bound
args.ubx(1:24:24*(N+1),1) = inf;  %state x1 upper bound

args.lbx(2:24:24*(N+1),1) = -inf; %state y1 lower bound
args.ubx(2:24:24*(N+1),1) = inf;  %state y1 upper bound

args.lbx(3:24:24*(N+1),1) = -inf; %state z1 lower bound
args.ubx(3:24:24*(N+1),1) = inf;  %state z1 upper bound

args.lbx(4:24:24*(N+1),1) = -inf; %state theta1 lower bound
args.ubx(4:24:24*(N+1),1) = inf;  %state theta1 upper bound

%% DRONE 2
args.lbx(5:24:24*(N+1),1) = -inf; %state x2 lower bound
args.ubx(5:24:24*(N+1),1) = inf;  %state x2 upper bound

args.lbx(6:24:24*(N+1),1) = -inf; %state y2 lower bound
args.ubx(6:24:24*(N+1),1) = inf;  %state y2 upper bound

args.lbx(7:24:24*(N+1),1) = -inf; %state z2 lower bound
args.ubx(7:24:24*(N+1),1) = inf;  %state z2 upper bound

args.lbx(8:24:24*(N+1),1) = -inf; %state theta2 lower bound
args.ubx(8:24:24*(N+1),1) = inf;  %state theta2 upper bound

%% DRONE 3
args.lbx(9:24:24*(N+1),1) = -inf; %state x3 lower bound
args.ubx(9:24:24*(N+1),1) = inf;  %state x3 upper bound

args.lbx(10:24:24*(N+1),1) = -inf; %state y3 lower bound
args.ubx(10:24:24*(N+1),1) = inf;  %state y3 upper bound

args.lbx(11:24:24*(N+1),1) = -inf; %state z3 lower bound
args.ubx(11:24:24*(N+1),1) = inf;  %state z3 upper bound

args.lbx(12:24:24*(N+1),1) = -inf; %state theta3 lower bound
args.ubx(12:24:24*(N+1),1) = inf;  %state theta3 upper bound

%% DRONE 1
args.lbx(13:24:24*(N+1),1) = -inf; %state ul1 lower bound
args.ubx(13:24:24*(N+1),1) = inf;  %state ul1 upper bound

args.lbx(14:24:24*(N+1),1) = -inf; %state um1 lower bound
args.ubx(14:24:24*(N+1),1) = inf;  %state um1 upper bound

args.lbx(15:24:24*(N+1),1) = -inf; %state un1 lower bound
args.ubx(15:24:24*(N+1),1) = inf;  %state un1 upper bound

args.lbx(16:24:24*(N+1),1) = -inf; %state w1 lower bound
args.ubx(16:24:24*(N+1),1) = inf;  %state w1 upper bound

%% DRONE 2
args.lbx(17:24:24*(N+1),1) = -inf; %state ul2 lower bound
args.ubx(17:24:24*(N+1),1) = inf;  %state ul2 upper bound

args.lbx(18:24:24*(N+1),1) = -inf; %state um2 lower bound
args.ubx(18:24:24*(N+1),1) = inf;  %state um2 upper bound

args.lbx(19:24:24*(N+1),1) = -inf; %state un2 lower bound
args.ubx(19:24:24*(N+1),1) = inf;  %state un2 upper bound

args.lbx(20:24:24*(N+1),1) = -inf; %state w2 lower bound
args.ubx(20:24:24*(N+1),1) = inf;  %state w2 upper bound

%% DRONE 3
args.lbx(21:24:24*(N+1),1) = -inf; %state ul3 lower bound
args.ubx(21:24:24*(N+1),1) = inf;  %state ul3 upper bound

args.lbx(22:24:24*(N+1),1) = -inf; %state um3 lower bound
args.ubx(22:24:24*(N+1),1) = inf;  %state um3 upper bound

args.lbx(23:24:24*(N+1),1) = -inf; %state un3 lower bound
args.ubx(23:24:24*(N+1),1) = inf;  %state un3 upper bound

args.lbx(24:24:24*(N+1),1) = -inf; %state w3 lower bound
args.ubx(24:24:24*(N+1),1) = inf;  %state w3 upper bound

%% Definicion de las restricciones de las acciones de control del sistema
% DRONE 1
args.lbx(24*(N+1)+1:12:24*(N+1)+12*N,1) = ul1_min;  %
args.ubx(24*(N+1)+1:12:24*(N+1)+12*N,1) = ul1_max;  %

args.lbx(24*(N+1)+2:12:24*(N+1)+12*N,1) = um1_min;  %
args.ubx(24*(N+1)+2:12:24*(N+1)+12*N,1) = um1_max;  % 

args.lbx(24*(N+1)+3:12:24*(N+1)+12*N,1) = un1_min;  %
args.ubx(24*(N+1)+3:12:24*(N+1)+12*N,1) = un1_max;  %

args.lbx(24*(N+1)+4:12:24*(N+1)+12*N,1) = w1_min;  %
args.ubx(24*(N+1)+4:12:24*(N+1)+12*N,1) = w1_max;  %

%% DRONE 2
args.lbx(24*(N+1)+5:12:24*(N+1)+12*N,1) = ul2_min;  %
args.ubx(24*(N+1)+5:12:24*(N+1)+12*N,1) = ul2_max;  %

args.lbx(24*(N+1)+6:12:24*(N+1)+12*N,1) = um2_min;  %
args.ubx(24*(N+1)+6:12:24*(N+1)+12*N,1) = um2_max;  % 

args.lbx(24*(N+1)+7:12:24*(N+1)+12*N,1) = un2_min;  %
args.ubx(24*(N+1)+7:12:24*(N+1)+12*N,1) = un2_max;  %

args.lbx(24*(N+1)+8:12:24*(N+1)+12*N,1) = w2_min;  %
args.ubx(24*(N+1)+8:12:24*(N+1)+12*N,1) = w2_max;  %

%% DRONE 3
args.lbx(24*(N+1)+9:12:24*(N+1)+12*N,1) = ul3_min;  %
args.ubx(24*(N+1)+9:12:24*(N+1)+12*N,1) = ul3_max;  %

args.lbx(24*(N+1)+10:12:24*(N+1)+12*N,1) = um3_min;  %
args.ubx(24*(N+1)+10:12:24*(N+1)+12*N,1) = um3_max;  % 

args.lbx(24*(N+1)+11:12:24*(N+1)+12*N,1) = un3_min;  %
args.ubx(24*(N+1)+11:12:24*(N+1)+12*N,1) = un3_max;  %

args.lbx(24*(N+1)+12:12:24*(N+1)+12*N,1) = w3_min;  %
args.ubx(24*(N+1)+12:12:24*(N+1)+12*N,1) = w3_max;  %

end