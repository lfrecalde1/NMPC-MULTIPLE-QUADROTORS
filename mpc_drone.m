 function [f,solver,args] = mpc_drone(bounded, N, L, x, ts)

addpath('/home/fer/casadi-linux-matlabR2014b-v3.4.5');
import casadi.*;

%% Dinamic Parameters
x1 = x(1);
x2 = x(2);
x3 = x(3);
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
ul_max = bounded(1); 
ul_min = bounded(2);

um_max = bounded(3);
um_min = bounded(4);

un_max = bounded(5);
un_min = bounded(6);

w_max = bounded(7); 
w_min = bounded(8);

%% Generacion de las variables simbolicas de los estados del sistema
x = SX.sym('x'); 
y = SX.sym('y');
z = SX.sym('z');
th = SX.sym('th');
ul = SX.sym('ul');
um = SX.sym('um');
un = SX.sym('un');
w = SX.sym('w');

%% Definicion de cuantos estados en el sistema
states = [x;y;z;th;ul;um;un;w];
n_states = length(states);

%% Generacion de las variables simbolicas de las acciones del control del sistema
ul_ref = SX.sym('ul_ref');
um_ref = SX.sym('um_ref');
un_ref = SX.sym('un_ref');
w_ref = SX.sym('w_ref');

%% Defincion de cuantas acciones del control tiene el sistema
controls = [ul_ref;um_ref;un_ref;w_ref]; 
n_control = length(controls);

%% Definicion de los las constantes dl sistema
a = L(1);
b = L(2);

%% Defincion del sistema pero usando espacios de estados todo el sistema de ecuaciones
J = [cos(th), -sin(th), 0, -(a*sin(th)+b*cos(th));...
     sin(th), cos(th), 0,  (a*cos(th)-b*sin(th));...
     0, 0, 1, 0;...
     0, 0, 0, 1]; 
 
%% INERTIAL MATRIX
M_1 = [x6/(x1*x6 - x2*x5), 0, 0, -x2/(x1*x6 - x2*x5);...
       0, 1/x3, 0, 0;...
       0, 0, 1/x4, 0;...
      -x5/(x1*x6 - x2*x5), 0, 0, x1/(x1*x6 - x2*x5)];

%% CENTRIFUGAL FORCES
C = [x7, x8 + w*x9, x10, x11;...
    x12 + w*x13, x14, x15, x16 + w*x17;...
    x18, x19, x20, x21;...
    x22, x23 + w*x24, x25, x26];

G = [0; 0; x27; 0];

A = [zeros(4,4),J;...
     zeros(4,4),-M_1*C];
 
B = [zeros(4,4);...
    M_1];

aux = [zeros(4,1);...
       -M_1*G];
rhs=(A*states+aux+B*controls);

%% Definicion de kas funciones del sistema
f = Function('f',{states,controls},{rhs}); 
U = SX.sym('U',n_control,N);
P = SX.sym('P',n_states + N*(n_states));
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
g = [g;X(:,1)-P(1:8)]; % initial condition constraints

%% Definicon del bucle para optimizacion a los largo del tiempo
for k = 1:N
    st = X(:,k);  con = U(:,k);

    %% Funcion costo a minimizar 
    he = [he;X(1:4,k)-P(8*k+1:8*k+4)];
    u = [u;con];
    %obj = obj+(st-P(4*k+1:4*k+4))'*Q*(st-P(4*k+1:4*k+4)) + con'*R*con;
    
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

% se crea el vector de desiscion solo de una columna
OPT_variables = [reshape(X,8*(N+1),1);reshape(U,4*N,1)];

nlprob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlprob,opts);

args = struct;

args.lbg(1:8*(N+1)) = 0;  %-1e-20  %Equality constraints
args.ubg(1:8*(N+1)) = 0;  %1e-20   %Equality constraints

args.lbx(1:8:8*(N+1),1) = -inf; %state x lower bound
args.ubx(1:8:8*(N+1),1) = inf;  %state x upper bound

args.lbx(2:8:8*(N+1),1) = -inf; %state y lower bound
args.ubx(2:8:8*(N+1),1) = inf;  %state y upper bound

args.lbx(3:8:8*(N+1),1) = -inf; %state z lower bound
args.ubx(3:8:8*(N+1),1) = inf;  %state z upper bound

args.lbx(4:8:8*(N+1),1) = -inf; %state theta lower bound
args.ubx(4:8:8*(N+1),1) = inf;  %state theta upper bound

args.lbx(5:8:8*(N+1),1) = -inf; %state x lower bound
args.ubx(5:8:8*(N+1),1) = inf;  %state x upper bound

args.lbx(6:8:8*(N+1),1) = -inf; %state y lower bound
args.ubx(6:8:8*(N+1),1) = inf;  %state y upper bound

args.lbx(7:8:8*(N+1),1) = -inf; %state z lower bound
args.ubx(7:8:8*(N+1),1) = inf;  %state z upper bound

args.lbx(8:8:8*(N+1),1) = -inf; %state theta lower bound
args.ubx(8:8:8*(N+1),1) = inf;  %state theta upper bound


%% Definicion de las restricciones de las acciones de control del sistema
args.lbx(8*(N+1)+1:4:8*(N+1)+4*N,1) = ul_min;  %
args.ubx(8*(N+1)+1:4:8*(N+1)+4*N,1) = ul_max;  %

args.lbx(8*(N+1)+2:4:8*(N+1)+4*N,1) = un_min;  %
args.ubx(8*(N+1)+2:4:8*(N+1)+4*N,1) = un_max;  % 

args.lbx(8*(N+1)+3:4:8*(N+1)+4*N,1) = um_min;  %
args.ubx(8*(N+1)+3:4:8*(N+1)+4*N,1) = um_max;  %

args.lbx(8*(N+1)+4:4:8*(N+1)+4*N,1) = w_min;  %
args.ubx(8*(N+1)+4:4:8*(N+1)+4*N,1) = w_max;  %

end