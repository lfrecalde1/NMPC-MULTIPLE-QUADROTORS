%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXX TRAJECTORY CONTROL DJI DRONE XXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

%% CLEAN VARIABLES
clc,clear all,close all;

%% DEFINITION OF TIME VARIABLES
ts = 0.1;
tf = 100;
to = 0;
t = (to:ts:tf);

%% CONSTANTS VALUES OF THE ROBOT 1
a1 = 0.0; 
b1 = 0.0;
c1 = 0.0;
r1 = 0.45*ones(1,length(t));
L1 = [a1, b1, c1, r1(1)];

%% INITIAL CONDITIONS 1
x1 = -5;
y1 = 0.0;
z1 = 0;
yaw1 = 0*(pi/180);

%% DIRECT KINEMATICS 1
x1 = x1 +a1*cos(yaw1) - b1*sin(yaw1);
y1 = y1 +a1*sin(yaw1) + b1*cos(yaw1);
z1 = z1 + c1;

h1 = [x1;...
     y1;...
     z1;...
     yaw1];
 
%% INITIAL GENERALIZE VELOCITIES 1
v1 = [0;...
     0;...
     0;...
     0];
 
%% CONSTANTS VALUES OF THE ROBOT 2
a2 = 0.0; 
b2 = 0.0;
c2 = 0.0;
r2 = 0.45*ones(1,length(t));
L2 = [a2, b2, c2, r2(1)];

%% INITIAL CONDITIONS 2
x2 = 0;
y2 = 0;
z2 = 0;
yaw2 = 0*(pi/180);

%% DIRECT KINEMATICS 2
x2 = x2 +a2*cos(yaw2) - b2*sin(yaw2);
y2 = y2 +a2*sin(yaw2) + b2*cos(yaw2);
z2 = z2 + c2;

h2 = [x2;...
     y2;...
     z2;...
     yaw2];
 
%% INITIAL GENERALIZE VELOCITIES
v2 = [0;...
     0;...
     0;...
     0];
 
%% CONSTANTS VALUES OF THE ROBOT 3
a3 = 0.0; 
b3 = 0.0;
c3 = 0.0;
r3 = 0.45*ones(1,length(t));
L3 = [a3, b3, c3, r3(1)];

%% INITIAL CONDITIONS 3
x3 = 5;
y3 = 0;
z3 = 0;
yaw3 = 0*(pi/180);

%% DIRECT KINEMATICS 3
x3 = x3 +a3*cos(yaw3) - b3*sin(yaw3);
y3 = y3 +a3*sin(yaw3) + b3*cos(yaw3);
z3 = z3 + c3;

h3 = [x3;...
     y3;...
     z3;...
     yaw3];
 
%% INITIAL GENERALIZE VELOCITIES
v3 = [0;...
     0;...
     0;...
     0];
 
H = [h1;h2;h3;v1;v2;v3];

%% DESIRED SIGNALS OF THE SYSYEM 1
[hxd1, hyd1, hzd1, hthd1] =  Trajectory(t,ts,1);

%% GENERALIZED DESIRED SIGNALS 1
hd1 = [hxd1;...
      hyd1;...
      hzd1;...
      hthd1];
  
%% DESIRED SIGNALS OF THE SYSYEM 2
[hxd2, hyd2, hzd2, hthd2] =  Trajectory(t,ts,3);

%% GENERALIZED DESIRED SIGNALS
hd2 = [hxd2;...
      hyd2;...
      hzd2;...
      hthd2];
  
%% DESIRED SIGNALS OF THE SYSYEM 3
[hxd3, hyd3, hzd3, hthd3] =  Trajectory(t,ts,4);

%% GENERALIZED DESIRED SIGNALS
hd3 = [hxd3;...
      hyd3;...
      hzd3;...
      hthd3];

%% COMPLETE TRAJECTORIES
HD = [hd1;...
      hd2;...
      hd3];
  
%% LOAD DYAMIC PARAMETERS DRONE
load("parameters.mat");


%% Definicion del horizonte de prediccion
N = 20; 

%% Definicion de los limites de las acciondes de control
bounded = [1.0; -1.0; 1.0; -1.0; 1.0; -1.0; 1.5; -1.5; 2.0; -2.0; 2.0; -2.0; 2.0; -2.0; 1.5; -1.5; 2.0; -2.0; 2.0; -2.0; 2.0; -2.0; 1.5; -1.5];
%% Definicion del vectro de control inicial del sistema
vc = zeros(N,12);
H0 = repmat(H,1,N+1)';

%% DEFINITION OBJECTS WE NEED THE KNOWLEAGE OF THE OBJECTS
ob1 = [-5.94*ones(1,length(t));...
       2.1*ones(1,length(t));...
       7.94*ones(1,length(t))];  
   
ob3 = [6.99+2*sin(0.3*t);...
       2.28*ones(1,length(t));...
       7.05*ones(1,length(t))];
   
ob4 = [4.05*ones(1,length(t));...
       1.87*ones(1,length(t));...
       7.98*ones(1,length(t))];
   
   
ob6 = [-9+2*sin(0.3*t);...
       1.7*ones(1,length(t));...
       7.1*ones(1,length(t))];
   
ob = [ob1(:,1),ob3(:,1),ob4(:,1),ob6(:,1)];
%% OPTIMIZATION SOLVER
%  [f, solver, args] = mpc_drone(bounded, N, L, chi, ts);
 [f, solver, args] = mpc_drone_cinematica(bounded, N, L1, L2, L3, chi, ob, ts);
%% SIMULATION 

%% DISTANCE TO OBJECTS
[dh1] = distance_objects(ob, h1);
[dh2] = distance_objects(ob, h2);
[dh3] = distance_objects(ob, h3);

[dh1h2] = distance_objects(h1, h2);
[dh1h3] = distance_objects(h1, h3);
[dh3h2] = distance_objects(h3, h2);


for k=1:1:length(t)-N
    tic; 
    %% GENERAL VECTOR OF ERROR SYSTEM
    he1(:, k) = hd1(1:4,k)-h1(1:4,k);
    he2(:, k) = hd2(1:4,k)-h2(1:4,k);
    he3(:, k) = hd3(1:4,k)-h3(1:4,k);
    
    %% GENERAL SYSTEM
    H = [h1(:,k);h2(:,k);h3(:,k);v1(:,k);v2(:,k);v3(:,k)];
%     [H0, control] = NMPC(h(:,k), v(:,k), hd(:,:), k, H0, vc, args, solver, N);
    [H0, control] = NMPC_cinematica(H, HD, k, H0, vc, args, solver, N, ob);

    %% OBTAIN CONTROL VALUES OF THE VECTOR
    ul1(k) = control(1,1);
    um1(k) = control(1,2);
    un1(k) = control(1,3);
    w1(k) = control(1,4);
    
    ul2(k) = control(1,5);
    um2(k) = control(1,6);
    un2(k) = control(1,7);
    w2(k) = control(1,8);
    
    ul3(k) = control(1,9);
    um3(k) = control(1,10);
    un3(k) = control(1,11);
    w3(k) = control(1,12);
    
    %% GET VALUES OF DRONE
    %% Drone 1
    v1(:,k+1) = system_dynamic(chi, v1(:,k), control(1,1:4)', ts);
    [h1(:,k+1), hp1(:,k+1)] = system_drone(h1(:,k), v1(:,k), ts, L1);
    
    %% DRONE 2
    v2(:,k+1) = system_dynamic(chi, v2(:,k), control(1,5:8)', ts);
    [h2(:,k+1), hp2(:,k+1)] = system_drone(h2(:,k), v2(:,k), ts, L2);
    
    %% DRONE 3
    v3(:,k+1) = system_dynamic(chi, v3(:,k), control(1,9:12)', ts);
    [h3(:,k+1), hp3(:,k+1)] = system_drone(h3(:,k), v3(:,k), ts, L3);
    
    
    %% OBJECTS
    ob = [ob1(:,k+1),ob3(:,k+1),ob4(:,k+1),ob6(:,k+1)];
    
    dh1(:,k+1) = distance_objects(ob, h1(:,k+1));
    dh2(:,k+1) = distance_objects(ob, h2(:,k+1));
    dh3(:,k+1) = distance_objects(ob, h3(:,k+1));
    
    dh1h2(:,k+1) = distance_objects(h1(:,k+1), h2(:,k+1));
    dh1h3(:,k+1) = distance_objects(h1(:,k+1), h3(:,k+1));
    dh3h2(:,k+1) = distance_objects(h2(:,k+1), h3(:,k+1));
    %% NEW VALUES OPTIMAL CONTROL
    vc = [control(2:end,:);control(end,:)];
    H0 = [H0(2:end,:);H0(end,:)];
    
    %% SAMPLE TIME
    t_sample(k) = toc;
    toc;
end

%%
close all; paso=1; 
%a) Parámetros del cuadro de animación
figure
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [4 2]);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperPosition', [0 0 8 3]);
myVideo = VideoWriter('myVideoFile'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)
luz = light;
luz.Color=[0.65,0.65,0.65];
luz.Style = 'infinite';
%b) Dimenciones del Robot
   Drone_Parameters(0.02);
%c) Dibujo del Robot 1
    G1=Drone_Plot_3D(h1(1,1),h1(2,1),h1(3,1),0,0,h1(4,1));hold on

    plot3(h1(1,1),h1(2,1),h1(3,1),'--','Color',[56,171,217]/255,'linewidth',1.3);hold on,grid on   
    plot3(hxd1(1),hyd1(1),hzd1(1),'Color',[32,185,29]/255,'linewidth',1.3);
    
    %c) Dibujo del Robot 2
    G2=Drone_Plot_3D(h2(1,1),h2(2,1),h2(3,1),0,0,h2(4,1));hold on

    plot3(h2(1,1),h2(2,1),h2(3,1),'--','Color',[219,177,79]/255,'linewidth',1.3);hold on,grid on   
    plot3(hxd2(1),hyd2(1),hzd2(1),'Color',[158,62,158]/255,'linewidth',1.3);
    
    %c) Dibujo del Robot 3
    G3=Drone_Plot_3D(h3(1,1),h3(2,1),h3(3,1),0,0,h3(4,1));hold on
    
    plot3(h3(1,1),h3(2,1),h3(3,1),'--','Color',[108,105,105]/255,'linewidth',1.3);hold on,grid on   
    plot3(hxd3(1),hyd3(1),hzd3(1),'Color',[198,87,67]/255,'linewidth',1.3);
    
    G_ob1 = plot3(ob1(1,1), ob1(2,1), ob1(3,1),'*','Color',[15,12,15]/255,'linewidth',1.3);
    G_ob3 = plot3(ob3(1,1), ob3(2,1), ob3(3,1),'*','Color',[15,12,15]/255,'linewidth',1.3);
    G_ob4 = plot3(ob4(1,1), ob4(2,1), ob4(3,1),'*','Color',[15,12,15]/255,'linewidth',1.3);
    G_ob6 = plot3(ob6(1,1), ob6(2,1), ob6(3,1),'*','Color',[15,12,15]/255,'linewidth',1.3);
%axis([-12 12 -12 15])
    
view([20 15])
% view([0 90])
for k = 1:30:length(t)-N
    drawnow
    delete(G1);
    delete(G2);
    delete(G3);
    delete(G_ob1);
    delete(G_ob3);
    delete(G_ob4);
    delete(G_ob6);
    
    G1=Drone_Plot_3D(h1(1,k),h1(2,k),h1(3,k),0,0,h1(4,k));hold on
    G2=Drone_Plot_3D(h2(1,k),h2(2,k),h2(3,k),0,0,h2(4,k));hold on
    G3=Drone_Plot_3D(h3(1,k),h3(2,k),h3(3,k),0,0,h3(4,k));hold on
    G_ob1 = plot3(ob1(1,k), ob1(2,k), ob1(3,k),'*','Color',[15,12,15]/255,'linewidth',1.3);
    G_ob3 = plot3(ob3(1,k), ob3(2,k), ob3(3,k),'*','Color',[15,12,15]/255,'linewidth',1.3);
    G_ob4 = plot3(ob4(1,k), ob4(2,k), ob4(3,k),'*','Color',[15,12,15]/255,'linewidth',1.3);
    G_ob6 = plot3(ob6(1,k), ob6(2,k), ob6(3,k),'*','Color',[15,12,15]/255,'linewidth',1.3);
    
    
    plot3(hxd1(1:k),hyd1(1:k),hzd1(1:k),'Color',[32,185,29]/255,'linewidth',1.3);
    plot3(h1(1,1:k),h1(2,1:k),h1(3,1:k),'--','Color',[56,171,217]/255,'linewidth',1.3);
    
    plot3(hxd2(1:k),hyd2(1:k),hzd2(1:k),'Color',[158,62,158]/255,'linewidth',1.3);
    plot3(h2(1,1:k),h2(2,1:k),h2(3,1:k),'--','Color',[219,177,79]/255,'linewidth',1.3);
    
    plot3(hxd3(1:k),hyd3(1:k),hzd3(1:k),'Color',[198,87,67]/255,'linewidth',1.3);
    plot3(h3(1,1:k),h3(2,1:k),h3(3,1:k),'--','Color',[108,105,105]/255,'linewidth',1.3);
    
    legend({'$\eta^{1}$','$\eta^{1}_{ref}$','$\eta^{2}$','$\eta^{2}_{ref}$','$\eta^{3}$','$\eta^{3}_{ref}$'},'Interpreter','latex','FontSize',11,'Location','northwest','Orientation','horizontal');legend('boxoff')
    title('$\textrm{Movement Executed by the Aerial Robots}$','Interpreter','latex','FontSize',11);
    xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
end
close(myVideo)
print -dpng SIMULATION_1
print -depsc SIMULATION_1

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(he1)),he1(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1:length(he1)),he1(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t(1:length(he1)),he1(3,:),'Color',[26,115,160]/255,'linewidth',1);hold on;
plot(t(1:length(he1)),he1(4,:),'Color',[83,57,217]/255,'linewidth',1);hold on;
grid('minor')
grid on;
legend({'$\tilde{\eta}^{1}_{x}$','$\tilde{\eta}^{1}_{y}$','$\tilde{\eta}^{1}_{z}$','$\tilde{\eta}^{1}_{\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(he2)),he2(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1:length(he2)),he2(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t(1:length(he2)),he2(3,:),'Color',[26,115,160]/255,'linewidth',1);hold on;
plot(t(1:length(he2)),he2(4,:),'Color',[83,57,217]/255,'linewidth',1);hold on;
grid('minor')
grid on;
legend({'$\tilde{\eta}^{2}_{x}$','$\tilde{\eta}^{2}_{y}$','$\tilde{\eta}^{2}_{z}$','$\tilde{\eta}^{2}_{\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(he3)),he3(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1:length(he3)),he3(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t(1:length(he3)),he3(3,:),'Color',[26,115,160]/255,'linewidth',1);hold on;
plot(t(1:length(he3)),he3(4,:),'Color',[83,57,217]/255,'linewidth',1);hold on;
grid('minor')
grid on;
legend({'$\tilde{\eta}^{3}_{x}$','$\tilde{\eta}^{3}_{y}$','$\tilde{\eta}^{3}_{z}$','$\tilde{\eta}^{3}_{\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(ul1)),ul1,'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(ul1)),um1,'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t(1:length(ul1)),un1,'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t(1:length(ul1)),w1,'Color',[83,57,217]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\mu^{1}_{lref}$','$\mu^{1}_{mref}$','$\mu^{1}_{nref}$','$\omega^{1}_{ref}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(ul2)),ul2,'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(ul2)),um2,'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t(1:length(ul2)),un2,'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t(1:length(ul2)),w2,'Color',[83,57,217]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\mu^{2}_{lref}$','$\mu^{2}_{mref}$','$\mu^{2}_{nref}$','$\omega^{2}_{ref}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(ul3)),ul3,'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(ul3)),um3,'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t(1:length(ul3)),un3,'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t(1:length(ul3)),w3,'Color',[83,57,217]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\mu^{3}_{lref}$','$\mu^{3}_{mref}$','$\mu^{3}_{nref}$','$\omega^{3}_{ref}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(t_sample)),t_sample,'Color',[108,105,105]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$t_{s}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);  
print -dpng SAMPLE_TIME
print -depsc SAMPLE_TIME

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(dh1)),dh1(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(dh1)),dh1(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t(1:length(dh1)),dh1(3,:),'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t(1:length(dh1)),dh1(4,:),'Color',[83,57,217]/255,'linewidth',1); hold on
plot(t(1:length(dh1)),r1(1,1:length(dh1)),'--','Color',[15,12,15]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$l^{1}_{obs_1}$','$l^{1}_{obs_2}$','$l^{1}_{obs_3}$','$l^{1}_{obs_4}$','$r^{1}_{obs}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff');
ylabel('$[m]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);