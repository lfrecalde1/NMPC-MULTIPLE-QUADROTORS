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

%% ROS PARAMETER FOR COMUNICATION
rosshutdown
rosinit('192.168.1.2', 'NodeHost', '192.168.1.2', 'Nodename', '/Matlab_Visual_Servoing');

%% OBJECTS CREATION OF TOPICS ROS
robot = rospublisher('/Mavic_2_PRO/cmd_vel');
velmsg = rosmessage(robot);
odom = rossubscriber('/Mavic_2_PRO/odom');

%% CONSTANTS VALUES OF THE ROBOT
a = 0.0; 
b = 0.0;
c = 0.0;
L = [a, b, c];

%% READ VALUES FOR INITIAL CONDITIONS
[h, hp] = odometry(odom, L);

%% DESIRED SIGNALS OF THE SYSYEM
hxd = 0.25*t+2;
hxdp = 0.25*ones(1,length(t));
hxdpp = 0*ones(1,length(t));

hyd = 2*sin(t/8)+0.05*t-4;
hydp = (1/8)*2*cos(t/8)+0.05;
hydpp = -(1/8)*(1/8)*2*sin(t/8);

hzd = 10+1.5*sin(t/10);
hzdp = 1.5*(1/10)*cos(t/10);
hzdpp = -1.5*(1/10)*1/10*sin(t/10);

hthd = (atan2(hydp,hxdp));
hthdp = diff([0 hthd])/ts;

%% GENERALIZED DESIRED SIGNALS
hd = [hxd;...
      hyd;...
      hzd;...
      hthd];
  
hdp = [hxdp;...
       hydp;...
       hzdp;...
       hthdp];
  
%% GAIN MATRICES OF THE SISTEM
K1 = 1;
K2 = 1;
%% SIMULATION 

for k=1:1:length(t)
    tic; 
    %% GENERAL VECTOR OF ERROR SYSTEM
    he(:, k) = hd(:,k)-h(:,k);
    
    %% GENERALIZED CONTROL LAW
    control = controller(h(:,k), hd(:,k), hdp(:,k), K1, K2, L);
    
    %% OBTAIN CONTROL VALUES OF THE VECTOR
    ul(k) = control(1);
    um(k) = control(2);
    un(k) = control(3);
    w(k) = control(4);
    
    %% SEND VALUES OF CONTROL ROBOT
    send_velocities(robot, velmsg, [ul(k), um(k), un(k), 0, 0 , w(k)]);
    
    %% GET VALUES OF DRONE
    [h(:,k+1), hp(:,k+1)] = odometry(odom, L);
    
    while(toc<ts)
    end
    toc;
end
%% SET VALUES TO ZERO ON THE DESIRED VELOCITIES
send_velocities(robot, velmsg, [0, 0, 0, 0, 0 , 0])
rosshutdown;

close all; paso=1; 
%a) Par??metros del cuadro de animaci??n
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 8 3]);
luz = light;
luz.Color=[0.65,0.65,0.65];
luz.Style = 'infinite';
%b) Dimenciones del Robot
   Drone_Parameters(0.02);
%c) Dibujo del Robot    
    G2=Drone_Plot_3D(h(1,1),h(2,1),h(3,1),0,0,h(4,1));hold on

    plot3(h(1,1),h(2,1),h(3,11),'--','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on   
    plot3(hxd(1),hyd(1),hzd(1),'Color',[32,185,29]/255,'linewidth',1.5);

view(20,15);
for k = 1:30:length(t)
    drawnow
    delete(G2);
   
    G2=Drone_Plot_3D(h(1,k),h(2,k),h(3,k),0,0,h(4,k));hold on
    
    plot3(hxd(1:k),hyd(1:k),hzd(1:k),'Color',[32,185,29]/255,'linewidth',1.5);
    plot3(h(1,1:k),h(2,1:k),h(3,1:k),'--','Color',[56,171,217]/255,'linewidth',1.5);
    
    legend({'$\mathbf{h}$','$\mathbf{h}_{des}$'},'Interpreter','latex','FontSize',11,'Location','northwest','Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Movement Executed by the Aerial Robot}$','Interpreter','latex','FontSize',11);
    xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
    
end
print -dpng SIMULATION_1
print -depsc SIMULATION_1

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(he)),he(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1:length(he)),he(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t(1:length(he)),he(3,:),'Color',[26,115,160]/255,'linewidth',1);hold on;
plot(t(1:length(he)),he(4,:),'Color',[83,57,217]/255,'linewidth',1);hold on;
grid on;
legend({'$\tilde{h_{x}}$','$\tilde{h_{y}}$','$\tilde{h_{z}}$','$\tilde{h_{\psi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolution of Control Errors}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(ul)),ul,'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(ul)),um,'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t(1:length(ul)),un,'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t(1:length(ul)),w,'Color',[83,57,217]/255,'linewidth',1); hold on
plot(t,hp(1,1:length(t)),'--','Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,hp(2,1:length(t)),'--','Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,hp(3,1:length(t)),'--','Color',[26,115,160]/255,'linewidth',1); hold on
plot(t,hp(4,1:length(t)),'--','Color',[83,57,217]/255,'linewidth',1); hold on
grid on;
legend({'$\mu_{lc}$','$\mu_{mc}$','$\mu_{nc}$','$\omega_{c}$','$\mu_{l}$','$\mu_{m}$','$\mu_{n}$','$\omega$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Control Values}$','Interpreter','latex','FontSize',9);
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
