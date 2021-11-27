%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXX SYSTEM IDENTIFICATION XXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

%% CLEAN VARIABLES
clc,clear all,close all;

%% LOAD VALUES OF DESIRED VELOCITIES
load('Signals.mat')

%%REFERENCE SIGNALS
ul_ref = Signals(1,:);
um_ref = Signals(2,:);
un_ref = Signals(3,:);
w_ref = Signals(4,:);

ts =0.2;
%% DEFINITION OF TIME VARIABLES
t = time_simulation(ul_ref, ts);

%% ROS PARAMETER FOR COMUNICATION
rosshutdown
rosinit('192.168.1.4', 'NodeHost', '192.168.1.4', 'Nodename', '/Matlab_Visual_Servoing');

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


for k=1:1:length(t)-1
    tic; 
    %% OBTAIN CONTROL VALUES OF THE VECTOR
    ul(k) = ul_ref(k);
    um(k) = um_ref(k);
    un(k) = un_ref(k);
    w(k) = w_ref(k);
    
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
%%REFERENCE SIGNALS
ul_ref = Signals(1,:);
um_ref = Signals(2,:);
un_ref = Signals(3,:);
w_ref = Signals(4,:);


figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(4,1,1)
plot(t(1:length(ul)),ul,'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,hp(1,1:length(t)),'--','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
legend({'$\mu_{lc}$','$\mu_{l}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(4,1,2)
plot(t(1:length(ul)),um,'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,hp(2,1:length(t)),'--','Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
legend({'$\mu_{mc}$','$\mu_{m}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(4,1,3)
plot(t(1:length(ul)),un,'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t,hp(3,1:length(t)),'--','Color',[26,115,160]/255,'linewidth',1); hold on
grid on;
legend({'$\mu_{nc}$','$\mu_{n}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(4,1,4)
plot(t(1:length(ul)),w,'Color',[83,57,217]/255,'linewidth',1); hold on
plot(t,hp(4,1:length(t)),'--','Color',[83,57,217]/255,'linewidth',1); hold on
grid on;
legend({'$\omega_{c}$','$\omega$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

%% Save Data
save("Drone_signals.mat", 'hp')
