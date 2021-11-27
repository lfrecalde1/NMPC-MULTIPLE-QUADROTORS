%% ESTIMATION FO PARAMETERS DORNE DYNAMIC %%

%% clear variables
clc, clear all, close all;

%% LOAD VALUES FROM MATRICES
load('Signals.mat')
load('Drone_signals')


%% REFERENCE SIGNALS
ul_ref = Signals(1,:);
um_ref = Signals(2,:);
un_ref = Signals(3,:);
w_ref = Signals(4,:);


vref = [ul_ref; um_ref; un_ref; w_ref];
%% SYSTEM TIME
ts =0.2;
t=0:ts:(length(ul_ref)-1)*ts;
N = length(t);
%% SYSTEM SIGNALS
ul = hp(1,1:length(ul_ref));
um = hp(2,1:length(um_ref));
un = hp(3,1:length(un_ref));
w = hp(4,1:length(w_ref));


%% ACELERATION SYSTEM
ulp = [0 , diff(ul)/ts];
ump = [0 , diff(um)/ts];
unp = [0 , diff(un)/ts];
wp = [0 , diff(w)/ts];

vp = [ulp; ump; unp; wp];

landa = 0.1;%lambda
F1=tf(landa,[1 landa]);


% ul=lsim(F1,ul,t)';
% um=lsim(F1,um,t)';
% un=lsim(F1,un,t)';
% w=lsim(F1,w,t)';
% 
% ulp=lsim(F1,ulp,t)';
% ump=lsim(F1,ump,t)';
% unp=lsim(F1,unp,t)';
% wp=lsim(F1,wp,t)';
% 
% 
% ul_ref=lsim(F1,ul_ref,t)';
% um_ref=lsim(F1,um_ref,t)';
% un_ref=lsim(F1,un_ref,t)';
% w_ref=lsim(F1,w_ref,t)';

v = [ul; um; un; w];
vp = [ulp; ump; unp; wp];
vref = [ul_ref; um_ref; un_ref; w_ref];

%% Parametros del optimizador
options = optimset('Display','iter',...
                'TolFun', 1e-8,...
                'MaxIter', 10000,...
                'Algorithm', 'active-set',...
                'FinDiffType', 'forward',...
                'RelLineSrchBnd', [],...
                'RelLineSrchBndDuration', 1,...
                'TolConSQP', 1e-6); 
x0=zeros(1,27)+0.1;           
f_obj1 = @(x)  cost_func_dynamic(x, vref, vp, v, N);
x = fmincon(f_obj1,x0,[],[],[],[],[],[],[],options);
chi = x;
%% SIMULATION DYNAMICS
v_estimate = v;
for k=1:length(t)
    v_estimate(:, k+1) = system_dynamic(x, v_estimate(:,k), vref(:,k), ts);
end
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(4,1,1)
plot(t(1:length(ul_ref)),ul_ref,'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,ul,'--','Color',[226,76,44]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\mu_{lref}$','$\mu_{l}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
%title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(4,1,2)
plot(t(1:length(ul_ref)),um_ref,'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,um,'--','Color',[46,188,89]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\mu_{mref}$','$\mu_{m}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(4,1,3)
plot(t(1:length(ul_ref)),un_ref,'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t,un,'--','Color',[26,115,160]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\mu_{nref}$','$\mu_{n}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(4,1,4)
plot(t(1:length(ul_ref)),w_ref,'Color',[83,57,217]/255,'linewidth',1); hold on
plot(t,w,'--','Color',[83,57,217]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\omega_{ref}$','$\omega$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
print -dpng Data_validation
print -depsc Data_validation

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(4,1,1)
plot(t,v_estimate(1,1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,ul,'--','Color',[226,76,44]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\mu_{lm}$','$\mu_{l}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
%title('$\textrm{Dynamic Model Identification}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(4,1,2)
plot(t,v_estimate(2,1:length(t)),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,um,'--','Color',[46,188,89]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\mu_{mm}$','$\mu_{m}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(4,1,3)
plot(t,v_estimate(3,1:length(t)),'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t,un,'--','Color',[26,115,160]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\mu_{nm}$','$\mu_{n}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m/s]$','Interpreter','latex','FontSize',9);

subplot(4,1,4)
plot(t,v_estimate(4,1:length(t)),'Color',[83,57,217]/255,'linewidth',1); hold on
plot(t,w,'--','Color',[83,57,217]/255,'linewidth',1); hold on
grid('minor')
grid on;
legend({'$\omega_{m}$','$\omega$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
print -dpng Data_model
print -depsc Data_model
save('parameters.mat','chi')