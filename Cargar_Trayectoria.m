% Cargar trayectoria
ts=.1;  tfin=100; 
t=[0:ts:tfin];


[Pox,Pox_p,Pox_2p,Poy,Poy_p,Poy_2p,Poz,Poz_p,Poz_2p,Popsi,Popsi_p]=Trajectory(t,ts,3);


figure(1)
plot3(Pox,Poy,Poz);grid on

figure (2)
subplot(3,1,1)
plot(Pox_p);grid on
subplot(3,1,2)
plot(Poy_p);grid on
subplot(3,1,3)
plot(Poz_p);grid on