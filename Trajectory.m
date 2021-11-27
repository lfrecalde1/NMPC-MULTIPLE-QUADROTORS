%% ************TRAYECOTRIAS EN XYZ *******************
% [Pox,Pox_p,Pox_2p,Poy,Poy_p,Poy_2p,Poz,Poz_p,Poz_2p,Popsi,Popsi_p]=Trajectory(t,ts,trayectoria);
function [x,y,z,psi] = Trajectory(t,ts,n)
switch n
    case 1 %silla
        no=1.5*2;
        x = 5*cos(0.05*no*t)-0.95;       xp = -5*0.05*no*sin(0.05*no*t);      xpp = -5*0.05*0.05*no*no*cos(0.05*no*t);
        y = 5*sin(0.05*no*t)+1.93;       yp =  5*0.05*no*cos(0.05*no*t);      ypp = -5*0.05*0.05*no*no*sin(0.05*no*t);
        z = 0.5*sin(0.1*no*t)+8;      zp =  0.5*0.1*no*cos(0.1*no*t);      zpp = -0.5*0.1*0.1*no*no*sin(0.1*no*t);
        
        psi = 45*(pi/180)*(ones(1,length(t)));
%         psi= atan2(yp,xp);
%         psip = (1./((yp./xp).^2+1)).*((ypp.*xp-yp.*xpp)./xp.^2);
%         psip(1)=0;
%         psip=(xp.*ypp - yp.*xpp)./(xp.^2 + yp.^2); %velangulo orientacion
                
    case 2 
        %Trayectoria circular en Y Z
        tt=t;
        no=0.04*4;
        x = 8*sin(no*tt)+5;
        y = 6*cos(no*tt)-4;
        z = 0.75*cos(0.2*4*tt)+10;
        
        [xp,xpp]=derivate(x,ts);
        [yp,ypp]=derivate(y,ts);
        [zp,zpp]=derivate(z,ts);  
        psi = 45*(pi/180)*(ones(1,length(t)));
%         psi= atan2(yp,xp);
%         psip = (1./((yp./xp).^2+1)).*((ypp.*xp-yp.*xpp)./xp.^2);
%         psip(1)=0;
%         psip=(xp.*ypp - yp.*xpp)./(xp.^2 + yp.^2); %velangulo orientacion

    case 3      %circulo con z variaciones
      
        tt=t;
        no=0.07*2.5;
        y = 12*cos(no*tt)+2;                              %Posici�n x
        x = 4*sin(2*no*tt)-1;
        z = 0.5*sin(3*no*tt)+7;
        [xp,xpp]=derivate(x,ts);
        [yp,ypp]=derivate(y,ts);
        [zp,zpp]=derivate(z,ts);
        psi = 45*(pi/180)*(ones(1,length(t)));
%         psi= atan2(yp,xp);
%         psip = (1./((yp./xp).^2+1)).*((ypp.*xp-yp.*xpp)./xp.^2);
%         psip(1)=0;
%         psip=(xp.*ypp - yp.*xpp)./(xp.^2 + yp.^2); %velangulo orientacion 
    case 4
        tt=t;
        no=0.07*2.5;
        y = 4*sin(2*no*tt)+2;                              %Posici�n x
        x = 8*cos(no*tt)-1;
        z = 0.5*sin(3*no*tt)+7;
        [xp,xpp]=derivate(x,ts);
        [yp,ypp]=derivate(y,ts);
        [zp,zpp]=derivate(z,ts);  
        psi = 45*(pi/180)*(ones(1,length(t)));
%         psi= atan2(yp,xp);
%         psip = (1./((yp./xp).^2+1)).*((ypp.*xp-yp.*xpp)./xp.^2);
%         psip(1)=0;
%         psip=(xp.*ypp - yp.*xpp)./(xp.^2 + yp.^2); %velangulo orientacio
    case 5
        tt=t;
        no=0.1*2;
        mo=50;
        x = 2*tt/mo.*cos(1.2*no*tt)-2;
        y = 3.5*tt/mo.*sin(1.2*no*tt)+1;
        z = 2*tt/mo+5;
        [xp,xpp]=derivate(x,ts);
        [yp,ypp]=derivate(y,ts);
        [zp,zpp]=derivate(z,ts);
        psi = 45*(pi/180)*(ones(1,length(t)));
%         psi= atan2(yp,xp);
%         psip = (1./((yp./xp).^2+1)).*((ypp.*xp-yp.*xpp)./xp.^2);
%         psip(1)=0;
%         psip=(xp.*ypp - yp.*xpp)./(xp.^2 + yp.^2); %velangulo orientacion
end

