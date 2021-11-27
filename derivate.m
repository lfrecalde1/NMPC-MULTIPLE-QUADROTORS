%% Funcion para calcular da rerivada de un vector 
% fx: representa la funcion derivar
% ts: Saple Time
function [fxp,fx2p] = derivate(fx,ts)
[m,n]=size(fx);
fxp(1)=(fx(2)-fx(1))/ts;
fx2p(1)=(fx(1)-2*fx(2)+fx(3))/(ts*ts);
for i=2:n-1 %se trabajan con los puntos intermedios
  fxp(i)=(fx(i+1)-fx(i-1))/(2*ts);
  fx2p(i)=(fx(i-1)-2*fx(i)+fx(i+1))/(ts*ts);
end
% fxp(n)=(fx(n)-fx(n-1))/ts;
fxp(n)=(2*fxp(n-1)-fxp(n-2));
fx2p(n)=(2*fx2p(n-1)-fx2p(n-2));  
end

