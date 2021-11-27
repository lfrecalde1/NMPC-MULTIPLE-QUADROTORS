function [H0, control] = NMPC_cinematica(h, hd, k, H0, vc, args, solver ,N, obs)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
args.p(1:24) = h;

for i = 1:N
    args.p(24*i+1:24*i+24)=[hd(:,k+i),hd(:,k+i)];
end

aux = 1;
for j =1:3:(size(obs,2)*size(obs,1))
   args.p(24+24*N+(j):24+24*N+(j+2))= obs(:,aux);
   aux =aux+1;
end


args.x0 = [reshape(H0',24*(N+1),1);reshape(vc',size(vc,2)*N,1)]; % initial value of the optimization variables

sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
    'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);

control = reshape(full(sol.x(24*(N+1)+1:end))',12,N)';
H0 = reshape(full(sol.x(1:24*(N+1)))',24,N+1)';
end


