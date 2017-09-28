function xdot = f(x)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab Function  Author: Ricardo Sanfelice (Revised by BMAlladi)
%
% Project: Simulation of a hybrid system (Spacecraft R&D)
%
% Name: f.m
%
% Description: Flow map
%
% Version: 1.0
% Required files: - 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global K
xsys = x(1:4);
h = x(5);
p = x(6);
mu = 3.98600444*10^14; ro = 7100*1000;
n = sqrt(mu/ro^3);
A = [0     0    1 0;
     0     0    0 1;
     3*n^2 0    0 2*n;
     0     0 -2*n 0];
m = 2500;  
B = [0  0; 0  0; 1/m  0;0  1/m];
%-----------------Input--------------------%

up = -B*K*(xsys-[0.1;20000;0;0]);
inp = up;
% if norm(up)>0.02
%     inp = 1*0.02*up/norm(up);
% else
%     inp = 1*up;
% end
%----------------------------------------------%
xsysdot = A*xsys+inp;



xdot = [xsysdot;1;0];

