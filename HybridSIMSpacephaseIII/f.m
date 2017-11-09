function xdot = f(x)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab Function  Author: Ricardo Sanfelice (Revised by Giampiero Campa)
%
% Project: Simulation of a hybrid system (Bouncing Ball)
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
xd = x(5:6);
p = x(7);
tau = x(8);
mu = 3.98600444*10^14; ro = 7100*1000;
n = sqrt(mu/ro^3);
A = [0     0    1 0;
     0     0    0 1;
     3*n^2 0    0 2*n;
     0     0 -2*n 0];
m = 1*500;  
B = [0  0; 0  0; 1/m  0;0  1/m];

r = norm([xsys(1) xsys(2)]);

if p==1
    up = -B*K*(xsys-[xd;0;0.0]);
    inp = up;
else if p==2
        k_1 = -0.0007;k_2 = -0.15;k_3 = -0.006;k_4 = -0.22;
        ax = 3*n^2*xsys(1)+k_1*xsys(1) +k_2*(xsys(3)-0);
        ay = k_3*xsys(2) +k_4*xsys(4);
        inp = [0 0 ax ay]';
    end
end
        
xsysdot = A*xsys+inp;
pd = 0;
xddot = [0;0];
taudot = 1;

xdot = [xsysdot;xddot;pd;taudot;0];

