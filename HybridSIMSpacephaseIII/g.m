function xplus = g(x)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab Function  Author: Ricardo Sanfelice (Revised by Giampiero Campa)
%
% Project: Simulation of a hybrid system (Bouncing ball)
%
% Name: g.m
%
% Description: Jump map
%
% Version: 1.0
% Required files: - 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global er K
mu = 3.98600444*10^14; ro = 7100*1000;
n = sqrt(mu/ro^3);
xsys = x(1:4);
xd = x(5:6);
p = x(7);
tau = x(8);
m = 500;
r = norm([xsys(1)-xd(1) xsys(2)-xd(2)]);
B = [0  0; 0  0; 1/m  0;0  1/m];
pp=p;
if r<=er && p==1
    pp = 3-p;
end
xsysp = xsys;
if (tau >=1) && p==1
taup = 0;
up = norm(-B*K*(xsys-[-50;0;0;0]));
else
    taup = 0;
    k_1 = -0.0007;k_2 = -0.15;k_3 = -0.006;k_4 = -0.22;
        ax = 3*n^2*xsys(1)+k_1*xsys(1) +k_2*(xsys(3)-0);
        ay = k_3*xsys(2) +k_4*xsys(4);
        inp = [0 0 ax ay]';
    up = norm(inp);
end

xdp = xd;

xplus =[xsysp;xdp;pp;taup;up];

end

