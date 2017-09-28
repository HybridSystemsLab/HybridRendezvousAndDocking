function xplus = g2(x)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab Function  Author: Ricardo Sanfelice (Revised by BMAlladi)
%
% Project: Simulation of a hybrid system (Spacecraft R&D)
%
% Name: g.m
%
% Description: Jump map
%
% Version: 1.0
% Required files: - 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

xsys = x(1:4);
h = x(5);
Th = (atan2(xsys(2),xsys(1)));
an = 179;
Ths = an*2*pi/360;
rho = 10*pi/180;
xsysp = xsys;
er = 10^-4;

if (Th >= rho && h ==-1) || (Th <= -rho && h ==1)
    hp = -h;
end

% if abs(h*(Th-Ths)+rho) <= er
%     hp = -h;
% end


xplus =[xsysp;hp];


