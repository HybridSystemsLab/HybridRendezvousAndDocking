function xdot = f2(x)

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

xsys = x(1:4);
h = x(5);
mu = 3.98600444*10^14; ro = 7100*1000;
n = sqrt(mu/ro^3);
A = [0     0    1 0;
     0     0    0 1;
     3*n^2 0    0 2*n;
     0     0 -2*n 0];
m = 1*500;  
B = [0  0; 0  0; 1/m  0;0  1/m];
r = sqrt(x(1)^2+x(2)^2);
Thh = (atan2(x(2),x(1)));
er = 10^-4;

%------------------------------------% 
% position error %                
xi(1) = xsys(1);
xi(2) = xsys(2);
xi(3) = xsys(3);
xi(4) = xsys(4);


%-------------- CONTROLLER Phase-II -------------%
w =n;
an = 179;
k1 = 30; k2 = .1;k3 = 25; k4 = 0.059;
Ths = h*an*2*pi/360;
Th = Thh;

cosThe = cos(Th)*cos(Ths) - sin(Th)*sin(Ths);
sinThe = sin(Th)*cos(Ths) - cos(Th)*sin(Ths);

Te = atan2(sinThe,cosThe);


vth = (-xi(3)*sin(Th)+xi(4)*cos(Th)); %Theta original
Thd = vth/r;

vr = (xi(3)*cos(Th)+xi(4)*sin(Th));  %Rho Original
rd = vr;
ur = -k1*(rd-0)-k2*(r-150);
wr = -((3*w^2*xi(1))+xi(4)*(2*w+Thd))*cos(Th)+xi(3)*(2*w+Thd)*sin(Th);  % omegar original
ar = ur+wr;
%ut = -r*(k3*(Thd-0)+k4*(Th-Ths));
ut = -r*(k3*(Thd-0)+k4*(Te));
wt = ((3*w^2*xi(1))+xi(4)*(2*w+Thd))*sin(Th)+xi(3)*(2*w+Thd)*cos(Th)+vr*Thd;   %omegatheta original
at = ut+wt;
inp = B*[cos(Th) -sin(Th);sin(Th) cos(Th)]*m*[ar;at];    % original input
if norm(inp) > 0.02
    inp = 0.02* (inp/norm(inp));
end

xdot = A*xsys+inp;
hd = 0;
xdot = [xdot;hd];

