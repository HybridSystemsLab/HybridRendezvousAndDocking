function v  = C2(x) 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab Function  Author: Ricardo Sanfelice (Revised by BMAlladi)
%
% Project: Simulation of a hybrid system (Spacecraft R&D)
%
% Name: C.m
%
% Description: Flow set
%
% Version: 1.0
% Required files: - 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

xsys = x(1:4);
h = x(5);
Th = (atan2(xsys(2),xsys(1)));
an = 179;
Ths = an*pi/180;
rho = 10*pi/180;
er = 10^-3 ;

v = 0;


if (Th <=rho && h==-1) || (Th >=-rho && h==1)
    v = 1;
end

