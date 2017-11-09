function v  = C(x) 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab Function  Author: Ricardo Sanfelice (Revised by Giampiero Campa)
%
% Project: Simulation of a hybrid system (Bouncing ball)
%
% Name: C.m
%
% Description: Flow set
%
% Version: 1.0
% Required files: - 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global er
xsys = x(1:4);
xd = x(5:6);
p = x(7);
r = norm([xsys(1)-xd(1) xsys(2)-xd(2)]);
v=1;
% 
% if (r>=er && p == 2) || (r>= er && p == 1) 
%     v=1;
% end
