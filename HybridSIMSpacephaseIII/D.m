function v  = D(x) 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab Function  Author: Ricardo Sanfelice (Revised by Giampiero Campa)
%
% Project: Simulation of a hybrid system (Bouncing ball)
%
% Name: D.m
%
% Description: Jump set
%
% Version: 1.0
% Required files: - 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global er

xsys = x(1:4);
xd = x(5:6);
p = x(7);
tau = x(8);
r = norm([xsys(1)-xd(1) xsys(2)-xd(2)]);
v=0;
if tau >=1
    v=1;
end
if r<er && p==1
    v=1;
end




    