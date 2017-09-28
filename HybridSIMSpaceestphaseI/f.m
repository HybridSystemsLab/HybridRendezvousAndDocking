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
global K NOISEP tnoise
xsys = x(1:4);
ysys = x(5:8);
xhat = x(9:12);
PO1 = x(13:16);
PO2 = x(17:20);
PO3 = x(21:24);
PO4 = x(25:28);
tau = x(29);
h = x(30);
tn = x(31);
mu = 3.98600444*10^14; ro = 7100*1000; rd = sqrt((ro+ysys(1))^2+ysys(2)^2);
n = sqrt(mu/ro^3);
A = [0     0    1 0;
     0     0    0 1;
     3*n^2 0    0 2*n;
     0     0 -2*n 0];
m = 1*500;  % Mass
B = [0  0; 0  0; 1/m  0;0  1/m];
%alp = atan2(xhat(2),xhat(1)); % gives singularity issues... not advisable to use   
%rho = norm(xhat(1),xhat(2));
%   

SOM =  (mu/ro^4)*[0;0;-3*x(1)^2+(3/2*x(2)^2);3*x(1)*x(2)];
SOMhat =  (mu/ro^4)*[0;0;-3*xhat(1)^2+(3/2*xhat(2)^2);3*xhat(1)*xhat(2)];
FNL =  [0;0;-2*n^2*ysys(1)+(mu/ro^2)-((mu/rd^3)*(ro+ysys(1)));n^2*ysys(2)-(mu/rd^3)*ysys(2)];

%------------Noise----------%
    tmp = abs(tnoise-tn);
    [row col] = min(tmp);
    col;
    pnoise = NOISEP(col);  % process noise

up = -1*B*K*xhat-B*m^2*B'*SOMhat;
inp = up;
nrinf = norm(up,inf);
if nrinf > 0.02
    inp = 0.02*(up/nrinf);
end


% state 
xdot = A*xsys+SOM+transpose([0 0 pnoise pnoise])+inp; 
%output
ydot = A*ysys+FNL+inp;
%estimated state
xhatdot = A*xhat+SOMhat+inp;

Ad = (mu/ro^4)*[0 0 0 0;0 0 0 0;-6*xhat(1) 3*xhat(2) 0 0;3*xhat(2) 3*xhat(1) 0 0];

Ft = [0 0 1 0;0 0 0 1;3*n^2 0 0 2*n;0 0 -2*n 0]+Ad;

%-------Ft with input-------%

%Ft = A-B*K;

Pta = [transpose(PO1);transpose(PO2);transpose(PO3);transpose(PO4)];

Pt = 0.5*(Pta+transpose(Pta));

Ptdot = Ft*Pt+Pt*transpose(Ft)+10^-8*[0 0 0 0;0 0 0 0;0 0 1 0;0 0 0 1];

POdot = reshape(Ptdot,[16,1]);


taudot = 1;
tndot = 1;


xdot = [xdot;ydot;xhatdot;POdot;taudot;0;tndot];

