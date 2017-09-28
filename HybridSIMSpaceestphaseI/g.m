function xplus = g(x)

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
global NOISE1 tnoise

xsys = x(1:4);
ysys = x(5:8);
xhat = x(9:12);
PO1 = x(13:16);
PO2 = x(17:20);
PO3 = x(21:24);
PO4 = x(25:28);
tn = x(31);

tmp = abs(tnoise-tn);
[row col] = min(tmp);
Ynoise = NOISE1(col);


Pkma = [transpose(PO1);transpose(PO2);transpose(PO3);transpose(PO4)];
Pkm = 0.5*(Pkma+transpose(Pkma));

%---- Angle only feedback----%

rho = [xhat(1);xhat(2)];
nrho = norm(rho);

H1 = 1/nrho *eye(2)-rho*transpose(rho)/nrho^3;
H2 = zeros(2,2);

Hkm = [H1 H2];


ys = [ysys(1)/sqrt(ysys(1)^2+ysys(2)^2); ysys(2)/sqrt(ysys(1)^2+ysys(2)^2)]+[(Ynoise); (Ynoise)];
yhat = [xhat(1)/sqrt(xhat(1)^2+xhat(2)^2); xhat(2)/sqrt(xhat(1)^2+xhat(2)^2)];   


Rk  = [((0.001))^2 0;0 ((0.001))^2];





Kk  = Pkm*transpose(Hkm)/(Hkm*Pkm*transpose(Hkm)+Rk);
xsysp = xsys;
ysysp = ysys;
xhatp = xhat+Kk*(ys-yhat);
%----regular update for the covariance matrix--------%
%Ptp   = (eye(4)-Kk*Hkm)*Pkm;
%--------Joseph form to avoid numerical issues with the Covariance matrix
%collapse or go to zero------------------%
Ptp   = (eye(4)-Kk*Hkm)*Pkm*transpose(eye(4)-Kk*Hkm) +Kk*Rk*transpose(Kk) ;
Ptk = reshape(Ptp,[16,1]);
hp = 1;
 


xplus =[xsysp;ysysp;xhatp;Ptk;0;hp;0];

end

