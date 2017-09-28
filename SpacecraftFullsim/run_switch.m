%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file       Project: HyEQ Toolbox  @ Hybrid Dynamics and Control
% Lab, http://www.u.arizona.edu/~sricardo/index.php?n=Main.Software
%
% Filename: run_switch.m
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global K1 K3 er xd K4 Ts NOISE1 tnoise NOISE2p NOISE3p NOISEP
er = 1;
xd = [-25;0];
Ts = 10;

%-----------------NOISE-----------%
N=150000;
Fs = 1000;
tnoise = (0:N-1)/Fs;
sigma = 1;
NOISE1 = (0.001)*sigma*randn(size(tnoise));
NOISE2p = (10)*sigma*randn(size(tnoise));
NOISE3p = (1/100)*sigma*randn(size(tnoise));
NOISEP = (10^-4)*sigma*randn(size(tnoise));
%---------------------------------------
mu = 3.98600444*10^14; ro = 7100*1000;
n = sqrt(mu/ro^3); 
A = [0     0    1 0;
     0     0    0 1;
     3*n^2 0    0 2*n;
     0     0 -2*n 0];
m = 0.5*1000;  
B = [0  0; 0  0; 1/m  0;0  1/m];
m2 = 2500;  
B2 = [0  0; 0  0; 1/m2  0;0  1/m2];

%xi = [1000 0 -0.1 1.69];
%xi = [0.0 2000 0.06639 74.95];
%     xi = [-6550.19 -5045.37 0.06639 .5];
%xi = [0 -5045.37 0.06639 7.495];
%     xi = [6550.19 5045.37 0.5 0.5];
%      xi = [5045.37 - 5045.37 0.5 .5];
%    xi = [-170 600 0.05 0.05];
%        xi = [-10000 0 0.5 0.5];
    xi = [0 -10000 0.5 0.5];
%     xi = [-120 0 0.05 0.05];
%-----------------------% Phase-I
% Q1 = 6e-1*eye(4);
%   R1 = 9.8e4*eye(2); 
Q1 = 1.5e-1*eye(4);
R1 = 18e5*eye(2); 
[K1,s,e] = lqr(A,B,Q1,R1);

%-------------------------- Estimation
% po = [5*10^2 0 0 0;
%       0 5*10^2 0 0;
%       0 0 .5*10^-1 0;
%       0 0 0 .5*10^-1];

po = [1*10^5 0 0 0;
      0 1*10^5 0 0;
      0 0 1*10^1 0;
      0 0 0 1*10^1];
  
poi = reshape(po,[1,16]);

%-----------------------------% Phase - III
Q3 = 38.4*eye(4);
R3 = 9.7e3*eye(2);
[K3,s,e] = lqr(A,B,Q3,R3);
%---------------------------% Phase -IV
  Q4 = 6e-1*eye(4);
  R4 = 11e4*eye(2);
[K4,s,e] = lqr(A,B2,Q4,R4);


%-------------------------
hint = -1;
pint = 1;
qint = 1;
xhati = xi + [1000 1000 0 0];
tau = 0;
yint = xi;
xint = [xi hint pint qint xhati tau poi yint]; 

  

x0 = xint;


% simulation horizon
T = [0 20000];                                                                
J = [0 10000];

% rule for jumps
% rule = 1 -> priority for jumps
% rule = 2 -> priority for flows
rule = 1;

options = odeset('RelTol',1e-6,'MaxStep',.1);

% simulate
[t j x] = HyEQsolver( @f,@g,@C,@D,x0',T,J,rule,options);

% if xi(1)<0
%     param1 = ['m',num2str(round(abs(xi(1))))];
% else
%     param1 = num2str(round(abs(xi(1))));
% end
% 
% if xi(2)<0
%     param2 = ['m',num2str(round(abs(xi(2))))];
% else
%     param2 = num2str(round(abs(xi(2))));
% end
% save(['ic_',param1,'_',param2,'.mat'])

%%
%plot hybrid arc   
figure(11)
plot(t,x(:,1)/1000); 
hold on
plot(t,x(:,2)/1000,'r');
grid on
xlabel('time (sec)')                    
ylabel('x,y (KM)')  
legend('x','y')

%%
%plot hybrid arc   
figure(12)
plot(t,x(:,3)/1000); 
hold on
plot(t,x(:,4)/1000,'r');
grid on
xlabel('time (sec)')                    
ylabel('$\dot x$,$\dot y$ (KM/sec)')                    
%zlabel('x1')    
legend('$\dot x$','$\dot y$')
%%
clear xnom
for i= 1:1:length(t)
  xnom(i,:)= norm([x(i,1)/1000 x(i,2)/1000],2);
end
figure(3)  
plot(t,xnom)
xlabel('time(sec)')
ylabel('\rho (KM)')
grid on
hold on
%%
figure(80)
set(gca,'Xdir','reverse')
sp = 1:1000:length(t);
plot(x(sp,2)/1000,x(sp,1)/1000,'b')
hold on
% plot(x(:,9)/1000,x(:,8)/1000,'b')
% hold on
angle = linspace(0,2*pi,360);
xc = 1000*cos(angle);
yc = 1000*sin(angle);
plot(xc/100,yc/100,'k')
plot(xc/1000,yc/1000,'g')
plot(xc/10000,yc/10000,'m')
cony = -linspace(0,1,500);
conxr = cony/tan(105*2*pi/360);
conxl = cony/tan(75*2*pi/360);
hold on
plot(conxl,cony,'b')
plot(conxr,cony,'b')
plot(0,0,'square')
xlabel('y-position in KM')
ylabel('x-position in KM')
legend('xy position','Target')
grid on
hold on
plot(30,0,'+','linewidth',5)
hold on
plot(20,0,'m o','linewidth',3)
hold on
plot(x(1,2)/1000,x(1,1)/1000,'r*','linewidth',5)
%%
clear xnom
%clear xnomi
for i= 1:1:length(t)
  xnom(i,:)= norm([x(i,1) x(i,2) x(i,3) x(i,4)]',2);
  unom(i,:)= norm(-B2*K1*[x(i,1) x(i,2) x(i,3) x(i,4)]',2);
  unomi(i,:)= norm(-B2*K1*[x(i,1) x(i,2) x(i,3) x(i,4)]',inf);
end
%%
figure(15)  
plot(t,unom)
 %hold on
 %plot(t,xnomi/m,'r');
grid on
hold on
plot(t,0.02*ones(size(t)),'r')
xlabel('time(sec)')
ylabel('$\frac{1}{m}|u|$ (m/sec$^2$)')
title('Control effor = $\int^t_0 |u|\ dt$ =  128.68 (m/sec): Calculated using trapz(X,Y)')
legend('calculated input','Max input')
CF = trapz(t,unomi)

%%
% %Animation
% 
% figure(64)
% plot(x(:,2),x(:,1))
% hold on
% p = plot(x(1,9),x(1,10),'o','MarkerFaceColor','red');
% hold off
% %axis manual
% for k = 2:length(x(:,1))
%     p.XData = x(k,9);
%     p.YData = x(k,10);
%     drawnow
% end
%%
for i= 1:1:length(t)
    if x(i,6) ==1
        tp1(i,:) = t(i,:);
        unomfull1(i,:)= norm(-B*K1*[x(i,1) x(i,2) x(i,3) x(i,4)]',inf);
        if norm(unomfull1(i,:)) > 0.02
            unomfull1(i,:) = 0.02;
        end
    end
    if x(i,6) == 2
        tp2(i,:) = t(i,:);
        mu = 3.98600444*10^14; ro = 7100*1000;
        n = sqrt(mu/ro^3);
        r = sqrt(x(i,1)^2+x(i,2)^2);
        Th = (atan2(x(i,2),x(i,1)));
        w =n;
        an = 179;
        Ths = x(i,5)*an*2*pi/360;
        %k1 = 80; k2 = .1;k3 = 25; k4 = .05;
        k1 = 30; k2 = .1;k3 = 25; k4 = 0.059;
        vth = (-x(i,3)*sin(Th)+x(i,4)*cos(Th)); %Theta original
        Thd = vth/r;
        
        vr = (x(i,3)*cos(Th)+x(i,4)*sin(Th));  %Rho Original
        rd = vr;
        ur = -k1*(rd-0)-k2*(r-150);
        wr = -((3*w^2*x(i,1))+x(i,4)*(2*w+Thd))*cos(Th)+x(i,3)*(2*w+Thd)*sin(Th);  % omegar original
        ar = ur+wr;
        ut = -r*(k3*(Thd-0)+k4*(Th-Ths));
        wt = ((3*w^2*x(i,1))+x(i,4)*(2*w+Thd))*sin(Th)+x(i,3)*(2*w+Thd)*cos(Th)+vr*Thd;   %omegatheta original
        at = ut+wt;
        inp = B*[cos(Th) -sin(Th);sin(Th) cos(Th)]*m*[ar;at];    % original input
        if norm(inp) > 0.02
            inp = 0.02* (inp/norm(inp));
        end
        unomfull2(i,:) = norm(inp,inf);
    end
    if x(i,6) == 3
        tp3(i,:) = t(i,:);
        if x(i,7) == 1
            up = -B*K3*([x(i,1) x(i,2) x(i,3) x(i,4)]'-[-25 0 0 0]');
            unomfull3(i,:) = norm(up,inf);
        else if x(i,7) == 2
                k_1 = -0.0007;k_2 = -0.15;k_3 = -0.006;k_4 = -0.22;
                ax = 3*n^2*x(i,1)+k_1*x(i,1) +k_2*(x(i,3)-0);
                ay = k_3*x(i,2) +k_4*x(i,4);
                inp = [0 0 ax ay]';
                unomfull3(i,:) = norm(inp,inf);
            end
        end
        
     end
    if x(i,6) == 4
        tp4(i,:) = t(i,:);
        unomfull4(i,:) = norm(-B2*K4*[x(i,1) x(i,2) x(i,3) x(i,4)]',inf);
    end
end
t1 = tp1;
u1 = unomfull1;
t2 = tp2(tp2~=0);
u2 = unomfull2(unomfull2~=0);
u2(1) = u1(end);
t3 = tp3(tp3~=0);
u3 = unomfull3(unomfull3~=0);
u3(1) = u2(end);
t4 = tp4(tp4~=0);
u4 = unomfull4(unomfull4~=0);
u4(1) = u3(end);
tu = [t1;t2;t3;t4];
ufull = [u1;u2;u3;u4];

%%

figure(20)
plot(t,0.02*ones(size(t)),'r')
hold on
sp1 = 1:100:length(t1);
plot(t1(sp1),u1(sp1),'g')
hold on
sp2 = 1:1:length(t2);
plot(t2(sp2),u2(sp2),'b')
hold on
sp3 = 1:1:length(t3);
plot(t3(sp3),u3(sp3),'m')
hold on
sp4 = 1:1000:length(t4);
plot (t4(sp4),u4(sp4),'c')
grid on
xlabel('time(sec)')
ylabel('$\frac{1}{m}|u|_{\infty}$ (m/sec$^2$)')
%title('Control effor = $\int^t_0 |u|\ dt$ =  142.9095 (m/sec): Calculated using trapz(X,Y)')
legend('Max input','Phase I input', 'Phase II input','Phase III input','Phase IV input')
CF = trapz(tu,ufull)

%%

figure(28)

for i= 1:1:length(t1)
   e = x(i,(1:4))' - x(i,(8:11))'; 
   % Pv(1,(1:16)) = x(1,(13:28));
    Pm = vec2mat(x(i,(13:28)),4);
   Lf(i) = norm(e'* Pm* e,2);
   clear e
   clear Pm
end

plot(t1,Lf)
hold on
xlabel('time(sec)')
ylabel('$|e^{\top} P  e|$')