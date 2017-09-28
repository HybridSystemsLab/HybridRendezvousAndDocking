%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file       Project: HyEQ Toolbox  @ Hybrid Dynamics and Control
% Lab, http://www.u.arizona.edu/~sricardo/index.php?n=Main.Software
%
% Filename: run_switch.m
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global NOISE1 tnoise K NOISEP

mu = 3.98600444*10^14; ro = 7100*1000;
n = sqrt(mu/ro^3); 
A = [0     0    1 0;
     0     0    0 1;
     3*n^2 0    0 2*n;
     0     0 -2*n 0];
m = 0.5*1000;  
B = [0  0; 0  0; 1/m  0;0  1/m];

% xint = [-35501.9 12453.7 0.06639 74.95]; 
% yint = [-35501.9 12453.7 0.06639 74.95]; 
% xhatint = [-35501.9 12453.7 0.06639 74.95]-[1000 1000 0 0]; 
% xint = [6550.19 -2453.7 0.5 0.5];
% yint = [6550.19 -2453.7 0.5 0.5];
% xhatint = [6550.19 -2453.7 0.5 0.5]+[300 200 0 0];
% xint = [-2453.7 6550.19  0.5 0.5];
% yint = [-2453.7 6550.19  0.5 0.5];
% xhatint = [-2453.7 6550.19 0.5 0.5]+[250 250 0.1 0.1];
% xint = [10000 0 .5 .5];
% yint = [10000 0 .5 .5];
% xhatint = [10000 0  .5 .5]+[-1000 -1000 0 0];
xint = [0 -10000  .5 .5];
yint = [0 -10000 .5 .5];
xhatint = [0 -10000 .5 .5]+[-1000 1000 0.0 0.0];
% xint = [8660.3 5000  .5 .5];
% yint = [8660.3 5000 .5 .5];
% xhatint = [8660.3 5000 .5 .5]+[10 -10 0.0 0.0];
% xint = [-7071 7071  .5 .5];
% yint = [-7071 7071 .5 .5];
% xhatint = [-7071 7071 .5 .5]+[-1000 1000 0.0 0.0];
% xint = [5000 8660.3 .5 .5];
% yint = [5000 8660.3 .5 .5];
% xhatint = [5000 8660.3 .5 .5]+[10 -10 0 0];
% xint = [1736.5 -9848.1 .5 .5];
% yint = [1736.5 -9848.1 .5 .5];
% xhatint = [1736.5 -9848.1 .5 .5]+[100 100 0.5 0.5];


% xint = [1107.8949    18.6642   -0.2632   -0.1745];
% yint = [1107.8949    18.6642   -0.2632   -0.1745];
% xhatint = [1107.8949    18.6642   -0.2632   -0.1745]+[-220.3376  154.8224   -0.0050    0.0361];



% %-------from full sim-----------%
po = [1*10^5 0 0 0;
      0 1*10^5 0 0;
      0 0 1*10^1 0;
      0 0 0 1*10^1];
  
poi = reshape(po,[1,16]);
tau = 0;
hin = 1;
tnN = 0;   

x0 = [xint yint xhatint poi tau hin tnN];

%-----------------NOISE-----------%
N=1100;
Fs = 100;
tnoise = (0:N-1)/Fs;
sigma = 1;
NOISE1 = (0.001)*sigma*randn(size(tnoise));
NOISEP = (10^-4)*sigma*randn(size(tnoise));

Q1 = 1.5e-1*eye(4);
R1 = [18e5 0;0 18e5];
 [K,s,e] = lqr(A,B,Q1,R1);


% simulation horizon
T = [0 10000];                                                                
J = [0 500000];
% rule for jumps
% rule = 1 -> priority for jumps
% rule = 2 -> priority for flows
rule = 1;

options = odeset('RelTol',1e-6,'MaxStep',.1);

% simulate
[t j x] = HyEQsolver( @f,@g,@C,@D,x0',T,J,rule,options);

%%
%plot hybrid arc   
figure(1)
plot(t,x(:,1)/1000); 
hold on
plot(t,x(:,9)/1000,'r'); 
%hold off
grid on
xlabel('t,j')                    
ylabel('x')                    
%zlabel('x1')    

%%
%plot hybrid arc   
figure(11)
plot(t,x(:,6)/1000); 
hold on
plot(t,x(:,10)/1000,'r'); 
hold off
grid on
xlabel('time (sec)')                    
ylabel('y (KM)')                    
%zlabel('x1')    
%%
clear xnom
for i= 1:1:length(t)
  xnomer(i,:)= norm([(x(i,1)-x(i,9)) (x(i,2)-x(i,10)) (x(i,3)-x(i,11)) (x(i,4)-x(i,12))],2);
  xnomerfnl(i,:)= norm([(x(i,5)-x(i,9)) (x(i,6)-x(i,10)) (x(i,7)-x(i,11)) (x(i,8)-x(i,12))],2);
end
figure(12)  
plot(t,xnomer,'b')
hold on
plot(t,xnomerfnl,'r')
xlabel('time(sec)')
ylabel('error $e = \eta-\eta_c$ (m)')
grid on
hold on
%%
figure(500)
set(gca,'Xdir','reverse')
sp = 1:1000:length(t);
angle = linspace(0,2*pi,360);
xc = 1000*cos(angle);
yc = 1000*sin(angle);
plot(xc/100,yc/100,'k') 
plot(x(sp,10)/1000,x(sp,9)/1000,'r')
hold on
plot(x(sp,2)/1000,x(sp,1)/1000,'k')
hold on
plot(x(1,10)/1000,x(1,9)/1000,'r*','linewidth',5)
hold on
plot(x(1,2)/1000,x(1,1)/1000,'b*','linewidth',1)
angle = linspace(0,2*pi,360);
xc = 1000*cos(angle);
yc = 1000*sin(angle);
plot(xc/1000,yc/1000,'g')
xlabel('True, Estimated y-position in km')
ylabel('True, Estimated x-position in km')
grid on
hold on
plot(xc/100,yc/100,'k')
%%
for i= 1:1:length(t)
    %NL = B*m^2*B'*(mu/ro^4)*[0;0;-3*x(i,9)^2+(3/2*x(i,10)^2);3*x(i,9)*x(i,10)];
    SOM = (mu/ro^4)*[0;0;-3*x(i,9)^2+(3/2*x(i,10)^2);3*x(i,9)*x(i,10)];
    unom(i,1)= norm((B*K*[x(i,9) x(i,10) x(i,11) x(i,12)]'+SOM),inf);
    if unom(i,1) >0.02
        unom(i,1) = 0.02;
    end
end

figure(5)
%plot(t,x(:,6))
grid on
hold on
plot(t,0.02*ones(size(t)),'r')
hold on
plot(t,unom)
xlabel('time(sec)')
ylabel('$\frac{1}{m}|u|_{\infty}$ (m/sec$^2$)')
legend('Max input','calculated input')
hold on
%CF = trapz(t,unom)

%%
figure(6)
plot(t,atan2(x(:,6),x(:,5)),'r')
grid on
hold on
plot(t,atan2(x(:,10),x(:,9)))
hold on
xlabel('time(sec)')
ylabel('angle (rad)')
%legend('Max input','calculated input')
hold on
%CF = trapz(t,unom)

%%
figure(200)
for i= 1:1:length(t)
gamaxhat = (mu/ro^4)*[0;0;-3*x(i,9).^2+(3/2*x(i,10).^2);3*x(i,9).*x(i,10)];
gamax = (mu/ro^4)*[0;0;-3*x(i,1).^2+(3/2*x(i,2).^2);3*x(i,1).*x(i,2)];
gamae(i) = norm((gamaxhat-gamax),inf);
xnom(i,:)= norm([x(i,1) x(i,2) x(i,3) x(i,4)]-[x(i,9) x(i,10) x(i,11) x(i,12)],2);
end



plot(t,gamae);
hold on
plot(t,xnom,'r');
grid on
xlabel('time(sec)')
ylabel('$|\Gamma(\eta)-\Gamma(\hat\eta)|_{\infty}$')