%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file       Project: HyEQ Toolbox  @ Hybrid Dynamics and Control
% Lab, http://www.u.arizona.edu/~sricardo/index.php?n=Main.Software
%
% Filename: run_switch.m
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global K
mu = 3.98600444*10^14; ro = 7100*1000;
n = sqrt(mu/ro^3); 
A = [0     0    1 0;
     0     0    0 1;
     3*n^2 0    0 2*n;
     0     0 -2*n 0];
m = 2500;  
B = [0  0; 0  0; 1/m  0;0  1/m];
%---------------ZERO INITIAL VELOCITY--------------%
% xi = [-0.0731   -0.0001    0.0003    0.0000];
%  xi = [-0.0199   -0.0000    0.0001    0.0000];
% xi = [-0.0734   0.0001    0.0004    0.0000];
% xi = [ -0.0728   -0.0001    0.0003    0.0000];
xi = [-0.0200   -0.0000    0.0001    0.0000];
 %  Q = 10e5*eye(4);
 %  R = 10e2*eye(2);
  Q = 6e-1*eye(4);
  R = 11e4*eye(2);
[K,s,e] = lqr(A,B,Q,R);

xint = [xi 0 0]; 

hin = 1;
  

x0 = xint;


% simulation horizon
T = [0 8000];                                                                
J = [0 1000];

% rule for jumps
% rule = 1 -> priority for jumps
% rule = 2 -> priority for flows
rule = 1;

options = odeset('RelTol',1e-6,'MaxStep',.1);

% simulate
[t j x] = HyEQsolver( @f,@g,@C,@D,x0',T,J,rule,options);
  

%% X and Y STAES INDIVIDUAL PLOTS
figure(1)
plot(t,x(:,1)/1000); 
hold on
plot(t,x(:,2)/1000,'r');
grid on
hold off
xlabel('time (sec)')                    
ylabel('x,y (KM)')  
legend('x','y')

%% velocities 
%plot hybrid arc   
figure(2)
plot(t,x(:,3)/1000); 
hold on
plot(t,x(:,4)/1000,'r');
grid on
hold off
xlabel('time (sec)')                    
ylabel('$\dot x$,$\dot y$ (KM/sec)')                    
%zlabel('x1')    
legend('$\dot x$','$\dot y$')
%% NORM OF THE STATES rho
clear xnom
for i= 1:1:length(t)
  xnom(i,:)= norm([x(i,1)/1000 x(i,2)/1000],2);
  unom(i,1)= norm((B*K*[x(i,1) x(i,2) x(i,3) x(i,4)]'),2);
end
figure(3)  
plot(t,xnom)
xlabel('time(sec)')
ylabel('\rho (KM)')
grid on

%% XY PLOT
figure(19)
set(gca,'Xdir','reverse')
plot(x(:,2)/1000,x(:,1)/1000,'b')
hold on
angle = linspace(0,2*pi,360);
xc = 1000*cos(angle);
yc = 1000*sin(angle);
plot(xc/10000000,yc/10000000,'r')
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
plot(30,0,'square')
hold on
plot(xc/100,yc/100,'k')
hold on
plot(30,0,'+','linewidth',5)
hold on
plot(20,0,'m o','linewidth',3)
%%
for i= 1:1:length(t)
  unom(i,1)= norm((B*K*[x(i,1) x(i,2) x(i,3) x(i,4)]'),inf);
end
%%
figure(15)
%plot(t,x(:,6))
grid on
hold on
plot(t,0.02*ones(size(t)),'r')
hold on
plot(t,unom,'k')
xlabel('time(sec)')
ylabel('$\frac{1}{m}|u|_{\infty}$ (m/sec$^2$)')
legend('Max input','calculated input')
hold on
%CF = trapz(t,unom)
%%
% %Animation
% 
% figure(4)
% plot(x(:,1),x(:,2))
% hold on
% p = plot(x(1,9),x(1,10),'o','MarkerFaceColor','red');
% hold off
% %axis manual
% for k = 2:length(x(:,1))
%     p.XData = x(k,9);
%     p.YData = x(k,10);
%     drawnow
% end
