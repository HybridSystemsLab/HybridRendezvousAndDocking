%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file       Project: HyEQ Toolbox  @ Hybrid Dynamics and Control
% Lab, http://www.u.arizona.edu/~sricardo/index.php?n=Main.Software
%
% Filename: run_switch.m
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global K er
er = 1;
mu = 3.98600444*10^14; ro = 7100*1000;
n = sqrt(mu/ro^3); 
A = [0     0    1 0;
     0     0    0 1;
     3*n^2 0    0 2*n;
     0     0 -2*n 0];
m = 0.5*1000;  
B = [0  0; 0  0; 1/m  0;0  1/m];

%xi = [-159.6767  10.0951    0.0240    -0.0162]; %IC1
%xi = [ -153.9680  -19.8469   -0.0032    0.0343]; %IC2 
%xi = [-154.0780  -18.1398   -0.0029    0.0308];   %IC3
%xi = [ -154.6125   13.2023   -0.0009   -0.0210];  %IC4 
%xi = [ -154.4601  -14.4355   -0.0015    0.0235];  %IC5
%xi = [-154.7417  -10.7563   -0.0005    0.0161 ];  %IC6
%xi = [-154.8150   10.3547   -0.0002   -0.0153];   %IC7
xi = [-154.9920    6.3253    0.0003   -0.0073];   %IC8
%xi = [-154.9835   -6.0357    0.0002    0.0067 ;   %IC9
xdi = [-25 0];
%-----------------------------%
Q = 38.4*eye(4);
R = 9.7e3*eye(2);
[K,s,e] = lqr(A,B,Q,R);
%---------------------------%
xint = [xi xdi 1 0 0]; 

x0 = xint;


% simulation horizon
T = [0 1800];                                                                
J = [0 10000];

% rule for jumps
% rule = 1 -> priority for jumps
% rule = 2 -> priority for flows
rule = 1;

options = odeset('RelTol',1e-3,'MaxStep',.1);

% simulate
[t j x] = HyEQsolver( @f,@g,@C,@D,x0',T,J,rule,options);
  

%%
% %plot hybrid arc   
% figure(1)
% plotHarc(t,j,x(:,1:2))
% grid on
% xlabel('time (sec)')                    
% ylabel('x,y (m)')  
% legend('x','y')

 %%
% %plot hybrid arc   
figure(2)
plotHarc(t,j,x(:,3:4))
grid on
xlabel('time (sec)')                    
ylabel('$\dot x$,$\dot y$ (m/sec)')                    
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

%%
figure(14)
set(gca,'Xdir','reverse')
angle = linspace(0,2*pi,360);
xc = 100*cos(angle);
yc = 100*sin(angle);
xcf = 0.1*cos(angle);
ycf = 0.1*sin(angle);
cony = -linspace(0,200,500);
conxr = cony/tan(120*2*pi/360);
conxl = cony/tan(60*2*pi/360);
hold on
plot(conxl,cony,'b')
plot(conxr,cony,'b')
plot(0,0,'square')
plot(xc,yc,'m')
hold on
plot(xcf,ycf,'r')
xlabel('y-position in m')
ylabel('x-position in m')
legend('xy position','Target')
grid on
%title('$t_f = 1000 sec$')
%hold off


plot(x(:,2),x(:,1),'b')
hold on
plot(x(1,6),x(1,5),'k O','linewidth',2)
hold on
hold on
plot(x(1,2),x(1,1),'r*','linewidth',2)


 %%
for i= 1:1:length(t)
  xdnom(i,:)= norm([x(i,3) x(i,4)]',2);
   unomi(i,:)= norm(-B*K*[x(i,1) x(i,2) x(i,3) x(i,4)]',inf);
  end


figure(6) 
plot(t,0.05*ones(size(t)),'r')
hold on
plot(t,xdnom,'m')
xlabel('time(sec)')
ylabel('Closing velocity $(m/sec)$')
legend('Max closing velocity','Calculated velocity')
grid on

% CF = trapz(t,unomi)
%%
figure(5) 
plot(t,0.02*ones(size(t)),'r')
hold on
plot(t,x(:,9),'m')
xlabel('time(sec)')
ylabel('$\frac{1}{m}|u|_{\infty}$ (m/sec$^2$)')
legend('Max input','Calculated input')
grid on
% 
% CF = trapz(t,unomi)
% %%
% % %Animation
% % 
% % figure(4)
% % plot(x(:,1),x(:,2))
% % hold on
% % p = plot(x(1,9),x(1,10),'o','MarkerFaceColor','red');
% % hold off
% % %axis manual
% % for k = 2:length(x(:,1))
% %     p.XData = x(k,9);
% %     p.YData = x(k,10);
% %     drawnow
% % end
