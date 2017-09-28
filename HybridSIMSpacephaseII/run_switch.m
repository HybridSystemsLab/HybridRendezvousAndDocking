%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file       Project: HyEQ Toolbox  @ Hybrid Dynamics and Control
% Lab, http://www.u.arizona.edu/~sricardo/index.php?n=Main.Software
%
% Filename: run_switch.m
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mu = 3.98600444*10^14; ro = 7100*1000;
n = sqrt(mu/ro^3); 
A = [0     0    1 0;
     0     0    0 1;
     3*n^2 0    0 2*n;
     0     0 -2*n 0];
m = 1*500;  
B = [0  0; 0  0; 1/m  0;0  1/m];

%----------Velocity------------%
%              d = [0 0 -0.45 -0.45];
                       d = [0 0 0.45 0.45];
%         d = [0 0 0 0];
%---------------------------------------------%
              xi = [700 0 0 0]+d; %-1
%           xi = [700*cosd(45) 700*sind(45) 0 0]+d;
%             xi = [-700*cosd(45) 700*sind(45) 0 0]+d;
%          xi = [-700*cosd(45) -700*sind(45) 0 0]+d;  
%           xi = [700*cosd(45) -700*sind(45) 0 0]+d; 
%            xi = [-700*cosd(90) -700*sind(90) 0 0]+d;
%              xi = [700*cosd(90) 700*sind(90) 0 0]+d;
%            xi = [-700*cosd(1) 700*sind(1) 0 0]+d;
%          xi = [-700*cosd(1) -700*sind(1) 0 0]+d;
%            xi = [-700 0 0 0]+d; %-1
%--------------------------------------------%


hin = 1;
xint = [xi hin]; 


  

x0 = xint;


% simulation horizon
T = [0 3000];                                                                
J = [0 10000];

% rule for jumps
% rule = 1 -> priority for jumps
% rule = 2 -> priority for flows
rule = 1;

options = odeset('RelTol',1e-6,'MaxStep',.1);

% simulate working old in the paper
[t j x] = HyEQsolver( @f2,@g2,@C2,@D2,x0',T,J,rule,options);

% % New files for updated A2 region
% [t j x] = HyEQsolver( @f4,@g4,@C4,@D4,x0',T,J,rule,options);
  

%%
%plot hybrid arc   
figure(1)
plot(t,x(:,1)/1000); 
hold on
plot(t,x(:,2)/1000,'r');
grid on
hold off
xlabel('time (sec)')                    
ylabel('x,y (KM)')  
legend('x','y')

%%
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
figure(4)
set(gca,'Xdir','reverse')
i = 1:300:length(t);
plot(x(i,2)/1000,x(i,1)/1000,'r')
hold on
angle = linspace(0,2*pi,360);
xc = 1000*cos(angle);
yc = 1000*sin(angle);

plot(xc/1000,yc/1000,'g')
plot(xc/10000,yc/10000,'m')
cony = -linspace(0,1,500);
conxr = cony/tan(105*2*pi/360);
conxl = cony/tan(75*2*pi/360);

xcd1 = 950*cos(angle);
ycd1 = 950*sin(angle);
xcd2 = 700*cos(angle);
ycd2 = 700*sin(angle);
plot(xcd1/1000,ycd1/1000,'c')
plot(xcd2/1000,ycd2/1000,'c')

hold on
plot(conxl,cony,'b')
plot(conxr,cony,'b')
xlabel('y-position in KM')
ylabel('x-position in KM')
legend('xy position','Target')
grid on
hold on
%%

figure(5)  
plot(t,x(:,5))
xlabel('time(sec)')
ylabel('Logic variable')
%title('Control effor = $\int^t_0 |u|\ dt$ = 0.2145 (KM/sec): Calculated using trapz(X,Y)')
grid on
%%
for i= 1:1:length(t)
        mu = 3.98600444*10^14; ro = 7100*1000;
        n = sqrt(mu/ro^3);
        r = sqrt(x(i,1)^2+x(i,2)^2);
        Thh = (atan2(x(i,2),x(i,1)));
        w =n;
        an = 179;
        h = x(i,5);
        Ths = h*an*2*pi/360;
        k1 = 30; k2 = .1;k3 = 25; k4 = 0.059;
        
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
        unorm(i) = norm(inp,2);
         if unorm(i) > 0.02
             unorm(i) = 0.02;
         end
end

 figure(7)  
plot(t,0.02*ones(size(t)),'b -.','linewidth',2)
hold on
plot(t,unorm,'r')   
grid on
ylabel('$\frac{1}{m}|u|_{\infty}$ (m/sec$^2$)')
legend('Max input','calculated input')