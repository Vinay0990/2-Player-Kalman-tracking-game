% One Dimentional Kalman Tracking Problem with two different trackers
% 
% Version 1
% System Definition
% x+ = x + u * dt + n
% y  = x+  + v

clc; clear all; close all;

dt = 0.1; N = 50; 
t = dt*(1:N);

F_x = 1;
F_u = dt;
F_n = 1;
H = 1;

Q = 16;
R1 = 4;
R2 = 9;

q = sqrt(Q);
r1 = sqrt(R1);
r2 = sqrt(R1);

% Trajectory simulation
xt0 = 10; u = 10;
xt = zeros(1,N);
xt(1) = xt0;

for i = 2:N
    xt(i) = F_x * xt(i-1) + F_u * u ;        
end

% measurement simulation
v1 = r1 * randn(1,N);    
y1 = H*xt + v1;

v2 = r2 *  randn(1,N);    
y2 = H*xt + v2;

x1 = zeros(1,N);
x1(1) = xt0;
x2 = zeros(1,N);
x2(1) = xt0;

P1 = 10; P2 = 10;

for i= 2:N 
    n1 = q * randn;   
    x1(i) = F_x * x1(i-1) + F_u * u + F_n * n1;
    P1(i) = F_x*P1(i-1)*F_x' + Q;
    
    K1(i) = P1(i)*H'/(H*P1(i)*H' + R1);
    x1(i) = x1(i) + K1(i)*(y1(i)- H*x1(i));
    P1(i) = (1 - K1(i)*H)*P1(i);
    
    n2 = q * randn;
    x2(i) = F_x * x2(i-1) + F_u * u + F_n * n2;
    P2(i) = F_x*P2(i-1)*F_x' + Q;
    
    K2(i) = P2(i)*H'/(H*P2(i)*H' + R2); 
    x2(i) = x2(i) + K2(i)*(y2(i) - H*x2(i));
    P2(i) = (1 - K2(i)*H)*P2(i);
    
    % plots
    plot(i,x1(i),'*r', i, x2(i), '*g', i, xt(i), '*b'); 
    xlabel('t(s)'); ylabel('x(m)'); title('Tracking of positional variable');
    legend('Tracker 1','Tracker 2','True Position');
    axis([0 N 0 100]);
    pause(0.5)
    drawnow
end

figure
plot(t,xt,'r',t,x1,'g',t,y1,'b', 'LineWidth', 2); hold on;
plot(t,x2,'c',t,y2,'m', 'LineWidth', 2);
xlabel('t(s)'); ylabel('x(m)'); title('Tracking of positional variable');
legend('truth','estimate1','measurement1','estimate2','measurement2');

