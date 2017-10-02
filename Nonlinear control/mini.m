clc
clear all

%0.9 <= l <= 1.1   0.5 <= m <= 1.5   0 <= k <= 0.2    |h| <= 1
global  l m k gain g epsilon 
g = 9.81;% m/s^2
l_min = 0.9;, l_max = 1.1; % length
m_min = 0.5;, m_max = 1.5; % mass
k_min = 0;, k_max = 0.2; % friction

l = l_min+(l_max-l_min)*rand;
m = m_min+(m_max-m_min)*rand;
k = k_min+(k_max-k_min)*rand;
epsilon = 0.004;
gain = 1; % u=-(gain)*x1
t = 0:0.1:100;
dummy = 1;
x0 = [5; 10]; % initial states
[t,x] = ode45(@xdot, t, x0); % solve 

for b = 1:length(x) 
s1 = x(b,2)+gain*x(b,1);

if s1/epsilon < -1
    sat1 = -1;
elseif s1/epsilon > 1
    sat1 = 1;
else
    sat1 = s1/epsilon;
end
%calculate the control input from the states, so it can be plotted
T(b,1) = -(16.1865*abs(x(b,1))+1.815*abs(x(b,2)) +2)*sat1; 
end

fontsize = 10;
figure(1)

subplot(2,3,1)
plot(t,x(:,1))
xlabel('Time [s]','FontSize',fontsize)
ylabel('Angle','FontSize',fontsize)
title1 = title('$$\theta$$');
set(title1,'Interpreter','latex');

subplot(2,3,2)
plot(t,x(:,2))
xlabel('Time [s]','FontSize',fontsize)
ylabel('Angle velocity','FontSize',fontsize)
title2 = title('$$\dot\theta$$');
set(title2,'Interpreter','latex');

subplot(2,3,3)
plot(x(:,1),x(:,2))
axis([-2 11 -11 11]);
title3 = title('s')
%  xlabel = title('$$\theta$$');
%  set(xlabel,'Interpreter','latex');
%  ylabel = title('$$\dot\theta$$');
%  set(ylabel,'Interpreter','latex');


subplot(2,3,4)
plot(t,x(:,1))
hold on
plot(xlim, [0.01 0.01], '-r')
hold on
plot(xlim, [-0.01 -0.01], '-r')
axis([2 100 -0.05 0.05]);
xlabel('Time [s]','FontSize',fontsize)
ylabel('Angle','FontSize',fontsize)
title4 = title('$$\theta$$');
set(title4,'Interpreter','latex');

subplot(2,3,5)
plot(t,x(:,2))
hold on
plot(xlim, [0.01 0.01], '-r')
hold on
plot(xlim, [-0.01 -0.01], '-r')
axis([2 100 -0.05 0.05]);
xlabel('Time [s]','FontSize',fontsize)
ylabel('Angle velocity','FontSize',fontsize)
title5 = title('$$\dot\theta$$');
set(title5,'Interpreter','latex');

subplot(2,3,6)
plot(t,T)
xlabel('Time [s]','FontSize',fontsize)
ylabel('Control input','FontSize',fontsize)
title6 = title('T')

function dx=xdot(t,x)
global   gain g k m l epsilon 

h = cos(t); % Disturbance
s = x(2)+gain*x(1);
sat=s/epsilon;
if sat < -1
    sat = -1;
elseif sat > 1
    sat = 1;
else
    sat = s/epsilon;
end

u = -(16.1865*abs(x(1))+1.815*abs(x(2)) +2)*sat;
dx(1,1)= x(2); 
dx(2,1) = (1/(m*l))*((m*h*cos(x(1)) - k*l*x(2) - m*g*l*sin(x(1)))) + u*(1/(m*l^2));
end
