clc; clear all; close all; 
%% For the miniproject in Nonlinear control. 
% Topic: Sliding mode control.

%% Variables and bounderies
global l m k g gain dummy int_x1 int_x2
l = rand(1)*(1.1 - 0.9) + 0.9; % Random pick lenght with boundery 0.9<= l <= 1.1
m = rand(1)*(1.5 - 0.5) + 0.5; % Random pick mass with boundery 0.5 <= m <= 1.5
k = rand(1)*(0.2 - 0);          % Random pick resistance with boundery 0 <= k <= 0.2
g = 9.81;                     % Gravity
gain = -10;
t = [0:1:500]';
dummy = 1;
              % Horizontal acceleration with random pick 
int_x1 = 0.5;
int_x2 = 3;

%% Dynamics 
% x1   = theta
% x2   = theta_d = x1_d
% x2_d = theta_dd

[t,x] = ode45(@xdot, t , [int_x1 int_x2]);

plot(t,x)
grid on 
xlabel('time')

%px = (0.01/l) - ((0.2*0.01)/m) - g*sin(0.01) - gain*0.01

function dx=xdot(t,x)
global l m k g gain dummy int_x1 int_x2
    
    if dummy == 1
    u = (-16.1865*norm(int_x1) + 1.815*norm(int_x2) + 2)%*sat((int_x2-gain*int_x2)/0.004)
    dummy = 0;
    end
    
    dx(1,1) = x(2);
    dx(2,1) = (1/(m*l))*(m*(sin(t*2*pi))*cos(x(1)) - k*l*x(2) - m*g*l*sin(x(1))) + (1/(m*l^2));   

if sign((x(2)-gain*x(2))/0.004) == 1
    u = -1*(-16.1865*norm(x(1)) + 1.815*abs(x(2)) + 2)
    'hi'
end

if sign((x(2)-gain*x(2))/0.004) == 0
    u = (-16.1865*norm(x(1)) + 1.815*norm(x(2)) + 2)
    'ih'
end

end

   



 

% eta_d = fa = x2 
% xi_d  = fb + g   =  (1/(m*l))*(m*h(t)*cos(theta) - k*l*theta_d - m*g*l*sin(theta) + 1/(m*l^2) * T

% fb = (1/(m*l))*(m*h(t)*cos(theta) - k*l*theta_d - m*g*l*sin(theta);
% g = 1/(m*l^2)
% T = u


% sliding manifold s 
% s = eta - phi(eta) = 0
% design phi(eta) to stabilize eta_d - for this we use state feedback 
%  u = -kx

% s_d = fb + g - K*fa
% delta = 1/g * (-fb + K*fa)

% s_d = g*v + delta 

% delta/g <= norm(delta,1)
% v  -beta*sgn(s) 
% beta < (norm(delta,1))