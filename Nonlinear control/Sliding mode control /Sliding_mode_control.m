clc; clear all; close all; 
%% For the miniproject in Nonlinear control. 
% Topic: Sliding mode control.

%% Variables and bounderies
global l m k g 
l = rand(1)*(1.1 - 0.9) + 0.9; % Random pick lenght with boundery 0.9<= l <= 1.1
m = rand(1)*(1.5 - 0.5) + 0.5; % Random pick mass with boundery 0.5 <= m <= 1.5
k = rand(1)*(0.2 - 0);          % Random pick resistance with boundery 0 <= k <= 0.2
g = 9.81;                     % Gravity

t = [0:1:50]';
              % Horizontal acceleration with random pick 



%% Dynamics 
% x1   = theta
% x2   = theta_d = x1_d
% x2_d = theta_dd

[t,x] = ode45(@xdot, t , [0.5 0.5]);




function dx=xdot(t,x)
global l m k g
    dx(1,1) = x(2);
    dx(2,1) = (1/(m*l))*(m*(sin(t*2*pi))*cos(x(1)) - k*l*x(2) - m*g*l*sin(x(1))) + (1/(m*l^2));
    
    
    
   
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