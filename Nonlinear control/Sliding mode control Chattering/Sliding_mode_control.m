clc; clear all; close all; 
%% For the miniproject in Nonlinear control. 
% Topic: Sliding mode control.
% The function of dynamics can be found at the buttom of this script.

% x1   = theta
% x2   = theta_d = x1_d
% x2_d = theta_dd


%% Variables and bounderies
global l m k g gain int_x1 int_x2 

l = rand(1)*(1.1 - 0.9) + 0.9;  % Random pick lenght with boundery 0.9<= l <= 1.1
m = rand(1)*(1.5 - 0.5) + 0.5;  % Random pick mass with boundery 0.5 <= m <= 1.5
k = rand(1)*(0.2 - 0);          % Random pick resistance with boundery 0 <= k <= 0.2
g = 9.81;                       % Gravity
gain = -1;                      % Stabilising feedback gain for eta_d
t = [0:0.01:50]';
time1 = cputime;                % Time spend on simulation
int_x1 = 0.5;                   % Initial values for x1 = Theta and x2 = Theta_d
int_x2 = 0.3;



%% Control problem 
[t,x] = ode45(@xdot, t , [int_x1 int_x2]);

%% Plot
% Recalculations of control u
for q = 1:length(x)
    s = x(q,2)-gain*x(q,1);
    if s < 0 
        u(q) = (16.1865*abs(x(1)) + 1.815*abs(x(2)) + 2); 
    end
    if s > 0 
        u(q) = (-1)*(16.1865*abs(x(1)) + 1.815*abs(x(2)) + 2);
    end
    if s == 0 
        u(q) = 0;
    end
end

% Sliding mainifold, disturbance and derivative of sliding manifold respectively
s = x(:,2)-gain*x(:,1);
h  = sin(t);
s_d = m*l^2 * (((h.*cos(x(:,1)))*(1/l)) - (k*x(:,2))*(1/l) - g*sin(x(:,1)) - gain*(x(:,2)));

% Start plotting
figure
subplot(2,3,1)
plot(t,x) % state value with respect to time
grid on 
xlabel('time')
title('State of the system Theta = x1 and Theta_dot = x2')
xlabel('Time [s]')
ylabel('State value')
legend('Theta','Theta_dot')

subplot(2,3,3)
plot(t,u)
grid on 
xlabel('Time')
ylabel('Input')
title('Control input')

subplot(2,3,2)
plot(x(:,1),x(:,2))
grid on
xlabel('Theta')
ylabel('Theta dot')
title('Theta = x1 and Theta_{dot} = x2')

subplot(2,3,4)
plot(x(:,1),x(:,2))
axis([-0.2 0.2 -0.2 0.2])
grid on
xlabel('Theta')
ylabel('Theta dot')
title('Zoom - Theta = x1 and Theta_dot = x2')

subplot(2,3,5)
plot(t,s)
grid on
xlabel('Time')
ylabel('s')
title('Sliding manifold')

subplot(2,3,6)
plot(t,s_d)
grid on
xlabel('Time')
ylabel('s_d')
title('Sliding mode derivative')

simulation_time = cputime-time1

%%

function dx=xdot(t,x)
global l m k g gain
    
    s = x(2)-gain*x(1);     % Sliding manifold
    
    % Control u calculated on sign value of s    
    beta = (16.1865*abs(x(1)) + 1.815*abs(x(2)) + 2);
   
    if s < 0 
        u = beta; 
    end
    
    if s > 0 
        u = (-1)*beta;
    end
    
    if s == 0 
        u = 0;
    end
    
    
    
    h = sin(t); % Disturbance
    % System dynamics
    dx(1,1) = x(2); 
    dx(2,1) = (1/(m*l))*(m*h*cos(x(1)) - k*l*x(2) - m*g*l*sin(x(1))) + (1/(m*l^2)*u);   
    
end

