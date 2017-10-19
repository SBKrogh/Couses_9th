clc; clear all; close all;
%% Exercise for nonlinear systems uppon 
% Kalman filters for linear systems 

%% System 

phi = 0.9; % System matrix
h = 1;     % Output matrix
q = 1      % Noice variance for system 

r1 = 0.1;  % Noice variance for measurement output 
r2 = 1;
r3 = 10;

sim_time = 100;

%% 1) Find the stationary solution k, p- and p+ for the above system description
% For this exercise go to slide 23

p0 = 3;
mean_x0 = 0;

x(1) = normrnd(mean_x0,sqrt(p0)); % inital 
k(1) = p0*transpose(h*(h))*(h*p0*transpose(h) + r1)^(-1); % initail

% update step


for n = 1:sym_time 
    x(n+1) = phi*x(n) + 


