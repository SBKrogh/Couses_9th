clc; clear all; close all; 
%% Lecture 4: Distributed optimization 

%% Exercise 1: Solve the following optimization problem using a centralized algorithm for method of multipliers
% (Augmented slide 16 / normal lagragian)  

% minimize alfa^(T)*x
% subject to Ax = b
% x >= 0

alfa = [1 2 2.3 2.5 3 2.4 2.1 2.2];

A = [1 -1 -1 0 0 0 0 0;
     0 1 0 -1 -1 0 0 0;
     0 0 1 0 0 -1 -1 0;
     0 0 0 0 -1 -1 0 1;
     0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 1];
 
 b = [0 0 0 0 1 2 3]';
 
%% slide 18
% step 0
lambda_init = ones(7,1)'; % page 6, lecture note
 
% step 1
fun = @(x)alfa*[x(1) x(2) x(3) x(4) x(5) x(6) x(7) x(8)]' + lambda_init*(A*[x(1) x(2) x(3) x(4) x(5) x(6) x(7) x(8)]' - b);
k = 0;
x0 = [1 1 1 1 1 1 1 1]';
x = fmincon(fun,x0,-eye(8),zeros(8,1));

% step 2
p = 1;
lambda = lambda_init + p*(A*x-b);

% step 3


