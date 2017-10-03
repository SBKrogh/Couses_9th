clc; clear all; close all; clear path
addpath(genpath('yalmip')); % Call yalmip
addpath sedumi-master
%% Certificates of Stability for LPV
A = sdpvar(2); 


A1 = [-1 0;1 -1]; % 1 <= p <= 2
A2 = [-1 0;2 -2];
P = sdpvar(2);

%Constraints = [A1, A2, P]
%Objective = transpose(A1)*P + P*A1 + transpose(A2)*P + P*A2
%F = constraints(P > 0) + constraints(transpose(A1)*P + P*A1) + constraints(transpose(A2)*P + P*A2)

constraint1 = A1'*P+P*A1 <= 0;
constraint2 = A2'*P+P*A2 <= 0;
constraint3 = P >= 0;

sol = optimize([constraint1, constraint2, constraint3],1);

P = double(P)

