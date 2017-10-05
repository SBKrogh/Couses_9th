clc; clear all; close all; clear path
addpath(genpath('yalmip')); % Call yalmip
addpath sedumi-master       % Call solver 
%% Control LPV 

A = [-1 3;0 -1]; B = [0; 1]; C = [1 0]; D = [0];
%g = 50;
g = sdpvar(1); 
P = sdpvar(2);

sdpsettings('solver','sedumi')

LMI1 = P > 0;
LMI2 = -[A'*P+P*A P*B C';
        (P*B)' -g D';
        C D -g] > 0;
%solvesdp(LMI1+LMI2, g)
optimize(LMI1+LMI2, g)
double(P)
double(g)