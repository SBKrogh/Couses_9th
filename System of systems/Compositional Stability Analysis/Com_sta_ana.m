clc; clear all; close all; clear path
addpath(genpath('YALMIP-master')); % Call yalmip
addpath sedumi-master
%% Compositional stability analysis

%% example blackboard 

% A = [-1 0;2 -2];
% %Lyaponov matrix 
% P = sdpvar(2,2,'symmetric');
% Objective = 1;
% Constrint1 = A'*P + P*A <= 0;
% Constrint2 = P >= 0;
% optimize([Constrint1, Constrint2], Objective);

%% Exercise 1 
%syms x y
%P(x) = 12*x^2 - 6.3*x^4 + x^6 +3*x*y - 12*y^2 + 12*y^4;

x = sdpvar(1,1);
y = sdpvar(1,1);

c = sdpvar(1,1);
f = 12*x^2 - 6.3*x^4 + x^6 + 3*x*y - 12*y^2 + 12*y^4;


con = sos(f-c);
%options = sdpsettings('solver', 'mosek');
optimize(con , -c)% ,options);

[x,y] = meshgrid(1:0.5:10,1:0.5:20);
f = 12*x.^2 - 6.3*x.^4 + x.^6 + 3*x*y - 12*y.^2 + 12*y.^4;
surf(y,x,f)