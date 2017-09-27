clc; clear all; close all;
% check if inverse of a22 is possible

syms p;
A = [-3 -p 0 1;
     p 1 0 0;
     0 -p 1 p;
     0 0 -p 1];

a11 = -3;
a12 = [-p 0 1]
a21 = transpose([p 0 0]);

a22 = [1 0 0; -p 1 p; 0 -p 1];
adjoint(a22); % cofactor
det(a22); % determinant of a22

inv_a22 = (1/det(a22))*adjoint(a22) % inverse of a22 


a11-a12*inv(a22)*a21