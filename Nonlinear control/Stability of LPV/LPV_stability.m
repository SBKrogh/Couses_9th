clc; clear all; close all;
%% Stability LPV MM8 

syms p %p -> [-1,-1/2 U 1/2, 1]
A(p) = [1 p; -4/p -3]; 
P(p) = [50+6*p^2 16*p; 16*p 1+7*p^2];


LPV_stab_con(p) = transpose(A(p))*P(p) + P(p)*A(p);

fprintf('We test and see if the eigenvalues in the exstremums are negative\nnote: in the lecture it has to be positive due to sign change.')
lpv_neg1 = eig(LPV_stab_con(-1))
lpv_neg05 = eig(LPV_stab_con(-0.5)) 
lpv_05 = eig(LPV_stab_con(0.5))
lpv_1 =eig(LPV_stab_con(1)) 
fprintf('It can be seen that p = [-0.5 0.5] eigenvalues is not negative\n\n')
