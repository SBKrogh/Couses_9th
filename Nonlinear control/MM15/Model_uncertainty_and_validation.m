clc; clear all; close all;
%%

rng default;  % For reproducibility
x = normrnd(0,1,1000,1); % (mu, step size, N samples, std diviation) 

figure;
normplot(x)