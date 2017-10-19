%LSim simulates the system
% x(i)= a*x(i-1)+b*u(i-1)+w(i-1);
% y(i)= c*x(i)+v(i);
% 
% External input: None

% Time-stamp: <2016-10-19 09:10:23 tk>
% Version 1: 2015
% Version 2: 2016-10-19 08:11:33 tk Added comments and minor changes
% Torben Knudsen
% Aalborg University, Dept. of Electronic Systems, Section of Automation
% and Control
% E-mail: tk@es.aau.dk

%% Parameters

n= 100; 
% $$$ n= 10000; 
a= 0.95; k= 1; b= k*(1-a); c= 1; fu= 0.02; x0= 0;
sigmaw= 0.05*sqrt(1-a^2); sigmav= 0.1; sigmax0= sigmaw; 
phif= 0; phih= 0;                       % Only used in EKF.m

%% Algorithm

w= sigmaw*randn(n,1);
v= sigmav*randn(n,1);
u= square((1:n)*fu*2*pi);
x= zeros(n,1);
y= zeros(n,1);
x(1)= sigmax0*randn+x0;
y(1)= c*x(1)+v(1);
for i= 2:n;
  x(i)= a*x(i-1)+b*u(i-1)+w(i-1);
  y(i)= c*x(i)+v(i);
end;

figure(1)
subplot(311)
plot(u);
title('u')
subplot(312)
plot(x)
title('x')
subplot(313)
plot(y)
title('y')
