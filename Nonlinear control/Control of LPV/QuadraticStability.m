clear all
A1 = [-1 3; 0 11];
A2 = [5 7; 0 -1];
A3 = [4 3;0 -2];
B = [0;1]; E = [0; 1]; C = [2 0;0 3];
D = [0; 0]; F = [0; 1];

X = sdpvar(2);
Y1 = sdpvar(1,2,'full');
Y2 = sdpvar(1,2,'full');
Y3 = sdpvar(1,2,'full');
g = sdpvar(1);

sdpsettings('solver','sedumi')

LMI1 = X > 0
R11_1 = A1*X+B*Y1;
R11_2 = A1*X+B*Y1;
R11_3 = A1*X+B*Y1;
R31 = C*X+D*Y1;
LMI2 = - [R11_1+R11_1' E R31'; E' -g F'; R31 F -g*eye(2)] > 0
LMI3 = - [R11_2+R11_2' E R31'; E' -g F'; R31 F -g*eye(2)] > 0
LMI4 = - [R11_3+R11_3' E R31'; E' -g F'; R31 F -g*eye(2)] > 0

solvesdp(LMI1+LMI2+LMI3+LMI4, 1)
eig(double(X))