pkg load control

g = 9.81;   # m/s
M = 2;      # kg
m = 0.1;    # kg
l = 0.5;    # m

A = [ 0             1 0 0;
      (M+m)/(M*l)*g 0 0 0;
      0             0 0 1;
      -m/M*g        0 0 0];
B = [ 0; -1/(M*l); 0; 1/M];
C = [0 0 1 0];

Ahat = [A zeros(4,1); -C 0];
Bhat = [B; 0];

J = [-1+j*sqrt(3) -1-j*sqrt(3) -5 -5 -5];

Khat = place(Ahat, Bhat, J);
