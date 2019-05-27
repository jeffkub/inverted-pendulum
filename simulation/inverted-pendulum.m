# inverted-pendulum.m
#
# Control system simulation for inverted pendulum. Based on Example 12-5 from
# Modern Control Engineering, Fourth Edition by K. Ogata.

pkg load control

clear all
close all

# Inverted pendulum parameters
g = 9.81;   # Gravity constant [m/s]
M = 0.05;   # Cart mass [kg]
m = 0.062;  # Pendulum mass [kg]
l = 0.02;   # Pendulum length to center of mass [m]

# Desired closed loop poles
J = [-1+j*sqrt(3) -1-j*sqrt(3) -5 -5 -5];

# State space representation of system
A = [             0 1 0 0;
      (M+m)/(M*l)*g 0 0 0;
                  0 0 0 1;
             -m/M*g 0 0 0];

B = [0 -1/(M*l) 0 1/M]';

C = [0 0 1 0];

D = [0];

Ahat = [ A zeros(4,1);
        -C          0];

Bhat = [B; 0];

# Determine the state feedback gain matrix
Khat = place(Ahat, Bhat, J)
K = Khat(1:4);
kI = -Khat(5);

# Calculate the step response
ts = 0.02;
t = 0:ts:6;

AA = [(A - B*K) B*kI;
             -C    0];

BB = [0 0 0 0 1]';

CC = [C 0];

DD = [0];

sys = ss(AA, BB, CC, DD);
[y, t, x] = step(sys, t);

u = (-Khat * x')';

# Plot the step response
subplot(3,2,1)
plot(t, x(:,1))
grid
title('\theta vs t')
xlabel('t [s]')
ylabel('\theta [rad]')

subplot(3,2,2)
plot(t, x(:,2))
grid
title('\theta dot vs t')
xlabel('t [s]')
ylabel('\theta dot [rad/s]')

subplot(3,2,3)
plot(t, x(:,3))
grid
title('x vs t')
xlabel('t [s]')
ylabel('x [m]')

subplot(3,2,4)
plot(t, x(:,4))
grid
title('x dot vs t')
xlabel('t [s]')
ylabel('x dot [m/s]')

subplot(3,2,5)
plot(t, x(:,5))
grid
title('\zeta vs t')
xlabel('t [s]')
ylabel('\zeta [m]')

subplot(3,2,6)
plot(t, u)
grid
title('u vs t')
xlabel('t [s]')
ylabel('u [N]')
