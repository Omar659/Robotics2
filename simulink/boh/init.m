clear all
clc

%condizioni iniziali
theta_0 = 0;
theta_0_dot = 0;

bm = 2;
bl = 3;
l = 1;
nr = 5;
g_0 = 9.81;
d = l*0.7;
m = 1.3;
I = 5.7;

viscous = bl + bm*nr^2;
U_0 = m*g_0*d;

A = 1;

T = 10;
t = linspace(0, T);
% syms t real
theta_d = A.*(1-cos(t));
theta_dot_d = A.*sin(t);
theta_dot_dot_d = A.*cos(t);
f = I.*theta_dot_dot_d + U_0.*sin(theta_d) + viscous.*theta_dot_d;

tau = f/(2*nr);
Fx = f./(2*l*cos(theta_d));
Fx1 = [t; Fx]';
tau1 = [t; tau]';


