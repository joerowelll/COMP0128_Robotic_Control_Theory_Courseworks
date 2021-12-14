clear;
clc;
% Symbolic variables used to compute analytical functions
%{
x 1-3: pos_x pos_y pos_z
x 4-6: posd_x posd_y posd_z
x 7-9: phi theta psi
x 10-12: omg1 omg2 omg3

xd 1-3: posd_x posd_y posd_z
xd 4-6: podd_x podd_y podd_z
xd 7-9: thd1 thd2 thd3
xd 10-12: omgd1 omgd2 omgd3
%}

syms x [12,1]
syms xd [12,1]
syms u [4,1]

%% define parameters
% Time step
dt = 0.05;
% Accleration due to gravity 
g = 9.2;
gravity=[0; 0; -g];
%Mass of qaudcopter
m = 0.2;
% Drag Coefficient
kd = 0.1;
k = 1;
% Quadcopter rotatinal inertia matrix
I = [1 , 0, 0;
    0, 1, 0;
    0, 0, 0.5];
L = 0.2;
b = 0.1;
% Target acceleration
a_target = [0;0;0]; % for equilibrium point
% Inputs for hovering 
equ_u = zeros(1,4);
equ_u(:) = m * g / 4;
% Equilibrium point when acceleration =0
equ_state = zeros(1,12);
equ_state(3) = 5;

%% Compute dynamics
% Roll, pitch, yaw
phi = x(7);
theta = x(8);
psi = x(9);
% Rotation Matrix
Rx = [1 0 0;
    0 cos(phi) -sin(phi); ... 
    0 sin(phi) cos(phi)];
Ry = [ cos(theta) 0 sin(theta);
    0 1 0; ...
    -sin(theta) 0 cos(theta)];
Rz = [cos(psi) -sin(psi) 0; ... 
    sin(psi) cos(psi) 0; ... 
    0 0 1];
R = Rz*Ry*Rx;

% Drag force 
Fd = -kd * [x(4); x(5); x(6)] ;
% Torques
tau = [
    L * k * (u(1) - u(3))
    L * k * (u(2) - u(4))
    b * (u(1) - u(2) + u(3) - u(4))];
% Thrust vector
Tb = [0; 0; k * sum(u)];

%% State space
xd(1:3) = x(4:6);
xd(4:6) = gravity + (1/m) * R *(Tb) + (1/m) *Fd;
xd(7:9) = [1, 0, -sin(x(8));
    0, cos(x(7)), cos(x(8)) * sin(x(7));
    0, -sin(x(7)), cos(x(8)) * cos(x(7))] \ x(10:12);
xd(10:12) = (I) \ (tau - cross(x(10:12), I * x(10:12)));

%Compute Jacobians 
Aj = jacobian(xd,x);
Bj = jacobian(xd,u);

A = subs(Aj,[x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12], equ_state);
A = double(subs(A,u,equ_u.'));
B = double (subs(Bj,[x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12],equ_state));

% Define C and D matrices
C = eye(size(A));
D = zeros(size(B));

cont_sys = ss(A,B,C,D);
disc_sys = c2d(cont_sys,dt,'zoh');

disp(disc_sys.A);
disp(disc_sys.B);


