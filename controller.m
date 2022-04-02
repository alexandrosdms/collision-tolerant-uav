function [F, M, ei_dot, eI_dot] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
%% Controller gains
k.x = 10;
k.v = 8;
k.i = 10;

% Attitude
k.R = 1.5;
k.W = 0.35;
k.I = 10;

% Yaw
k.y = 0.8;
k.wy = 0.15;
k.yI = 2;

sigma = params.sigma;
c1 = params.c1;
m = params.mass;
g = params.gravity;
e3 = [0, 0, 1]';

error.pos = state.pos - des_state.pos;
error.vel = state.vel - des_state.vel;
A = - k.x * error.pos ...
    - k.v * error.vel ...
    - m * g * e3 ...
    + m * des_state.acc ...
    - k.i * sat(sigma, state.ei);

ei_dot = error.vel + c1 * error.pos;
b3 = state.rotm * e3;
F = -dot(A, b3);
ea = g * e3 ...
    - F / m * b3 ...
    - des_state.acc ...
    + params.x_delta / m;
A_dot = - k.x * error.vel ...
    - k.v * ea ...
    + m * des_state.jerk ...
    - k.i * satdot(sigma, state.ei, ei_dot);

ei_ddot = ea + c1 * error.vel;
b3_dot = state.rotm * hat(state.omega) * e3;
f_dot = -dot(A_dot, b3) - dot(A, b3_dot);
eb = - f_dot / m * b3 - F / m * b3_dot - des_state.jerk;
A_ddot = - k.x * ea ...
    - k.v * eb ...
    + m * des_state.snap ...
    - k.i * satdot(sigma, state.ei, ei_ddot);

[b3c, b3c_dot, b3c_ddot] = deriv_unit_vector(-A, -A_dot, -A_ddot);

A2 = -hat(des_state.b1) * b3c;
A2_dot = -hat(des_state.b1_dot) * b3c - hat(des_state.b1) * b3c_dot;
A2_ddot = - hat(des_state.b1_2dot) * b3c ...
    - 2 * hat(des_state.b1_dot) * b3c_dot ...
    - hat(des_state.b1) * b3c_ddot;

[b2c, b2c_dot, b2c_ddot] = deriv_unit_vector(A2, A2_dot, A2_ddot);

b1c = hat(b2c) * b3c;
b1c_dot = hat(b2c_dot) * b3c + hat(b2c) * b3c_dot;
b1c_ddot = hat(b2c_ddot) * b3c ...
    + 2 * hat(b2c_dot) * b3c_dot ...
    + hat(b2c) * b3c_ddot;

Rc = [b1c, b2c, b3c];
Rc_dot = [b1c_dot, b2c_dot, b3c_dot];
Rc_ddot = [b1c_ddot, b2c_ddot, b3c_ddot];

Wc = vee(Rc' * Rc_dot);
Wc_dot = vee(Rc' * Rc_ddot - hat(Wc)^2);

W3 = dot(state.rotm * e3, Rc * Wc);
W3_dot = dot(state.rotm * e3, Rc * Wc_dot) ...
    + dot(state.rotm * hat(state.omega) * e3, Rc * Wc);

% Decoupled Yaw Attitude Controller
[M, eI_dot, error.R, error.W] = attitude_control( ...
    state.rotm, state.omega, state.eI, ...
    Rc, Wc, Wc_dot, ...
    k, params);
error.y = 0;
error.Wy = 0;
end
