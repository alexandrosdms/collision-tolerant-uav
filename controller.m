% 
% Copyright (c) 2020 Flight Dynamics and Control Lab
% 
% Permission is hereby granted, free of charge, to any person obtaining a
% copy of this software and associated documentation files (the 
% "Software"), to deal in the Software without restriction, including 
% without limitation the rights to use, copy, modify, merge, publish, 
% distribute, sublicense, and/or sell copies of the Software, and to permit
% persons to whom the Software is furnished to do so, subject to the 
% following conditions:
% 
% The above copyright notice and this permission notice shall be included
%  in all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
% OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
% MERCHANTABILITY,FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
% IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
% CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
% TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
% SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

function [F, M, ei_dot, eI_dot, error, calculated] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
% Position controller that uses decoupled-yaw controller as the attitude
% controller
% 
%   Caluclates the force and moments required for a UAV to reach a given 
%   set of desired position commands using a decoupled-yaw controller
%   defined in https://ieeexplore.ieee.org/document/8815189.
%
%   Inputs:
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.omega = [p; q; r], state.rotm = [r11:r13,r21:r23,r31:r33]',
%   state.ei = [ei1; ei2; ei3], state.eI = [eI1; eI2; eI3]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot],
%   des_state.jerk = [x_3dot; y_3dot; z_3dot],
%   des_state.snap = [x_4dot; y_4dot; z_4dot],
%   des_state.b1 = [b1_1; b1_2; b1_3], des_state.b1_dot, des_state.b1_2dot
%
%   params: robot parameters
%  Outputs:
%    F: (scalar) required motor force
%    M: (3x1 matrix) control moment required to reach desired conditions
%    ei_dot: (3x1 matrix) position integral change rate
%    eI_dot: (3x1 matrix) attitude integral change rate
%    error: (struct) errors for attitude and position control (for data
%    logging)
%    calculated: (struct) calculated desired commands (for data logging)

% Use this flag to enable or disable the decoupled-yaw attitude controller.
use_decoupled = true;

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

%% Run attitude controller
if use_decoupled
    [M, eI_dot, error.b, error.W, error.y, error.Wy] ...
        = attitude_control_decoupled_yaw( ...
        state.rotm, state.omega, state.eI, ...
        b3c, b3c_dot, b3c_ddot, b1c, W3, W3_dot, ...
        k, params);
    
    % Only used for comparison between two controllers
    error.R = 1 / 2 * vee(Rc' * state.rotm - state.rotm' * Rc);
else
    [M, eI_dot, error.R, error.W] = attitude_control( ...
        state.rotm, state.omega, state.eI, ...
        Rc, Wc, Wc_dot, ...
        k, params);
    error.y = 0;
    error.Wy = 0;
end

%% Saving data
calculated.b3 = b3c;
calculated.b3_dot = b3c_dot;
calculated.b3_ddot = b3c_ddot;
calculated.b1 = b1c;
calculated.R = Rc;
calculated.W = Wc;
calculated.W_dot = Wc_dot;
calculated.W3 = dot(state.rotm * e3, Rc * Wc);
calculated.W3_dot = dot(state.rotm * e3, Rc * Wc_dot) ...
    + dot(state.rotm * hat(state.omega) * e3, Rc * Wc);
end
