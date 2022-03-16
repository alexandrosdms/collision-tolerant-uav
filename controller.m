function [F, M] = controller(t, state, des_state, params)
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


% =================== Your code goes here ===================
% Tuning gains for line and helix
Kp_p = [30 30 30];
Kv_p = [400 400 400];

Kp_a = [800 800 800];
Kv_a = [1200 1200 1200];

% Kp_p = [10 10 80];
% Kv_p = [10 10 35];
% 
% Kp_a = [200 200 200];
% Kv_a = [.1 .1 .1];
% Givens
psi_des = des_state.yaw;
r_des = des_state.yawdot;
p_des = 0;
q_des = 0;

for i = 1:3
    rddot_des(i) = des_state.acc(i) + Kv_p(i)*(des_state.vel(i)-state.vel(i)) ...
        + Kp_p(i)*(des_state.pos(i)-state.pos(i));
end

phi_des = (rddot_des(1)*sin(psi_des) ...
            - rddot_des(2)*cos(psi_des))/params.gravity; %14a

theta_des = (rddot_des(1)*cos(psi_des)...
            + rddot_des(2)*sin(psi_des))/params.gravity; %14b

% Inputs
u1 = params.mass * params.gravity +...
    params.mass * rddot_des(3);
    
u2 = [Kp_a(1)*(phi_des-state.rot(1)) + Kv_a(1)*(p_des-state.omega(1));... % Eq 10
      Kp_a(2)*(theta_des-state.rot(2)) + Kv_a(2)*(q_des-state.omega(2));
      Kp_a(3)*(psi_des-state.rot(3)) + Kv_a(3)*(r_des-state.omega(3))];

% Thrust
F = u1;

% Moment
M = params.I*u2;

% =================== Your code ends here ===================

end
