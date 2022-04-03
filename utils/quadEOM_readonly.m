function sdot = quadEOM_readonly(t, s, F, M, ei_dot, eI_dot, params)
% QUADEOM_READONLY Solve quadrotor equation of motion
%   quadEOM_readonly calculate the derivative of the state vector
%
% INPUTS:
% t      - 1 x 1, time
% s      - 24 x 1, state vector = [x, y, z, xd, yd, zd, p, q, r, reshape(R,1,9), ei, eI]
% F      - 1 x 1, thrust output from controller (only used in simulation)
% M      - 3 x 1, moments output from controller (only used in simulation)
% params - struct, output from nanoplus() and whatever parameters you want to pass in
%
% OUTPUTS:
% sdot   - 24 x 1, derivative of state vector s
%
% NOTE: You should not modify this function
% See Also: quadEOM_readonly, nanoplus

%************ EQUATIONS OF MOTION ************************
% Limit the force and moments due to actuator limits
A = [0.25,                      0, -0.5/params.arm_length;
     0.25,  0.5/params.arm_length,                      0;
     0.25,                      0,  0.5/params.arm_length;
     0.25, -0.5/params.arm_length,                      0];

prop_thrusts = A*[F;M(1:2)]; % Not using moment about Z-axis for limits
prop_thrusts_clamped = max(min(prop_thrusts, params.maxF/4), params.minF/4);


B = [                 1,                 1,                 1,                  1;
                      0, params.arm_length,                 0, -params.arm_length;
     -params.arm_length,                 0, params.arm_length,                 0];
F = B(1,:)*prop_thrusts_clamped;
M = [B(2:3,:)*prop_thrusts_clamped; M(3)];

e3 = [0, 0, 1]';
m = params.mass;
I = params.I;

[~, v, R, W, ~, ~] = split_to_states(s);

xdot = v;
vdot = params.gravity * e3 ...
    - F / m * R * e3 + params.x_delta / m;
Wdot = I \ (-hat(W) * I * W + M + params.R_delta);
Rdot = R * hat(W);

sdot=[xdot; vdot; Wdot; reshape(Rdot,9,1); ei_dot; eI_dot];
end
