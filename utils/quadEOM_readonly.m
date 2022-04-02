function sdot = quadEOM_readonly(t, s, F, M, params)
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
