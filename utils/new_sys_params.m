function params = new_sys_params()
m = 2;
g = 9.81;
d = 0.169; %m
I1 = 0.02;
I2 = 0.02;
I3 = 0.04;
I = diag([I1, I2, I3]);

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.gravity = g;
params.arm_length = d;

% Save these for when thrust moment limits are introduced to the EOM
% params.minF = 0.0;
% params.maxF = 2.0*m*g;

% Verify what these signify
params.ctf = 0.0135;
% Fixed disturbance
params.x_delta = [0.5, 0.8, -1];
param.R_delta = [0.2, 1.0, -0.1]';

% Geometric Controller
params.c1 = 1.5;
params.sigma = 10;
params.c2 = 2;
params.c3 = 2;
end