function [qd] = stateToQd(x)
%Converts x vector used in simulation to qd struct used in hardware
% x is 1 x 24 vector of state variables [pos vel quat omega]
% qd is a struct including the fields pos, vel, euler, and omega

Rot = reshape(x(10:18),3,3);

%current state
qd.pos      = x(1:3);
qd.vel      = x(4:6);
qd.omega    = x(7:9);
qd.rotm     = Rot;
qd.ei       = x(19:21);
qd.eI       = x(22:24);

end
