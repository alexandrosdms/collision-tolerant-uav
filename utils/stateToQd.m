function [qd] = stateToQd(x)
%Converts qd struct used in hardware to x vector used in simulation
% x is 1 x 24 vector of state variables [pos vel omega rotm ei eI]
% qd is a struct including the fields pos, vel, rotm, and omega

%current state
qd.pos = x(1:3);
qd.vel = x(4:6);

Rot = reshape(x(10:18),3,3);
qd.rotm = Rot;

qd.omega = x(7:9);

end
