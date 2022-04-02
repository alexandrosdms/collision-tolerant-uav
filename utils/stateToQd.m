function [qd] = stateToQd(x)
<<<<<<< HEAD
%Converts x vector used in simulation to qd struct used in hardware
% x is 1 x 13 vector of state variables [pos vel quat omega]
% qd is a struct including the fields pos, vel, euler, and omega
=======
%Converts qd struct used in hardware to x vector used in simulation
% x is 1 x 24 vector of state variables [pos vel omega rotm ei eI]
% qd is a struct including the fields pos, vel, rotm, and omega
>>>>>>> change-state-representation

%current state
qd.pos = x(1:3);
qd.vel = x(4:6);

Rot = reshape(x(10:18),3,3);
qd.rotm = Rot;

qd.omega = x(7:9);

end
