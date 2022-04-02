function [qd] = stateToQd(x)
%Converts x vector used in simulation to qd struct used in hardware
% x is 1 x 13 vector of state variables [pos vel quat omega]
% qd is a struct including the fields pos, vel, euler, and omega

%current state
qd.pos = x(1:3);
qd.vel = x(4:6);

Rot = QuatToRot(x(7:10)');
[phi, theta, yaw] = RotToRPY_ZXY(Rot);

qd.rot = [phi; theta; yaw];
qd.omega = x(11:13);

end
