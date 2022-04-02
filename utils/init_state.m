function [ s ] = init_state( start, yaw )
%INIT_STATE Initialize 24 x 1 state vector

s     = zeros(13,1);
% phi0   = 0.0;
% theta0 = 0.0;
% psi0   = yaw;
Rot0   = expm(pi * hat([0, 0, 1]'));
s(1)  = start(1);  %x
s(2)  = start(2);  %y
s(3)  = start(3);  %z
s(4)  = 0;         %xdot
s(5)  = 0;         %ydot
s(6)  = 0;         %zdot
s(16) = 0;         %p
s(17) = 0;         %q
s(18) = 0;         %r
s(7)  = Rot0(1,1); %r11
s(8)  = Rot0(1,1); %r12
s(9)  = Rot0(1,1); %r13
s(10) = Rot0(1,1); %r21
s(11) = Rot0(1,1); %r22
s(12) = Rot0(1,1); %r23
s(13) = Rot0(1,1); %r31
s(14) = Rot0(1,1); %r32
s(15) = Rot0(1,1); %r33
s(19) = 0;         %ei1
s(20) = 0;         %ei2
s(21) = 0;         %ei3
s(22) = 0;         %eI1
s(23) = 0;         %eI2
s(24) = 0;         %eI3

end