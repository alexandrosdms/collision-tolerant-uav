function [ s ] = init_state( start, yaw )
%INIT_STATE Initialize 24 x 1 state vector

s     = zeros(13,1);
Rot0   = expm(pi * hat([0, 0, 1]'));
s(1)  = start(1);  %x
s(2)  = start(2);  %y
s(3)  = start(3);  %z
s(4)  = 0;         %xdot
s(5)  = 0;         %ydot
s(6)  = 0;         %zdot
s(7) = 0;         %p
s(8) = 0;         %q
s(9) = 0;         %r
s(10:18) = reshape(Rot0, 9, 1);
s(19) = 0;         %ei1
s(20) = 0;         %ei2
s(21) = 0;         %ei3
s(22) = 0;         %eI1
s(23) = 0;         %eI2
s(24) = 0;         %eI3

end