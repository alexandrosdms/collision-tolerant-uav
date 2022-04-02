function [desired_state] = traj_lissajous(t, state)
% TRAJ_LISSAJOUS generates a lissajous

A = 100;
B = 100;
C = 2;

d = pi/2 * 0;

a = 1;
b = 2;
c = 2;
alt = -1;

T = 10;

if t >= T
    t = T;
    pos     = [A * sin(a * t + d), B * sin(b * t), alt + C * cos(2 * t)]';
    vel     = zeros(3,1);
    acc     = zeros(3,1);
else
    pos = [A * sin(a * t + d), B * sin(b * t), alt + C * cos(2 * t)]';
    
    vel = [A * a * cos(a * t + d), ...
        B * b * cos(b * t), ...
        C * c * -sin(c * t)]';
    
    acc = [A * a^2 * -sin(a * t + d), ...
        B * b^2 * -sin(b * t), ...
        C * c^2 * -cos(c * t)]';

    jerk = [A * a^3 * -cos(a * t + d), ...
        B * b^3 * -cos(b * t), ...
        C * c^3 * sin(c * t)]';

    snap = [A * a^4 * sin(a * t + d), ...
        B * b^4 * sin(b * t), ...
        C * c^4 * cos(c * t)]';
end

yaw = 0;
yawdot = 0;

desired_state.pos   = pos(:);
desired_state.vel   = vel(:);
desired_state.acc   = acc(:);
desired_state.jerk  = jerk(:);
desired_state.snap  = snap(:);

desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end