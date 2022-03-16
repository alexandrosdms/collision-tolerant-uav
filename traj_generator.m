function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent waypoints0 traj_time d0 coffx coffy coffz

if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;

%     coffs = xlsread('coefficients_correct.xlsx');
%     coffx = coffs(:,1);
%     coffy = 
%     coffz = ;
    [coffx,A,b] = getCoff(waypoints0(1,:));
%     xlswrite('A2.xlsx',A);
%     xlswrite('b2.xlsx',b);
    coffy = getCoff(waypoints0(2,:));
    coffz = getCoff(waypoints0(3,:));
else
    if(t > traj_time(end))
        t = traj_time(end)-0.001;
    end
    t_index = find(traj_time >= t,1); % return the first element that sutisfies the condition

    if(t_index > 1)
        t = t - traj_time(t_index-1);
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
    else
        scale = t/d0(t_index-1);
        segment_index = t_index-1;
        % desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%         desired_state.pos = [polyT(8,0,t)*coffx((segment_index-1)*8+1:segment_index*8,1);...
%             polyT(8,0,t)*coffy((segment_index-1)*8+1:segment_index*8,1);...
%             polyT(8,0,t)*coffz((segment_index-1)*8+1:segment_index*8,1)];
%         desired_state.vel = [polyT(8,1,t)*coffx((segment_index-1)*8+1:segment_index*8,1);...
%             polyT(8,1,t)*coffy((segment_index-1)*8+1:segment_index*8,1);...
%             polyT(8,1,t)*coffz((segment_index-1)*8+1:segment_index*8,1)];
%         desired_state.acc = [polyT(8,2,t)*coffx((segment_index-1)*8+1:segment_index*8,1);...
%             polyT(8,2,t)*coffy((segment_index-1)*8+1:segment_index*8,1);...
%             polyT(8,2,t)*coffz((segment_index-1)*8+1:segment_index*8,1)];

        dt_dt = 1/(traj_time(t_index-1));
%         vel_corr = [1 1 dt_dt dt_dt dt_dt dt_dt dt_dt dt_dt];
%         acc_corr = vel_corr.^2;
        desired_state.pos = [polyT(8,0,scale)*coffx((segment_index-1)*8+1:segment_index*8,1);...
            polyT(8,0,scale)*coffy((segment_index-1)*8+1:segment_index*8,1);...
            polyT(8,0,scale)*coffz((segment_index-1)*8+1:segment_index*8,1)];
        desired_state.vel = [polyT(8,1,scale)*coffx((segment_index-1)*8+1:segment_index*8,1);...
            polyT(8,1,scale)*coffy((segment_index-1)*8+1:segment_index*8,1);...
            polyT(8,1,scale)*coffz((segment_index-1)*8+1:segment_index*8,1)].* (1/d0(segment_index)) ;
        desired_state.acc = [polyT(8,2,scale)*coffx((segment_index-1)*8+1:segment_index*8,1);...
            polyT(8,2,scale)*coffy((segment_index-1)*8+1:segment_index*8,1);...
            polyT(8,2,scale)*coffz((segment_index-1)*8+1:segment_index*8,1)].* (1/d0(segment_index)) ;
    end
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
%


%% Fill in your code here

% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end

function [coff, A, b] = getCoff(waypoints)
    n = size(waypoints,2)-1; % number of segments P1..n
    A = zeros(8*n, 8*n);
    b = zeros(1,8*n);
    
    % --------------------------------------------
    
    % YOUR CODE HERE 
    % Fill A and b matices with values using loops
    
    % Example for first 4 equations:
    
    column = 1;
    for i=1:n
        b(1,column) = waypoints(i);
        column = column + 1;
    end
    
    row = 1;
    for i=1:n
        A(row,8*(i-1)+1:8*i) = polyT(8,0,0); 
        row = row + 1;
    end
    
    % --------------------------------------------

    % Second set of equations
    
    for i = 1:n
        b(1,column) = waypoints(i+1);
        column = column + 1;
    end

    for i=1:n
        A(row,8*(i-1)+1:8*i) = polyT(8,0,1);
        row = row + 1;
    end

    % --------------------------------------------

    % Third set of equations

    for i = 1:n-1
        A(row, 1:8) = polyT(8,i,0);
        row = row + 1;
    end

    % --------------------------------------------

    % Fourth set of equations

    for i = 1:n-1
        A(row,25:32) = polyT(8,i,1);
        row = row + 1;
    end

    % --------------------------------------------

    % Fifth set of equations

    for i = 1:n+2
        for j = 1:n-1
            A(row, 8*(j-1)+1:8*j) = polyT(8,i,1);
            A(row, 8*j+1:8*(j+1)) = -polyT(8,i,0);
            row = row + 1;
        end
    end
    
    % --------------------------------------------
    coff = inv(A)*b';


end

function [T] = polyT(n,k,t)
    %n is the polynom number of coefficients, k is the requested derivative and
    %t is the actual value of t (this can be anything, not just 0 or 1).
    T = zeros(n,1);
    D = zeros(n,1);
    %Init:
    for i=1:n
    D(i) = i-1;
    T(i) = 1;
    end
    %Derivative:
    for j=1:k
        for i=1:n
            T(i) = T(i) * D(i);
    
            if D(i) > 0
                D(i) = D(i) - 1;
            end
        end
    end
    %put t value
    for i=1:n
    T(i) = T(i) * t^D(i);
    end
    T = T';
end