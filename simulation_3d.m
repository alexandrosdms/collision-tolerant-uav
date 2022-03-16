function [t_out, s_out] = simulation_3d(trajhandle, controlhandle)
% NOTE: This srcipt will not run as expected unless you fill in proper
% code in trajhandle and controlhandle
% You should not modify any part of this script except for the
% visualization part
%
% ***************** QUADROTOR SIMULATION *****************

% *********** YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW **********

addpath('utils');

video = false;

% real-time
real_time = true;

% max time
max_time = 20;

% parameters for simulation
params = sys_params;

%% **************************** FIGURES *****************************
disp('Initializing figures...');
if video
    video_writer = VideoWriter('test_control_3d.avi', 'Uncompressed AVI');
    open(video_writer);
end
h_fig = figure;
h_3d = subplot(3,3,[1,2,4,5,7,8]);
axis equal
grid on
view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(1);

set(gcf,'Renderer','OpenGL')

%% *********************** INITIAL CONDITIONS ***********************
disp('Setting initial conditions...');
tstep    = 0.01; % this determines the time step at which the solution is given
cstep    = 0.05; % image capture time interval
max_iter = max_time/cstep; % max iteration
nstep    = cstep/tstep;
time     = 0; % current time
err = []; % runtime errors

% Get start and stop position
des_start = trajhandle(0, []);
des_stop  = trajhandle(inf, []);
stop_pos  = des_stop.pos;
x0    = init_state(des_start.pos, 0);
xtraj = zeros(max_iter*nstep, length(x0));
ttraj = zeros(max_iter*nstep, 1);

x       = x0;        % state

pos_tol = 0.01;
vel_tol = 0.01;

%% ************************* RUN SIMULATION *************************
disp('Simulation Running....');
% Main loop
for iter = 1:max_iter

    timeint = time:tstep:time+cstep;

    tic;

    % Initialize quad plot
    if iter == 1
        pause;
        subplot(3,3,[1,2,4,5,7,8]);
        QP = QuadPlot(1, x0, 0.1, 0.04, quadcolors(1,:), max_iter, h_3d);
        current_state = stateToQd(x);
        desired_state = trajhandle(time, current_state);
        QP.UpdateQuadPlot(x, [desired_state.pos; desired_state.vel], time);
        h_title = title(h_3d, sprintf('iteration: %d, time: %4.2f', iter, time));
    end

    % Run simulation
    [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, controlhandle, trajhandle, params), timeint, x);
    x    = xsave(end, :)';

    % Save to traj
    xtraj((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
    ttraj((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);

    % Update quad plot
    current_state = stateToQd(x);
    desired_state = trajhandle(time + cstep, current_state);
    QP.UpdateQuadPlot(x, [desired_state.pos; desired_state.vel], time + cstep);
    subplot(3,3,[1,2,4,5,7,8]);
    set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep))

    time = time + cstep; % Update simulation time
    if video
        writeVideo(video_writer, getframe(h_fig));
    end
    
    line_width = 2;
    line_color = 'r';
    line_color_reference = 'b';

%     subplot(3,3,3)
%     plot(ttraj(1:iter*nstep), xtraj(1:iter*nstep,1), LineWidth=line_width, Color=line_color);
%     xlabel('t [s]'); ylabel('x [m]');
%     grid on;
%     subplot(3,3,6)
%     plot(ttraj(1:iter*nstep), xtraj(1:iter*nstep,2), LineWidth=line_width, Color=line_color);
%     xlabel('t [s]'); ylabel('y [m]');
%     grid on;
%     subplot(3,3,9)
%     plot(ttraj(1:iter*nstep), 180/pi*xtraj(1:iter*nstep,3), LineWidth=line_width, Color=line_color);
%     grid on;
%     xlabel('t [s]'); ylabel('z [m]');

    t = toc;
    % Check to make sure ode45 is not timing out
    if(t> cstep*500)
        err = 'Ode45 Unstable';
        break;
    end

    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end

    % Check termination criteria
    if terminate_check(x, time, stop_pos, pos_tol, vel_tol, max_time)
        break
    end
end

%% ************************* POST PROCESSING *************************
% Truncate xtraj and ttraj
xtraj = xtraj(1:iter*nstep,:);
ttraj = ttraj(1:iter*nstep);

% Truncate saved variables
QP.TruncateHist();

% Plot position
h_pos = figure('Name', ['Quad position']);
plot_state(h_pos, QP.state_hist(1:3,:), QP.time_hist, 'pos', 'vic');
plot_state(h_pos, QP.state_des_hist(1:3,:), QP.time_hist, 'pos', 'des');
% Plot velocity
h_vel = figure('Name', ['Quad velocity']);
plot_state(h_vel, QP.state_hist(4:6,:), QP.time_hist, 'vel', 'vic');
plot_state(h_vel, QP.state_des_hist(4:6,:), QP.time_hist, 'vel', 'des');
% % Plot acceleration
% h_acc = figure('Name', ['Quad acceleration']);
% plot_state(h_acc, QP.state_hist(7:9,:), QP.time_hist, 'vel', 'vic');
% plot_state(h_acc, QP.state_des_hist(7:9,:), QP.time_hist, 'vel', 'des');

if(~isempty(err))
    error(err);
end

disp('finished.')

if video
    close(video_writer);
end

t_out = ttraj;
s_out = xtraj;

end
