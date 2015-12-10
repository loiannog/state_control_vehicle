addpath(genpath('~/git/trajgen/matlab'));
clear
close all
clc

z_clear = 2; % z-height that is considered out of ground effect
r = 0.1; % m
bottom_height = 1*r;
rest_height = bottom_height + (z_clear - bottom_height) / 2;

if 1
    % For trials in motion
    transition_time = 1.8; % s
    reverse_time = 2.9;
    bottom_distance = 2.2; % m
    bottom_velocity = 1; % m/s
    bottom_time = bottom_distance / bottom_velocity;
    xmax = 1.8; % Extent in the x-plane of the trajectory
else
    transition_time = 5; % s
    bottom_time = 3;
    bottom_distance = 0; % m
    bottom_velocity = bottom_distance / bottom_time; % m/s
    xmax = 0; % Extent in the x-plane of the trajectory
end

% The time vector
t = [0, ...
    transition_time, ...
    transition_time + bottom_time, ...
    transition_time + bottom_time + reverse_time / 2, ...
    transition_time + bottom_time + reverse_time, ...
    transition_time + 2 * bottom_time + reverse_time, ...
    2 * transition_time + 2 * bottom_time + reverse_time];

% Return
% t = [t, t(end) + t(2:end)];

% The gains
traj_kx = [3.7*ones(1,2), 8.0];
traj_kv = [2.4*ones(1,2), 3.0];

%% Waypoints
%
% Note that we only need to plan for x and z since the other dimensions are
% identically zero

d = 2;

% Trajectory Start
waypoints(1) = ZeroWaypoint(t(1), d);
waypoints(1).pos = [-xmax; rest_height];

% Start of bottom
waypoints(end+1) = ZeroWaypoint(t(2), d);
waypoints(end).pos = [-bottom_distance/2, bottom_height];
waypoints(end).vel(1) = bottom_velocity;

% End of bottom
waypoints(end+1) = ZeroWaypoint(t(3), d);
waypoints(end).pos = [bottom_distance/2, bottom_height];
waypoints(end).vel(1) = bottom_velocity;

% Trajectory End (xmax)
waypoints(end+1) = NanWaypoint(t(4), d);
waypoints(end).pos = [xmax; rest_height];

%% Trajectory Return

% Start of Top
waypoints(end+1) = ZeroWaypoint(t(5), d);
waypoints(end).pos = [bottom_distance/2; z_clear];
waypoints(end).vel(1) = -bottom_velocity;

% End of Top
waypoints(end+1) = ZeroWaypoint(t(6), d);
waypoints(end).pos = [-bottom_distance/2; z_clear];
waypoints(end).vel(1) = -bottom_velocity;

% xmin
waypoints(end+1) = ZeroWaypoint(t(7), d);
waypoints(end).pos = [-xmax; rest_height];

% xbound = xmax + 0.001;
% bounds = SetBound([t(1), t(2)], 'pos', 'lb', [-xbound, nan]);
% bounds(end+1) = SetBound([t(3), t(4)], 'pos', 'ub', [xbound, nan]);
% bounds(end+1) = SetBound([t(4), t(5)], 'pos', 'ub', [xbound, nan]);
% bounds(end+1) = SetBound([t(6), t(7)], 'pos', 'lb', [-xbound, nan]);

options = {'ndim',2 ,'order',11, 'minderiv',[4,4]};

% Generate the trajectory
traj = trajgen(waypoints, options);

%% Plotting

PlotTraj(traj)

ntraj = TrajEval(traj, 0:0.001:traj.keytimes(end));

figure();
plot(ntraj(:,1,1), ntraj(:,2,1), '.')
axis equal

% Some safety checks
max_acc = max(sqrt(ntraj(:,1,3).^2 + ntraj(:,2,3).^2));
fprintf('Max Acceleration: %2.2f m/s^s\n', max_acc);

%% Output to a csv

gains = [bsxfun(@times, ones(size(ntraj,1), 1), traj_kx), ...
         bsxfun(@times, ones(size(ntraj,1), 1), traj_kv)];

% Generate the array
zvec = zeros(size(ntraj,1), 1);
array = [...
    ntraj(:,1,1), zvec, ntraj(:,2,1), zvec, ...
    ntraj(:,1,2), zvec, ntraj(:,2,2), zvec, ...
    ntraj(:,1,3), zvec, ntraj(:,2,3), zvec, ...
    gains];

% Write to file and write dimensions on the first line
filename = 'traj.csv';
csvwrite(filename, array);
system(['printf "', num2str(size(array,1)), ',', '4,3,6\n',...
    '$( cat ', filename, ' )" > ', filename]);
save('workspace.mat')