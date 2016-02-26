addpath(genpath('.'));

MASS = 0.230; % 230g with camera
%MASS = 0.176; % 176g no camera

% when true, stop at each waypoint; when false, fit spline
% safer to use larger TARGET_ACC, like 4, if SIMPLE_TRAJ=true
SIMPLE_TRAJ = false;

% if > 0, create intermediate waypoints so that the maximum distance
% between consecutive waypoints is bounded by this value
% this is only useful when SIMPLE_TRAJ=false and the path has few points
% and sharp turns; in this case the refined path will keep the trajectory
% closer to the line between waypoints and won't make large curves far away
% from desired path
MAX_WAYPT_DIST = 0;

% target acceleration for trajectory. 4 is very agressive.
TARGET_ACC = 3;