addpath(genpath('.'));

MASS = 0.230; % 230g with camera
%MASS = 0.176; % 176g no camera

% when true, stop at each waypoint; when false, fit spline
% safer to use larger TARGET_ACC, like 4, if SIMPLE_TRAJ=true
SIMPLE_TRAJ = false;

% target acceleration for trajectory. 4 is very aagressive.
TARGET_ACC = 4;