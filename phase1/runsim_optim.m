function pos_err = runsim_optim(traj, gains)
% runsim_optim - run simulation for opitmization; no plots, return error

addpath('utils')
addpath('trajectories')

% You need to implement trajhandle and controlhandle

% trajectory generator
trajhandle = traj;

% controller
gains
controlhandle = @(qd, t, qn, params) controller_optim(qd, t, qn, params, gains);

% real-time 
real_time = false;

% *********** YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW **********
% number of quadrotors
nquad = 1;

% max time
time_tol = 30;

% parameters for simulation
params = nanoplus();

%% **************************** FIGURES *****************************
h_3d = gca;
quadcolors = lines(nquad);

set(gcf,'Renderer','OpenGL')

%% *********************** INITIAL CONDITIONS ***********************
fprintf('Setting initial conditions...\n')
max_iter  = 5000;      % max iteration
starttime = 0;         % start of simulation in seconds
tstep     = 0.01;      % this determines the time step at which the solution is given
cstep     = 0.05;      % image capture time interval
nstep     = cstep/tstep;
time      = starttime; % current time
err = []; % runtime errors
for qn = 1:nquad
    % Get start and stop position
    des_start = trajhandle(0, qn);
    des_stop  = trajhandle(inf, qn);
    stop{qn}  = des_stop.pos;
    x0{qn}    = init_state( des_start.pos, 0 );
    xtraj{qn} = zeros(max_iter*nstep, length(x0{qn}));
    ttraj{qn} = zeros(max_iter*nstep, 1);
end

x         = x0;        % state

pos_tol   = 0.01;
vel_tol   = 0.01;

%% ************************* RUN SIMULATION *************************
fprintf('Simulation Running....')
% Main loop
for iter = 1:max_iter

    timeint = time:tstep:time+cstep;

    tic;
    % Iterate over each quad
    for qn = 1:nquad
        % Initialize quad plot
        if iter == 1
            QP{qn} = QuadPlot(qn, x0{qn}, 0.1, 0.04, quadcolors(qn,:), max_iter, h_3d);
            desired_state = trajhandle(time, qn);
            QP{qn}.UpdateQuadNoPlot(x{qn}, [desired_state.pos; desired_state.vel], time);
        end

        % Run simulation
        [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, qn, controlhandle, trajhandle, params), timeint, x{qn});
        x{qn}    = xsave(end, :)';
        
        % Save to traj
        xtraj{qn}((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
        ttraj{qn}((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);

        % Update quad plot
        desired_state = trajhandle(time + cstep, qn);
        QP{qn}.UpdateQuadNoPlot(x{qn}, [desired_state.pos; desired_state.vel], time + cstep);
    end
    time = time + cstep; % Update simulation time
    t = toc;
    % Check to make sure ode45 is not timing out
    if(t> cstep*50)
        err = 'Ode45 Unstable';
        break;
    end

    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end

    % Check termination criteria
    if terminate_check(x, time, stop, pos_tol, vel_tol, time_tol)
        break
    end
end

%% ************************* POST PROCESSING *************************
% Truncate xtraj and ttraj
for qn = 1:nquad
    xtraj{qn} = xtraj{qn}(1:iter*nstep,:);
    ttraj{qn} = ttraj{qn}(1:iter*nstep);
end

pos_err = (QP{1}.state_hist(1:3,:) - QP{1}.state_des_hist(1:3,:))';
pos_err = sum(arrayfun(@(x)norm(pos_err(x,:)), 1:size(pos_err,1)));

if(~isempty(err))
    pos_err = 1000;
end

close all;