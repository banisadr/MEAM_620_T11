function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.

% pairwise quintic; start and stop at end of each leg
% WARNING: djikstra.m must also have this option on!!!
SIMPLE_TRAJ = false;
AVGSPEED = 10;
MAXACC = 4; % passed all tests with 4
TSTEP = 1e-2; % to store traj
EPS = 1e-6;

persistent map0 path0
persistent times pos vel acc

if isempty(t) && isempty(qn)
    % init call
    map0 = map;
    path0 = path;

    times = [];
    pos = [];
    vel = [];
    acc = [];

    path = path{1};

    npts = size(path,1);

    if isempty(path)
        return;
    elseif size(path,1) == 1
        times = [0, 1];
        pos = [path; path];
        vel = [0,0,0; 0,0,0];
        acc = [0,0,0; 0,0,0];
        return
    end

    %% simpler trajectory: start/stop at each
    if SIMPLE_TRAJ
        MINSPEED = .1;
        MAXACC = 4;
        ACCMARGIN = 0.1;
        for(i=1:size(path,1)-1)
            % find bst time to cover this leg
            len = norm(path(i+1,:)-path(i,:));
            tlim = [0, len/MINSPEED];
            converged = false;
            while ~converged
                T = mean(tlim);
                T = ceil(T/TSTEP)*TSTEP; % round to TSTEP
                [curr_t, curr_p, curr_v, curr_a] = fit_traj([0,T], [path(i,:); path(i+1,:)], TSTEP);
                accT = max(sqrt(sum(curr_a.^2,2)));
                if accT > MAXACC + ACCMARGIN
                    tlim(1) = T;
                elseif accT < MAXACC - ACCMARGIN
                    tlim(2) = T;
                else
                    converged = true;
                end
                if tlim(2)-tlim(1) <= 2*TSTEP
                    converged = true;
                end
            end

            [curr_t, curr_p, curr_v, curr_a] = fit_traj([0,T], [path(i,:); path(i+1,:)], TSTEP);
            if isempty(times)
                times = [times, curr_t];
            else
                times = [times, times(end)+TSTEP+curr_t];
            end
            pos = [pos; curr_p];
            vel = [vel; curr_v];
            acc = [acc; curr_a];
        end
        return;
    end

    %% find desirable velocity at each point
    leglen = sqrt(sum(diff(path).^2,2));
    des_v = AVGSPEED*ones([npts,1]); % each pt has desited velocity
    acc_sign = zeros([npts,1]); % accelerating or decelarating?
    % handle 1st acceleration
    curr_v = 0;
    i = 1;
    while curr_v < AVGSPEED && i <= npts-1
        des_v(i) = curr_v;
        t = get_time_cte_acc(leglen(i), curr_v, MAXACC);
        curr_v = min([curr_v + t*MAXACC, AVGSPEED]);
        i = i+1;
    end

    % handle last acceleration
    curr_v = 0;
    i = npts;
    while curr_v < AVGSPEED && i >=2
        if (des_v(i) < curr_v)
            break;
        end
        des_v(i) = min(curr_v, des_v(i));
        t = get_time_cte_acc(leglen(i-1), curr_v, MAXACC);
        curr_v = min([curr_v + t*MAXACC, AVGSPEED]);
        i = i-1;
    end

    % handle accelerating/breaking around corners
    for i=2:npts-1
        back = path(i,:) - path(i-1,:);
        forw = path(i+1,:) - path(i,:);
        if dot(back,forw)/norm(back)/norm(forw) < 0.99
            % radius of turn
            r = radius_3pts(path(i-1,:), path(i,:), path(i+1,:));
            curr_v = min(sqrt(MAXACC*r), des_v(i));

            des_v(i) = curr_v;

            % walk around corner finding desired velocity
            j = i-1; % walk left
            while curr_v < AVGSPEED && j >= 2
                if (des_v(j) < curr_v)
                    break;
                end
                des_v(j) = min(des_v(j), curr_v);
                t = get_time_cte_acc(leglen(j-1), curr_v, MAXACC);
                curr_v = min([curr_v + t*MAXACC, AVGSPEED]);
                j = j-1;
            end
            j = i+1; % walk right
            curr_v = des_v(i);
            while curr_v < AVGSPEED && j <= npts
                if (des_v(j) < curr_v)
                    break;
                end
                des_v(j) = min(des_v(j), curr_v);
                t = get_time_cte_acc(leglen(j), curr_v, MAXACC);
                curr_v = min([curr_v + t*MAXACC, AVGSPEED]);
                j = j+1;
            end

        end
    end

    % find desired times given velocities
    for(i=1:npts-1)
        des_t(i) = get_time_vel_diff(leglen(i), des_v(i), des_v(i+1));
        if ~isreal(get_time_vel_diff(leglen(i), des_v(i), des_v(i+1)))
            error('value not real!');
        end
    end

    intervals = des_t';

    % warning: got instability when rounding this
    %knots_t = ceil(cumsum([0; intervals])/TSTEP)*TSTEP;
    knots_t = cumsum([0; intervals]);
    % start and final point are the same, only two points
    [times, pos, vel, acc] = fit_traj(knots_t, path, TSTEP);

    % for extrapolation
    times = [times,times(end)+TSTEP,times(end)+2];
    pos = [pos; pos(end,:); pos(end,:)];
    vel = [vel; zeros([2,3])];
    acc = [acc; zeros([2,3])];

    % edge case where all points are the same -- no movement required.
    if numel(times) == 1
        times = [0, 1];
        pos = [path(1:2,:)];
        vel = [0,0,0; 0,0,0];
        acc = [0,0,0; 0,0,0];
    end

    % debug
    % figure;
    % subplot(3,1,1); plot(times, pos);
    % subplot(3,1,2); plot(times, vel);
    % subplot(3,1,3); plot(times, acc);
    % legend('x', 'y', 'z');
    % figure;
    % plot3(pos(:,1), pos(:,2), pos(:,3), path(:,1), path(:,2), path(:,3));
else
    if(~isempty(times))
        desired_state.pos = interp1(times, pos, t, 'linear', 'extrap')';
        desired_state.vel = interp1(times, vel, t, 'linear', 'extrap')';
        desired_state.acc = interp1(times, acc, t, 'linear', 'extrap')';
    else
        desired_state.pos = [0;0;0];
        desired_state.vel = [0;0;0];
        desired_state.acc = [0;0;0];
    end
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end

end

function [times, pos, vel, acc, jer, sna] = fit_traj(knots_t, path, TSTEP)
    [tx, px, vx, ax, jx, sx] = fit_quintic(path(:,1), knots_t, [0,0], [0,0], TSTEP);
    [ty, py, vy, ay, jy, sy] = fit_quintic(path(:,2), knots_t, [0,0], [0,0], TSTEP);
    [tz, pz, vz, az, jz, sz] = fit_quintic(path(:,3), knots_t, [0,0], [0,0], TSTEP);

    % if any(tx ~= ty) || any(ty ~= tz)
    %     error('fit_quintic outputs not matching!');
    % end

    pos = [px', py', pz'];
    vel = [vx', vy', vz'];
    acc = [ax', ay', az'];
    jer = [jx', jy', jz'];
    sna = [sx', sy', sz'];
    times = tx;
end

function t = get_time_cte_acc(x, v, a);
% compute time to cover distance X at contant acceleration A starting with velocity V
  % some tiny imaginary part may appear due to numerical errors
  t = real(roots([a/2, v, -x]));
  assert(~all(t < 0));
  t = min(t(t >= 0));
end

function t = get_time_vel_diff(x, v1, v2);
% compute time to cover distance X at constant acceleration A starting with velocity V
  MAXACC = 3;
  EPS = 1e-6;

  if v1 < EPS && v2 < EPS
      t = sqrt(4*x/MAXACC);
      return;
  end
  a = (v2^2-v1^2)/2/x;
  t = get_time_cte_acc(x, v1, a);
end


function r = radius_3pts(p1, p2, p3)
% radius of circle between 3 points is product of sides over 4 times the area
% see: http://www.mathworks.com/matlabcentral/newsreader/view_thread/128429
  a = norm(p1-p2);
  b = norm(p1-p3);
  c = norm(p2-p3);
  s = (a+b+c)/2;
  A = sqrt(s*(s-a)*(s-b)*(s-c)); % Area of triangle
  r = a*b*c/(4*A); % Radius of circumscribing circle
end
