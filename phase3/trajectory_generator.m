function [timelist, pos, vel, acc] = trajectory_generator(path, simple_traj, target_acc, max_waypt_dist, show_plots)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% path: n x 3
% simple_traj: when true, stop at each waypoint
% target_acc: target max acceleration to follow; actual acceleration commanded may be higher than this but only for short periods
% max_waypt_dist: if > 0, create intermediate waypoints so that distances are at most this value of each other
% timelist, pos, vel, acc: desired trajectory

    if exist('simple_traj') ~= 1
        simple_traj = false;
    end
    if exist('target_acc') ~= 1
        target_acc = 2;
    end
    if exist('show_plots') ~= 1
        show_plots = false;
    end

    AVGSPEED = 10;
    TSTEP = 1e-2; % to store traj
    TEXTRAP = 600; % how long to maintain last command
    EPS = 1e-6;

    timelist = [];
    pos = [];
    vel = [];
    acc = [];

    %% remove repeated points from path
    i=1;
    while (i <= size(path,1)-1)
        % find bst time to cover this leg
        len = norm(path(i+1,:)-path(i,:));
        if len < EPS
            path(i+1, :) = [];
            i=i-1;
        end
        i=i+1;
    end

    %% handle empty and 1 point paths
    if isempty(path)
        return;
    elseif size(path,1) == 1
        timelist = [0, TEXTRAP];
        pos = [path; path];
        vel = [0,0,0; 0,0,0];
        acc = [0,0,0; 0,0,0];
        return
    end

    %% simpler trajectory: start/stop at each point
    if simple_traj
        MINSPEED = .1;
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
                if accT > target_acc + ACCMARGIN
                    tlim(1) = T;
                elseif accT < target_acc - ACCMARGIN
                    tlim(2) = T;
                else
                    converged = true;
                end
                if tlim(2)-tlim(1) <= 2*TSTEP
                    converged = true;
                end
            end

            [curr_t, curr_p, curr_v, curr_a] = fit_traj([0,T], [path(i,:); path(i+1,:)], TSTEP);
            if isempty(timelist)
                timelist = [timelist, curr_t];
            else
                timelist = [timelist, timelist(end)+TSTEP+curr_t];
            end
            pos = [pos; curr_p];
            vel = [vel; curr_v];
            acc = [acc; curr_a];
        end
    else
        % refine trajectory
        if max_waypt_dist > 0
            finepath = path(1,:);
            for i=1:size(path,1)-1
                v = path(i+1,:) - path(i,:);
                % ignore repeated points
                if norm(v) < EPS
                    continue;
                end
                u = v/norm(v); % unit vector
                % we add some points along the line, according to given resolution
                ncells = ceil(norm(v)/max_waypt_dist);
                finepath = [finepath;
                            bsxfun(@plus, path(i,:), ...
                                   bsxfun(@times, ...
                                          linspace(norm(v)/ncells,norm(v),ncells)', ...
                                          repmat(u,[ncells,1])))];
            end
            path = finepath;
        end

        % handle special case where traj goes to some point and back via the same line
        % we can't fit a circle on 3 colinear points, so we add another point to make a smooth curve
        i=2;
        while i<=size(path,1)-1
            back = path(i,:) - path(i-1,:);
            forw = path(i+1,:) - path(i,:);
            % the point added is the sum of same-length parallel and perpendicular
            % components w.r.t the desired forward vector
            if dot(back,forw)/norm(back)/norm(forw) < -0.99
                path = [path(1:i, :);
                        path(i,:) + forw/2 + [-forw(2), forw(1), forw(3)]/2;
                        path(i+1:end,:)];
            end
            i=i+1;
        end

        npts = size(path,1);
        leglen = sqrt(sum(diff(path).^2,2));
        des_v = AVGSPEED*ones([npts,1]); % each pt has desited velocity
        acc_sign = zeros([npts,1]); % accelerating or decelarating?
                                    % handle 1st acceleration

        %% find desirable velocity at each point
        curr_v = 0;
        i = 1;
        while curr_v < AVGSPEED && i <= npts-1
            des_v(i) = curr_v;
            t = get_time_cte_acc(leglen(i), curr_v, target_acc);
            curr_v = min([curr_v + t*target_acc, AVGSPEED]);
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
            t = get_time_cte_acc(leglen(i-1), curr_v, target_acc);
            curr_v = min([curr_v + t*target_acc, AVGSPEED]);
            i = i-1;
        end

        % handle accelerating/breaking around corners
        for i=2:npts-1
            back = path(i,:) - path(i-1,:);
            forw = path(i+1,:) - path(i,:);
            if dot(back,forw)/norm(back)/norm(forw) < 0.99
                % radius of turn
                r = radius_3pts(path(i-1,:), path(i,:), path(i+1,:));
                curr_v = min(sqrt(target_acc*r), des_v(i));

                des_v(i) = curr_v;

                % walk around corner finding desired velocity
                j = i-1; % walk left
                while curr_v < AVGSPEED && j >= 2
                    if (des_v(j) < curr_v)
                        break;
                    end
                    des_v(j) = min(des_v(j), curr_v);
                    t = get_time_cte_acc(leglen(j-1), curr_v, target_acc);
                    curr_v = min([curr_v + t*target_acc, AVGSPEED]);
                    j = j-1;
                end
                j = i+1; % walk right
                curr_v = des_v(i);
                while curr_v < AVGSPEED && j <= npts
                    if (des_v(j) < curr_v)
                        break;
                    end
                    des_v(j) = min(des_v(j), curr_v);
                    t = get_time_cte_acc(leglen(j), curr_v, target_acc);
                    curr_v = min([curr_v + t*target_acc, AVGSPEED]);
                    j = j+1;
                end

            end
        end

        % find desired timelist given velocities
        for(i=1:npts-1)
            des_t(i) = get_time_vel_diff(leglen(i), des_v(i), des_v(i+1), target_acc);
            if ~isreal(get_time_vel_diff(leglen(i), des_v(i), des_v(i+1), target_acc))
                error('value not real!');
            end
        end

        intervals = des_t';

        % warning: got instability when rounding this
        %knots_t = ceil(cumsum([0; intervals])/TSTEP)*TSTEP;
        knots_t = cumsum([0; intervals]);
        % start and final point are the same, only two points
        [timelist, pos, vel, acc] = fit_traj(knots_t, path, TSTEP);
    end

    % for extrapolation
    timelist = [timelist,timelist(end)+TSTEP,timelist(end)+TEXTRAP];
    pos = [pos; pos(end,:); pos(end,:)];
    vel = [vel; zeros([2,3])];
    acc = [acc; zeros([2,3])];

    % edge case where all points are the same -- no movement required.
    if numel(timelist) == 1
        timelist = [0, 1];
        pos = [path(1:2,:)];
        vel = [0,0,0; 0,0,0];
        acc = [0,0,0; 0,0,0];
    end

    if show_plots
        idx = 1:(length(timelist)-1);
        figure;
        subplot(3,1,1); plot(timelist(idx), pos(idx,:)); grid; title('pos');
        subplot(3,1,2); plot(timelist(idx), vel(idx,:)); grid; title('vel');
        subplot(3,1,3); plot(timelist(idx), acc(idx,:)); grid; title('acc');
        legend('x', 'y', 'z');
        figure;
        plot3(pos(:,1), pos(:,2), pos(:,3), path(:,1), path(:,2), path(:,3)); grid;
        legend('planned', 'desired');
    end
end

function [timelist, pos, vel, acc, jer, sna] = fit_traj(knots_t, path, TSTEP)
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
    timelist = tx;
end

function t = get_time_cte_acc(x, v, a);
% compute time to cover distance X at constant acceleration A starting with velocity V
% some tiny imaginary part may appear due to numerical errors
    t = real(roots([a/2, v, -x]));
    assert(~all(t < 0));
    t = min(t(t >= 0));
end

function t = get_time_vel_diff(x, v1, v2, target_acc);
% compute time to cover distance X at constant acceleration A starting with velocity V
    EPS = 1e-6;

    if v1 < EPS && v2 < EPS
        t = sqrt(4*x/target_acc);
        return;
    end
    if abs(x) < EPS
        t = 0;
    else
        a = (v2^2-v1^2)/2/x;
        t = get_time_cte_acc(x, v1, a);
    end
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
