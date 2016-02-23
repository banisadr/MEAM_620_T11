function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.

    SIMPLE_TRAJ = true;
    EPS = 1e-3;

    if nargin < 4
        astar = false;
    end
    path = [];
    num_expanded = 0;

    % sanity check; start and stop must not collide with any blocks
    % and must 'collide' with map boundaries
    if (collide(map, start) || ...
        collide(map, goal) || ...
        ~collide(struct('block', map.boundary), start)  || ...
        ~collide(struct('block', map.boundary), goal))
        warning('Initial conditions invalid, no path returned!');
        return;
    end

    n = prod(map.dims);

    % using custom function that only uses xy diagonals
    % WARNING! this must be changed if map discretization changes,
    % i.e. if other diagonals are included
    % heuristic = @(x,y)norm(x-y);
    % heuristic = @(x,y)norm(x(1:2)-y(1:2))+abs(x(3)-y(3));
    heuristic = @(x,y) + min(abs(x(1)-y(1)), abs(x(2)-y(2)))*sqrt(2) + ...
        abs(abs(x(1)-y(1)) - abs(x(2)-y(2))) +  ...
        abs(x(3)-y(3));
    % caching distance of each node to goal to improve A* performance
    % TODO: since goal point is different than goal cell, we must compute distances
    % to all neighboring cells and add euclidean to goal
    distgoal = pdist2(goal, ...
                      [map.gridx(:) map.gridy(:) map.gridz(:)], ...
                      heuristic);

    % check if we need to modify map to add start and end points
    startcellid = point2ind(start, map.x, map.y, map.z);
    startcellpt = [map.gridx(startcellid), map.gridy(startcellid), map.gridz(startcellid)];
    % add new start cell
    if norm(startcellpt - start) > EPS
        neigid = [startcellid; find(map.graph(:,startcellid))];
        neigpos = [map.gridx(neigid), map.gridy(neigid), map.gridz(neigid)];
        dists = sqrt(sum(bsxfun(@minus, start, neigpos).^2,2));
        % adjacency line for extra cell
        adj = zeros([n,1]);
        adj(neigid) = dists;
        map.graph = [map.graph adj; adj' 0];
        n = n+1;
        startid = n;
        distgoal = [distgoal, 0]; % doesn't matter
    else
        startid = startcellid;
    end

    goalcellid = point2ind(goal, map.x, map.y, map.z);
    goalcellpt = [map.gridx(goalcellid), map.gridy(goalcellid), map.gridz(goalcellid)];
    if norm(goalcellpt - goal) > EPS
        neigid = [goalcellid; find(map.graph(:,goalcellid))];
        % may fail if new startid is in here
        nid = neigid(neigid <= prod(map.dims));
        neigpos = [map.gridx(nid), map.gridy(nid), map.gridz(nid)];
        dists = sqrt(sum(bsxfun(@minus, goal, neigpos).^2,2));
        if any(neigid > prod(map.dims)) % new start point is neighbor
            dists = [dists; norm(goal-start)];
        end
        % adjacency line for extra cell
        adj = zeros([n,1]);
        adj(neigid) = dists;
        map.graph = [map.graph adj; adj' 0];
        n = n+1;
        goalid = n;
        distgoal = [distgoal, 0];
    else
        goalid = goalcellid;
    end

    % TODO: we probably don't need this many variables: queue, ranking, parents and visited

    % TODO: this should be a priority queue for best efficiency
    % the way it is now, we're taking the min in O(n) every step
    % however, tests with prio queue showed worst results, probably bc of my bad implementation

    % queue(id) stores priority for node id, which is the distance
    % from start node to it (A* will add a term to it)
    queue = inf*ones(n, 1);
    ranking = inf*ones(n, 1); % ranking is queue + heuristic

    queue(startid) = 0;
    ranking(startid) = 0;
    % store the parents of each node to trace back the path
    parents = zeros(n, 1);
    visited = logical(zeros(n, 1)); % mark visited nodes

    currid = 0;
    while (currid ~= goalid)
        % this line takes 50% of computing time; priority queue would certainly help!
        [valmin, currid] = min(ranking);
        if isinf(valmin)
            path = zeros(0,3);
            num_expanded = sum(visited);
            warning('No path found!');
            return;
        end

        visited(currid) = true;
        % DEBUG!!!
        % plot coordinates of node being visited
        % [cx, cy, cz] = ind2sub(map.dims, currid);
        % hold on; plot3(map.x(cx), map.y(cy), map.z(cz)', 'bo');

        %% WARNING: if we do map.graph(currid, :), this gets MUCH slower!
        children = find(map.graph(:, currid));
        % checkme! need isinf? may we be overwritting ranking without it?
        children = children(~visited(children) & isinf(ranking(children)));
        parents(children) = currid;
        % update cost of children (this is a costly operation!)
        queue(children) = queue(currid) + map.graph(currid, children);
        ranking(children) = queue(children);

        if astar
            % include A* heuristic
            ranking(children) = ranking(children) + distgoal(children)';

            % reduce cost by a small amount if children and parent are aligned
            % the idea is to break ties in favor of the trajectory with less turns
            % to improve performance: use distance from cost matrix instead of dotunit
            currpt = []; %may be empty
            try
                currpt = [map.gridx(currid), map.gridy(currid), map.gridz(currid)];
            end
            cid = children(children ~= goalid);
            childpt = []; % may be empty
            try
                childpt = [map.gridx(cid), map.gridy(cid), map.gridz(cid)];
            end

            % annoyance when child is goal and we added a special node for it
            % (this would be fixed by storing each ndgrid x,y,z instead of just indices...)
            if any(children == goalid)
                childpt = [childpt; goal];
            end
            pid = parents(currid);
            parpt = [];
            try
                parpt = [map.gridx(pid), map.gridy(pid), map.gridz(pid)];
            end

            if ~isempty(parpt) && ~isempty(childpt) && ~isempty(currpt)
                % first node has no parent; last node can't visit its children
                idpar = dotunit(bsxfun(@minus, childpt, currpt), ...
                                     currpt-parpt) > .999;
                ranking(children(idpar)) = ranking(children(idpar)) - ...
                    min([map.cost_dx, map.cost_dy, map.cost_dz])/10;
            end
        end
        ranking(currid) = inf; % won't revisit
    end
    num_expanded = sum(visited);


    if currid == startid
        % start = end
        path = startid;
    else
        % trace back path from parents
        currid = parents(currid);
        while (currid ~= startid)
            path = [path, currid];
            currid = parents(currid);
        end
    end
    % reverse direction
    path = path(end:-1:1);
    % compute path coordinates
    [sx, sy, sz] = ind2sub(map.dims, path);
    path = [map.x(sx)' map.y(sy)' map.z(sz)'];
    % and start and finish points
    path = [start; path; goal];

    % we have our path; now we try to optimize it a little bit
    % first, we eliminate all colinear points
    i=2;
    while i < size(path,1)-1;
        back = path(i,:) - path(i-1,:);
        forw = path(i+1,:) - path(i,:);
        if dot(back,forw)/norm(back)/norm(forw) > 1-EPS
            % remove point
            path(i,:) = [];
            i = i-1;
        end
        i=i+1;
    end

    % finer resolution taking margin into account
    res = [min(map.cost_dx, map.margin), ...
           min(map.cost_dy, map.margin), ...
           min(map.cost_dz, map.margin)];

    % now we track every corner and check if we can cut it
    % if a->b->c and c is a corner, we check if a->c doesn't
    % collide with anything
    i=2;
    while i <= size(path,1)-1
        back = path(i,:) - path(i-1,:);
        forw = path(i+1,:) - path(i,:);
        if dot(back,forw)/norm(back)/norm(forw) < 1-EPS
            v = path(i+1,:) - path(i-1,:);
            u = v/norm(v); % unit vector
            % we check some points along the line, according to given resolution
            ncells = ceil(norm(abs(v)./res));

            if ~any(collide_margin(map, bsxfun(@plus, path(i-1,:), ...
                        bsxfun(@times, linspace(0,norm(v),ncells)', repmat(u,[ncells,1])))))
                % safe to remove corner
                path(i,:) = [];
                i = i-1; % analyze same id since we've reduced the vector size
            end
        end
        i=i+1;
    end

    % for simple path, we're done
    if SIMPLE_TRAJ
        return
    end

    %% rebuild finer trajectory and do moving average to smooth corners out
    finepath = [];
    for i=1:size(path,1)-1
        v = path(i+1,:) - path(i,:);
        u = v/norm(v); % unit vector
        % we check some points along the line, according to given resolution
        % note: if res < margin we use the margin
        ncells = ceil(norm(abs(v)./res));
        finepath = [finepath;
                    bsxfun(@plus, path(i,:), ...
                        bsxfun(@times, linspace(0,norm(v),ncells)', repmat(u,[ncells,1])))];
    end
    path = finepath;

    %% moving average between points
    % this won't remove points from their cells
    for (i=2:size(path,1)-1)
        path(i,:) = (path(i-1,:) + path(i+1,:))/2;
    end
end
