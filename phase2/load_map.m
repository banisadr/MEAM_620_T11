function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.

    USE_XY_DIAG = true;
    % z is coarse, we don't gain much by using its diagonals
    % the code is also not well tested; there may be extra edges!
    USE_XZ_DIAG = false;
    USE_YZ_DIAG = false;

    map.block = [];
    map.block_color = [];
    map.block_inflated = [];

    fid = fopen(filename);
    while (~feof(fid))
        linetype = textscan(fid, '%s', 1, 'CommentStyle', '#');
        if strcmp(linetype{:}, 'boundary')
            map.boundary = cell2mat(textscan(fid, '%f %f %f %f %f %f', 1, 'CommentStyle', '#'));
        elseif strcmp(linetype{:}, 'block')
            block = textscan(fid, '%f %f %f %f %f %f %d %d %d', 1, 'CommentStyle', '#');
            map.block = [map.block; cell2mat(block(1:6))];
            map.block_color = [map.block_color; cell2mat(block(7:9))];
        elseif ~isempty(linetype{:})
            error('Unrecognized file format!')
        end
    end
    fclose(fid);

    % inflate blocks because of margin
    sblk = size(map.block);
    if sblk > 0
        map.block_inflated = map.block + [-margin*ones(sblk(1), 3) margin*ones(sblk(1), 3)];
    end

    % list of possible x, y, z of node positions
    % we avoid an ndgrid to save memory; this makes the code uglier and
    % forbids us to use nice functions like interp3
    % TODO: we do call ndgrid later, so we might as well call it here once and keep it...
    % TODO: perhaps it makes sense to use max(1,... ) instead of max(2, ...)
    bd = map.boundary;
    map.x = linspace(bd(1)+xy_res/2., bd(4)-xy_res/2., max(2, (bd(4)-bd(1))/xy_res));
    map.y = linspace(bd(2)+xy_res/2., bd(5)-xy_res/2., max(2, (bd(5)-bd(2))/xy_res));

    if z_res > (bd(6)-bd(3))/2
        z_res = (bd(6)-bd(3))/2
    end
    map.z = linspace(bd(3)+ z_res/2., bd(6)- z_res/2., max(2, (bd(6)-bd(3))/z_res));

    [map.gridx, map.gridy, map.gridz] = ndgrid(map.x, map.y, map.z);

    map.cost_dx = map.x(2) - map.x(1);
    map.cost_dy = map.y(2) - map.y(1);
    map.cost_dz = map.z(2) - map.z(1);
    map.margin = margin;

    nx = numel(map.x);
    ny = numel(map.y);
    nz = numel(map.z);
    map.dims = [nx, ny, nz];
    n = prod(map.dims); % total number of cells

    % index
    dx = sub2ind(map.dims, 2, 1, 1) - sub2ind(map.dims, 1, 1, 1);
    dy = sub2ind(map.dims, 1, 2, 1) - sub2ind(map.dims, 1, 1, 1);
    dz = sub2ind(map.dims, 1, 1, 2) - sub2ind(map.dims, 1, 1, 1);
    % horizontal connections
    edges = [(1:n-dx)' (1+dx:n)' map.cost_dx*ones(size((1+dx:n)'))];
    % vertical
    edges = [edges; (1:n-dy)' (1+dy:n)' map.cost_dy*ones(size((1+dy:n)'))];
    % depth
    edges = [edges; (1:n-dz)' (1+dz:n)' map.cost_dz*ones(size((1+dz:n)'))];
    if USE_XY_DIAG
        % diagonal xy right
        edges = [edges; (1:n-dy-1)' (2+dy:n)' norm([map.cost_dx, map.cost_dy])*ones(size((2+dy:n)'))];
        % diagonal xy left
        edges = [edges; (2:n-dy+1)' (1+dy:n)' norm([map.cost_dx, map.cost_dy])*ones(size((1+dy:n)'))];
    end

    if USE_XZ_DIAG
        % diagonal xz forward
        edges = [edges; (1:n-dz-1)' (2+dz:n)' norm([map.cost_dx, map.cost_dz])*ones(size((2+dz:n)'))];
        % diagonal xz backward
        edges = [edges; (2:n-dz+1)' (1+dz:n)' norm([map.cost_dx, map.cost_dz])*ones(size((1+dz:n)'))];
    end

    if USE_YZ_DIAG
        % diagonal yz forward
        edges = [edges; (1:n-dy-dz)' (1+dy+dz:n)' norm([map.cost_dy, map.cost_dz])*ones(size((1+dy+dz:n)'))];
        % diagonal yz backward
        edges = [edges; (1+dy:n-dz)' (1+dz:n-dy)' norm([map.cost_dy, map.cost_dz])*ones(size((1+dz:n-dy)'))];
    end

    % create adjacency matrix
    % we set max # of zeros as number of edges (we can remove edges after, but not add)
    map.graph = sparse(edges(:,1), edges(:,2), edges(:,3), n, n, size(edges,1));
    % the matrix has a few extra connections that must be eliminated
    % for instance, a point on the rightmost column is connected to
    % the leftmost point of the next line
    % TODO: perhaps we can make this more efficient and/or more readable...
    for i=nx:nx:(n-dx)
        map.graph(i, i+dx) = 0; % horizontal
    end
    if USE_XY_DIAG
        for i=nx:nx:(n-dy-1)
            map.graph(i, i+dy+1) = 0; % diagonal xy right
        end
        for i=nx+1:nx:(n-dy+1)
            map.graph(i, i+dy-1) = 0; % diagonal xy left
        end
    end
    if USE_XZ_DIAG
        for i=nx:nx:(n-dz-1)
            map.graph(i, i+dz+1) = 0; % diagonal xz forward
        end
        for i=nx+1:nx:(n-dz+1)
            map.graph(i, i+dz-1) = 0; % diagonal xz backward
        end
        % assert(all(size(map.graph) == size(map.graph')));
    end

    for i=(nx*ny):(nx*ny):(n-1) % i is point with max x and max y at each z
        for j=1:nx % i-j+1 are the points with max y for each z
            map.graph(i-j+1, i-j+1+dy) = 0; % vertical (y)
            if USE_XY_DIAG
                map.graph(i-j+1, i-j+dy) = 0; % diagonal xy right
            end

            % assert(all(size(map.graph) == size(map.graph')));
        end
        if USE_XY_DIAG
            for j=2:nx
                map.graph(i-j+1, i-j+2+dy) = 0; % diagonal xy right
                % assert(all(size(map.graph) == size(map.graph')));
            end
        end
    end

    if USE_YZ_DIAG
        % remove extra diagonal yz forward edges
        for i=dz:dz:n-dz-1 % i is point with max x and max y at each z except the two last
            for j=1:nx % i-j+1 are the points with max y for each z
                map.graph(i-j+1, i-j+1+dy+dz) = 0;
                % assert(all(size(map.graph) == size(map.graph')));
            end
        end

        % TODO: remove extra diagonal yz backward edges from first line of each layer
        for i=1:dz:n % i is point with max x and max y at each z except the two last
            for j=1:nx % i-j+1 are the points with max y for each z
                map.graph(i+j-1, i+j-1-dy+dz) = 0;
                % assert(all(size(map.graph) == size(map.graph')));
            end
        end
    end

    map.graph = map.graph + map.graph'; % make it symmetric

    % handle obstacles by removing paths from the graph
    for(i=1:size(map.block,1))
        map.graph = remove_block_edges(map, map.block_inflated(i,:));
    end
end

function graph = remove_block_edges(map, blk)
%

    % whether to disconnect only surface of obstacle or the whole volume
    REMOVESURFACE = false;

    % we inflate by the resolution to make sure the node closest
    % to the boundary is included in the list of possible disconnected
    % nodes
    pts = [blk(1)-map.cost_dx, blk(2)-map.cost_dy, blk(3)-map.cost_dz;
           blk(1)-map.cost_dx, blk(2)-map.cost_dy, blk(6)+map.cost_dz;
           blk(1)-map.cost_dx, blk(5)+map.cost_dy, blk(3)-map.cost_dz;
           blk(1)-map.cost_dx, blk(5)+map.cost_dy, blk(6)+map.cost_dz;
           blk(4)+map.cost_dx, blk(2)-map.cost_dy, blk(3)-map.cost_dz;
           blk(4)+map.cost_dx, blk(2)-map.cost_dy, blk(6)+map.cost_dz;
           blk(4)+map.cost_dx, blk(5)+map.cost_dy, blk(3)-map.cost_dz;
           blk(4)+map.cost_dx, blk(5)+map.cost_dy, blk(6)+map.cost_dz];

    sub = zeros(size(pts));
    for(i=1:size(pts,1))
        sub(i,:) = point2sub(pts(i,:), map.x, map.y, map.z);
    end

    graph = map.graph;
    % there is a trade-off here; we can remove each surface of the obstacle
    % to make it unreachable, or we can alse remove the inner-nodes.
    % the former may be faster but the latter makes it easier to check
    % for collisions.
    if REMOVESURFACE
        error('REMOVESURFACE is probably not working anymore. FIXME!');
        % we have 6 faces to disconnect
        faces = [1,2,3,4;
                 5,6,7,8;
                 1,2,5,6;
                 3,4,7,8;
                 1,3,5,7;
                 2,4,6,8];
        for(i=1:size(faces,1))
            face = faces(i,:);
            [x,y,z] = ndgrid(min(sub(face,1)):max(sub(face,1)), ...
                             min(sub(face,2)):max(sub(face,2)), ...
                             min(sub(face,3)):max(sub(face,3)));
            ids = sub2ind(map.dims, x(:), y(:), z(:));
            % disconnect nodes
            graph(ids, :) = 0;
            graph(:, ids) = 0;
        end
    else
        [x,y,z] = ndgrid(min(sub(:,1)):max(sub(:,1)), ...
                         min(sub(:,2)):max(sub(:,2)), ...
                         min(sub(:,3)):max(sub(:,3)));
        ids = sub2ind(map.dims, x(:), y(:), z(:));

        % make sure nodes removed are colliding
        % there are combinations of resolution and margins
        % that may cause the closest node to the margin not collide
        idcol = collide(struct('block', blk), ...
                        [map.x(x(:))', map.y(y(:))', map.z(z(:))']);

        % disconnect nodes
        graph(ids(idcol), :) = 0;
        graph(:, ids(idcol)) = 0;
    end
end

