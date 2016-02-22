function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.

    SHOW_MARGIN = true;
    SHOW_NODES = false;

    % find figure bounds
    bdnblk = [map.boundary; map.block_inflated];

    faces = [1,3,7,5;
             2,4,8,6;
             1,2,6,5;
             3,4,8,7;
             1,2,4,3;
             5,6,8,7];
    % show blocks and inflated blocks
    figure;
    for(i=1:size(map.block,1))
        blk = map.block(i,:);
        [x,y,z] = ndgrid([blk(1), blk(4)],[blk(2), blk(5)],[blk(3), blk(6)]);
        pts = [x(:) y(:) z(:)];
        patch('Faces', faces, 'Vertices', pts, 'FaceColor', double(map.block_color(i,:)/255));
        hold on;

        if SHOW_MARGIN
            blk = map.block_inflated(i,:);
            [x,y,z] = ndgrid([blk(1), blk(4)],[blk(2), blk(5)],[blk(3), blk(6)]);
            pts = [x(:) y(:) z(:)];
            patch('Faces', faces, 'Vertices', pts, ...
                  'FaceColor', 'None');
            hold on;
        end
    end

    if SHOW_NODES
        [x,y,z] = ndgrid(map.x, map.y, map.z);
        % plot unreachable nodes as red
        idreach = find(any(map.graph));
        idunreach = find(any(map.graph) == 0);
        plot3(x(idreach), y(idreach), z(idreach), 'b.');
        plot3(x(idunreach), y(idunreach), z(idunreach), 'r.');
        hold on;
    end

    % show path
    if ~isempty(path)
        plot3(path(:,1), path(:,2), path(:,3), '-o');
        hold on;
    end

    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    axis([-0.5+min(bdnblk(:,1)), 0.5+max(bdnblk(:,4)), ...
          -0.5+min(bdnblk(:,2)), 0.5+max(bdnblk(:,5)), ...
          -0.5+min(bdnblk(:,3)), 0.5+max(bdnblk(:,6))]);
end