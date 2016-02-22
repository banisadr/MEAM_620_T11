function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.
    blk = map.block;
    C = logical(zeros([size(points,1),1]));
    for(i=1:size(points,1))
        pt = points(i,:);
        if ~isempty(blk)
            C(i) = any(pt(1) >= blk(:,1) & pt(1) <= blk(:,4) & ...
                       pt(2) >= blk(:,2) & pt(2) <= blk(:,5) & ...
                       pt(3) >= blk(:,3) & pt(3) <= blk(:,6));
        end
    end
end
