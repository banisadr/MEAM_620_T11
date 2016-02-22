function ind = point2ind(pt, x, y, z)
%point2ind - find cell index in grid X x Y x Z closest to PT
%
% Syntax:  sub = point2ind(pt, x, y, z)
%
% Inputs:
%    pt - point to test
%    x, y, z - vectors of discrete x, y and z positions
%
% Outputs:
%    ind - index of the closest cell centroid
    sub = point2sub(pt, x, y, z);
    ind = sub2ind([numel(x), numel(y), numel(z)], sub(1), sub(2), sub(3));
end

