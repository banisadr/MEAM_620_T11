function pt = ind2point(ind, x, y, z)
%point2ind - find point in grid X x Y x Z correspondent to IND
%
% Syntax:  sub = point2ind(pt, x, y, z)
%
% Inputs:
%    ind - index of cell centroid
%    x, y, z - vectors of discrete x, y and z positions
%
% Outputs:
%    pt - 3-d point
%
    error('this is slow! you shouldnt be using it');
    [cx, cy, cz] = ind2sub([numel(x), numel(y), numel(z)], ind);
    pt = [];
    try
        pt = [x(cx)', y(cy)', z(cz)'];
    end
end

