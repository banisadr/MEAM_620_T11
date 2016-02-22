function sub = point2sub(pt, x, y, z)
%point2sub - find cell in grid X x Y x Z closest to PT
%
% Syntax:  sub = point2sub(pt, x, y, z)
%
% Inputs:
%    pt - point to test
%    x, y, z - vectors of discrete x, y and z positions
%
% Outputs:
%    sub - subscripts of the closest cell centroid

% TODO: make this work on vectors
    idx = closest_id_vector(pt(1), x);
    idy = closest_id_vector(pt(2), y);
    idz = closest_id_vector(pt(3), z);
    sub = [idx, idy, idz];
end

function id = closest_id_vector(val, vec)
% closest_id_vector - fast to way to find point in sorted vector VEC that is closest to VAL
    if val <= vec(1)
        id = 1;
    elseif val >= vec(end)
        id = length(vec);
    else
        [~, id] = histc(val, vec);
        % just ignore out of bond error
        try
            if vec(id+1) - val < val - vec(id)
                id = id + 1;
            end
        end
    end
end