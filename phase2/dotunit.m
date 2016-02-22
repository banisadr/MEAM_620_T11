function res = dotunit(x,y)
% dotunit - dot product between unit vectors
%
% RES = DOTUNIT(X,Y)
%
% X - n x m list of n vectors of m dimensions
% Y - 1 x m vector of m dimensions
% res = n x 1 list of dot products between the unit vectors parallel to each x and y

res = x*y'./sqrt(sum(x.^2,2))./sqrt(sum(y.^2,2));