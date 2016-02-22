function [times, pos, vel, acc, jer, sna] = fit_quintic(knots_x, knots_t, v0f, a0f, tstep)
%fit_quintic - fit quintic spline between knots
%
% Syntax:  [times, pos, vel, acc, jer, sna] = fit_quintic(knots_x, knots_t, v0f, a0f, tstep)
%
% Inputs:
%    knots_x, knots_t - times and positions of each knot
%    v0f, a0f - initial and final velocities and accelerations (two 1x2 vectors)
%    tstep - timestep of returned trajectory
%
% Outputs:
%    times, pos, vel, acc
%
% Example:
%
% See also:

n = numel(knots_t); % n pts; n-1 segments
% matrix A definition
% x1, x2, x', x'', x''', x''''
% first constant is of higher power (t^5)
A = zeros([6*(n-1), 6*(n-1)]); % each segment has 6 ctes
X = zeros([6*(n-1), 1]);

% 1st segment
X(1:4) = [knots_x(1), knots_x(2), v0f(1), a0f(1)];
A(1:4, 1:6) = [knots_t(1)^5 knots_t(1)^4 knots_t(1)^3 knots_t(1)^2 knots_t(1) 1;
               knots_t(2)^5 knots_t(2)^4 knots_t(2)^3 knots_t(2)^2 knots_t(2) 1;
               5*knots_t(1)^4 4*knots_t(1)^3 3*knots_t(1)^2 2*knots_t(1) 1 0;
               20*knots_t(1)^3 12*knots_t(1)^2 6*knots_t(1) 2 0 0];

for(i=2:numel(knots_t)-1)
  t1 = 0;
  t2 = knots_t(i+1) - knots_t(i);
  pt2 = knots_t(i) - knots_t(i-1); % previous t2
  lin = 6*(i-1)-1;  % initial line in matrix A, vector X
  cols = 6*(i-1)+1:6*(i-1)+6; % cols in A

  X(lin:lin+5) = [knots_x(i), knots_x(i+1), 0, 0, 0, 0];
  A(lin:lin+1, cols) = [t1^5 t1^4 t1^3 t1^2 t1^1 1;
                        t2^5 t2^4 t2^3 t2^2 t2^1 1];
  % continuity: x'
  A(lin+2, cols(1)-6:cols(1)+5) = [-5*pt2^4 -4*pt2^3 -3*pt2^2 -2*pt2 -1 0 5*t1^4 4*t1^3 3*t1^2 2*t1 1 0];
  % continuity: x''
  A(lin+3, cols(1)-6:cols(1)+5) = [-20*pt2^3 -12*pt2^2 -6*pt2 -2 0 0 20*t1^3 12*t1^2 6*t1 2 0 0];
  % continuity: x'''
  A(lin+4, cols(1)-6:cols(1)+5) = [-60*pt2^2 -24*pt2 -6 0 0 0 60*t1^2 24*t1 6 0 0 0 ];
  % continuity: x''''
  A(lin+5, cols(1)-6:cols(1)+5) = [-120*pt2 -24 0 0 0 0 120*t1 24 0 0 0 0];
end

% last segment
tf = knots_t(end) - knots_t(end-1);
X(end-1:end) = [v0f(2), a0f(2)];
A(end-1:end, end-5:end) = [5*tf^4 4*tf^3 3*tf^2 2*tf 1 0;
                           20*tf^3 12*tf^2 6*tf 2 0 0];

C = A\X;

pos = [];
vel = [];
acc = [];
jer = [];
sna = [];
times = [];
for(i=1:numel(knots_t)-1)
  curr_c = C(6*(i-1)+1:6*(i-1)+6);
  if i == numel(knots_t)-1
      curr_t = 0:tstep:knots_t(i+1)-knots_t(i);
  else
      curr_t = 0:tstep:knots_t(i+1)-knots_t(i)-tstep;
  end

  times = [times, curr_t + knots_t(i)];

  pos = [pos, polyval(curr_c, curr_t)];
  vel = [vel, polyval(polyder(curr_c), curr_t)];
  acc = [acc, polyval(polyder(polyder(curr_c)), curr_t)];
  jer = [jer, polyval(polyder(polyder(polyder(curr_c))), curr_t)];
  sna = [sna, polyval(polyder(polyder(polyder(polyder(curr_c)))), curr_t)];
end
