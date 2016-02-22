function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

points = [0,0,0;
         .25, sqrt(2), sqrt(2);
         0.50, 0, 2*sqrt(2);
         0.75, -sqrt(2), sqrt(2);
         1,0,0];

leglen = norm(points(2,:));
nlegs = size(points,1) - 1;
totallen = leglen*nlegs;
tleg = 1.1;
totaltime = nlegs*tleg;
timelegs = (0:nlegs) * totaltime/nlegs;

T = tleg;
A = [0 0 0 0 0 1;
     T^5 T^4 T^3 T^2 T 1;
     0 0 0 0 1 0;
     5*T^4 4*T^3 3*T^2 2*T 1 0;
     0 0 0 2 0 0;
     20*T^3 12*T^2 6*T 2 0 0];
c = inv(A)*[0; leglen; 0; 0; 0; 0];

posfun = @(t)c(1)*t^5 + c(2)*t^4 + c(3)*t^3 + c(4)*t^2 + c(5)*t + c(6);
velfun = @(t)5*c(1)*t^4 + 4*c(2)*t^3 + 3*c(3)*t^2 + 2*c(4)*t + c(5);
accfun = @(t)20*c(1)*t^3 + 12*c(2)*t^2 + 6*c(3)*t + 2*c(4);

if t > totaltime
    t = totaltime;
    vel = [0;0;0];
    pos = [1;0;0];
    acc = [0,0,0];
else
    leg = ceil((t+eps)*nlegs/totaltime);
    t0 = (leg-1)*tleg;
    linpos = posfun(t-t0);
    linvel = velfun(t-t0);
    linacc = accfun(t-t0);

    u = (points(leg+1,:) - points(leg,:))/norm(points(leg+1,:) - points(leg,:));
    pos = points(leg,:) + linpos*u;
    vel = linvel*u;
    acc = linacc*u;
end

yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
