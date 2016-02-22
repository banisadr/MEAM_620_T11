function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

R = 5; % radius
H = 2.5; % height

trajlength = 2*pi*sqrt(R^2 + H^2/4/pi^2);
trajtime = 8;

T = trajtime;
A = [0 0 0 0 0 1;
     T^5 T^4 T^3 T^2 T 1;
     0 0 0 0 1 0;
     5*T^4 4*T^3 3*T^2 2*T 1 0;
     0 0 0 2 0 0;
     20*T^3 12*T^2 6*T 2 0 0];
c = inv(A)*[0; trajlength; 0; 0; 0; 0];

posfun = @(t)c(1)*t^5 + c(2)*t^4 + c(3)*t^3 + c(4)*t^2 + c(5)*t + c(6);
velfun = @(t)5*c(1)*t^4 + 4*c(2)*t^3 + 3*c(3)*t^2 + 2*c(4)*t + c(5);
accfun = @(t)20*c(1)*t^3 + 12*c(2)*t^2 + 6*c(3)*t + 2*c(4);

if t > trajtime
    pos = [5; 0; 2.5];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
else
    linpos = posfun(t);
    linvel = velfun(t);
    linacc = accfun(t);

    % to make it clockwise: change sign to -1
    ccwise = +1;

    ang = ccwise*2*pi*linpos/trajlength;

    pos = [R*cos(ang);
           R*sin(ang);
           H*linpos/trajlength];
    vel = [-ccwise*2*pi*R/trajtime*sin(ang);
           ccwise*2*pi*R/trajtime*cos(ang);
           H/trajtime];
    acc = [-4*pi^2*R/trajtime^2*cos(ang);
           -4*pi^2*R/trajtime^2*sin(ang);
           0];

    % correct acc and vel magnitudes
    t_u = vel/norm(vel); % TODO: check when 0
    n_u = acc/norm(acc);
    vel = linvel*t_u;
    acc = linvel^2/R*n_u + linacc*t_u;
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
