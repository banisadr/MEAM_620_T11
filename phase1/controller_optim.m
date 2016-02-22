function [F, M, trpy, drpy] = controller_optim(qd, t, qn, params, gains)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

EPS = 1e-6; % for float vs 0 comparisons
% wheter to ignore error projection tangent to the trajectory
PROJECT_ERROR = false;

% valid types: pd-linear, pd-nonlinear, lqr
ctr_type = 'pd-nonlinear';

m = 0.176;
g = 9.81;

% angular controller gains, to compute desired momentum
kpang = [0.698; 0.648; 1.04]; % phi, theta, psi
kdang = [0.0111; 0.0103; 0.0166]; % phi, theta, psi

% position controller gains, to compute desired force and attitude
kp = gains(1);
kd = gains(2);
% kp = 4.93;
% kd = 2.72;

% lqr gains
Klqr = [0.0000    0.0000   10.7910    4.7693   -0.0000   -0.0000;
        -0.1720   -0.1204   -0.0000   -0.0000    0.4050    0.0328];

pos = qd{qn}.pos;
vel = qd{qn}.vel;
euler = qd{qn}.euler;
omega = qd{qn}.omega;
pos_des = qd{qn}.pos_des;
vel_des = qd{qn}.vel_des;
acc_des = qd{qn}.acc_des;
yaw_des = qd{qn}.yaw_des;
yawdot_des = qd{qn}.yawdot_des;

ep = pos_des-pos; % position error
% ignore position errors on tangent direction
% TODO: this is not correct, we should use the closest traj point, not the point @ curr time
if PROJECT_ERROR
    if norm(vel_des) > EPS
        t_u = vel_des/norm(vel_des); % tangent unit vector
        if norm(acc_des) > EPS
            n_u = acc_des/norm(acc_des); % normal unit vector
        else
            % if acc==0, we want any vector normal to vel
            % we try two of them to cope with degenerate cases
            n_u1 = [0; -t_u(3); t_u(2)];
            n_u2 = [-t_u(2); t_u(1); 0]; % normal unit vector
            if norm(n_u1) > norm(n_u2)
                n_u = n_u1/norm(n_u1);
            else
                n_u = n_u2/norm(n_u2);
            end
        end
        assert(abs(dot(t_u, n_u)) < EPS)
        b_u = cross(t_u, n_u); % binormal unit vector
                               % projecting the position error
        ep = dot(ep, n_u)*n_u + dot(ep, b_u)*b_u;
    end
end
ev = vel_des-vel;

if strcmp(ctr_type, 'pd-linear')
    acc_cmd = acc_des + kd*ev + kp*ep;
    % Desired roll, pitch and yaw
    phi_des = 1/g*(acc_cmd(1)*sin(yaw_des) - acc_cmd(2)*cos(yaw_des));
    theta_des = 1/g*(acc_cmd(1)*cos(yaw_des) + acc_cmd(2)*sin(yaw_des));
    
    % Thrust
    F    = m*g + m*acc_cmd(3);
    % Moment
    M    = [kpang(1)*(phi_des - euler(1)) + kdang(1)*(0 - omega(1));
            kpang(2)*(theta_des - euler(2)) + kdang(2)*(0 - omega(2));
            kpang(3)*(yaw_des - euler(3)) + kdang(3)*(yawdot_des - omega(3))];
elseif strcmp(ctr_type, 'pd-nonlinear')
    cp = cos(euler(1)); % phi
    sp = sin(euler(1));
    ct = cos(euler(2)); % theta
    st = sin(euler(2));
    cs = cos(euler(3)); % psi
    ss = sin(euler(3));
    Rba = [cs*ct-sp*ss*st, -cp*ss, cs*st+ct*sp*ss;
           ct*ss+cs*sp*st,  cp*cs, ss*st-cs*ct*sp;
           -cp*st, sp, cp*ct];

    acc_cmd = acc_des + kd*ev + kp*ep;

    % project required force into b3
    b3 = Rba*[0;0;1];
    acc_cmd_proj = dot([0;0;g] + acc_cmd, b3);
    F    = m*acc_cmd_proj;

    % compute error to turn the quadrotor to align b3 with desired acc vel
    if norm(acc_cmd) > EPS % TODO: handle acc_cmd = 0
        err_phi = dot(cross(b3, [0;0;g] + acc_cmd)/norm([0;0;g] + acc_cmd), Rba*[1;0;0]);
        err_theta = dot(cross(b3, [0;0;g] + acc_cmd)/norm([0;0;g] + acc_cmd), Rba*[0;1;0]);
    else
        err_phi = 0;
        err_theta = 0;
    end

    % Moment
    M    = [kpang(1)*err_phi + kdang(1)*(0 - omega(1));
            kpang(2)*err_theta + kdang(2)*(0 - omega(2));
            kpang(3)*(yaw_des - euler(3)) + kdang(3)*(yawdot_des - omega(3))];
elseif strcmp(ctr_type, 'lqr')
    acc_cmd = acc_des + Klqr(1,4)*ev + Klqr(1,3)*ep;
    % Desired roll, pitch and yaw
    phi_des = 1/g*(acc_cmd(1)*sin(yaw_des) - acc_cmd(2)*cos(yaw_des));
    theta_des = 1/g*(acc_cmd(1)*cos(yaw_des) + acc_cmd(2)*sin(yaw_des));
    
    % Thrust
    F    = m*g + m*acc_cmd(3);
    % Moment
    M    = [Klqr(2,5)*(phi_des - euler(1)) + Klqr(2,6)*(0 - omega(1)) + Klqr(2,1)*ep(2) + Klqr(2,2)*ev(2);
            Klqr(2,5)*(theta_des - euler(2)) + Klqr(2,6)*(0 - omega(2)) + Klqr(2,1)*ep(1) + Klqr(2,2)*ev(1);
            Klqr(2,5)*(yaw_des - euler(3)) + Klqr(2,6)*(yawdot_des - omega(3))];
else
    error(['Unrecognized controller type: ' ctr_type]);
end

% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
%trpy = [F, phi_des, theta_des, yaw_des];
drpy = [0, 0,       0,         0];

end
