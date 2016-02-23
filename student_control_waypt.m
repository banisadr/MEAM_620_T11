% MEAM 620 Student One Waypoint code

if (setitM(qn)~=902) %The variable setitM(qn) tracks what type of sequence each quad is in. 
                     %If the quad switches sequences, this if statement will be active.
    setitM(qn)=902;  %This changes setitM(qn) to the current sequence type so that this code only runs once.
    
    %PUT ANY INITIALIZATION HERE
    t0 = GetUnixTime;
    pos0 = qd{qn}.pos;

    path = [pos0';pos0'+[1 1 1];pos0'];
    [times, pos_des, vel_des, acc_des] = trajectory_generator(path, SIMPLE_TRAJ, TARGET_ACC);
    %END INTITIALIZATION

end %everything beyond this point runs every control loop iteration

%COMPUTE CONTROL HERE
t = GetUnixTime - t0;

% get desired state
qd{qn}.pos_des = interp1(times, pos_des, t, 'linear', 'extrap')';
qd{qn}.vel_des = interp1(times, vel_des, t, 'linear', 'extrap')';
qd{qn}.acc_des = interp1(times, acc_des, t, 'linear', 'extrap')';
qd{qn}.yaw_des      = 0;
qd{qn}.yawdot_des   = 0;

[F, M, trpy, drpy] = controller(qd, t, qn, struct('mass', MASS));
qd{qn}.euler_des = trpy(2:end);
