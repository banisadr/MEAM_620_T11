% MEAM 620 Student One Waypoint code

if (setitM(qn)~=902) %The variable setitM(qn) tracks what type of sequence each quad is in. 
                     %If the quad switches sequences, this if statement will be active.
    setitM(qn)=902;  %This changes setitM(qn) to the current sequence type so that this code only runs once.
    
    %PUT ANY INITIALIZATION HERE
    t0 = tic;
    pos0 = qd{qn}.pos;
    times = seqM(qn).seq(seq_cntM(qn)).times;
    pos_des = seqM(qn).seq(seq_cntM(qn)).pos_des;
    vel_des = seqM(qn).seq(seq_cntM(qn)).vel_des;
    acc_des = seqM(qn).seq(seq_cntM(qn)).acc_des;
    %END INTITIALIZATION

end %everything beyond this point runs every control loop iteration

%COMPUTE CONTROL HERE
t = toc - t0;

% get desired state
% checkme! add pos0 here???
qd{qn}.pos_des = interp1(times, pos_des, t, 'linear', 'extrap')';
qd{qn}.vel_des = interp1(times, vel_des, t, 'linear', 'extrap')';
qd{qn}.acc_des = interp1(times, acc_des, t, 'linear', 'extrap')';
qd{qn}.yaw_des      = 0;
qd{qn}.yawdot_des   = 0;

[F, M, trpy, drpy] = controller(qd, t, qn, []);
qd{qn}.euler_des = trpy(2:end);