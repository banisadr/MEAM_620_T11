% MEAM 620 Student Hover code

if (setitM(qn)~=901) %The variable setitM(qn) tracks what type of sequence each quad is in. 
                     %If the quad switches sequences, this if statement will be active.
    setitM(qn)=901;  %This changes setitM(qn) to the current sequence type so that this code only runs once.
    
    %PUT ANY INITIALIZATION HERE
    pos0 = qd{qn}.pos;
    pos_des = pos0 + [0,0,seqM(qn).seq(seq_cntM(qn)).hoverheight];
    t0 = tic;     % initial time
    %END INTITIALIZATION

end %everything beyond this point runs every control loop iteration

%COMPUTE CONTROL HERE
% and time at this step
t = toc - t0;

% The desired_state is set in the trajectory generator
qd{qn}.pos_des      = pos_des;
[F, M, trpy, drpy] = controller(qd, t, qn, []);
qd{qn}.euler_des = trpy(2:end);