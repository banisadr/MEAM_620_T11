function [desired_state] = traj_interp(t, times, pos_des, vel_des, acc_des)
    desired_state.pos = interp1(times, pos_des, t, 'linear', 'extrap')';
    desired_state.vel = interp1(times, vel_des, t, 'linear', 'extrap')';
    desired_state.acc = interp1(times, acc_des, t, 'linear', 'extrap')';
    desired_state.yaw      = 0;
    desired_state.yawdot   = 0;

