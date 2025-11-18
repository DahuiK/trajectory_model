function [x_ref, y_ref] = generate_min_jerk_traj(p0, p1, T, t_vec)
%GENERATE_MIN_JERK_TRAJ 5th-order (minimum jerk) trajectory from p0 to p1 over [0, T].
% p0, p1 are 2x1 vectors [x; y]
% t_vec is 1xN time vector

x0 = p0(1);  y0 = p0(2);
x1 = p1(1);  y1 = p1(2);

tau = t_vec / T;
tau(tau>1) = 1;

s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;  % scalar blend from 0→1

x_ref = x0 + (x1 - x0)*s;
y_ref = y0 + (y1 - y0)*s;
end
