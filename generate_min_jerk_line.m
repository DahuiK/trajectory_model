function [x_ref, y_ref] = generate_min_jerk_line(p0, p1, T, t_vec)
%GENERATE_MIN_JERK_LINE 5th-order min-jerk trajectory from p0 to p1.
% p0, p1 are 2x1 [x; y] positions.
% T is total time, t_vec is time vector.

x0 = p0(1);  y0 = p0(2);
x1 = p1(1);  y1 = p1(2);

tau = t_vec / T;
tau(tau > 1) = 1;

% min-jerk scalar 0→1
s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;

x_ref = x0 + (x1 - x0) .* s;
y_ref = y0 + (y1 - y0) .* s;
end
