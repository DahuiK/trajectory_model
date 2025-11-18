function [x_ref, y_ref] = generate_spiral_to_center( ...
    p0, p1, T, t_vec, num_turns, r_max)
% p0 = [x0; y0], p1 = [x1; y1] (usually [0;0])
% r_max = max radius allowed (e.g., 0.8 * plate_radius)

if nargin < 5
    num_turns = 2;
end
if nargin < 6
    r_max = 0.08; % default 8 cm
end

x0 = p0(1);  y0 = p0(2);
x1 = p1(1);  y1 = p1(2); %#ok<NASGU>

r0     = hypot(x0, y0);
r0     = min(r0, r_max);      % clamp starting radius
theta0 = atan2(y0, x0);

tau = t_vec / T;
tau(tau > 1) = 1;

% smooth scalar 0→1
s = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;

% radius shrinks from r0 → 0 inside allowed margin
r = r0 .* (1 - s);

% angle winds num_turns revolutions
theta = theta0 + 2*pi*num_turns .* s;

x_ref = r .* cos(theta);
y_ref = r .* sin(theta);
end
