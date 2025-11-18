% Ball-on-platform simulation:
% - Click to choose starting position on the plate
% - Smooth min-jerk trajectory from start to center
% - PID on x, y controlling platform roll & pitch
% - Animation with ball path + roll/pitch

clear; clc; close all;

%% config 

R_plate = 0.1;  % platform radius [m]

% Position limits (clamp to plate radius)
pos_min = -R_plate;
pos_max =  R_plate;

% PID gains (can tune)
Kp_x = 15.0;  Ki_x = 0.0;  Kd_x = 3.0;
Kp_y = 15.0;  Ki_y = 0.0;  Kd_y = 3.0;

% Platform max tilt (deg) relative to level
max_roll_deg  = 8.0;
max_pitch_deg = 8.0;

% Simulation time
T_total = 15.0;     % [s]
dt      = 0.01;     % [s]
t_vec   = 0:dt:T_total;
N       = numel(t_vec);

% Ball dynamics 
g       = 9.81;
k_accel = 0.7 * g;  % tilt -> acceleration gain
c_damp  = 3.0;      % viscous damping

% center 
x_target = 0; 
y_target = 0;

%% get initial position (mouse click) 

x0 = 0.06;    % 6 cm
y0 = -0.04;   % -4 cm

useMouseStart = true;  % set false if you don't want to click

if useMouseStart
    fig_sel = figure('Name','Select Ball Start Position','Color','w');
    ax_sel  = axes(fig_sel);
    hold(ax_sel,'on'); axis(ax_sel,'equal'); grid(ax_sel,'on');
    set(ax_sel,'FontSize',12);
    th = linspace(0,2*pi,300);
    plot(ax_sel, R_plate*cos(th), R_plate*sin(th), 'k-', 'LineWidth', 2);
    title(ax_sel,'Click inside the plate to set starting ball position');
    xlabel(ax_sel,'x [m]'); ylabel(ax_sel,'y [m]');
    xlim(ax_sel,1.1*[-R_plate R_plate]);
    ylim(ax_sel,1.1*[-R_plate R_plate]);

    [x_click, y_click] = ginput(1);   % wait for one click

    % Clamp to plate radius (just in case click is slightly outside)
    r_click = hypot(x_click, y_click);
    if r_click > R_plate
        x_click = x_click * (R_plate / r_click);
        y_click = y_click * (R_plate / r_click);
    end

    x0 = x_click;
    y0 = y_click;

    close(fig_sel);
end

%% generate reference trajectory line

[x_ref, y_ref] = generate_min_jerk_line([x0; y0], [x_target; y_target], ...
                                        T_total, t_vec);

%% state initialize 

% Ball states
x    = x0;   xdot = 0;
y    = y0;   ydot = 0;

% PID states
int_ex = 0;  int_ey = 0;
prev_ex = 0; prev_ey = 0;

% Logs
x_log     = zeros(1,N);
y_log     = zeros(1,N);
xr_log    = zeros(1,N);
yr_log    = zeros(1,N);
roll_log  = zeros(1,N);
pitch_log = zeros(1,N);

%% simulation loop 

for k = 1:N
    % ref
    xr = x_ref(k);
    yr = y_ref(k);

    % Error = ball - ref
    ex = x - xr;
    ey = y - yr;

    % PID update
    int_ex = int_ex + ex*dt;
    int_ey = int_ey + ey*dt;

    dex = (ex - prev_ex)/dt;
    dey = (ey - prev_ey)/dt;

    prev_ex = ex;
    prev_ey = ey;

    % Controller outputs (dimensionless)
    u_x = Kp_x*ex + Ki_x*int_ex + Kd_x*dex;
    u_y = Kp_y*ey + Ki_y*int_ey + Kd_y*dey;

    % Tilt opposite to error so ball rolls toward reference
    theta_x = -u_x;  % roll
    theta_y = -u_y;  % pitch

    % Convert to deg and saturate
    theta_x_deg = max(min(theta_x * 180/pi, max_roll_deg ), -max_roll_deg );
    theta_y_deg = max(min(theta_y * 180/pi, max_pitch_deg), -max_pitch_deg);

    % Back to rad for dynamics
    theta_x = deg2rad(theta_x_deg);
    theta_y = deg2rad(theta_y_deg);

    % Damped dynamics
    % x aligned with roll, y with pitch
    xddot =  k_accel * theta_x - c_damp * xdot;
    yddot =  k_accel * theta_y - c_damp * ydot;

    % Integrate
    xdot = xdot + xddot*dt;
    x    = x    + xdot*dt;

    ydot = ydot + yddot*dt;
    y    = y    + ydot*dt;

    % Clamp to plate
    x = min(max(x, pos_min), pos_max);
    y = min(max(y, pos_min), pos_max);

    % Log
    x_log(k)  = x;
    y_log(k)  = y;
    xr_log(k) = xr;
    yr_log(k) = yr;
    roll_log(k)  = theta_x_deg;
    pitch_log(k) = theta_y_deg;
end

%% Animation

fig = figure('Name','Ball to Center - Trajectory Tracking', ...
             'Color','w', ...
             'Position',[100 100 1200 700]);

tl = tiledlayout(fig,2,1);
tl.TileSpacing = 'compact';
tl.Padding     = 'compact';

% plate top view
ax1 = nexttile(tl,1);
hold(ax1,'on'); axis(ax1,'equal'); grid(ax1,'on');
set(ax1,'FontSize',12);
title(ax1,'Top View: Ball Path vs Reference','FontSize',14);
xlabel(ax1,'x [m]','FontSize',12);
ylabel(ax1,'y [m]','FontSize',12);

th = linspace(0,2*pi,300);
plot(ax1, R_plate*cos(th), R_plate*sin(th),'k-','LineWidth',2); 

h_ref   = plot(ax1, x_ref, y_ref, '--', 'LineWidth', 1.5);
h_ball  = plot(ax1, x_log(1), y_log(1), 'o', ...
               'MarkerSize', 10, ...
               'MarkerFaceColor',[0.2 0.7 1], ...
               'MarkerEdgeColor','k', ...
               'LineWidth',1.5);
h_trail = plot(ax1, x_log(1), y_log(1), '-', 'LineWidth',1.0);

legend(ax1, {'Platform edge','Reference','Ball','Ball path'}, ...
       'Location','bestoutside');

xlim(ax1, 1.1*[-R_plate R_plate]);
ylim(ax1, 1.1*[-R_plate R_plate]);

% roll and pitch
ax2 = nexttile(tl,2);
hold(ax2,'on'); grid(ax2,'on');
set(ax2,'FontSize',12);
title(ax2,'Platform Roll & Pitch (relative to level)','FontSize',14);
xlabel(ax2,'Time [s]','FontSize',12);
ylabel(ax2,'Angle [deg]','FontSize',12);

h_roll  = plot(ax2, t_vec(1), roll_log(1), 'LineWidth',2);
h_pitch = plot(ax2, t_vec(1), pitch_log(1),'--','LineWidth',2);
legend(ax2, {'Roll (x-axis)','Pitch (y-axis)'}, 'Location','best');

% play back
for k = 1:4:N
    if ~isvalid(h_ball)
        break;
    end

    % Ball + trail
    set(h_ball, 'XData', x_log(k),   'YData', y_log(k));
    set(h_trail,'XData', x_log(1:k), 'YData', y_log(1:k));

    % Roll/pitch
    set(h_roll, 'XData', t_vec(1:k), 'YData', roll_log(1:k));
    set(h_pitch,'XData', t_vec(1:k), 'YData', pitch_log(1:k));

    drawnow;
    pause(0.02);
end

disp('Simulation complete.');
