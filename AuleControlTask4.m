% Parameters
T_f = 10; % Total time for trajectory (s)
dt = 0.01; % Time step (s)
time = 0:dt:T_f; % Time vector
n_steps = length(time);

% Initial conditions (Chaser position, velocity, orientation)
x0 = 0; y0 = 0; theta0 = 30 * pi / 180;
vx0 = 0; vy0 = 0; omega0 = 0; % Initial velocities and angular velocity

% Target position (fixed for simplicity)
x_target = 5; y_target = 3;

% Trajectory generation (same as in Task 3)
% Define boundary conditions for x, y, and theta
xf = 5; yf = 3; % Final positions
vxf = 0; vyf = 0; % Final velocities
axf = 0; ayf = 0; % Final accelerations

thetaf = 0; omega0 = 0; alphaf = 0; % Final orientation (aligned)
theta_traj = linspace(theta0, thetaf, n_steps);
x_traj = linspace(x0, xf, n_steps);
y_traj = linspace(y0, yf, n_steps);

% Create figure for animation
figure;
axis equal;
xlim([-1, 6]);
ylim([-1, 4]);

% Loop to animate the docking process
for i = 2:n_steps
    % Clear previous plot elements
    clf;
    axis equal;
    xlim([-1, 6]);
    ylim([-1, 4]);
    hold on;

    % Plot Target position (static)
    plot(x_target, y_target, 'rx', 'MarkerSize', 10); % Target position
    
    % Plot Chaser position (dynamic)
    h_chaser = plot(x_traj(i), y_traj(i), 'bo', 'MarkerSize', 10); % Chaser position
    
    % Plot Chaser orientation (arrow representing orientation)
    h_orientation = plot([x_traj(i), x_traj(i) + cos(theta_traj(i))], ...
                         [y_traj(i), y_traj(i) + sin(theta_traj(i))], 'g', 'LineWidth', 2); % Orientation
    
    % Initialize thruster arrows (just as an example)
    h_thruster_x = quiver(x_traj(i), y_traj(i), 0.1, 0, 'r', 'LineWidth', 2);
    h_thruster_y = quiver(x_traj(i), y_traj(i), 0, 0.1, 'b', 'LineWidth', 2);
    h_thruster_torque = quiver(x_traj(i), y_traj(i), 0, 0, 'm', 'LineWidth', 2); % Torque (not used initially)
    
    % Pause for a short time to update the animation
    pause(0.01);
end

title('Chaser Docking Animation');
xlabel('X Position (m)');
ylabel('Y Position (m)');
grid on;
