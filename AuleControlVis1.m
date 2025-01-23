% Parameters
m = 10; % Mass of Chaser (kg)
I = 0.1; % Moment of Inertia (kg.m^2)

% State-space matrices for translational dynamics
A_trans = [0 0 1 0; 
           0 0 0 1; 
           0 0 0 0; 
           0 0 0 0];

B_trans = [0 0; 
           0 0; 
           1/m 0; 
           0 1/m];

% State-space matrices for rotational dynamics
A_rot = [0 1; 
         0 0];

B_rot = [0; 
         1/I];

% Combine translational and rotational into one model
A = blkdiag(A_trans, A_rot);
B = [B_trans, zeros(4, 1); zeros(2, 2), B_rot];

% Define control inputs (forces and torque)
F_x = 0.2; % Force in the X-direction (N)
F_y = 0.3; % Force in the Y-direction (N)
tau = 0.1; % Torque (Nm)

% Combine Inputs
u = [F_x; F_y; tau]; % Input vector (forces and torque)

% Simulation parameters
dt = 0.01; % Time step (s)
T = 10; % Total simulation time (s)
time = 0:dt:T; % Time vector
n_steps = length(time);

% Initial state
x0 = [0; 0; 0; 0; 0; 0]; % [x, y, vx, vy, theta, omega]
state = zeros(length(x0), n_steps);
state(:, 1) = x0;

% Simulate state trajectories
for i = 2:n_steps
    state(:, i) = state(:, i-1) + dt * (A * state(:, i-1) + B * [u(1:2); u(3)]);
end

% Extract individual states
x_pos = state(1, :); % X position
y_pos = state(2, :); % Y position
theta = state(5, :); % Orientation (theta)

% Plot results
figure;

subplot(3, 1, 1);
plot(time, x_pos, 'r', 'LineWidth', 1.5);
title('X Position vs Time');
xlabel('Time (s)');
ylabel('X Position (m)');
grid on;

subplot(3, 1, 2);
plot(time, y_pos, 'g', 'LineWidth', 1.5);
title('Y Position vs Time');
xlabel('Time (s)');
ylabel('Y Position (m)');
grid on;

subplot(3, 1, 3);
plot(time, theta, 'b', 'LineWidth', 1.5);
title('Orientation (\theta) vs Time');
xlabel('Time (s)');
ylabel('\theta (rad)');
grid on;

% Optional: 2D Trajectory Plot
figure;
plot(x_pos, y_pos, 'm', 'LineWidth', 1.5);
title('2D Trajectory of the Chaser');
xlabel('X Position (m)');
ylabel('Y Position (m)');
grid on;
axis equal;
