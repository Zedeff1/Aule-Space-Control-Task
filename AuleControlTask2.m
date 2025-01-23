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

% Define LQR parameters
Q = diag([10, 10, 1, 1, 10, 1]); % Penalizes state errors
R = diag([0.1, 0.1, 0.05]);       % Penalizes control effort

% Compute optimal gain matrix
K = lqr(A, B, Q, R);

% Initial conditions
x0 = [0.5; -0.5; 0; 0; 30*pi/180; 0]; % Initial [x, y, vx, vy, theta, omega]

% Simulation parameters
dt = 0.01; % Time step (s)
T = 10; % Total simulation time (s)
time = 0:dt:T; % Time vector
n_steps = length(time);

% Preallocate arrays for states and control inputs
state = zeros(length(x0), n_steps);
state(:, 1) = x0;
u_profile = zeros(3, n_steps); % Thruster profile

% Simulate system with LQR control
for i = 2:n_steps
    % Calculate control input
    u = -K * state(:, i-1); 
    
    % Store thruster profile
    u_profile(:, i-1) = u;
    
    % Update state using discrete-time approximation
    state(:, i) = state(:, i-1) + dt * (A * state(:, i-1) + B * u);
end

% Calculate energy consumption
gas_rate = 0.05; % g/s per thruster
energy = cumsum(sum(abs(u_profile), 1) * gas_rate * dt); % Total gas used

% Extract states
relative_orientation = state(5, :); % Theta (orientation)
thruster_profile = u_profile; % Thruster force/torque

% Plot results
figure;

subplot(3, 1, 1);
plot(time, relative_orientation * (180/pi), 'r', 'LineWidth', 1.5);
title('Relative Orientation (\theta) vs Time');
xlabel('Time (s)');
ylabel('\theta (deg)');
grid on;

subplot(3, 1, 2);
plot(time, thruster_profile(1, :), 'b', 'LineWidth', 1.5); hold on;
plot(time, thruster_profile(2, :), 'g', 'LineWidth', 1.5);
plot(time, thruster_profile(3, :), 'm', 'LineWidth', 1.5);
title('Thruster Profile vs Time');
xlabel('Time (s)');
ylabel('Thruster Force/Torque (N or Nm)');
legend('F_x', 'F_y', '\tau');
grid on;

subplot(3, 1, 3);
plot(time, energy, 'k', 'LineWidth', 1.5);
title('Energy Consumption vs Time');
xlabel('Time (s)');
ylabel('Energy (g)');
grid on;

% Optional: Display results
disp('Final Energy Consumption (g):');
disp(energy(end));
