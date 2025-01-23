% Parameters
T_f = 10; % Total time for trajectory (s)
dt = 0.01; % Time step (s)
time = 0:dt:T_f; % Time vector
n_steps = length(time);

% Boundary conditions for x, y, and theta
x0 = 0; xf = 5; % Initial and final x positions
vx0 = 0; vxf = 0; % Initial and final x velocities
ax0 = 0; axf = 0; % Initial and final x accelerations

y0 = 0; yf = 3; % Initial and final y positions
vy0 = 0; vyf = 0; % Initial and final y velocities
ay0 = 0; ayf = 0; % Initial and final y accelerations

theta0 = 30 * pi / 180; thetaf = 0; % Initial and final orientation (rad)
omega0 = 0; omegaf = 0; % Initial and final angular velocities
alpha0 = 0; alphaf = 0; % Initial and final angular accelerations

% Solve for polynomial coefficients (5th-order)
poly_coeffs = @(p0, pf, v0, vf, a0, af, T) ...
    [p0; v0; a0/2; ...
     (20*pf - 20*p0 - (8*vf + 12*v0)*T - (3*a0 - af)*T^2)/(2*T^3); ...
     (30*p0 - 30*pf + (14*vf + 16*v0)*T + (3*a0 - 2*af)*T^2)/(2*T^4); ...
     (12*pf - 12*p0 - (6*v0 + 6*vf)*T - (a0 - af)*T^2)/(2*T^5)];

% Calculate coefficients for x, y, and theta
a_x = poly_coeffs(x0, xf, vx0, vxf, ax0, axf, T_f);
a_y = poly_coeffs(y0, yf, vy0, vyf, ay0, ayf, T_f);
a_theta = poly_coeffs(theta0, thetaf, omega0, omegaf, alpha0, alphaf, T_f);

% Trajectory generation
x_traj = polyval(flip(a_x), time);
y_traj = polyval(flip(a_y), time);
theta_traj = polyval(flip(a_theta), time);

% Velocity and acceleration profiles
x_vel = polyval(polyder(flip(a_x)), time);
y_vel = polyval(polyder(flip(a_y)), time);
theta_vel = polyval(polyder(flip(a_theta)), time);

x_acc = polyval(polyder(polyder(flip(a_x))), time);
y_acc = polyval(polyder(polyder(flip(a_y))), time);
theta_acc = polyval(polyder(polyder(flip(a_theta))), time);

% Energy consumption (based on forces and torques)
m = 10; % Mass (kg)
I = 0.1; % Moment of inertia (kg·m^2)
F_x = m * x_acc; % Force in x
F_y = m * y_acc; % Force in y
tau = I * theta_acc; % Torque

gas_rate = 0.05; % g/s per thruster
energy = cumsum((abs(F_x) + abs(F_y) + abs(tau)) * gas_rate * dt);

% Plot results
figure;

% 2D Trajectory
subplot(3, 1, 1);
plot(x_traj, y_traj, 'b', 'LineWidth', 1.5);
title('2D Chaser Trajectory');
xlabel('X Position (m)');
ylabel('Y Position (m)');
grid on;
axis equal;

% Velocity and Acceleration Profiles
subplot(3, 1, 2);
plot(time, x_vel, 'r', 'LineWidth', 1.5); hold on;
plot(time, y_vel, 'g', 'LineWidth', 1.5);
plot(time, theta_vel, 'b', 'LineWidth', 1.5);
title('Velocity Profiles');
xlabel('Time (s)');
ylabel('Velocity (m/s or rad/s)');
legend('V_x', 'V_y', '\omega');
grid on;

subplot(3, 1, 3);
plot(time, x_acc, 'r', 'LineWidth', 1.5); hold on;
plot(time, y_acc, 'g', 'LineWidth', 1.5);
plot(time, theta_acc, 'b', 'LineWidth', 1.5);
title('Acceleration Profiles');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2 or rad/s^2)');
legend('A_x', 'A_y', '\alpha');
grid on;

% Energy Consumption
figure;
plot(time, energy, 'k', 'LineWidth', 1.5);
title('Energy Consumption vs. Time');
xlabel('Time (s)');
ylabel('Energy (g)');
grid on;

disp('Final Energy Consumption (g):');
disp(energy(end));
