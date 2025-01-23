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
B = [B_trans; zeros(2, 2)];
B_rot_combined = [zeros(4, 1); B_rot];

% Define control inputs (forces and torque)
F_x = 0.2; % Force in the X-direction (N)
F_y = 0.3; % Force in the Y-direction (N)
tau = 0.1; % Torque (Nm)

% Combine Inputs
u = [F_x; F_y; tau]; % Input vector (forces and torque)

% Display matrices for debugging purposes
disp('State-Space Matrix A:');
disp(A);

disp('State-Space Matrix B:');
disp(B);

disp('Input Vector u:');
disp(u);
