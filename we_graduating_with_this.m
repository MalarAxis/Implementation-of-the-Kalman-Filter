% Parameters
T = 1; % Sampling time
Q = [10, 0; 
    0, 10]; % Process noise covariance
G = [0, 0;
     0, 0;
     1, 0;
     0, 1]; % Noise Influence
R = [5, 0; 
    0, 5];  % Measurement noise covariance
A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, 0, 0, 0;
     0, 0, 0, 0]; % Process Influence

% Initial conditions
x_true = [20; 
          20; 
         -0.2; 
           0]; % True initial state

x_est = [x_true(1) + randn * 100;  % Random x-position 
         x_true(2) + randn * 100;  % Random y-position 
         x_true(3) + randn * 11;   % Random x-velocity 
         x_true(4) + randn * 13];  % Random y-velocity

P = [1, 0, 0, 0; 
     0, 1, 0, 0; 
     0, 0, 0.01, 0; 
     0, 0, 0, 0.01]; % Initial covariance matrix

Q_d = G * Q * G' * T; % Discretize the Noise of process

% State transition matrix (Phi_k)
Phi = eye(4) + A * T;

% Measurement function
h = @(x) [sqrt(x(1)^2 + x(2)^2); 
          atan2(x(2), x(1))];

% Simulation parameters
N = 200; % Number of time steps
x_history = zeros(4, N); % Store true state
x_est_history = zeros(4, N); % Store estimated state
z_history = zeros(2, N); % Store measurements
P_history = zeros(4, 4, N); % To store the full covariance matrices
variance_history = zeros(4, N); % To store variances

% EKF loop
for k = 1:N
    % True state dynamics (with process noise)
    w = mvnrnd([0; 0], Q)'; % Process noise
    x_true = Phi * x_true + G*w; % True state update
    x_history(:, k) = x_true;
    
    % Measurement (with measurement noise)
    z = h(x_true) + mvnrnd([0; 0], R)'; % Measurement with noise
    z_history(:, k) = z;
    
    % Prediction step
    x_pred = Phi * x_est; % Predicted state
    P_pred = Phi * P * Phi' + Q_d; % Predicted covariance
    
    % Linearize measurement function (Jacobian H_k)
    r = sqrt(x_pred(1)^2 + x_pred(2)^2);

    H = [x_pred(1)/r,      x_pred(2)/r,    0, 0;
         -x_pred(2)/(r^2),  x_pred(1)/(r^2),  0, 0];
    
    % **Compute the predicted measurement**
    z_pred = h(x_pred);
    
    % **Calculate the innovation with angle wrapping**
    % Compute the difference in the angle and wrap it
    angle_diff = z(2) - z_pred(2);
    angle_diff = wrapToPi(angle_diff); % Wrap to [-pi, pi]
    
    % Construct the innovation vector
    innovation = [z(1) - z_pred(1); angle_diff];

    % Update step
    K = (P_pred * H') / (H * P_pred * H' + R); % Kalman gain
    x_est = x_pred + K * innovation; % Updated state estimate
    P = (eye(4) - K * H) * P_pred; % Updated covariance matrix
    
     % Store the full covariance matrix
    P_history(:, :, k) = P;
    % Store variances (diagonal elements)
    variance_history(:, k) = diag(P);

    % Store estimated state
    x_est_history(:, k) = x_est;

    % Extract the covariances between x-position and x-velocity
    cov_xpos_xvel = squeeze(P_history(1, 3, :));

    % Extract the covariances between y-position and y-velocity
    cov_ypos_yvel = squeeze(P_history(2, 4, :));

end

% Plot results
time = 1:N;

figure;
subplot(2, 1, 1);
plot(time, x_history(1, :), 'b', 'LineWidth', 1.5);
hold on;
plot(time, x_est_history(1, :), 'r--', 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('x-position');
legend('True', 'Estimated');
title('Position Estimation');

subplot(2, 1, 2);
plot(time, x_history(2, :), 'b', 'LineWidth', 1.5);
hold on;
plot(time, x_est_history(2, :), 'r--', 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('y-position');
legend('True', 'Estimated');
title('Position Estimation');

figure;
subplot(2, 1, 1);
plot(time, x_history(3, :), 'b', 'LineWidth', 1.5);
hold on;
plot(time, x_est_history(3, :), 'r--', 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('x-velocity');
legend('True', 'Estimated');
title('Velocity Estimation');

subplot(2, 1, 2);
plot(time, x_history(4, :), 'b', 'LineWidth', 1.5);
hold on;
plot(time, x_est_history(4, :), 'r--', 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('y-velocity');
legend('True', 'Estimated');
title('Velocity Estimation');

% Plot true vs estimated trajectory
figure;
plot(x_history(1, :), x_history(2, :), 'b', 'LineWidth', 1.5);
hold on;
plot(x_est_history(1, :), x_est_history(2, :), 'r--', 'LineWidth', 1.5);
xlabel('x-position');
ylabel('y-position');
legend('True Trajectory', 'Estimated Trajectory');
title('True vs Estimated Trajectory');
grid on;

% Plot variances of position estimates
figure;
subplot(2, 1, 1);
plot(time, variance_history(1, :), 'r', 'LineWidth', 1.5);
hold on;
plot(time, variance_history(2, :), 'b', 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('Variance');
title('Position Estimate Variances');
legend('Variance of x-position', 'Variance of y-position');
grid on;

% Plot variances of velocity estimates
subplot(2, 1, 2);
plot(time, variance_history(3, :), 'g', 'LineWidth', 1.5);
hold on;
plot(time, variance_history(4, :), 'm', 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('Variance');
title('Velocity Estimate Variances');
legend('Variance of x-velocity', 'Variance of y-velocity');
grid on;

% Plot Cross Vars of Position/Velocity
figure;
subplot(2, 1, 1);
plot(time, cov_xpos_xvel, 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('Covariance');
title('Covariance: x-position and x-velocity');
grid on;

subplot(2, 1, 2);
plot(time, cov_ypos_yvel, 'LineWidth', 1.5);
xlabel('Time Step');
ylabel('Covariance');
title('Covariance: y-position and y-velocity');
grid on;