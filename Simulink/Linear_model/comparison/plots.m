% Comparison of Non-Linear and Linear Models

% Subplot 1: Input Torque vs Time
sp1 = subplot(4,1,1);
n = abs(length(out.tout) - size(out.Torque, 1));  % Correct the size difference between time and torque data
plot(out.tout, [[0.007 * ones(n, 1), zeros(n, 2)]; out.Torque]);  % Ensure the data is concatenated correctly
title('Non-Linear and Linear Model Comparison: Input Torque');
ylabel('Input Torque (N.m)');
xlabel('Time (s)');
legend('Torque X', 'Torque Y', 'Torque Z');
grid on;

% Subplot 2: Euler Angles vs Time
sp2 = subplot(4,1,2);
plot(out.tout, [out.Euler, out.nonLinearEuler]);  % Plot both linear and non-linear Euler angles
title('Non-Linear and Linear Model Comparison: Euler Angles');
ylabel('Euler Angles (deg)');
xlabel('Time (s)');
legend('Roll (Linear)', 'Pitch (Linear)', 'Yaw (Linear)', ...
       'Roll (Non-Linear)', 'Pitch (Non-Linear)', 'Yaw (Non-Linear)');
grid on;

% Subplot 3: Angular Rates vs Time
sp3 = subplot(4,1,3);
plot(out.tout, [out.angularRate, out.nonLinearRates]);  % Compare linear and non-linear angular rates
title('Non-Linear and Linear Model Comparison: Angular Rates');
ylabel('Angular Rates (RPM)');
xlabel('Time (s)');
legend('X (Linear)', 'Y (Linear)', 'Z (Linear)', ...
       'X (Non-Linear)', 'Y (Non-Linear)', 'Z (Non-Linear)');
grid on;

% Subplot 4: Reaction Wheel Rates vs Time
sp4 = subplot(4,1,4);
plot(out.tout, [out.RwRate, out.nonLinearRwRates]);  % Compare reaction wheel rates
title('Non-Linear and Linear Model Comparison: Reaction Wheel Rates');
ylabel('Reaction Wheel Rates (RPM)');
xlabel('Time (s)');
legend('X (Linear)', 'Y (Linear)', 'Z (Linear)', ...
       'X (Non-Linear)', 'Y (Non-Linear)', 'Z (Non-Linear)');
grid on;

% Link the x-axes of all subplots for synchronized zooming/panning
linkaxes([sp1, sp2, sp3, sp4], 'x');

% Set x-axis limits for all subplots
xlim([0 2]);
