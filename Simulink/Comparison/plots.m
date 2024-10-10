% Comparison of Non-Linear and Linear Models

% Subplot 1: Input Torque vs Time
subplot(4,1,1);
plot(out.t, [[0.007 * ones(6, 1), zeros(6, 1), zeros(6, 1)]; out.Torque]);
title('Non-Linear and Linear Model Comparison');
ylabel('Input Torque (N.m)');
xlabel('Time (s)');
legend('X', 'Y', 'Z');
grid on;

% Subplot 2: Euler Angles vs Time
subplot(4,1,2);
plot(out.t, [out.Euler, out.nonLinearEuler]);
title('Euler Angles Comparison');
ylabel('Euler Angles (deg)');
xlabel('Time (s)');
legend('Roll (Linear)', 'Pitch (Linear)', 'Yaw (Linear)', 'Roll (Non-Linear)', 'Pitch (Non-Linear)', 'Yaw (Non-Linear)');
grid on;

% Subplot 3: Angular Rates vs Time
subplot(4,1,3);
plot(out.t, [out.angularRate, out.nonLinearRates]);
title('Angular Rates Comparison');
ylabel('Angular Rates (RPM)');
xlabel('Time (s)');
legend('X (Linear)', 'Y (Linear)', 'Z (Linear)', 'X (Non-Linear)', 'Y (Non-Linear)', 'Z (Non-Linear)');
grid on;

% Subplot 4: Reaction Wheel Rates vs Time
subplot(4,1,4);
plot(out.t, [out.RwRate, out.nonLinearRwRates]);
title('Reaction Wheel Rates Comparison');
ylabel('Reaction Wheel Rates (RPM)');
xlabel('Time (s)');
legend('X (Linear)', 'Y (Linear)', 'Z (Linear)', 'X (Non-Linear)', 'Y (Non-Linear)', 'Z (Non-Linear)');
grid on;