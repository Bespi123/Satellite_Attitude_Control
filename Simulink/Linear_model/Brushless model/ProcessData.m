%**************************************************************************
% AUTHOR: Brayan Espinoza 23/10/2020
% DESCRIPTION: 
% This program processes the data obtained from the reaction wheel test. 
% It applies a discrete moving average filter to clean the data (e.g., the 
% motor current). The code also plots the raw and filtered signals for 
% better visualization of the reaction wheel performance.
%
% IMPORTANT:
% Ensure that the file 'RawDataObtained' is imported into the workspace as 
% a numeric matrix. This program reads the motor data from two different 
% input files: 'stepInput.txt' and 'trapesoidalInput.txt'.
%
% The following key variables are computed:
% - V: Applied voltage (scaled from raw data)
% - W: Reaction wheel angular rate (in rad/s)
% - i: Brushless motor current (in Amps)
% - t: Time vector (in seconds)
%
% *************************************************************************

% 1. Read and process data from step input
step1 = readmatrix('stepInput.txt');

%% OBTAINING DATA from stepInput.txt
V = step1(130:430,1)' * 20 / 255;           % Applied voltage (scaled)
W = 60 * step1(130:430,2)' / (6 * 0.2) * (2 * pi / 60); % Angular rate (rad/s)
i = 10^(-3) * step1(130:430,3)';            % Motor current (A)
t = 0.2 * (1:1:length(i));                  % Time vector (seconds)

%% Apply Moving Average Filter
windowSize = 3;                             % Define window size for moving average
b = (1/windowSize) * ones(1, windowSize);   % Filter coefficients
a = 1;                                      % Denominator coefficients for filter
i_m = filter(b, a, i);                      % Apply the filter to motor current

%% Prepare input and output data
y = [W',i'];                                % Angular rate and current as output
u = V';                                     % Applied voltage as input

%% Plot Data for step input
figure();
h1 = subplot(3,1,1); 
    plot(t, V, 'LineWidth', 2); grid on;
    xlabel('Time (s)'); ylabel('Voltage (V)');
    title('REACTION WHEEL DATA - Step Input'); 

h2 = subplot(3,1,2); 
    plot(t, W, 'LineWidth', 2); grid on;
    xlabel('Time (s)'); ylabel('Angular Rate (rad/s)');

h3 = subplot(3,1,3); 
    plot(t, i, 'LineWidth', 2); hold on;
    plot(t, i_m, '--', 'LineWidth', 2); grid on;   % Filtered signal with dashed line
    xlabel('Time (s)'); ylabel('Current (A)');
    legend('Non-filtered signal', 'Filtered signal');
    hold off;

% Link x-axes of all subplots
linkaxes([h1, h2, h3], 'x');

% % % 2. Read and process data from trapezoidal input
% % trapezoidal1 = readmatrix('trapesoidalInput.txt');
% % 
% % %% OBTAINING DATA from trapesoidalInput.txt
% % V = trapezoidal1(:,1)' * 20 / 255;                 % Applied voltage (scaled)
% % W = 60 * trapezoidal1(:,2)' / (6 * 0.2) * (2 * pi / 60); % Angular rate (rad/s)
% % i = 10^(-3) * trapezoidal1(:,3)';                  % Motor current (A)
% % t = 0.2 * (1:1:length(i));                         % Time vector (seconds)
% % 
% % %% Apply Moving Average Filter
% % windowSize = 3;                                    % Define window size for moving average
% % b = (1/windowSize) * ones(1, windowSize);          % Filter coefficients
% % a = 1;                                             % Denominator coefficients for filter
% % i_m = filter(b, a, i);                             % Apply the filter to motor current
% % 
% % %% Prepare input and output data
% % y = W';                                            % Angular rate as output
% % u = V';                                            % Applied voltage as input
% % 
% % %% Plot Data for trapezoidal input
% % figure();
% % h1 = subplot(3,1,1); 
% %     plot(t, V, 'LineWidth', 2); grid on;
% %     xlabel('Time (s)'); ylabel('Voltage (V)');
% %     title('REACTION WHEEL DATA - Trapezoidal Input'); 
% % 
% % h2 = subplot(3,1,2); 
% %     plot(t, W, 'LineWidth', 2); grid on;
% %     xlabel('Time (s)'); ylabel('Angular Rate (rad/s)');
% % 
% % h3 = subplot(3,1,3); 
% %     plot(t, i, 'LineWidth', 2); hold on;
% %     plot(t, i_m, '--', 'LineWidth', 2); grid on;   % Filtered signal with dashed line
% %     xlabel('Time (s)'); ylabel('Current (A)');
% %     legend('Non-filtered signal', 'Filtered signal');
% %     hold off;
% % 
% % % Link x-axes of all subplots
% % linkaxes([h1, h2, h3], 'x');