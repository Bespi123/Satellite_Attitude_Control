function [k_dot] = Gain_estimator_bosk(w, wd, dq, delta, gamma, k, Umax)
% Inputs:
%   w:     Satellite's angular velocity;
%   wd:    Desired angular velocity;
%   dq:    Error quaternion;
%   delta: Gain;
%   gamma: Learning rate;
%   k:     Adaptive gain;
%   Umax:  Saturation torque;
%
% Calculations
dq0 = dq(1);                    % Scalar part of the quaternion error
dq13 = dq(2:4);                 % Vector part of the quaternion error
R = quat2rotm([dq0, dq13']);    % Rotation matrix from the quaternion

we = w - R * wd;     % Error in angular velocity

% Initialize summation variable
sum_val = 0;         % Use a more descriptive name for the variable

% Sliding surface
S = we + k^2 * dq13;

% Summation
for i = 1:3
    sum_val = sum_val + (we(i) * dq13(i)) / (abs(S(i)) + k^2 * delta) ...
             - (abs(we(i)) * (1 + delta)) / (abs(we(i)) + k^2 * (1 + delta));
end

% Gain Estimator
k_dot = gamma * k / (1 + 4 * gamma * (1 - dq0)) * ...
        (Umax * sum_val * we' * dq13 - k^2 * (dq13') * dq13);
end

