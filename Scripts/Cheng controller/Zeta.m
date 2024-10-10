function [zeta_dot] = Zeta(dq, w, wd, u, zeta, alpha1, alpha2, n, gamma, gamma1, zeta_o, k1, k2, Umax, a)
% Zeta - Auxiliary system to compensate for actuator saturation
%
% Inputs:
%    dq: Quaternion error (4x1 vector)
%    w:  Satellite's angular velocity (3x1 vector)
%    wd: Desired angular velocity (3x1 vector)
%    u:  Control input torque (3x1 vector)
%    zeta: Sliding surface auxiliary variable (3x1 vector)
%    alpha1: Gain for quaternion error
%    alpha2: Gain for Beta function
%    n, gamma: Parameters for Beta function
%    gamma1: Exponent for compensative control
%    zeta_o: Threshold for the auxiliary variable
%    k1, k2: Gains for the auxiliary system
%    Umax: Maximum allowable torque
%    a: Saturation parameter
%
% Outputs:
%    zeta_dot: Time derivative of zeta (3x1 vector)

% Quaternion error decomposition
dq0 = dq(1);        % Scalar part of quaternion error
dq13 = dq(2:4);     % Vector part of quaternion error

% Rotation matrix from quaternion error
C = quat2rotm([dq0, dq13']);

% Angular velocity error
we = w - C * wd;

% Sliding surface (s)
s = we + alpha1 * dq13 + alpha2 * Beta(dq13, n, gamma);

% Delta control input (difference between saturated and actual control input)
delta_u = sat(Umax, a, u) - u;

% Check if norm of zeta is below the threshold zeta_o
if norm(zeta) <= zeta_o
    % If norm is below threshold, zeta_dot is zero
    zeta_dot = [0; 0; 0];
else
    % Otherwise, compute zeta_dot using the given formula
    zeta_dot = -k1 * zeta - k2 * [sign(zeta(1)) * abs(zeta(1))^gamma1;
                                  sign(zeta(2)) * abs(zeta(2))^gamma1;
                                  sign(zeta(3)) * abs(zeta(3))^gamma1] ...
                - (norm(s' * delta_u, 1) + 0.5 * (delta_u' * delta_u)) / (norm(zeta)^2) * zeta ...
                + delta_u;
end

% Ensure zeta remains a column vector (in case it's modified elsewhere)
%zeta = zeta(:);

end