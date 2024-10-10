function [U] = Cheng_controller(J_tilde, dq, dq_dot, w, wd, wd_dot, Beta_dot, c, zeta, alpha1, alpha2, ko, tao1, tao2, rho, k3, k4, gamma1, n, gamma)
% Cheng_controller - Implements Cheng's controller for satellite attitude control.
%
% Inputs:
%    J_tilde:   Calculated inertia tensor (flattened 6-element vector)
%    w:         Current angular velocity of the satellite (3x1 vector)
%    wd:        Desired angular velocity (3x1 vector)
%    dq:        Quaternion error (4x1 vector)
%    dq_dot:    Time derivative of the quaternion error (4x1 vector)
%    wd_dot:    Desired angular velocity derivative (3x1 vector)
%    Beta_dot:  Time derivative of the Beta function
%    c:         Estimated parameters for compensative control
%    zeta:      Sliding surface gain
%    alpha1:    Gain for quaternion error
%    alpha2:    Gain for Beta function
%    ko:        Gain for compensative control
%    tao1, tao2: Gains for robust control components
%    rho:       Exponent for sliding mode control
%    k3, k4:    Gains for additional control components
%    gamma1:    Exponent for compensative control
%    n, gamma:  Parameters for Beta function
%
% Outputs:
%    U:         Control torque (3x1 vector)

% Inertia tensor construction from J_tilde (3x3 matrix)
Jo = [J_tilde(1), J_tilde(4), J_tilde(5);
      J_tilde(4), J_tilde(2), J_tilde(6);
      J_tilde(5), J_tilde(6), J_tilde(3)];

% Quaternion error decomposition
dq0 = dq(1);        % Scalar part of quaternion error
dq13 = dq(2:4);     % Vector part of quaternion error
dq13_dot = dq_dot(2:4);  % Time derivative of the quaternion error (vector part)

% Rotation matrix from quaternion error
C = quat2rotm([dq0, dq13']);

% Angular velocity error
we = w - C * wd;

% Calculate F term for control law
F = -skew(we + C * wd) * Jo * (we + C * wd) + Jo * (skew(we) * C * wd - C * wd_dot);

% Sliding surface
s = we + alpha1 * dq13 + alpha2 * Beta(dq13, n, gamma);

% Compensative control parameters
som_u = c(1) + c(2) * norm(we) + c(3) * norm(we)^2;
e = ko / (1 + som_u);

% Control components ur and un
ur = zeros(3, 1);
un = zeros(3, 1);
for i = 1:3
    ur(i) = -tao1 * s(i) - tao2 * sign(s(i)) * abs(s(i))^rho;
    un(i) = -k4 * sign(s(i)) * abs(s(i))^gamma1;
end

% Adaptive control component ua
ua = -som_u * (s / (norm(s) + e));

% Control torque calculation
U = -F - alpha1 * Jo * dq13_dot - alpha2 * Jo * Beta_dot - k3 * zeta - 0.5 * s + ur + un + ua;

end

