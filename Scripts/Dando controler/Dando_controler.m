function [u] = Dando_controler(wd_dot, wd, w, dq, dq_dot, J_tilde, theta, lambda, Kd)
% Dando_controler - Implements Dando's control law for satellite attitude control.
%
% Inputs:
%    wd:       Desired angular velocity (3x1 vector)
%    wd_dot:   Time derivative of desired angular velocity (3x1 vector)
%    w:        Current angular velocity of the satellite (3x1 vector)
%    dq:       Quaternion error (4x1 vector)
%    dq_dot:   Derivative of quaternion error (4x1 vector)
%    J_tilde:  Calculated inertia tensor (flattened 6-element vector)
%    theta:    Error in calculated inertia tensor (adaptation parameter)
%    lambda:   Control gain for quaternion error
%    Kd:       Control gain for sliding surface
%
% Outputs:
%    u:        Control torque (3x1 vector)

% Reconstruct the inertia tensor (J_tilde) into a 3x3 matrix
J = [J_tilde(1), J_tilde(4), J_tilde(5);
     J_tilde(4), J_tilde(2), J_tilde(6);
     J_tilde(5), J_tilde(6), J_tilde(3)];

% Quaternion error decomposition
dq0 = dq(1);        % Scalar part of the quaternion error
dq13 = dq(2:4);     % Vector part of the quaternion error
dq13_dot = dq_dot(2:4);  % Derivative of the vector part of quaternion error

% Rotation matrix from quaternion error
R = quat2rotm([dq0, dq13']);

% Reference angular velocity
wr = R * wd - lambda * sign(dq0) * dq13;

% Angular velocity error
wc_tilde = w - R * wd;

% Sliding surface (S)
S = w - wr;

% Auxiliary term alpha_r
alpha_r = R * wd_dot - skew(wc_tilde) * R * wd - lambda * dq13_dot;

% Phi matrix (inertia tensor adaptation term)
phi = -(funcion_M(alpha_r) + skew(wr) * funcion_M(w));

% Inertia tensor adaptation term (tilde_u)
tilde_u = phi * theta;

% Control law calculation
u = -Kd * S + J * alpha_r + skew(wr) * J * w + tilde_u;

end