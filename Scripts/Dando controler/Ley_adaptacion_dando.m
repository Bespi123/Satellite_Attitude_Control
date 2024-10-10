function [theta_dot] = Ley_adaptacion_dando(wd_dot, wd, w, dq, dq_dot, lambda, gamma)
% Ley_adaptacion_dando - Implements Dando's adaptation law for satellite attitude control.
%
% Inputs:
%    wd:        Desired angular velocity (3x1 vector);
%    wd_dot:    Desired angular velocity derivative (3x1 vector); 
%    w:         Current angular velocity of the spacecraft (3x1 vector);
%    dq:        Quaternion error (4x1 vector);
%    dq_dot:    Quaternion error derivative (4x1 vector);
%    lambda:    Control gain;
%    gamma:     Adaptation gain (scalar);
%
% Outputs:
%    theta_dot: Adaptation law update (3x1 vector).
%
% Quaternion error decomposition
dq0 = dq(1);        % Scalar part of the quaternion error
dq13 = dq(2:4);     % Vector part of the quaternion error
dq13_dot = dq_dot(2:4);  % Derivative of the quaternion error (vector part)

% Convert quaternion error to a rotation matrix
R = quat2rotm(dq');  % Rotation matrix from the quaternion error

% Reference angular velocity (wr)
wr = R * wd - lambda * sign(dq0) * dq13;

% Angular velocity error
wc_tilde = w - R * wd;  % The angular velocity error between the spacecraft and the desired one

% Sliding surface (S)
S = w - wr;  % Sliding surface based on the difference between actual and reference angular velocities

% Calculate alpha_r (auxiliary term for the adaptation law)
alpha_r = R * wd_dot - skew(wc_tilde) * R * wd - lambda * dq13_dot;

% Phi function (an auxiliary term based on alpha_r and wr)
phi = -(funcion_M(alpha_r) + skew(wr) * funcion_M(w))';

% Adaptation law (theta_dot)
Gamma = gamma;  % Learning rate (or adaptation gain)
theta_dot = -Gamma * phi * S;  % Adaptation law update

end