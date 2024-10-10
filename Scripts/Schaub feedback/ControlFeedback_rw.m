function [U] = ControlFeedback_rw(J_tilde, Irw_per, Irw_par, x, dq, Wr, Wr_dot, P, K)
% Control law based on Lyapunov and Feedback Control
%---------------------Defining Variables---------------------------
%   Td: Disturbance Torque
%   U: Input Torque (torque of reaction wheels)
%   Irw_par: Inertia parallel to the reaction wheels
%   Irw_per: Inertia perpendicular to the reaction wheels
%   J_tilde: Inertia matrix of the rigid body
%   q: Attitude quaternion
%   w: Angular velocities of the body [wx, wy, wz]
%   Wrw: Angular velocities of the reaction wheels
%   P, K: Control gains
W = x(5:7);                     % Angular velocities of the body
Wrw = x(8:10);                  % Angular velocities of the reaction wheels
dq13 = [dq(2), dq(3), dq(4)]';  % Vector part of the quaternion error

% Tensor of inertia for the rigid body
J_tilde = [J_tilde(1), J_tilde(4), J_tilde(5);
           J_tilde(4), J_tilde(2), J_tilde(6);
           J_tilde(5), J_tilde(6), J_tilde(3)];

%---------------------Calculations----------------------------------
J = J_tilde + diag(2 * Irw_per * ones(1, 3));  % General inertia tensor
dW = W - Wr;  % Compute error in angular velocities
R = diag(Irw_par * ones(1, 3));  % Parallel inertia matrix
U = P * dW + K * dq13 - skew(W) * (J * W + R * (W + Wrw)) - J * (Wr_dot - skew(W) * Wr);

end

