function [x_dot] = Equation_state_Euler(entr)
% EQUATION_STATE_EULER: Computes the time derivative of the state vector 
% for a spacecraft with reaction wheels using Euler's equations.
%
% INPUT:
%   entr - A vector containing the following elements:
%       [1-3]    Td: Disturbance torques [Td_x, Td_y, Td_z]
%       [4-6]    U: Input torques from the reaction wheels [Ux, Uy, Uz]
%       [7]      Irw_per: Perpendicular inertia of the reaction wheels
%       [8]      Irw_par: Parallel inertia of the reaction wheels
%       [9-11]   J_tilde diagonal elements: Rigid body inertia matrix (diagonal)
%       [12-14]  J_tilde off-diagonal elements: Rigid body inertia matrix
%       [15-17]  Euler: Euler angles [roll, pitch, yaw]
%       [18-20]  w: Angular velocities of the rigid body [wx, wy, wz]
%       [21-23]  Wrw: Angular velocities of the reaction wheels [wrx, wry, wrz]
%
% OUTPUT:
%   x_dot - Time derivative of the state vector (Euler angles, angular
%           velocities of the body, angular velocities of reaction wheels)

%----------------------------- Extract Inputs -----------------------------
Td = [entr(1), entr(2), entr(3)]';        % Disturbance torques
U = [entr(4), entr(5), entr(6)]';         % Input torques from reaction wheels
Irw_per = entr(7);                        % Perpendicular inertia of reaction wheels
Irw_par = entr(8);                        % Parallel inertia of reaction wheels
Euler = [entr(15), entr(16), entr(17)]';  % Euler angles (roll, pitch, yaw)
w = [entr(18), entr(19), entr(20)]';      % Angular velocities of the rigid body
Wrw = [entr(21), entr(22), entr(23)]';    % Angular velocities of reaction wheels

% Rigid body inertia matrix (J_tilde)
J_tilde = [entr(9),  entr(12), entr(13);
           entr(12), entr(10), entr(14);
           entr(13), entr(14), entr(11)];

% Inertia matrix for the reaction wheels (parallel inertia)
R = diag(Irw_par * ones(1, 3));  % Parallel inertia matrix

% General inertia tensor for the rigid body with reaction wheels
J = J_tilde + diag(4 * Irw_per * ones(1, 3));  % Total inertia tensor

%----------------- Kinematic Equation (Euler angles) ----------------------
% Kinematic equation to calculate Euler angle derivatives.
% Reference: Fundamentals of Spacecraft Attitude Determination and Control
% Eq. (3.38) with a 3-2-1 rotation sequence (yaw-pitch-roll)
% The transformation matrix (B) accounts for the Euler angle rates

B = [1, sin(Euler(1)) * tan(Euler(2)), cos(Euler(1)) * tan(Euler(2));
     0, cos(Euler(1)),                -sin(Euler(1));
     0, sin(Euler(1)) / cos(Euler(2)), cos(Euler(1)) / cos(Euler(2))];

Euler_dot = B * w;  % Euler angle rate of change

%----------------- Dynamic Equation (Rigid Body + Reaction Wheels) --------
% Equation for the angular acceleration of the rigid body with reaction wheels
% It includes contributions from the disturbance torque, input torque, and
% the coupling effects between the body and the reaction wheels (via cross products).

w_dot = J \ (Td - U - cross(w, J * w + R * (w + Wrw)));  % Angular acceleration

%----------------- Reaction Wheel Dynamics -------------------------------
% Reaction wheel angular velocity rate of change, influenced by the input 
% torque and the angular acceleration of the body.

Wrw_dot = (R \ U) - w_dot;  % Reaction wheel dynamics

%----------------- State Derivative Vector (x_dot) ------------------------
% Combine all calculated derivatives into the state derivative vector.
% x_dot contains: [Euler angle rates; body angular velocity rates; reaction wheel velocity rates]

x_dot = [Euler_dot; w_dot; Wrw_dot];  % State vector derivative

end
