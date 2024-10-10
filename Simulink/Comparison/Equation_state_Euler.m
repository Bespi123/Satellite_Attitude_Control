function [x_dot] = Equation_state_Euler(entr)
%---------------------- Defining Variables --------------------------------
%   Td:         Disturbance torque [Td_x, Td_y, Td_z]
%   U:          Input torque (reaction wheel torques) [Ux, Uy, Uz]
%   Irw_par:    Parallel inertia of reaction wheels
%   Irw_per:    Perpendicular inertia of reaction wheels
%   J_tilde:    Rigid body inertia matrix
%   Euler:      Euler angles [roll, pitch, yaw]
%   w:          Angular velocities of the body [wx, wy, wz]
%   Wrw:        Angular velocities of reaction wheels [wrx, wry, wrz]

%----------------------------- Calculations ---------------------------------
Td = [entr(1), entr(2), entr(3)]';       % Disturbance torque
U = [entr(4), entr(5), entr(6)]';        % Input torque (reaction wheels)
Irw_per = entr(7);                       % Perpendicular inertia of reaction wheels
Irw_par = entr(8);                       % Parallel inertia of reaction wheels
Euler = [entr(15), entr(16), entr(17)]'; % Euler angles (roll, pitch, yaw)
w = [entr(18), entr(19), entr(20)]';     % Angular velocities of the body
Wrw = [entr(21), entr(22), entr(23)]';   % Reaction wheel angular velocities

% Inertia tensor of the rigid body
J_tilde = [entr(09), entr(12), entr(13);
           entr(12), entr(10), entr(14);
           entr(13), entr(14), entr(11)];

% Inertia matrix for parallel inertias (reaction wheels)
R = diag(Irw_par * ones(1, 3));

% General inertia tensor (body + reaction wheels)
J = J_tilde + diag(4 * Irw_per * ones(1, 3));

%----------------- Kinematic Equation -------------------------------------
% Reference:
% Fundamentals of Spacecraft Attitude Determination and Control
% Authors: F. Landis Markley & John L. Crassidis
%
% Euler angle kinematics (equation 3.38, sec. 3.147)
% Note: Singularity occurs at pitch = 90 degrees (gimbal lock)
% Assuming a 3-2-1 rotation sequence (yaw-pitch-roll, x1_y -> pitch)
B = [1, sin(Euler(1)) * tan(Euler(2)), cos(Euler(1)) * tan(Euler(2));
     0, cos(Euler(1)),                 -sin(Euler(1));
     0, sin(Euler(1)) / cos(Euler(2)), cos(Euler(1)) / cos(Euler(2))];

Euler_dot = B * w;  % Kinematic equation for Euler angles

%----------------- Dynamic Equation ---------------------------------------
% Dynamics of the rigid body with reaction wheels
w_dot = J \ (Td - U - cross(w, J * w + R * (w + Wrw)));  % Dynamic equation

%----------------- Reaction Wheel Dynamics --------------------------------
Wrw_dot = (R \ U) - w_dot;  % Reaction wheel dynamics

%----------------- State Derivative Vector --------------------------------
x_dot = [Euler_dot; w_dot; Wrw_dot];  % State vector derivative
end