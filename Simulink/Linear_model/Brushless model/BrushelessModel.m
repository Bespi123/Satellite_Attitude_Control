function [dx, y] = BrushelessModel(t, x, u, kt, J, B, Kc, L, R, Ke, varargin)
% BrushelessModel - Simulates a simplified brushless motor model.
% 
% This function calculates the time derivatives of the state variables 
% (angular velocity and current) and outputs the current state values.
%
% Syntax:
%   [dx, y] = BrushelessModel(t, x, u, kt, J, B, Kc, L, R, Ke)
%
% Inputs:
%   t   - Time (not used in the equations but kept for compatibility with ODE solvers)
%   x   - State vector:
%           x(1) -> Angular velocity (rad/s)
%           x(2) -> Current (A)
%   u   - Input voltage (Vmean)
%   kt  - Torque constant (N*m/A)
%   J   - Rotor inertia (kg*m^2)
%   B   - Viscous friction constant (N*m*s)
%   Kc  - Coulomb friction constant (N*m)
%   L   - Inductance (H)
%   R   - Resistance (ohms)
%   Ke  - Back EMF constant (V/(rad/s))
% 
% Outputs:
%   dx  - Derivative of the state vector:
%           dx(1) -> Angular acceleration (rad/s^2)
%           dx(2) -> Current derivative (A/s)
%   y   - Output vector (same as input state vector for this model):
%           y(1) -> Angular velocity (rad/s)
%           y(2) -> Current (A)
%
% Example:
%   [dx, y] = BrushelessModel(0, [0; 0], 24, 0.0255, 0.001, 0.0001, 0.01, 0.56e-3, 1.2, 0.85);
%
% The function simulates a brushless motor under the influence of input
% voltage, motor parameters such as inertia, friction, and back EMF.

    % State variables
    w = x(1);  % Angular velocity (rad/s)
    i = x(2);  % Current (A)

    % Equations of motion
    % Angular acceleration (rad/s^2)
    w_dot = (1/J) * (kt * i - B * w - Kc * sign(w));

    % Current derivative (A/s)
    i_dot = (1/L) * (u - R * i - Ke * w);

    % Derivative of state vector
    dx = [w_dot; i_dot];

    % Output (state variables)
    y = [w; i];
end
