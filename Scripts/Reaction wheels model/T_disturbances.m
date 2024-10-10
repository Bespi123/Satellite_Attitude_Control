function [Tgg] = T_disturbances(rc, J_tilde, Irw_per, q)
% T_disturbances - Calculates the gravity gradient torque acting on the satellite.
%
% Inputs:
%    rc:        Distance from the Earth's center (m)
%    J_tilde:   Rigid body inertia tensor (3x3 matrix, flattened as vector)
%    Irw_per:   Perpendicular inertia of the reaction wheels (scalar)
%    q:         Attitude quaternion [q0, q1, q2, q3]'
%
% Outputs:
%    Tgg:       Gravity gradient torque (3x1 vector)
%
% Constants:
    mu = 3.986 * 10^14;  % Earth's gravitational constant (m^3/s^2)

% Reconstruct the inertia tensor (J_tilde) from the input vector
J_tilde = [J_tilde(1), J_tilde(4), J_tilde(5);
           J_tilde(4), J_tilde(2), J_tilde(6);
           J_tilde(5), J_tilde(6), J_tilde(3)];

% General inertia tensor (including reaction wheel perpendicular inertia)
J = J_tilde + diag(2 * Irw_per * ones(1, 3));

% Compute the gravitational gradient constant (ω_o^2)
wo2 = mu / rc^3;

% Compute the nadir-pointing unit vector (c3) using the quaternion
c3 = [2 * (q(2) * q(4) - q(3) * q(1));  % First component of c3
      2 * (q(3) * q(4) + q(2) * q(1));  % Second component of c3
      1 - 2 * (q(2)^2 + q(3)^2)];       % Third component of c3

% Compute the gravity gradient torque (Tgg)
Tgg = cross(3 * wo2 * c3, J * c3);

end


