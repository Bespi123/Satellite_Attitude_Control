function [U] = Boskovic_control(w, wd, dq, delta, k, Umax)
% Inputs:
%    w:     Satellite's angular velocity;
%    wd:    Desired angular velocity;
%    dq:    Error quaternion;
%    delta: Gain related to the quaternion;
%    k:     Adaptive gain;
%    Umax:  Saturation torque;
%
% Calculations
dq0 = dq(1);                    % Scalar part of the quaternion error
dq13 = dq(2:4);                 % Vector part of the quaternion error
R = quat2rotm([dq0, dq13']);    % Rotation matrix from quaternion

we = w - R * wd;    % Error in angular velocity

% Sliding surface
S = we + k^2 * dq13;

% Control signal
U = zeros(3,1);  % Initialize control vector
for i = 1:3
    U(i) = -Umax * S(i) / (abs(S(i)) + k^2 * delta);
end

end