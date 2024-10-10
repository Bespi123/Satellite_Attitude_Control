function [Sat] = sat(Umax, a, u)
% sat - Applies vector saturation to the input vector `u`.
%
% Inputs:
%    Umax: Maximum allowed value for the control signal
%    a:    Parameter for the tanh saturation function
%    u:    Input control signal (3x1 vector)
%
% Outputs:
%    Sat: Saturated control signal (3x1 vector)

% Pre-allocate the output vector
Sat = zeros(3, 1);

% Loop through the 3 components of the input vector u
for i = 1:3
    if u(i) >= (Umax - a)
        % Case when u(i) is large (saturation upper bound)
        Sat(i) = (Umax - a) + a * tanh((u(i) - Umax + a) / a);
    elseif ((a - Umax) < u(i)) && (u(i) < (Umax - a))
        % Case when u(i) is within the bounds (no saturation)
        Sat(i) = u(i);
    else
        % Case when u(i) is small (saturation lower bound)
        Sat(i) = (a - Umax) + a * tanh((u(i) + Umax - a) / a);
    end
end

end