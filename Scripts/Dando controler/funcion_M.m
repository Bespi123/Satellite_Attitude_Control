function [M] = funcion_M(x)
% funcion_M - Generates a specific 3x6 matrix based on a 3-component vector `x`.
%
% Inputs:
%    x: A 3-component vector [x1, x2, x3]'
%
% Outputs:
%    M: A 3x6 matrix constructed from the components of the input vector

% Ensure that the input is a 3-component vector
if length(x) ~= 3
    error('Input vector must have 3 components.');
end

% Define the matrix M using the components of the input vector `x`
M = [x(1), 0,    0,    0, x(3), x(2);
     0,    x(2), 0,    x(3), 0,    x(1);
     0,    0,    x(3), x(2), x(1), 0];

end
