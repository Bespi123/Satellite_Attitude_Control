function [B] = Beta(dq13, n, gamma)
% Beta - Computes the Beta function for quaternion error vector dq13.
%
% Inputs:
%    dq13:  3-component quaternion error vector (vector part)
%    n:     Threshold parameter
%    gamma: Exponent for the large dq13 values
%
% Outputs:
%    B:     3x1 vector calculated based on dq13, n, and gamma
%
% Pre-allocate the output vector
B = zeros(3, 1);

% Coefficients for small dq13 values (|dq13| <= n)
r1 = (2 - gamma) * n^(gamma - 1);
r2 = (gamma - 1) * n^(gamma - 2);

% Loop over the 3 components of dq13
for i = 1:3
    if abs(dq13(i)) > n
        % Case for large dq13 values (|dq13| > n)
        B(i) = sign(dq13(i)) * abs(dq13(i))^gamma;
    else
        % Case for small dq13 values (|dq13| <= n)
        B(i) = r1 * dq13(i) + r2 * sign(dq13(i)) * dq13(i)^2;
    end
end

end