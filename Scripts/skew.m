% AUTHOR: 
%  Brayan Espinoza 1/10/2020
% DESCRIPTION: 
% This function calculates the skew-symmetric matrix for a given 3D vector.
% The skew-symmetric matrix is useful for cross-product operations and
% rotation calculations in attitude control and kinematics.
%
% INPUT: 
%    vector: A 3-component vector [x, y, z]
%
% OUTPUT: 
%    skewsim: The 3x3 skew-symmetric matrix of the input vector
%
function [skewsim] = skew(vector)

% Ensure the input is a 3-component vector
if length(vector) ~= 3
    error('Input must be a 3-component vector');
end

% Compute the skew-symmetric matrix
skewsim = [  0,         -vector(3),  vector(2);
           vector(3),    0,         -vector(1);
          -vector(2),  vector(1),    0      ];

end