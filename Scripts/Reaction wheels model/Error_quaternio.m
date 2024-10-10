function [q_err] = Error_quaternio(qd, q)
% Error_quaternio - Calculates the quaternion error between desired (qd) and current (q) quaternions.
%
% Inputs:
%    qd: Desired quaternion [qd0, qd1, qd2, qd3]';
%    q:  Current quaternion [q0, q1, q2, q3]';
%
% Outputs:
%    q_err: Quaternion error [q0_err, q13_err] where q0_err is the scalar part, and q13_err is the vector part.

% Quaternion error matrix (Xi)
Xi = [-qd(2), -qd(3), -qd(4);
       qd(1), -qd(4),  qd(3);
       qd(4),  qd(1), -qd(2);
      -qd(3),  qd(2),  qd(1)];

% Vector part of the quaternion error (q13_err)
q13_err = Xi' * q;

% Scalar part of the quaternion error (q0_err)
q0_err = qd' * q;

% Quaternion error output [scalar part; vector part]
q_err = [q0_err; q13_err];

end