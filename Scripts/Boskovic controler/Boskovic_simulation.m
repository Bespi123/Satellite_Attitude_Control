function [angArray, xArray, uiArray, TdArray, L_dArray, EULERINT, ASCCT, T, ts] = Boskovic_simulation(gamma, delta, k, rc, n, t, Umax, J_tilde, Irw_par, Irw_per, qd_Array, wd_Array, ui, x, b, c, d, ws, alpha1, alpha2, alpha3, beta1, beta2, beta3)
% Inputs:
% gamma:     Learning rate;
% delta:     Gain;
% k:         Adaptive gain;
% rc:        CubeSat's distance from Earth's center;
% n:         Number of simulation steps;
% t:         Time vector;
% Umax:      Maximum torque;
% J_tilde:   Rigid body's inertia matrix;
% Irw_par:   Parallel inertia of reaction wheels;
% Irw_per:   Perpendicular inertia of reaction wheels;
% qd_Array:  Desired attitude quaternion array;
% wd_Array:  Desired angular velocity array;
% ui:        Initial control input;
% x:         State vector [attitude quaternion, angular velocities];
% ...       Miscellaneous input parameters.

% Outputs:
% angArray:  Array of Euler angles over time;
% xArray:    State vector array;
% uiArray:   Control input array;
% TdArray:   Disturbance torque array;
% L_dArray:  Friction torque array;
% EULERINT:  Euler angle error integration over time;
% ASCCT:     Accumulated square of control torque;
% T:         Computation times for control signals;
% ts:        Settling time when error reaches a certain threshold.

    % Initialize variables and arrays
    angArray = quat2eul(x(1:4)')';
    euler_ant = 0;
    k_ant = k;
    k_dot_ant = 0;
    EULERINT = 0;
    ASCCT = 0;
    ascct_ant = 0;
    ascct_dot_ant = 0;
    euint_ant = 0;
    xArray = x;
    uiArray = ui;
    TdArray = zeros(3,1);
    L_dArray = zeros(3,1);
    deltaD = [0, alpha2*sin(beta2), alpha3*cos(beta3);
              alpha1*cos(beta1), 0, alpha3*sin(beta3);
              alpha1*sin(beta1), alpha2*cos(beta2), 0];
    ok = 0;
    ts = 0;
    
    % Loop through the simulation steps
    for i = 1:n-2
        dt = t(2) - t(1);
        qd = qd_Array(:,i);        % Desired attitude quaternion
        Wd = wd_Array(:,i);        % Desired angular velocity
        
        % Gravity gradient torque
        Tgg = T_disturbances(rc, J_tilde, Irw_per, x(1:4));
        
        % Friction torque (assuming 0 for now)
        L_d = [0, 0, 0]';
        
        % Miscellaneous torque (assuming 0 for now)
        L_mis = [0, 0, 0]';
        
        % Total disturbance torque
        Td = Tgg - L_d + L_mis;
        
        % 4th Order Runge-Kutta for state propagation
        g1 = dt * Equation_state_rw(Td, Irw_per, Irw_par, J_tilde, ui, x);
        g2 = dt * Equation_state_rw(Td, Irw_per, Irw_par, J_tilde, ui, x + 0.5 .* g1);
        g3 = dt * Equation_state_rw(Td, Irw_per, Irw_par, J_tilde, ui, x + 0.5 .* g2);
        g4 = dt * Equation_state_rw(Td, Irw_per, Irw_par, J_tilde, ui, x + 0.5 .* g3);
        x = x + (1/6) * (g1 + 2 * g2 + 2 * g3 + g4);
        
        % Quaternion error
        dq = Error_quaternio(qd, x(1:4));
        
        % Gain estimator using Simpson's integration method
        k_dot = Gain_estimator_bosk(x(5:7), Wd, dq, delta, gamma, k, Umax);
        k = k_ant + dt / 6 * (k_dot_ant + 2 * (k_dot_ant + k_dot) + k_dot);
        
        % Boskovic control input
        ui = Boskovic_control(x(5:7), Wd, dq, delta, k, Umax);
        T(i) = toc;
        ui = ui * -1;
        
        % Saturation of control inputs
        for j = 1:3
            if abs(ui(j)) > Umax
                ui(j) = sign(ui(j)) * Umax;
            end
        end
        
        % Apply disturbance delta
        ui = eye(3) * ui + deltaD * ui;
        
        % Convert quaternion to XYZ Euler angles
        ang = quat2eul(x(1:4)')';
        if sum(abs(quat2eul(qd')' - ang) < 0.05 * quat2eul(qd')') == 3 && ok == 0
            ts = i * dt;
            ok = 1;
        end
        
        % Euler angle error integration (Simpson's method)
        dqq = eul2quat(pi/180 * [10, 20, 30] - ang');
        eulerang = 2 * acos(dqq(1));
        euint = euint_ant + dt / 6 * (euler_ant + 2 * (euler_ant + eulerang) + eulerang);
        
        % ASCCT calculation (Simpson's method)
        ascct_dot = norm(ui)^2;
        ascct = ascct_ant + dt / 6 * (ascct_dot_ant + 2 * (ascct_dot_ant + ascct_dot) + ascct_dot);
        
        % Update previous values
        k_ant = k;
        k_dot_ant = k_dot;
        euler_ant = eulerang;
        euint_ant = euint;
        ascct_ant = ascct;
        ascct_dot_ant = ascct_dot;
        
        % Store results in arrays
        ASCCT = [ASCCT ascct];
        EULERINT = [EULERINT euint];
        TdArray = [TdArray Tgg];
        L_dArray = [L_dArray L_d];
        angArray = [angArray ang];
        xArray = [xArray x];
        uiArray = [uiArray ui];
    end
end
