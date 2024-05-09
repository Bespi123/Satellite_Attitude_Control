function J=myObjectiveFunction(k)
% Template for Creating an Objective Function Using a Simulink Block Diagram
% x contains all the controller variables of the problem in a column vector.
% Acknoledgment: 
% Professor Juan Pablo Requez Vivas for the Intelligent Control
% course - UNEXPO - 2023 - jrequez@unexpo.edu.ve
% Modified by Bespi123 on 26/02/2024

%%%%%%%%%%%%%%%%%%%     SECTION 1: Variables          %%%%%%%%%%%%%%%%%%%%%
% Separate the variables into their appropriate names, according to the
% problem at hand. These variables correspond to the elements of x, which
% must be n different elements.
pid.kp = k(1); 
pid.ki = k(2);
pid.kd = k(3);

%%%%%%%%%%%%%%%%%%%     SECTION 2: Conditions         %%%%%%%%%%%%%%%%%%%%%
% Operating conditions of the problem. If the problem has parameters or 
% data that you need to add, place them in this section.
%%%x-coil
%%Nominal motor values (estimated from real Data)
motor.kt = 0.000872234;  %kt 
motor.Jrw  = 2.55E-05;   %J
motor.b  = 1.00023E-05;  %B
motor.c = 0.000435786;   %kc
motor.L  = 0.000176564;  %H  
motor.R  = 1.23424;      %R    
motor.ke = 0.00966471;   %ke

%%Initial conditions
init.w_rw = 0;
init.current = 0; 

%%%%%%%%%%%%%%%%%%%     SECTION 3: Pre-Calculations   %%%%%%%%%%%%%%%%%%%%%
% Pre-calculations
% If the problem requires performing pre-calculations to facilitate the
% evaluation of the objective function, place them here.

%%%%%%%%%%%%%%%%%%% SECTION 4: Simulate the Process  %%%%%%%%%%%%%%%%%%%%%%
% Simulink is called to simulate the process of interest and calculate the
% simulation outputs
% WARNING!!! The model name must be changed in the following line
try
   %salidas=sim('xcoilSintonization','SrcWorkspace','current');
   %salidas=sim('ycoilSintonization','SrcWorkspace','current');
   salidas=sim('closeLoop','SrcWorkspace','current');
   stable = 1;
catch exception
   
   if strcmp(exception.identifier,'Simulink:Engine:DerivNotFinite')
    stable = 0;
    disp('System No stable')
   else
       stable = 0;
       disp('Otro error')
   end
end

if(stable == 1)
% The simulation outputs are divided into two groups, the time and the
% outputs themselves
yout = salidas.get('yout');
t    = salidas.get('tout');

% It is common to have the system error as out1, the controller output u as
% out2, and the output y as out3
u=yout(:,1); y=yout(:,2); e=yout(:,4);%Define error, control signal and output

%%%%%%%%%%%%%%%%%%% SECTION 5: Calculate the Fitness  %%%%%%%%%%%%%%%%%%%%%%
% The response of the process is now analyzed. If the input is not a step, 
% other variables or representative calculations must be chosen in this
% section and what is shown may not be applicable.

%-----Indice preestablecido------------
%digamos que se desea que el tiempo de establecimiento sea un valor
%específico
%desired_setlement_time = 376;
%desired_rising_time    = 80;
desired_setlement_time = 0;
desired_rising_time    = 0;

% calculate rising time
tr = calculateRissingTime(e, t, 50/100);
d_tr = abs(desired_rising_time-tr);

% Calculate settlement time
ts = calculateSettlementTime(e, t, 5/100);
d_ts = abs(desired_setlement_time-ts);

%-----índices de error integral --------
%itse=trapz(t,t.*e.^2);
itse = trapz(t,e.^2);

itsy = trapz(t,y.^2);

entropy = calculate_entropy(y);
% % %-----
%overshoot = max(y);
[overshoot, ~] = calculateOvershoot(y);

uwork = trapz(t,u.^2);

%%%-----Elección del índice deseado como fitness-------
%%% Se determina la salida del la función fitness
%%%J=itse+overshoot+umax+uwork+timeWorking;
%%%J=itse+setlement_time_error+overshoot+rising_time_error;
%%%J=itse+setlement_time_error+rising_time_error;
if isempty(tr) || isempty(ts)
    J = 5E4;
else
    J = d_tr+d_ts+itse+uwork+itsy+entropy+overshoot*100; %%%High value of J
end
else
    J = 5E4;
end

end

function entropy = calculate_entropy(signal)
    % Calculate the Shannon entropy of a signal.
    % Input: signal (1D array)
    % Output: entropy

    unique_values = unique(signal);
    probabilities = histcounts(signal, unique_values) / numel(signal);
    entropy = -sum(probabilities .* log2(probabilities));
end