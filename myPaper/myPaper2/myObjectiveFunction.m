function J=myObjectiveFunction(k)
% Template for Creating an Objective Function Using a Simulink Block Diagram
% x contains all the controller variables of the problem in a column vector.
% Acknoledgment: 
% Professor Juan Pablo Requez Vivas for the Intelligent Control
% course - UNEXPO - 2023 - jrequez@unexpo.edu.ve
% Modified by Bespi123 on 26/02/2024
global initialConditions parameters 
%initialConditions = [];

%%%%%%%%%%%%%%%%%%%     SECTION 1: Variables          %%%%%%%%%%%%%%%%%%%%%
% Separate the variables into their appropriate names, according to the
% problem at hand. These variables correspond to the elements of x, which
% must be n different elements.
feedback.P = [k(1),k(2),k(3)];
feedback.K = [k(4),k(5),k(6)];

%%%%%%%%%%%%%%%%%%%     SECTION 2: Conditions         %%%%%%%%%%%%%%%%%%%%%
% Operating conditions of the problem. If the problem has parameters or 
% data that you need to add, place them in this section.
motor.Jrw = 2.55e-05;
motor.b   = 1.00023e-05;
motor.c   = 0.000435786;

sat.J     = [3.084,3.132,3.540,0.082,-0.054,0.016];

init.q    = [1,0,0,0];
init.w    = [0,0,0];
init.W_rw = [0,0,0];

setPoint.angd = 40*rand(3,1)-20;
setPoint.wd   = [0,0,0]';

initialConditions = [initialConditions,setPoint.angd];
%%%%%%%%%%%%%%%%%%%     SECTION 3: Pre-Calculations   %%%%%%%%%%%%%%%%%%%%%
% Pre-calculations
% If the problem requires performing pre-calculations to facilitate the
% evaluation of the objective function, place them here.

%%%%%%%%%%%%%%%%%%% SECTION 4: Simulate the Process  %%%%%%%%%%%%%%%%%%%%%%
% Simulink is called to simulate the process of interest and calculate the
% simulation outputs
% WARNING!!! The model name must be changed in the following line
try
   salidas=sim('closeLoopSimulation','SrcWorkspace','current');
   stable = 1;
catch exception
   
   if strcmp(exception.identifier,'Simulink:Engine:DerivNotFinite')
    stable = 0;
    disp('System No stable')
   else
       disp('Otro error')
       stable = 0;
   end
end

if(stable == 1)
% The simulation outputs are divided into two groups, the time and the
% outputs themselves
yout = salidas.get('yout');
t    = salidas.get('tout');

% It is common to have the system error as out1, the controller output u as
% out2, and the output y as out3
q=yout(:,1:4); w=yout(:,5:7); Wrw=yout(:,8:10);%Define error, control signal and output
dw = yout(:,11:13); dq=yout(:,14:17); u=yout(:,18:20);
%%%%%%%%%%%%%%%%%%% SECTION 5: Calculate the Fitness  %%%%%%%%%%%%%%%%%%%%%%
% The response of the process is now analyzed. If the input is not a step, 
% other variables or representative calculations must be chosen in this
% section and what is shown may not be applicable.

%-----Indice preestablecido------------
%digamos que se desea que el tiempo de establecimiento sea un valor
%específico

%%%EULERINT calculation
ang_and_axis = quat2axang(dq); 
eulerang = ang_and_axis(:,4);
eulerInt = cumtrapz(t,eulerang); 
EULERINT = eulerInt(end);

%%%ASCCT Calculation
ascct_dot=vecnorm(u,2,2).^2;
ascct = cumtrapz(t,ascct_dot); 
ASCCT = ascct(end);

%%%Settlement time calculation
tol = 5/100; % 2%
ts = calculateSettlementTime(180/pi*quat2eul(dq), t, tol);

%eulerAngs=180/pi*quat2eul(q);
entropy1 = calculate_entropy(q(:,1));
entropy2 = calculate_entropy(q(:,2));
entropy3 = calculate_entropy(q(:,3));
entropy4 = calculate_entropy(q(:,4));
entropy  = entropy1+entropy2+entropy3+entropy4;

% % %-----

%overshoot = max(y);
[overshoot] = calculateOvershoot(q,eul2quat(pi/180*setPoint.angd'));
arr = [EULERINT;ASCCT;ts;entropy;sum(overshoot)];
parameters = [parameters,arr];

%%%-----Elección del índice deseado como fitness-------
%%% Se determina la salida del la función fitness
%%%J=itse+overshoot+umax+uwork+timeWorking;
%%%J=itse+setlement_time_error+overshoot+rising_time_error;
%%%J=itse+setlement_time_error+rising_time_error;
if  isempty(ts)
    J = 5E4;
else
    J = EULERINT+ASCCT+ts+entropy+sum(overshoot);
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