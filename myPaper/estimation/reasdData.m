%%%Nominal motor values
init.J_11 = 3.084E-3;
init.J_22 = 3.132E-3;
init.J_33 = 3.540E-3;
init.J_12 = 0.082E-3;
init.J_23 = 0.016E-3;
init.J_13 = -0.054E-3;
init.cx   = 0; %m
init.cy   = 0; %m
init.cz   = 0; %m
init.m_sat = 1.317; %kg
init.alpha = 0;  
init.miss_x = 0; 
init.miss_y = 0;
init.miss_z = 0;

initial = cell2mat(struct2cell(init));

%%%1. Read data measured from the experiment
% Especifica la ruta del archivo CSV
filename = 'FeedbackControl.csv'; %%%Free movement
% Especificar tiempo de muestreo
dt = 1E-3;
% Leer el archivo CSV y almacenar los datos en una tabla
data = readtable(filename);

% Mostrar las primeras filas de la tabla
disp(data(1:5, :));

%%%Time vector 
measured.t = data.t; 

%%%Measured data respect to body frame
q = angle2quat(deg2rad(data.Yaw),deg2rad(data.Pitch),deg2rad(data.Roll),'ZXY');
%%%Get x matrix
measured.x = [q, data.Ang_vel_x, data.Ang_vel_y, data.Ang_vel_z]';
%%%Uniform the sample time
[t_uniform, x_uniform] = setSampleTime(measured.t,measured.x',dt);
%%%Measured data respect to body frame
q_meas = measured.x(1:4,:); %%Assuming quaternions measurement errors is low
w_meas_body = measured.x(5:7,:); %%Angular rate in body frame

%%%Measured torque delivered
[~, u_uniform] = setSampleTime(measured.t,[zeros(length(data.u_z_),2),data.u_z_],dt);

%% 2. Represent the estimation data as an |iddata| object. 
y = x_uniform;
u = u_uniform;

z = iddata(y, u, dt, 'Name', 'ADCS-module');
%%
% 3. Specify input and output signal names, start time and time units. 
z.InputName = {'Torque_x','Torque_y','Torque_z'};
z.InputUnit =  {'Nm','Nm','Nm'};
z.OutputName = {'q0', 'q1', 'q2', 'q3', 'ang Rate x', 'ang Rate y', 'ang Rate z'};
z.OutputUnit = {'-', '-', '-', '-', 'rad/s' ,'rad/s', 'rad/s'};
z.Tstart = 0;
z.TimeUnit = 's';

% 4. Plot the data.
%
% The data is shown in two plot windows.
figure('Name', [z.Name ': Torque input -> sat quat']);
plot(z(:, 1:4, 1:3)); grid on;  % Plot first input-output pair (t -> q).

% The data is shown in two plot windows.
figure('Name', [z.Name ': Torque input -> sat ang rates']);
plot(z(:, 5:7, 1:3)); grid on;  % Plot first input-output pair (t -> q).

%% 2. Represent the satellite dynamics using an |idnlgrey| object.
%
% The model describes how the inputs generate the outputs using the state
% equation(s).
FileName      = 'adcsModuleModel';       % File describing the model structure.
Order         = [7 3 7];           % Model orders [ny nu nx].
Parameters    = initial';          % Initial parameters. Np = 11.
InitialStates = [-0.9999,-0.0045,-0.0103,0.0098,...
                  0.0100,-0.0300,-0.0300]';            % Initial initial states.
Ts            = 0;                 % Time-continuous system.
nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, ...
                'Name', 'ADCS-module');
%%
%
% 3. Specify input and output names, and units.
set(nlgr, 'InputName', {'Torque_x','Torque_y','Torque_z'}, ...
          'InputUnit', {'Nm','Nm','Nm'},               ...
          'OutputName', {'q0', 'q1', 'q2', 'q3', 'ang Rate x', 'ang Rate y', 'ang Rate z'}, ...
          'OutputUnit', {'-', '-', '-', '-', 'rad/s' ,'rad/s', 'rad/s'},                         ...
          'TimeUnit', 's');

%%
% 
% 4. Specify names and units of the initial states and parameters.
nlgr = setinit(nlgr, 'Name',  {'q0', 'q1', 'q2', 'q3', 'ang Rate x', 'ang Rate y', 'ang Rate z'});
nlgr = setinit(nlgr, 'Unit', {'-', '-', '-', '-', 'rad/s' ,'rad/s', 'rad/s'});

nlgr = setpar(nlgr, 'Name', {'J_11','J_22','J_33','J_12','J_13','J_23','cx','cy','cz','m_sat','alpha','miss_x','miss_y','miss_z'});
nlgr = setpar(nlgr, 'Unit', {'kg⋅m²','kg⋅m²','kg⋅m²','kg⋅m²','kg⋅m²','kg⋅m²','m','m','m','kg','-','Nm','Nm','Nm'});

%% 
%
% You can also use |setinit| and |setpar| to assign values, minima, maxima,
% and estimation status for all initial states or parameters
% simultaneously.
%
% 5. View the initial model.
%
% a. Get basic information about the model.
%
% The DC-motor has 2 (initial) states and 2 model parameters.
size(nlgr)

%%
% b. View the initial states and parameters.
%
% Both the initial states and parameters are structure arrays. The fields
% specify the properties of an individual initial state or parameter. Type
% |help idnlgrey.InitialStates| and |help idnlgrey.Parameters| for
% more information.
nlgr.Parameters(1).Minimum = eps(0);
nlgr.Parameters(1).Maximum = 1E-2;
nlgr.Parameters(1)

nlgr.Parameters(2).Minimum = eps(0);
nlgr.Parameters(2).Maximum = 1E-2;
nlgr.Parameters(2)

nlgr.Parameters(3).Minimum = eps(0);
nlgr.Parameters(3).Maximum = 1E-2;
nlgr.Parameters(3)

nlgr.Parameters(4).Minimum = -1E-3;
nlgr.Parameters(4).Maximum = 1E-3;
nlgr.Parameters(4)

nlgr.Parameters(5).Minimum = -1E-3;
nlgr.Parameters(5).Maximum = 1E-3;
nlgr.Parameters(5)

nlgr.Parameters(6).Minimum = -1E-3;
nlgr.Parameters(6).Maximum = 1E-3;
nlgr.Parameters(6)

nlgr.Parameters(7).Minimum = -5E-2;
nlgr.Parameters(7).Maximum = 5E-2;
nlgr.Parameters(7)

nlgr.Parameters(8).Minimum = -5E-2;
nlgr.Parameters(8).Maximum = 5E-2;
nlgr.Parameters(8)

nlgr.Parameters(9).Minimum = -20E-2;
nlgr.Parameters(9).Maximum =  20E-2;
nlgr.Parameters(9)

nlgr.Parameters(10).Fixed = 1;
nlgr.Parameters(10)

nlgr.Parameters(11).Minimum = 0;
nlgr.Parameters(11).Maximum = 5E-2;
nlgr.Parameters(11).Fixed = 0;
nlgr.Parameters(11)

nlgr.Parameters(12).Minimum = -1E-2;
nlgr.Parameters(12).Maximum = 1E-2;
nlgr.Parameters(12).Fixed = 0;
nlgr.Parameters(12)

nlgr.Parameters(13).Minimum = -1E-2;
nlgr.Parameters(13).Maximum = 1E-2;
nlgr.Parameters(13).Fixed = 0;
nlgr.Parameters(13)

nlgr.Parameters(14).Minimum = -1E-2;
nlgr.Parameters(14).Maximum = 1E-2;
nlgr.Parameters(14).Fixed = 0;
nlgr.Parameters(14)

%%
%
% c. Retrieve information for all initial states or model parameters in one
% call.
% 
% For example, obtain information on initial states that are fixed (not
% estimated) and the minima of all model parameters.
getinit(nlgr, 'Fixed')

%%
%
% d. Obtain basic information about the object:
nlgr

%%
% Use |get| to obtain more information about the model properties. The
% |idnlgrey| object shares many properties of parametric linear model
% objects.
get(nlgr)

%% Performance Evaluation of the Initial DC-Motor Model
%
% Before estimating the parameters |tau| and |k|, simulate the output of
% the system with the parameter guesses using the default differential
% equation solver (a Runge-Kutta 45 solver with adaptive step length
% adjustment). The simulation options are specified using the
% "SimulationOptions" model property.
%
% 1. Set the absolute and relative error tolerances to
% small values (|1e-6| and |1e-5|, respectively).
nlgr.SimulationOptions.AbsTol = 1e-6;
nlgr.SimulationOptions.RelTol = 1e-5;

%%
%
% 2. Compare the simulated output with the measured data.
%
% |compare| displays both measured and simulated outputs of one or more
% models, whereas |predict|, called with the same input arguments, displays
% the simulated outputs.
%
% The simulated and measured outputs are shown in a plot window.
compare(z, nlgr);

%% Parameter Estimation
% Estimate the parameters and initial states using |nlgreyest|, which is a
% prediction error minimization method for nonlinear grey box models. The
% estimation options, such as the choice of estimation progress display,
% are specified using the "nlgreyestOptions" option set.
nlgr = setinit(nlgr, 'Fixed', {false false false false false false false}); % Estimate the initial states.
opt = nlgreyestOptions('Display', 'on');
opt.SearchOptions.Advanced.UseParallel = 'on';
nlgr = nlgreyest(z, nlgr, opt);

%% Performance Evaluation of the Estimated DC-Motor Model
% 1. Review the information about the estimation process.
%
% This information is stored in the |Report| property of the
% |idnlgrey| object. The property also contains information about how the
% model was estimated, such as solver and search method, data set, and why
% the estimation was terminated.
nlgr.Report
fprintf('\n\nThe search termination condition:\n')
nlgr.Report.Termination
% % % % %% Plot measured data
% % % % figure()
% % % % sp1 = subplot(3,1,1);
% % % %     plot(measured.t,measured.x(5,:),'.'); grid on; hold on;
% % % %     plot(t_uniform,x_uniform(:,5),'--');
% % % %     title('Measured Ang rates (Body Frame)'); xlabel('time (s)'); ylabel('\omega_x(rad/s)');
% % % % sp2 = subplot(3,1,2);
% % % %     plot(measured.t,measured.x(6,:),'.'); grid on; hold on;
% % % %     plot(t_uniform,x_uniform(:,6),'--');
% % % %     xlabel('time (s)'); ylabel('\omega_y(rad/s)');
% % % % sp3 = subplot(3,1,3);
% % % %     plot(measured.t,measured.x(7,:),'.'); grid on; hold on;
% % % %     plot(t_uniform,x_uniform(:,7),'--');
% % % %     xlabel('time (s)'); ylabel('\omega_z(rad/s)');
% % % % linkaxes([sp1, sp2, sp3],'x');
% % % % 
% % % % %%Plot measured data
% % % % figure()
% % % % sp1 = subplot(4,1,1);
% % % %     plot(measured.t,measured.x(1,:),'.'); grid on; hold on;
% % % %     plot(t_uniform,x_uniform(:,1),'--');
% % % %     title('Measured quaternions'); xlabel('time (s)'); ylabel('q_0');
% % % % sp2 = subplot(4,1,2);
% % % %     plot(measured.t,measured.x(2,:),'.'); grid on; hold on;
% % % %     plot(t_uniform,x_uniform(:,2),'--');
% % % %     xlabel('time (s)'); ylabel('q_1');
% % % % sp3 = subplot(4,1,3);
% % % %     plot(measured.t,measured.x(3,:),'.'); grid on; hold on;
% % % %     plot(t_uniform,x_uniform(:,3),'--');
% % % %     xlabel('time (s)'); ylabel('q_2');
% % % % sp4 = subplot(4,1,4);
% % % %     plot(measured.t,measured.x(4,:),'.'); grid on; hold on;
% % % %     plot(t_uniform,x_uniform(:,4),'--');
% % % %     xlabel('time (s)'); ylabel('q_3');
% % % % linkaxes([sp1, sp2, sp3, sp4],'x');
% % % % 
% % % % figure()
% % % % sp1 = subplot(3,1,1);
% % % %     plot(measured.t,zeros(length(data.u_z_),1),'.'); grid on; hold on;
% % % %     plot(t_uniform,u_uniform(:,1),'--');
% % % %     title('Inputs(Nm)'); xlabel('time (s)'); ylabel('T_x(Nm)');
% % % % sp2 = subplot(3,1,2);
% % % %     plot(measured.t,zeros(length(data.u_z_),1),'.'); grid on; hold on;
% % % %     plot(t_uniform,u_uniform(:,2),'--');
% % % %     xlabel('time (s)'); ylabel('T_y(Nm)');
% % % % sp3 = subplot(3,1,3);
% % % %     plot(measured.t,data.u_z_*1E-4,'.'); grid on; hold on;
% % % %     plot(t_uniform,u_uniform(:,3),'--');
% % % %     xlabel('time (s)'); ylabel('T_z(Nm)');
% % % % linkaxes([sp1, sp2, sp3],'x');


function [ t_uniform, x_uniform] = setSampleTime(tout,data,t_sample)
    %%% Interpolation to have uniform sampling
    %temp = diff(tout);               %%Temporal time
    %t_sample = mean(temp(1:end-1));                             %%Get sample time
    t_uniform = tout(1):t_sample:tout(end); %%Define a uniform time vector
    %%% Start interpolation
    x_uniform = interp1(tout, data, t_uniform, 'spline');  % Interpolación
end
