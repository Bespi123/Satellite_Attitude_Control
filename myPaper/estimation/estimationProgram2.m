%%%Read csv Data
global measured
% Especifica la ruta del archivo CSV
filename = 'uncontrolledMode.csv'; %%%Frre movement
% Especificar tiempo de muestreo
t_sample = 5E-2;
no_var = 13;  %number of variables

% Leer el archivo CSV y almacenar los datos en una tabla
data = readtable(filename);

% Mostrar las primeras filas de la tabla
%disp(data(1:5, :));

measured.t_raw = data.x__time-data.x__time(1);
measured.x_raw = [data.x_imu_orientation_w,data.x_imu_orientation_x,data.x_imu_orientation_y,data.x_imu_orientation_z...
            data.x_imu_angular_velocity_x,data.x_imu_angular_velocity_y,data.x_imu_angular_velocity_z]';
[measured.t, measured.x] = setSampleTime(measured.t_raw,measured.x_raw',t_sample);
measured.x = measured.x';

%%%Parameters to estimate
initial=zeros(1,no_var);
%%%Inertia Tensor Values
initial(1) = 0.009613843687391;
initial(2) = 0.098790159821859;
initial(3) = 0.097785883899461;
%initial(4)=-0.000682426613639;
%initial(5)=0.000864367715638;
%initial(6)=0.000991821289062;
initial(4) =  0.000720683076672;
initial(5) = -0.004991779490391;
initial(6) = -0.008665462907103;

%%%Gravity center offset
initial(7:9)= [0.056625804154980, -0.001000000000000, -0.000823642439759];

%%%Unmodeled 
initial(10)  = 0.01;
initial(11:13)  = [-0.009887425098015, 0.009893183003803,0.000100000000000];
lb = [0, 0, 0, -1E-2, -1E-2, -1E-2,...
      -1E-1, -1E-1, -1E-1,...
      0,...
      -1E-2, -1E-2, -1E-2]; % lower bound
up = [1E-1, 1E-1, 1E-1, 1E-2, 1E-2, 1E-2, ...
      1E-1, 1E-1, 1E-1,...
      1E-2, ...
      1E-2, 1E-2, 1E-2]; % high bound

%GA OPTIONS
%try
ga_opt = gaoptimset('Display','off','Generations',10,'PopulationSize',100, ...
    'InitialPopulation',initial,'PlotFcns',@gaplotbestf);
obj_fun = @(k_est)costFunction1(k_est);

[k_est,bestblk] = ga((obj_fun),no_var,[],[],[],[],lb,up,[],ga_opt);

function [ t_uniform, x_uniform] = setSampleTime(tout,data,t_sample)
    %%% Interpolation to have uniform sampling
    %temp = diff(tout);               %%Temporal time
    %t_sample = mean(temp(1:end-1));                             %%Get sample time
    t_uniform = tout(1):t_sample:tout(end); %%Define a uniform time vector
    %%% Start interpolation
    x_uniform = interp1(tout, data, t_uniform, 'spline');  % Interpolación
end