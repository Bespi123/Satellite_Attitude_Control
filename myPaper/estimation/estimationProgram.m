%%%Read csv Data
% Especifica la ruta del archivo CSV
filename = 'uncontrolledMode.csv'; %%%Frre movement

% Leer el archivo CSV y almacenar los datos en una tabla
data = readtable(filename);

% Mostrar las primeras filas de la tabla
%disp(data(1:5, :));

measured.t = data.x__time-data.x__time(1);
measured.x = [data.x_imu_orientation_w,data.x_imu_orientation_x,data.x_imu_orientation_y,data.x_imu_orientation_z...
            data.x_imu_angular_velocity_x,data.x_imu_angular_velocity_y,data.x_imu_angular_velocity_z]';

%%%Parameters to estimate
K=zeros(1,20);
%%%Inertia Tensor Values
K(1)=0.006052624839162;
K(2)=0.003577842742920;
K(3)=0.002497219771078;
%K(4)=0.082E-3;
%K(5)=0.054E-3;
%K(6)=0.016E-3;
K(4)=6.093978881835938e-04;
K(5)=7.066875696182251e-04;
K(6)=2.573514043829434e-04;

%%%Gravity center offset
%K(7:9)= [-0.78, 37.84, 149.19]*1E-3;
K(7:9)= [0, 0, -3.352165222167969e-04];
%%%Satellite mass
K(10) = 1.244977993011475;
%%%airbearing Friction Coef
K(11) = 0.012187498192474;
%%%Unmodeled friction coef
K(12:14)=[0,0,0];
K(15:17)=[0,8.937320788259939e-04,-2.056360244750977e-06];
K(18:20)=[0.010000000000000,8.296966552734375e-05,0];

no_var = 20;  %number of variables
lb = [0 0 0 0 0 0 -3E-2 -3E-2 -3E-2 1.243 0 -1E-2 -1E-2 -1E-2 -1E-2 -1E-2 -1E-2 1E-2 -1E-2 -1E-2]; % lower bound
up = [1E-2,1E-2,1E-2,1E-3,1E-3,1E-3,3E-2,3E-2,3E-2,1.245,2E-2,1E-2,1E-2,1E-2,1E-2,1E-2,1E-2,1E-2,1E-2,1E-2]; % high bound
initial = K;

%GA OPTIONS
%try
ga_opt = gaoptimset('Display','off','Generations',200,'PopulationSize',100, ...
    'InitialPopulation',initial,'PlotFcns',@gaplotbestf);
obj_fun = @(k_est)costFunction(k_est);

[k_est,bestblk] = ga((obj_fun),no_var,[],[],[],[],lb,up,[],ga_opt);
