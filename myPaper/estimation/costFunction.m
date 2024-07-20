function J = costFunction(k)
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


%Simulation
t=measured.t;
x=zeros(7,length(t)+1);
x(:,1) = measured.x(:,1);

for i = 1:length(t)
    %%Get step size
    if i == 1
        dt = t(i+1);
    else
        dt=t(i)-t(i-1);
    end
      
    %Start RK 4 de cuarto orden
    g1=dt*testBed_model(t(i), x(:,i), k);
    g2=dt*testBed_model(t(i), x(:,i)+0.5.*g1, k);
    g3=dt*testBed_model(t(i), x(:,i)+0.5.*g2, k);
    g4=dt*testBed_model(t(i), x(:,i)+0.5.*g3, k);
    x(:,i+1)=x(:,i)+(1/6).*(g1+2.*g2+2.*g3+g4);
    
    %%% Progress bar configuration
    %%%% Calculate the progress percentage
    %progress = t(i) / settings.tfinal;
    
    %if isa(settings.hWaitbar,'handle') && isvalid(settings.hWaitbar)
    %    if t(i) == t(end)
            % Close the progress bar when the simulation is complete
    %        close(settings.hWaitbar);
    %    else
            % Update the progress bar
    %        waitbar(progress, settings.hWaitbar, sprintf('Progress: %.1f%%', progress*100));
    %    end
    %end
end

%%%Compute Residual Errors
J=sum(vecnorm(measured.x-x(:,1:end-1),2,1).^2)+norm(k(12:20))^2;
if isnan(J) 
    J = Inf;
end
end