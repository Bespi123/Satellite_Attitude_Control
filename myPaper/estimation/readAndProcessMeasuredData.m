%%%Read csv Data
% Especifica la ruta del archivo CSV
filename = 'uncontrolledMode.csv'; %%%Free movement
% Especificar tiempo de muestreo
t_sample = 1E-3;
% Leer el archivo CSV y almacenar los datos en una tabla
data = readtable(filename);

% Mostrar las primeras filas de la tabla
disp(data(1:5, :));

%%%Time
measured.t = data.x__time-data.x__time(1); 
%%%Measured data respect to body frame
measured.x = [data.x_imu_orientation_w,data.x_imu_orientation_x,data.x_imu_orientation_y,data.x_imu_orientation_z...
              data.x_imu_angular_velocity_x,data.x_imu_angular_velocity_y,data.x_imu_angular_velocity_z]';
%%%Uniform the sample time
[t_uniform, x_uniform] = setSampleTime(measured.t,measured.x',t_sample);
%%%Measured data respect to body frame
q_meas = measured.x(1:4,:); %%Assuming quaternions measurement errors is low
w_meas_body = measured.x(5:7,:); %%Angular rate in body frame

%%Measurements data respect to inertial frame
w_meas_inertial = zeros(3,length(w_meas_body));
for i = 1:length(q_meas)
    w_meas_inertial(:,i) = quatRotation(q_meas(:,i),w_meas_body(:,i));
end

%% 3D plot
n = 450244/3;
figure()
plotVector([0,0,0],[1,0,0],1,'k','X_B','m');hold on;
title('Angular Rates (Body Frame)')
plotVector([0,0,0],[0,1,0],1,'k','Y_B','m');
plotVector([0,0,0],[0,0,1],1,'k','Z_B','m'); 
for i = 1:length(q_meas)
    if mod(i,n)==0
        x = w_meas_body(:,i);
        plotVector([0,0,0],x,50,'r--','\omega','m'); axis equal;
    end
end 

figure()
plotVector([0,0,0],[1,0,0],1,'k','X_I','m');hold on;
title('Angular Rates (Inertial Frame)')
plotVector([0,0,0],[0,1,0],1,'k','Y_I','m');
plotVector([0,0,0],[0,0,1],1,'k','Z_I','m'); 
for i = 1:length(q_meas)
    if mod(i,n)==0
        x = w_meas_inertial(:,i);
        plotVector([0,0,0],x,50,'r--','\omega','m'); axis equal;
        plotVector([0,0,0],quatRotation((q_meas(:,i)'),[1,0,0]),1,'k--','X_B','m');
        plotVector([0,0,0],quatRotation((q_meas(:,i)'),[0,1,0]),1,'k--','Y_B','m');
        plotVector([0,0,0],quatRotation((q_meas(:,i)'),[0,0,1]),1,'k--','Z_B','m'); axis equal;
    end
end

%% Plot measured data
figure()
sp1 = subplot(3,1,1);
    plot(measured.t,measured.x(5,:),'.'); grid on; hold on;
    plot(t_uniform,x_uniform(:,5),'--');
    title('Measured Ang rates (Body Frame)'); xlabel('time (s)'); ylabel('\omega_x(rad/s)');
sp2 = subplot(3,1,2);
    plot(measured.t,measured.x(6,:),'.'); grid on; hold on;
    plot(t_uniform,x_uniform(:,6),'--');
    xlabel('time (s)'); ylabel('\omega_y(rad/s)');
sp3 = subplot(3,1,3);
    plot(measured.t,measured.x(7,:),'.'); grid on; hold on;
    plot(t_uniform,x_uniform(:,7),'--');
    xlabel('time (s)'); ylabel('\omega_z(rad/s)');
linkaxes([sp1, sp2, sp3],'x');

%%Plot measured data
figure()
sp1 = subplot(4,1,1);
    plot(measured.t,measured.x(1,:),'.'); grid on; hold on;
    plot(t_uniform,x_uniform(:,1),'--');
    title('Measured quaternions'); xlabel('time (s)'); ylabel('q_0');
sp2 = subplot(4,1,2);
    plot(measured.t,measured.x(2,:),'.'); grid on; hold on;
    plot(t_uniform,x_uniform(:,2),'--');
    xlabel('time (s)'); ylabel('q_1');
sp3 = subplot(4,1,3);
    plot(measured.t,measured.x(3,:),'.'); grid on; hold on;
    plot(t_uniform,x_uniform(:,3),'--');
    xlabel('time (s)'); ylabel('q_2');
sp4 = subplot(4,1,4);
    plot(measured.t,measured.x(4,:),'.'); grid on; hold on;
    plot(t_uniform,x_uniform(:,4),'--');
    xlabel('time (s)'); ylabel('q_3');
linkaxes([sp1, sp2, sp3, sp4],'x');

figure()
sp1 = subplot(3,1,1);
    plot(measured.t,w_meas_inertial(1,:),'.'); grid on;
    title('Measured Ang rates (Inertial Frame)'); xlabel('time (s)'); ylabel('\omega_x(rad/s)');
sp2 = subplot(3,1,2);
    plot(measured.t,w_meas_inertial(2,:),'.'); grid on;
    xlabel('time (s)'); ylabel('\omega_y(rad/s)');
sp3 = subplot(3,1,3);
    plot(measured.t,w_meas_inertial(3,:),'.'); grid on;
    xlabel('time (s)'); ylabel('\omega_z(rad/s)');
linkaxes([sp1, sp2, sp3],'x');

%%%
%  %%Calculate the norm
%  % Número de quaternions
% q=[data.x_imu_orientation_w,data.x_imu_orientation_x,data.x_imu_orientation_y,data.x_imu_orientation_z];
% n = size(q, 1);
% % Inicializar un vector para almacenar las normas
% normas = zeros(n, 1);
% % Calcular la norma de cada quaternion
% for i = 1:n
%     normas(i) = norm(q(i, :));
% end
% 
% figure()
% plot(data.x__time-data.x__time(1),normas,'.')
% 
% %%
% %Simulation
% t=data.x__time-data.x__time(1);
% x=zeros(4,length(t)+1);
% x(:,1) = [data.x_imu_orientation_w(1);data.x_imu_orientation_x(1);data.x_imu_orientation_y(1);data.x_imu_orientation_z(1)];
% u=[data.x_imu_angular_velocity_x,data.x_imu_angular_velocity_y,data.x_imu_angular_velocity_z]';
% for i = 1:length(t)
%     %%Get step size
%     if i == 1
%         dt = t(i+1);
%     else
%         dt=t(i)-t(i-1);
%     end
% 
%     %Start RK 4 de cuarto orden
%     g1=dt*kinematicEq(t(i), x(:,i), u(:,i));
%     g2=dt*kinematicEq(t(i), x(:,i)+0.5.*g1, u(:,i));
%     g3=dt*kinematicEq(t(i), x(:,i)+0.5.*g2, u(:,i));
%     g4=dt*kinematicEq(t(i), x(:,i)+0.5.*g3, u(:,i));
%     x(:,i+1)=x(:,i)+(1/6).*(g1+2.*g2+2.*g3+g4);
% end
% 
% figure()
% plot(t,x(:,1:end-1));grid on; hold on;
% plot(t,[data.x_imu_orientation_w,data.x_imu_orientation_x,data.x_imu_orientation_y,data.x_imu_orientation_z]','.')
% 
% function q_dot = kinematicEq(t, q, w)
% %------------------------Ecuaciones cinematica------------------
% %Libro: Fundamentals of Spacecraft Attitude Determination and Control
% %Autor: F. Landis Markley & John L. Crassidis
% %Cinemática de cuaternos..Ecuación (2.88 modificada)
% %Cinemática de cuaternos..Ecuación (2.88)
% Xi=[-q(2),-q(3),-q(4);
%      q(1),-q(4),q(3);
%      q(4),q(1),-q(2);
%     -q(3),q(2),q(1)]; 
% q_dot=1/2*Xi*w;                   %Ecuacion cinematica (3.21)
% end
% 
function rotX = quatRotation(q,x)
     qx = [0,x(1),x(2),x(3)]';
     q = [q(1),q(2),q(3),q(4)]';
     qrotX = quatmultiply(quatmultiply(q', qx'), quatconj(q'));
     rotX = qrotX(2:4);
end

function [h1,h2] = plotVector(v0,v,scale,style,label,labelLocation)
    if labelLocation=='m'
        scale = scale*1.1;
        h1 = quiver3(v0(1), v0(2), v0(3), v(1)*scale, v(2)*scale, v(3)*scale, style);
        h2 = text(v0(1)+v(1)*scale/2,v0(2)+v(2)*scale/2, v0(3)+v(3)*scale/2, label,'Color',style(1));
    else
        h1 = quiver3(v0(1)*scale, v0(2)*scale, v0(3)*scale, v(1)*scale, v(2)*scale, v(3)*scale, style);
        h2 = text((v0(1)+v(1))*scale, (v0(2)+v(2))*scale, (v0(3)+v(3))*scale, label,'Color',style(1));
    end
end

function [ t_uniform, x_uniform] = setSampleTime(tout,data,t_sample)
    %%% Interpolation to have uniform sampling
    %temp = diff(tout);               %%Temporal time
    %t_sample = mean(temp(1:end-1));                             %%Get sample time
    t_uniform = tout(1):t_sample:tout(end); %%Define a uniform time vector
    %%% Start interpolation
    x_uniform = interp1(tout, data, t_uniform, 'spline');  % Interpolación
end

% % % % Suavizar la señal usando un filtro de media móvil
% % %  windowSize_x = 5; % Tamaño de la ventana de media móvil
% % %  windowSize_y = 5; % Tamaño de la ventana de media móvil
% % %  windowSize_z = 100; % Tamaño de la ventana de media móvil
% % %  w_x_smooth = movmean(data.x_imu_angular_velocity_x, windowSize_x);
% % %  w_y_smooth = movmean(data.x_imu_angular_velocity_y, windowSize_y);
% % %  w_z_smooth = movmean(data.x_imu_angular_velocity_z, windowSize_z);
% % % 
% % % get omega_dot
% % % wdot_x=diff(w_x_smooth)/0.05;
% % % wdot_y=diff(w_y_smooth)/0.05;
% % % wdot_z=diff(w_z_smooth)/0.05;
% % % 
% % % %%Filter
% % %  w_dot_x_smooth = movmean(wdot_x, 10);
% % %  w_dot_y_smooth = movmean(wdot_y, 10);
% % %  w_dot_z_smooth = movmean(wdot_z, 10);