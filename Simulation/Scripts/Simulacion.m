clc
clear
close all;
% For a rigid body Satellite
% ********************Simulation control parameters**********************
    n=100000;    %numero de iteraciones
    Tf=50;     %tiempo final
%********************* Parametros del modelo*****************************
    J_tilde=[8.46,8.5,8.33,1.1,1.5,1.6]*10^(-3); %Tensor de inercia real
    J_est=[1,1,1,0,0,0];               %Tensor de inercia estimado
    Sat=1.343*10^(-2);                 %Limites de saturación
%********************* Parametros ruedas de reacción*********************  
    rc=(6371+100)*10^(3);               %Distancia desde centro de la tierra
    m_rw=0.025;                         %masa ruedas de reacción
    r_rw=0.043;                         %radio ruedas de reacción
    d_rw=0.015;                         %grosor ruedas de reacción
    b=4.83*10^(-6);                     %coeficiende de fricción viscosa
    c=0.8795*10^(-3);                   %Frincción de Coulumb
    d=0.9055*10^(-3)-c;                 %Starting torque
    ws=0.418;                           %Velocidad de Stribeck
      b=0;                     %coeficiende de fricción viscosa
      c=0;                   %Frincción de Coulumb
      d=0;                 %Starting torque
    Irw_per=(1/4)*m_rw*r_rw^2+(1/12)*m_rw*d_rw^2;
    Irw_par=(1/2)*m_rw*r_rw^2;
%*********************Parametros de desalineaciones**********************
    alpha1=3*pi/180;
    alpha2=-4*pi/180;
    alpha3=5*pi/180;
    beta1=10*pi/180;
    beta2=-50*pi/180;
    beta3=70*pi/180;
%********************* Vectores a priori ********************************
    t=linspace(0,Tf,n);                   %Vector de tiempo
    %qd_Array=[ones(n,1),zeros(n,3)]';      %Vector qd
    qd=eul2quat(pi/180*[10,20,30]);
    qd_Array=[qd(1)*(ones(n,1)),qd(2)*(ones(n,1)),qd(3)*(ones(n,1)),qd(4)*(ones(n,1))]';      %Vector qd
    wd_Array=zeros(3,n);                   %Vector wd
    %wd_Array=ones(3,n);                   %Vector wd
%*********************Definiendo condiciones iniciales******************* 
    ang=[0,0,0];
    %ang=[0,0,0];
    q=eul2quat(pi/180*ang)';
    w=pi/180*[0,0,30]';
    %w=pi/180*[0,0,0]';
    wrw=pi/180*[0,0,30]';
    %wrw=pi/180*[0,0,0]';
    ui=zeros(3,1);
    x=[q;w;wrw];
%************************************************************************
%**                 Quaternion Feedback Controller                     **
%************************************************************************    
%*************************Parametros de simulación***********************
    %Ganancias
    P=1*eye(3,3);
    K=1*eye(3,3);
    %P=1*eye(3,3);
    %K=0.5*eye(3,3);
    m_est=0.025;                       %masa ruedas de reacción estimada
    r_est=0.043;                       %radio ruedas de reacción estimada
    d_est=0.015;                       %grosor ruedas de reacción estimada
%************************************************************************
%**                   Boskovic Adaptive Controller                     **
%************************************************************************ 
    gamma=0.001;
    delta=0.01;
    k=1;
%************************************************************************
%**                   Dando Adaptive Controller                     **
%************************************************************************ 
    lambda=1;
    Kd=1;
    theta=[1,1,1,0,0,0]';
%**************************Perform simulation****************************
    [angArray1,xArray1,uiArray1,TdArray1,L_dArray1,EULERINT1,ASCCT1,T1,ts1] = Feedback_simulation(P,K,rc,n,t,Sat,J_tilde,J_est,Irw_par,Irw_per,qd_Array,wd_Array,ui,x,b,c,d,ws,alpha1,alpha2,alpha3,beta1,beta2,beta3);  
    [angArray2,xArray2,uiArray2,TdArray2,L_dArray2,EULERINT2,ASCCT2,T2,ts2] = Boskovic_simulation(gamma,delta,k,rc,n,t,Sat,J_tilde,Irw_par,Irw_per,qd_Array,wd_Array,ui,x,b,c,d,ws,alpha1,alpha2,alpha3,beta1,beta2,beta3);
    [angArray3,xArray3,uiArray3,TdArray3,L_dArray3,EULERINT3,ASCCT3,T3,ts3] = Dando_simulation(gamma,lambda,Kd,theta,rc,n,t,Sat,J_tilde,J_est,Irw_par,Irw_per,qd_Array,wd_Array,ui,x,b,c,d,ws,alpha1,alpha2,alpha3,beta1,beta2,beta3);
%********************************Figuras********************************
 figure(1)
 subplot(3,1,1);plot(t(1:n-1),180/pi*angArray1)
    grid on;
    xlabel('Time(s)');
    ylabel('Euler angles(deg)');
    title('Quaternion feedback controller')
    legend('Roll','Pitch','Yaw');
 subplot(3,1,2);plot(t(1:n-1),xArray1(5:7,:))
    grid on;
    xlabel('Time(s)');
    ylabel('Angular velocity error(rad/s)');
    legend('X','Y','Z')
subplot(3,1,3);plot(t(1:n-1),uiArray1)
    grid on;
    xlabel('Time(s)');
    ylabel('Control torque(Nm)');
    legend('X','Y','Z')
figure(2)
 subplot(3,1,1);plot(t(1:n-1),180/pi*angArray2)
    grid on;
    xlabel('Time(s)');
    ylabel('Euler angles(deg)');
    title('Boskovic adaptive controller')
    legend('Roll','Pitch','Yaw');
 subplot(3,1,2);plot(t(1:n-1),xArray2(5:7,:))
    grid on;
    xlabel('Time(s)');
    ylabel('Angular velocity error(rad/s)');
    legend('X','Y','Z')
 subplot(3,1,3);plot(t(1:n-1),uiArray2)
    grid on;
    xlabel('Time(s)');
    ylabel('Control torque(Nm)');
    legend('X','Y','Z')
figure(3)
 subplot(3,1,1);plot(t(1:n-1),180/pi*angArray3)
    grid on;
    xlabel('Time(s)');
    ylabel('Euler angles(deg)');
    title('Dando adaptive controller')
    legend('Roll','Pitch','Yaw');
 subplot(3,1,2);plot(t(1:n-1),xArray3(5:7,:))
    grid on;
    xlabel('Time(s)');
    ylabel('Angular velocity error(rad/s)');
    legend('X','Y','Z')
 subplot(3,1,3);plot(t(1:n-1),uiArray3)
    grid on;
    xlabel('Time(s)');
    ylabel('Control torque(Nm)');
    legend('X','Y','Z')

figure(4)
 subplot(3,1,1);plot(t(1:n-1),[TdArray1(1,:);TdArray2(1,:);TdArray3(1,:)])
    grid on
    xlabel('Time(s)');
    ylabel('X (Nm)');
    legend('Quaternion feedback controller','Boskovic adaptive controller','Dando adaptive controller');
    title('Gravity gradient torque simulated')
 subplot(3,1,2);plot(t(1:n-1),[TdArray1(2,:);TdArray2(2,:);TdArray3(2,:)])
    grid on
    xlabel('Time(s)');
    ylabel('Y (Nm)');
    %legend('Quaternion feedback controller','Boskovic adaptive controller','Dando adaptive controller');
 subplot(3,1,3);plot(t(1:n-1),[TdArray1(3,:);TdArray2(3,:);TdArray3(3,:)])
    grid on
    xlabel('Time(s)');
    ylabel('Z (Nm)');
    %legend('Quaternion feedback controller','Boskovic adaptive controller','Dando adaptive controller');
figure(5)
 subplot(3,1,1);plot(t(1:n-1),[L_dArray1(1,:);L_dArray2(1,:);L_dArray3(1,:)])
    grid on
    xlabel('Time(s)');
    ylabel('X (Nm)');
    legend('Quaternion feedback controller','Boskovic adaptive controller','Dando adaptive controller');
    title('Reaction wheel drag')
 subplot(3,1,2);plot(t(1:n-1),[L_dArray1(2,:);L_dArray2(2,:);L_dArray3(2,:)])
    grid on
    xlabel('Time(s)');
    ylabel('Y (Nm)');
    %legend('Quaternion feedback controller','Boskovic adaptive controller','Dando adaptive controller');
 subplot(3,1,3);plot(t(1:n-1),[L_dArray1(3,:);L_dArray2(3,:);L_dArray3(3,:)])
    grid on
    xlabel('Time(s)');
    ylabel('Z (Nm)');
figure(6)
 subplot(1,2,1); plot(t(1:n-1),[EULERINT1;EULERINT2;EULERINT3])
    grid on
    xlabel('Time(s)');
    ylabel('EULERINT (rad.s)');
    legend('Quaternion feedback controller','Boskovic adaptive controller','Dando adaptive controller');
    title('Control laws Performance')
 subplot(1,2,2); plot(t(1:n-1),(1/Tf)*[ASCCT1;ASCCT2;ASCCT3])
    grid on
    xlabel('Time(s)');
    ylabel('ASCCT (Nm)');
figure(7)
    subplot(3,1,1);plot(t(1:n-1),xArray1(8:10,:))
    grid on;
    xlabel('Time(s)');
    ylabel('Angular velocity (rad/s)');
    legend('X','Y','Z')
    title('Reaction wheels velocity (Quaternion feedback controller)')
    subplot(3,1,2);plot(t(1:n-1),xArray2(8:10,:))
    grid on;
    xlabel('Time(s)');
    ylabel('Angular velocity (rad/s)');
    legend('X','Y','Z')
    title('Reaction wheels velocity (Boskovic adaptive controller)')
    subplot(3,1,3);plot(t(1:n-1),xArray3(8:10,:))
    grid on;
    xlabel('Time(s)');
    ylabel('Angular velocity (rad/s)');
    legend('X','Y','Z')
    title('Reaction wheels velocity (Dando adaptive controller)')
    
disp('Costo computacional estimado (Quaternion feedback controller): ');disp(mean(T1));
disp('Costo computacional estimado (Boskovic adaptive controller): ');disp(mean(T2));
disp('Costo computacional estimado (Dando adaptive controller):');disp(mean(T3));
disp('Tiempo de asentamiento (Quaternion feedback controller): ');disp(ts1);
disp('Tiempo de asentamiento (Boskovic adaptive controller): ');disp(ts2);
disp('Tiempo de asentamiento (Dando adaptive controller):');disp(ts3);
disp('Error estacionario (Quaternion feedback controller): ');disp(180/pi*angArray1(:,end)-[10,20,30]');
disp('Error estacionario (Boskovic adaptive controller): ');disp(180/pi*angArray2(:,end)-[10,20,30]');
disp('Error estacionario (Dando adaptive controller):');disp(180/pi*angArray3(:,end)-[10,20,30]');



    