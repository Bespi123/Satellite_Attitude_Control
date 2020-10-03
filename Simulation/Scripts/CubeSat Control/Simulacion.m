%**************************************************************************
% AUTHOR: Brayan Espinoza 1/10/2020
% DESCRIPTION: 
% This program perform the attitude simulation between three diferent 
% control laws. The CubeSat rigid body model takes into acount the reaction
% wheels inertia contributions to the Body inertia tensor. The inertial frame 
% and the reference frame in this simulation are considered equal and all
% the paramteters are expresed in the body frame. The script also models
% the gravity gradient torque and the disturbances produced by reaction
% wheels misaligments.
% IMPORTANT: 
% The CubeSat model doesn't takes into account the electric torque and
% the dinamic model for brushless motor used in reaction wheels. The rw
% friction is removed because this will be take into account in the
% brushless motor model that will be implemeted in the later versions.
% *************************************************************************

% Clear old variables and close all windows and graphs 
clc
clear
close all;

% Simulation control parameters
n=100000;            %Number of iterations
Tf=200;              %Simulation time
t=linspace(0,Tf,n);  %Vector time

%****************************MODEL PARAMETERS******************************
% CubeSat's rigid model parameters
J_tilde=[8.46,8.5,8.33,1.1,1.5,1.6]*10^(-3); %Real inertia tensor (Nm)
J_est=[1,1,1,0,0,0];                         %Estimated inertia tensor (Nm)
                                             %for all the controllers
                                             
% Reaction wheel's parameters
m_rw=0.025;                         %rw mass(kg)
r_rw=0.043;                         %rw radius(m)
d_rw=0.015;                         %rw thickness(m)
Sat=1.343*10^(-2);                  %Reaction wheel max inertia (Nm)
Irw_per=(1/4)*m_rw*r_rw^2+(1/12)*m_rw*d_rw^2; %rw perpendicular inertia
Irw_par=(1/2)*m_rw*r_rw^2;                    %rw paralell inertia

% Reactions wheel's friction parameters (Not used in this version)
b=4.83*10^(-6);                     %Viscous coefficient of friction 
c=0.8795*10^(-3);                   %Frincción de Coulumb
d=0.9055*10^(-3)-c;                 %Starting torque
ws=0.418;                           %Stribeck rate
      
% Reaction wheels misaligments parameters (rad)
alpha1=3*pi/180;            
alpha2=-4*pi/180;
alpha3=5*pi/180;
beta1=10*pi/180;
beta2=-50*pi/180;
beta3=70*pi/180;

% Gravity gradient parameters
rc=(6371+100)*10^(3);               %Distance from center of earth (m)
    
%****************************INITIAL CONDITIONS**************************** 
ang=[0,0,0];                %Initial euler angles (rad)
q=eul2quat(pi/180*ang)';    %Initial quaternion     
w=pi/180*[0,0,30]';         %Initial body rates (rad/s)    
wrw=pi/180*[0,0,30]';       %Initial rw rates (rad/s)
ui=zeros(3,1);              %Initial control torque (Nm)
x=[q;w;wrw];                %State Space Vector
    
%*************************CONTROL REFERENCES******************************* 
%qd_Array=[ones(n,1),zeros(n,3)]';             %Desired attitude
qd=eul2quat(pi/180*[10,20,30]);
wd_Array=zeros(3,n);                           %Desired angular rate array
qd_Array=[qd(1)*(ones(n,1)),qd(2)*(ones(n,1)),...
            qd(3)*(ones(n,1)),qd(4)*(ones(n,1))]'; %Desired attitude array      

%*************************CONTROL PARAMETERS*******************************
% Quaternion Feedback Controller
P=1*eye(3,3);           %Positive definite matrix
K=1*eye(3,3);           %Positive definite matrix
%P=1*eye(3,3);
%K=0.5*eye(3,3);
m_est=0.025;            %RW estimated mass (kg)
r_est=0.043;            %RW estimated radious (m)
d_est=0.015;            %RW estimated thickness (m)   

% Boskovic Robust Controller
gamma=0.001;            %Gain
delta=0.01;             %Gain
k=1;                    %Initial condition for the adaptive gain

% Dando Adaptive Controller
lambda=1;               %Learning rate
Kd=1;                   %Gain
theta=[1,1,1,0,0,0]';   %Initial condition for the estimated inertia error

% Chen Robust Controller (Difficult to tune)
falpha1=1;
falpha2=0.5;
fko=0.0005;
ftao1=50;                   %Positive
ftao2=10;                   %Positive
frho=0.7;                   
fzeta_o=0.0001;             %Positive
%fk1=3;                     %Positive k1-1/2*k3^2-1/2>0
fk1=2;
fk2=1;                      %Positive
%fk3=2;                     %Positive                           
fk3=0.3;
fk4=1;                      %Positive
fa=0.5;                     %Must be <1 
fgamma1=0.7;                %Must be 0<gamma1<1
fn=0.0004;                  %Must be <1
fgamma=0.6;                 %Must be >0                        
fp=[0.1,0.1,0.1];           
fx=[0.001,0.001,0.001];

%**************************PERFORM SIMULATIONS****************************
[angArray1,xArray1,uiArray1,TdArray1,L_dArray1,EULERINT1,ASCCT1,T1,ts1] = ...
    Feedback_simulation(P,K,rc,n,t,Sat,J_tilde,J_est,Irw_par,Irw_per,qd_Array,...
    wd_Array,ui,x,b,c,d,ws,alpha1,alpha2,alpha3,beta1,beta2,beta3);  
[angArray2,xArray2,uiArray2,TdArray2,L_dArray2,EULERINT2,ASCCT2,T2,ts2] = ...
    Boskovic_simulation(gamma,delta,k,rc,n,t,Sat,J_tilde,Irw_par,Irw_per,qd_Array,...
    wd_Array,ui,x,b,c,d,ws,alpha1,alpha2,alpha3,beta1,beta2,beta3);
[angArray3,xArray3,uiArray3,TdArray3,L_dArray3,EULERINT3,ASCCT3,T3,ts3] = ...
    Dando_simulation(gamma,lambda,Kd,theta,rc,n,t,Sat,J_tilde,J_est,Irw_par,Irw_per,...
    qd_Array,wd_Array,ui,x,b,c,d,ws,alpha1,alpha2,alpha3,beta1,beta2,beta3);
[angArray4,xArray4,uiArray4,TdArray4,L_dArray4,EULERINT4,ASCCT4,T4,ts4] = ...
    Cheng_simulation(falpha1,falpha2,fko,ftao1,ftao2,frho,fzeta_o,fk1,fk2,fk3,fk4,fa,...
    fgamma1,fn,fgamma,fp,fx,rc,n,t,Sat,J_tilde,J_est,Irw_par,Irw_per,qd_Array,wd_Array,...
    ui,x,b,c,d,ws,alpha1,alpha2,alpha3,beta1,beta2,beta3);

%********************************Figures***********************************
figure(1)
subplot(3,1,1);plot(t(1:n-1),180/pi*angArray1)
    grid on; xlabel('Time(s)');
    ylabel('Euler Angles(deg)');
    legend('Roll','Pitch','Yaw');
    title('Quaternion Feedback Controller');
subplot(3,1,2);plot(t(1:n-1),xArray1(5:7,:))
    grid on; xlabel('Time(s)');
    legend('X','Y','Z');
    ylabel('Angular Rates Error(rad/s)');
subplot(3,1,3);plot(t(1:n-1),uiArray1)
    grid on; xlabel('Time(s)');
    legend('X','Y','Z');
    ylabel('Control Torque(Nm)');  
    
figure(2)
 subplot(3,1,1);plot(t(1:n-1),180/pi*angArray2)
    grid on;
    xlabel('Time(s)');
    ylabel('Euler Angles(deg)');
    title('Boskovic Robust Controller')
    legend('Roll','Pitch','Yaw');
 subplot(3,1,2);plot(t(1:n-1),xArray2(5:7,:))
    grid on;
    xlabel('Time(s)');
    ylabel('Angular Rates Error(rad/s)');
    legend('X','Y','Z')
 subplot(3,1,3);plot(t(1:n-1),uiArray2)
    grid on;
    xlabel('Time(s)');
    ylabel('Control Torque(Nm)');
    legend('X','Y','Z')

figure(3)
subplot(3,1,1);plot(t(1:n-1),180/pi*angArray3)
    grid on; xlabel('Time(s)');
    ylabel('Euler Angles(deg)');
    title('Dando Adaptive Controller')
    legend('Roll','Pitch','Yaw');
subplot(3,1,2);plot(t(1:n-1),xArray3(5:7,:))
    grid on; xlabel('Time(s)');
    ylabel('Angular Rates Error(rad/s)');
    legend('X','Y','Z')
subplot(3,1,3);plot(t(1:n-1),uiArray3)
    grid on; xlabel('Time(s)');
    ylabel('Control Torque(Nm)');
    legend('X','Y','Z')

figure(4)
subplot(3,1,1);plot(t(1:n-1),180/pi*angArray4)
    grid on; xlabel('Time(s)');
    ylabel('Euler Angles(deg)');
    title('Chen Robust Controller')
    legend('Roll','Pitch','Yaw');
subplot(3,1,2);plot(t(1:n-1),xArray4(5:7,:))
    grid on; xlabel('Time(s)');
    ylabel('Angular Rates Error(rad/s)');
    legend('X','Y','Z')
subplot(3,1,3);plot(t(1:n-1),uiArray4)
    grid on; xlabel('Time(s)');
    ylabel('Control Torque(Nm)');
    legend('X','Y','Z')

 figure(5)
 subplot(3,1,1);plot(t(1:n-2),[TdArray1(1,2:end);TdArray2(1,2:end);...
     TdArray3(1,2:end);TdArray4(1,2:end)])
    grid on; xlabel('Time(s)');
    ylabel('X (Nm)');
    legend('Quaternion Feedback Controller','Boskovic Robust Controller',...
        'Dando Adaptive Controller','Chen Robust Controller');
    title('Gravity gradient torque simulated')
 subplot(3,1,2);plot(t(1:n-2),[TdArray1(2,2:end);TdArray2(2,2:end);...
     TdArray3(2,2:end);TdArray4(2,2:end)])
    grid on; xlabel('Time(s)'); ylabel('Y (Nm)');
 subplot(3,1,3);plot(t(1:n-2),[TdArray1(3,2:end);TdArray2(3,2:end);...
     TdArray3(3,2:end);TdArray4(3,2:end)])
    grid on; xlabel('Time(s)'); ylabel('Z (Nm)');

figure(6)
 subplot(3,1,1);plot(t(1:n-2),[L_dArray1(1,2:end);L_dArray2(1,2:end);...
     L_dArray3(1,2:end);L_dArray4(1,2:end)])
    grid on; xlabel('Time(s)'); ylabel('X (Nm)'); title('Reaction wheel drag');
    legend('Quaternion Feedback Controller','Boskovic Robust Controller',...
        'Dando Adaptive Controller','Chen Robust Controller');
 subplot(3,1,2);plot(t(1:n-2),[L_dArray1(2,2:end);L_dArray2(2,2:end);...
     L_dArray3(2,2:end);L_dArray4(1,2:end)])
    grid on; xlabel('Time(s)'); ylabel('Y (Nm)');
 subplot(3,1,3);plot(t(1:n-2),[L_dArray1(3,2:end);L_dArray2(3,2:end);...
     L_dArray3(3,2:end);L_dArray4(3,2:end)])
    grid on; xlabel('Time(s)'); ylabel('Z (Nm)');
    
figure(7)
subplot(1,2,1); plot(t(1:n-1),[EULERINT1;EULERINT2;EULERINT3;EULERINT4])
    grid on; xlabel('Time(s)'); ylabel('EULERINT (rad.s)');
    legend('Quaternion Feedback Controller','Boskovic Robust Controller',...
        'Dando Adaptive Controller','Chen Robust Controller');
    title('Control laws Performance')
 subplot(1,2,2); plot(t(1:n-1),(1/Tf)*[ASCCT1;ASCCT2;ASCCT3;ASCCT4])
    grid on; xlabel('Time(s)'); ylabel('ASCCT (Nm)');

figure(8)
subplot(4,1,1);plot(t(1:n-1),xArray1(8:10,:))
    grid on; xlabel('Time(s)'); ylabel('Angular Rate (rad/s)');
    legend('X','Y','Z');
    title('Reaction wheels velocity (Quaternion Feedback Controller)')
subplot(4,1,2);plot(t(1:n-1),xArray2(8:10,:))
    grid on; xlabel('Time(s)'); ylabel('Angular Rate (rad/s)');
    legend('X','Y','Z');
    title('Reaction wheels velocity (Boskovic Robust Controller)')
subplot(4,1,3);plot(t(1:n-1),xArray3(8:10,:))
    grid on; xlabel('Time(s)'); ylabel('Angular Rate (rad/s)');
    legend('X','Y','Z')
    title('Reaction wheels velocity (Dando Adaptive Controller)')
subplot(4,1,4);plot(t(1:n-1),xArray4(8:10,:))
    grid on; xlabel('Time(s)'); ylabel('Angular Rate (rad/s)');
    legend('X','Y','Z');
    title('Reaction wheels velocity (Chen Robust Controller)')
     
%*******************CONTROL FEATURES**************************************
disp('Estimated Computational Cost: ')
    disp('Quaternion Feedback Controller: ');disp(mean(T1));
    disp('Boskovic Robust Controller: ');disp(mean(T2));
    disp('Dando Adaptive Controller:');disp(mean(T3));
    disp('Cheng adaptive controller:');disp(mean(T4));

disp('Settlement time: ')
    disp('Quaternion Feedback Controller: ');disp(ts1);
    disp('Boskovic Robust Controller: ');disp(ts2);
    disp('Dando Adaptive Controller:');disp(ts3);
    disp('Chen Adaptive Controller:');disp(ts4);

disp('Stady-state error: ')
    disp('Quaternion Feedback Controller: ');disp(180/pi*angArray1(:,end)-[10,20,30]');
    disp('Boskovic Robust Controller: ');disp(180/pi*angArray2(:,end)-[10,20,30]');
    disp('Dando Adaptive Controller:');disp(180/pi*angArray3(:,end)-[10,20,30]');
    disp('Chen Robust Controller:');disp(180/pi*angArray4(:,end)-[10,20,30]');
   


    