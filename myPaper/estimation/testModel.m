close all

%%Simulation
dt = 0.05;
t = 0:dt:1000;
n = length(t);

%%%containers
x = zeros(7,length(t)+1);
x(:,1) = [1,0,0,0,0,0,0]';
t_air_body = zeros(3,length(t)+1);
t_air_friction = zeros(3,length(t)+1);
t_gg_body = zeros(3,length(t)+1);

%%Simulation parameters
K = [3.084E-3, 3.132E-3, 3.540E-3,...
    0,0,0,...
    0,0,0,...
    0, 0];

for i = 1:length(t)    
    q = x(1:4,i);
    w = x(5:7,i);
    %%%Gravity center offset (body Frame)
    cg= K(7:9)';
    %%%Satellite mass
    m_sat = 1.244;
    %%%Unmodeled friction coef (Inetial Frame)
    alpha = K(10);
    A = K(11);
    
    %%%AirBearing undesired Torque
    t_air_inertial = [0,0,A]'; %%% Inertial frame
    t_air_body(:,i) = quatRotation(quatconj(q'),t_air_inertial);

    %%%AirBearing friction Torque
    %coef= [0,0,aplha];  %%%Inertial Frame
    %diag(quatRotation(quatconj(q'),coef))
    t_air_friction(:,i) = -alpha*w;
    
    %%%Gravity Gradient Torque
    t_gg_body(:,i) = T_disturbances(m_sat,cg,q);  %%Body Frame
                             
    %%%Total torque 
    Td_body = t_gg_body(:,i)+t_air_friction(:,i)+t_air_body(:,i);
    
    %Start RK 4 de cuarto orden
    g1=dt*testBed_model(t(i), Td_body, x(:,i), K);
    g2=dt*testBed_model(t(i), Td_body, x(:,i)+0.5.*g1, K);
    g3=dt*testBed_model(t(i), Td_body, x(:,i)+0.5.*g2, K);
    g4=dt*testBed_model(t(i), Td_body, x(:,i)+0.5.*g3, K);
    x(:,i+1)=x(:,i)+(1/6).*(g1+2.*g2+2.*g3+g4);
    if(isnan(x(:,i+1)))
        disp('Unstable system')
        break;
    end
end

figure()
subplot(2,1,1)
plot(t,x(1:4,1:end-1));
subplot(2,1,2)
plot(t,x(5:7,1:end-1));

figure()
subplot(3,1,1)
plot(t,t_air_body(:,1:end-1));
subplot(3,1,2)
plot(t,t_air_friction(:,1:end-1));
subplot(3,1,3)
plot(t,t_gg_body(:,1:end-1));

function [ Tgg ] = T_disturbances(Ms,R_cm,q)
    g_inertial     = [0,0,-9.81]';
    g_body         = quatRotation(quatconj(q'),g_inertial);
    Tgg = cross(R_cm,Ms*g_body);
end

function rotX = quatRotation(q,x)
    qx = [0,    x(1), x(2), x(3)];
    q  = [q(1), q(2), q(3), q(4)];
    qrotX = quatmultiply(quatmultiply(q, qx), quatconj(q));
    rotX = [qrotX(2);qrotX(3);qrotX(4)];
end
