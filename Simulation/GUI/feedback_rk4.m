function [x, u, eulerInt, ascct, o, ts] = feedback_rk4(app,disturbances,simParameters,time)

%%%Program developed by: bespi123
%%Recover variables
t = time.t; n=time.n;
P = simParameters.feedback.Peye; K = simParameters.feedback.Peye;
qd = repmat(simParameters.setPoint.qd',n, 1)';    %%Desired attitude array
wd = repmat(simParameters.setPoint.Wd',n, 1)';    %%Desired angular rate array 
Td = disturbances; I = simParameters.initialValues.I;

%%%Simulation containers
x = zeros(7,n); u = zeros(3,n); o = zeros(1,n); dq = zeros(4,n);
%%%eulerInt = zeros(1,n); asscct = zeros(1,n); 
%%%Initial conditions
x(:,1)=[simParameters.initialValues.q0; simParameters.initialValues.Wo];

%%%Calculate Wd_dot
Wd_dot = diff(wd')'./diff(t);

% % % %% Create wait bar
% % %%%Progress bar settings
hWaitbar = waitbar(0, 'Progress: 0%','Name', 'Attitude simulation progress..');

for i=1:n-1
    % Calculate step
    dt=t(2)-t(1);                   
    
    %fourth order RK 4 
    g1=dt*cubeSatEquationState(Td(:,i), I, u(:,i), x(:,i));
    g2=dt*cubeSatEquationState(Td(:,i), I, u(:,i), x(:,i)+0.5.*g1);
    g3=dt*cubeSatEquationState(Td(:,i), I, u(:,i), x(:,i)+0.5.*g2);
    g4=dt*cubeSatEquationState(Td(:,i), I, u(:,i), x(:,i)+0.5.*g3);
    x(:,i+1)=x(:,i)+(1/6).*(g1+2.*g2+2.*g3+g4);
    %RK 4 end
    
    % Calculate quaternion error
    dq(:,i)=Error_quaternio(qd(:,i),x(1:4,i));
    
    % Caculate control law and stimate its computational cost
    tic
    u(:,i+1)=ControlFeedback_rw(I,x(:,i),dq(:,i),wd(:,i),Wd_dot(:,i),P,K);
    o(i)=toc;

    %% Progress bar configuration
    %%%% Calculate the progress percentage
    if (mod(i,round(n/10))==0)
    progress = i / (n-1);
    if isa(hWaitbar,'handle') && isvalid(hWaitbar)
    %%% Update the progress bar
        waitbar(progress, hWaitbar, sprintf('Progress: %.1f%%', progress*100));
    end
    end
    %%%%Plot simulation
    % if(i==1)
    %     plot(app.UIAxes,t(i),x(1:4,i),'r.');
    % end
end
%%%Close wait bar
close(hWaitbar);

%%%EULERINT calculation
ang_and_axis = quat2axang(dq'); 
eulerang = ang_and_axis(:,4);
eulerInt = cumtrapz(t,eulerang); 
%%%ASCCT Calculation
ascct_dot=vecnorm(u,2,1).^2;
ascct = cumtrapz(t,ascct_dot); 
%%%Settlement time calculation
tol = 5/100; % 2%
ts = calculateSettlementTime(180/pi*quat2eul(dq'), t, tol) ;
end