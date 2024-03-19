function [angArray,xArray,uiArray,TdArray,EULERINT,ASCCT,T,ts] = Feedback_simulation(Td,P,K,n,t,I,qd_Array,wd_Array,ui,x)
%LS2125204: Brayan Espinoza
%Initial variables    
EULERINT=0; ASCCT=0; ts=0; 
wd_ant=0; ascct_ant=0; ascct_dot_ant=0;
euler_ant=0; euint_ant=0;
angArray=quat2eul(x(1:4)')';
xArray=x; uiArray=ui;
TdArray=zeros(3,1);
ok=0;   %Flag
    
for i=1:n-2
    dt=t(2)-t(1);                   % Calculate step
    qd=qd_Array(:,i);               % Reference point
    Wd=wd_Array(:,i);               % Reference velocity
    Wd_dot=(Wd-wd_ant)/dt;          % Derivative of wd
    Tdi=Td(:,i);                    % Disturbance torque    
        
    %fourth order RK 4 
    g1=dt*cubeSatEquationState(Tdi,I,ui,x);
    g2=dt*cubeSatEquationState(Tdi,I,ui,x+0.5.*g1);
    g3=dt*cubeSatEquationState(Tdi,I,ui,x+0.5.*g2);
    g4=dt*cubeSatEquationState(Tdi,I,ui,x+0.5.*g3);
    x=x+(1/6).*(g1+2.*g2+2.*g3+g4);
    %RK 4 end
    
    % Calculate quaternion error
    dq=Error_quaternio(qd,x(1:4));
    
    % Caculate control law and stimate its computational cost
    tic
    ui=ControlFeedback_rw(I,x,dq,Wd,Wd_dot,P,K);
    T(i)=toc;
      
    % Calculate settlement time
    ang=quat2eul(x(1:4)')';
    
    if sum(abs(quat2eul(qd')'-ang) < abs(0.05*quat2eul(qd')'))==3 && ok==0 
                ts=i*dt;
                ok=1;
    elseif sum(abs(quat2eul(qd')'-ang) < abs(0.05*quat2eul(qd')'))~=3 && ok==1
                ok=0;
    end
    
    % EULERINT calculation
    dqq=eul2quat(quat2eul(qd')-ang');
    %dqq=eul2quat((pi/180*[10,20,30]-ang'));
    eulerang=2*acos(dqq(1));
    %Second order Simpson integration
    euint=euint_ant+dt/6*(euler_ant+2*(euler_ant+eulerang)+eulerang);
    %End of Second order Simpson integration
     
    %ASCCT Calculation
    ascct_dot=norm(ui)^2;
    %Second order Simpson integration
    ascct=ascct_ant+dt/6*(ascct_dot_ant+2*(ascct_dot_ant+ascct_dot)+ascct_dot);
    %End of Second order Simpson integration
    
    %Update values
    euler_ant=eulerang;
    euint_ant=euint;
    ascct_ant=ascct;
    ascct_dot_ant=ascct_dot;
    
    %Arrays
    ASCCT=[ASCCT,ascct];
    EULERINT=[EULERINT,euint];
    TdArray=[TdArray Tdi];
    angArray=[angArray ang];
    xArray=[xArray x];
    uiArray=[uiArray ui];
    wd_ant=Wd;    
end
end


