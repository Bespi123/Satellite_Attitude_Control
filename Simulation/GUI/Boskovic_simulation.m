function [kArray,angArray,xArray,uiArray,TdArray,EULERINT,ASCCT,T,ts] = Boskovic_simulation(Td,gamma,delta,k,n,t,Umax,I,qd_Array,wd_Array,ui,x)
%LS2125204: Brayan Espinoza
%Initial variables
EULERINT=0;ASCCT=0;ts=0;
euler_ant=0;euint_ant=0; 
ascct_ant=0;ascct_dot_ant=0;
k_ant=k;k_dot_ant=0;
angArray=quat2eul(x(1:4)')';
xArray=x;uiArray=ui;  
kArray=k;
TdArray=zeros(3,1);
ok=0;
   
for i=1:n-2
    dt=t(2)-t(1);
    qd=qd_Array(:,i);                            % Desired tracking
    Wd=wd_Array(:,i);                            % Desired tracking
    Tdi=Td(:,i);                                 % Disturbance torque 
    
    %fourth order RK 4
    g1=dt*cubeSatEquationState(Tdi,I,ui,x);
    g2=dt*cubeSatEquationState(Tdi,I,ui,x+0.5.*g1);
    g3=dt*cubeSatEquationState(Tdi,I,ui,x+0.5.*g2);
    g4=dt*cubeSatEquationState(Tdi,i,ui,x+0.5.*g3);
    x=x+(1/6).*(g1+2.*g2+2.*g3+g4);
    %RK 4 end
    
    %Calculate quaternion error
    dq=Error_quaternio(qd,x(1:4));
    
    % Caculate control law and stimate its computational cost
    tic
    k_dot=Gain_estimator_bosk(x(5:7),Wd,dq,delta,gamma,k,Umax);
    %Second order Simpson integration
    k=k_ant+dt/6*(k_dot_ant+2*(k_dot_ant+k_dot)+k_dot);
    %End of Second order Simpson integration
    ui=Boskovic_control(x(5:7),Wd,dq,delta,k,Umax);
    T(i)=toc;
     
    % Calculate settlement time
    ang=quat2eul(x(1:4)')';
    if sum(abs(quat2eul(qd')'-ang) < abs(0.05*quat2eul(qd')'))==3 && ok==0 
                ts=i*dt;
                ok=1;
    elseif sum(abs(quat2eul(qd')'-ang) < abs(0.05*quat2eul(qd')'))~=3 && ok==1
                ok=0;
    end
    
    %EULERINT calculation    
    dqq=eul2quat(quat2eul(qd')-ang');    
    eulerang=2*acos(dqq(1));    
    %Second order Simpson integratio
    euint=euint_ant+dt/6*(euler_ant+2*(euler_ant+eulerang)+eulerang);
    %End of Second order Simpson integration
         
    %ASCCT Calculation
    ascct_dot=norm(ui)^2;
    %Second order Simpson integration
    ascct=ascct_ant+dt/6*(ascct_dot_ant+2*(ascct_dot_ant+ascct_dot)+ascct_dot);
    %End of Second order Simpson integration
    
    %Update values
    k_ant=k;
    k_dot_ant=k_dot;
    euler_ant=eulerang;
    euint_ant=euint;
    ascct_ant=ascct;
    ascct_dot_ant=ascct_dot;
    %Arrays
    ASCCT=[ASCCT ascct];
    kArray=[kArray,k];
    EULERINT=[EULERINT,euint];
    TdArray=[TdArray Tdi];
    angArray=[angArray ang];
    xArray=[xArray x];
    uiArray=[uiArray ui];
end
end