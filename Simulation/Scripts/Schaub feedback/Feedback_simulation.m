function [angArray,xArray,uiArray,TdArray,L_dArray,EULERINT,ASCCT,T,ts] = Feedback_simulation(P,K,rc,n,t,Sat,J_tilde,J_est,Irw_par,Irw_per,qd_Array,wd_Array,ui,x,b,c,d,ws,alpha1,alpha2,alpha3,beta1,beta2,beta3)
    wd_ant=0;
    EULERINT=0;
    ASCCT=0;
    ascct_ant=0;
    ascct_dot_ant=0;
    euler_ant=0;
    euint_ant=0;
    angArray=quat2eul(x(1:4)')';
    xArray=x;
    uiArray=ui;
    TdArray=zeros(3,1);
    L_dArray=zeros(3,1);
    deltaD=[0,alpha2*sin(beta2),alpha3*cos(beta3);
            alpha1*cos(beta1),0,alpha3*sin(beta3);
            alpha1*sin(beta1),alpha2*cos(beta2),0];
    ok=0;
    ts=0;
    for i=1:n-2
        dt=t(2)-t(1);
        qd=qd_Array(:,i);                                 % tracking deseado
        Wd=wd_Array(:,i);                                 % tracking deseado
        Wd_dot=(Wd-wd_ant)/dt;                            % derivando wd
        %Torque por gradiente de gravedad
        Tgg=T_disturbances(rc,J_tilde,Irw_per,x(1:4));
        %Torque por fricción de ruedas de reacción
        L_d=b*x(8:10)+sign(x(8:10))*(c+d*exp(-(x(8:10)/ws)'*(x(8:10)/ws)));
        %L_d=[0,0,0]';
        %Disturbio de torques
        Td=Tgg-L_d;
        %Td=Td;
        %Empezamos en RK 4 de cuarto orden
        g1=dt*Equation_state_rw(Td,Irw_per,Irw_par,J_tilde,ui,x);
        g2=dt*Equation_state_rw(Td,Irw_per,Irw_par,J_tilde,ui,x+0.5.*g1);
        g3=dt*Equation_state_rw(Td,Irw_per,Irw_par,J_tilde,ui,x+0.5.*g2);
        g4=dt*Equation_state_rw(Td,Irw_per,Irw_par,J_tilde,ui,x+0.5.*g3);
        x=x+(1/6).*(g1+2.*g2+2.*g3+g4);
        %Final del RK 4
        dq=x(1:4)-qd(1:4); 
        dq1=Error_quaternio(qd,x(1:4));
        tic
        ui=ControlFeedback_rw(J_est,Irw_per,Irw_par,x,dq,Wd,Wd_dot,P,K);
        T(i)=toc;
        for j=1:3
            if (abs(ui(j))>Sat)
                ui(j)=sign(ui(j))*Sat;
            end
        end
        ui=eye(3)*(ui)+deltaD*(ui);
        ang=quat2eul(x(1:4)')';
        if sum(abs(quat2eul(qd')'-ang)< 0.05*quat2eul(qd')')==3 && ok==0
                ts=i*dt;
                ok=1;
        end 
        %Calculando EULERINT
        eulerang=2*acos(dq1(1));
        %Integración por metodo de Simpson de segundo orden
            euint=euint_ant+dt/6*(euler_ant+2*(euler_ant+eulerang)+eulerang);
        %Final de la integración por metodo de Simpson 
         %Calculando ASCCT
            ascct_dot=norm(ui)^2;
        %Integración por metodo de Simpson de segundo orden
            ascct=ascct_ant+dt/6*(ascct_dot_ant+2*(ascct_dot_ant+ascct_dot)+ascct_dot);
        %Final de la integración por metodo de Simpson
        %Actualizando valores
        euler_ant=eulerang;
        euint_ant=euint;
        ascct_ant=ascct;
        ascct_dot_ant=ascct_dot;
        %Armando Arrays
        ASCCT=[ASCCT,ascct];
        L_dArray=[L_dArray,L_d];
        EULERINT=[EULERINT,euint];
        TdArray=[TdArray Tgg];
        angArray=[angArray ang];
        xArray=[xArray x];
        uiArray=[uiArray ui];
        wd_ant=Wd;    
    end
end

