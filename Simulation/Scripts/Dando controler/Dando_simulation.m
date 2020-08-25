function [angArray,xArray,uiArray,TdArray,L_dArray,EULERINT,ASCCT,T,ts] = Dando_simulation(gamma,lambda,Kd,theta,rc,n,t,Umax,J_tilde,J_est,Irw_par,Irw_per,qd_Array,wd_Array,ui,x,b,c,d,ws,alpha1,alpha2,alpha3,beta1,beta2,beta3)
    angArray=quat2eul(x(1:4)')';
    EULERINT=0;
    ASCCT=0;
    ascct_ant=0;
    ascct_dot_ant=0;
    theta_dot_ant=theta;
    theta_ant=theta;
    euler_ant=0;
    euint_ant=0;
    dq_ant=0;
    Wd_ant=0;
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
        Wd_dot=(Wd-Wd_ant)/dt;
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
        dq=Error_quaternio(qd,x(1:4));
        dq_dot=(dq-dq_ant)/dt;
        tic
        theta_dot=Ley_adaptacion_dando(Wd_dot,Wd,x(5:7),dq,dq_dot,lambda,gamma);
        %Integración por metodo de Simpson de segundo orden
            theta= theta_ant + dt/6*(theta_dot_ant+2*(theta_dot_ant+theta_dot)+theta_dot);
        %Final de la integración por metodo de Simpson 
            %theta=dt/2*theta_dot+dt/2*theta_dot_ant+theta_ant;
        ui=Dando_controler(Wd_dot,Wd,x(5:7),dq,dq_dot,J_est,theta,lambda,Kd);
        T(i)=toc;
        ui=ui*-1;
        for j=1:3
            if (abs(ui(j))>Umax)
                ui(j)=sign(ui(j))*Umax;
            end
        end       
        ui=eye(3)*(ui)+deltaD*(ui);
        ang=quat2eul(x(1:4)')';
        if sum(abs(quat2eul(qd')'-ang)< 0.05*quat2eul(qd')')==3 && ok==0
                ts=i*dt;
                ok=1;
        end 
        %Calculando EULERINT
        eulerang=2*acos(dq(1));
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
        ascct_dot_ant=ascct_dot;
        ascct_ant=ascct;
        theta_dot_ant=theta_dot;
        theta_ant=theta;
        dq_ant=dq;
        Wd_ant=Wd;
        %Armando Arrays
        ASCCT=[ASCCT,ascct];
        EULERINT=[EULERINT,euint];
        angArray=[angArray ang];
        xArray=[xArray x];
        uiArray=[uiArray ui];
        TdArray=[TdArray Tgg];
        L_dArray=[L_dArray,L_d];
    end
end