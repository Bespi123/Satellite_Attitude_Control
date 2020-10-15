%**************************************************************************
% AUTHOR: Brayan Espinoza 13/10/2020
% DESCRIPTION: This function represents a adaptation law to stimate the
% inertia in each axis.
%
% IMPORTANT: 
%
% *************************************************************************
function sal = ley_adapt(entr)
%sigma=0.5;
%lambda=6;
%lambda=100;
%++++++++++++++++++Parameters++++++++++++++++++++
sigma=entr(1);
lambda=entr(2);
ym_dotdot=[entr(3),entr(4),entr(5)]';
x_err=[entr(6),entr(7),entr(8)]';
x_errdot=[entr(9),entr(10),entr(11)]';

%+++++++++++++++++Calculus+++++++++++++++++++++++
s=x_errdot+lambda*x_err; %Sliding surface
v=ym_dotdot-2*lambda*x_errdot-lambda^2*x_err;
m_est_dot=-1*sigma*v.*s;
sal=[v;
    m_est_dot];
end

