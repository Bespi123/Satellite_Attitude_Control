function sal = ley_adapt(x_mdotdot,x_err,x_errdot)
sigma=0.5;
lambda=6;
%lambda=100;
s=x_errdot+lambda*x_err;
v=x_mdotdot-2*lambda*x_errdot-lambda^2*x_err;
m_est_dot=-1*sigma*v*s;
sal=[v;
    m_est_dot];
end

