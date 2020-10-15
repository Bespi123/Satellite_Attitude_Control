function u = Controlador(entr)
I_est=[entr(1),entr(2),entr(3)]';
v=[entr(4),entr(5),entr(6)]';
u=I_est.*(v);
end

