no_var = 3;  %number of variables
lb = [0 0 0]; % lower bound
up = [Inf Inf Inf]; % high bound
initial = [1 1 1];

%GA OPTIONS
%try
ga_opt = gaoptimset('Display','off','Generations',10,'PopulationSize',200, ...
    'InitialPopulation',initial,'PlotFcns',@gaplotbestf);
obj_fun = @(k)myObjectiveFunction(k);

[k,bestblk] = ga((obj_fun),no_var,[],[],[],[],lb,up,[],ga_opt);

disp(k);