function [pop] = randomize_pop(opt, pop)
% Takes the members which violate constraints and replaces them with random
% members. This part of the population is generated uniformly.

nVar = opt.numVar;
type = opt.vartype;

lb = opt.lb;
ub = opt.ub;
popsize = length(pop);
% Mmbers are already sorted, so find the first constraint-violating member
% index
idx = 1;
for i = 1:popsize
    if pop(i).nViol > 0
        break
    end
    idx = idx + 1;
end

% Replace members from idx to end
for i = idx:popsize
    var(1:nVar) = lb(1:nVar) + rand(1, nVar) .* (ub(1:nVar)-lb(1:nVar));
%     var(1) = normrnd(-1.7501,0.07);
%     var(2) = normrnd(0.8817,0.06);
%     var(3) = normrnd(0.5618,0.1);
%     var(4) = normrnd(-0.7901,0.01);
%     var(5) = normrnd(-1.2286,0.05);
%     var(6) = normrnd(1.2069,0.1);
    % if desing variable is integer, round to the nearest integer
    for v = 1:nVar
        if( type(v) == 2)
            var(v) = round(var(v));
        end
    end
    
    % limit in the lower and upper bound
    var = varlimit(var, lb, ub);

    pop(i).var = var;
end

end

