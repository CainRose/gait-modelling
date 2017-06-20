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

if popsize-idx+1 > 0
    fprintf('Randomizing %d Members\n', popsize-idx+1);
    % Generate Latin Hypercube Sample of design space
    newVars = lhsdesign(popsize-idx+1, nVar);
    newVars = newVars .* repmat(ub - lb, [popsize-idx+1,1]);
    newVars = newVars + repmat(lb, [popsize-idx+1,1]);

    % Replace members from idx to end
    for i = idx:popsize
        var(1:nVar) = newVars(i-idx+1,:);
        % if desing variable is integer, round to the nearest integer
        for v = 1:nVar
            if( type(v) == 2)
                var(v) = round(var(v));
            end
        end
        pop(i).var = var;
    end
end

end

