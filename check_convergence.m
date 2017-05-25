function [ flag ] = check_convergence(pop)
flag = 0;
N = length(pop);

% mean of best 10% population
energy_mean_10p = mean([pop(1:floor(N/10)).obj]);

% average of population... 
energy_avg = mean([pop.obj]);

if (abs(energy_avg - energy_mean_10p) < (0.01*min(energy_avg, energy_mean_10p)))
    flag = 1;
end

for i = 1:length(pop)
    if pop(i).nViol > 0
        flag = 0;
        break
    end
end

end

