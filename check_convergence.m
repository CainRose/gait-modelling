function [ flag ] = check_convergence(pop)
flag = 0;
N = length(pop);

% mean of best 10% population
energy_mean_10p = mean([pop(1:floor(N/10)).obj]);

% mean of best 50% population
energy_mean_50p = mean([pop(1:floor(N/2)).obj]);

if (abs(energy_mean_50p - energy_mean_10p) < abs(0.01*mean([energy_mean_50p, energy_mean_10p])))
    flag = 1;
end

for i = 1:length(pop)
    if pop(i).nViol > 0
        flag = 0;
        break
    end
end

end

