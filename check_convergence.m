function [ flag ] = check_convergence(pop)
flag = 0;
N = length(pop);

for i = 1:(N/2)
    energy_values(i) = pop(i).obj(1);
end

% mean of best 50% population 
energy_mean_50p = mean(energy_values);
clear energy_values

for i = 1:(N/5)
    energy_values(i) = pop(i).obj(1);
end

% mean of best 20% population
energy_mean_20p = mean(energy_values);
clear energy_values

for i = 1:(N/10)
    energy_values(i) = pop(i).obj(1);
end

% mean of best 10% population
energy_mean_10p = mean(energy_values);
clear energy_values

% average of means... 
energy_avg = (1/3)*(energy_mean_50p+energy_mean_20p+energy_mean_10p);

c_e = 0;
c_p = 0;
c_h = 0;

if (abs(energy_avg - energy_mean_50p)< (0.01*energy_avg))&&(abs(energy_avg - energy_mean_20p)< (0.01*energy_avg))&&(abs(energy_avg - energy_mean_10p)< (0.01*energy_avg))
    c_e = 1;
end

if (c_e)
    flag = 1;
end

for i = 1:length(pop)
    if pop(i).nViol > 0
        flag = 0;
        break
    end
end

end

