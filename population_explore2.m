clear all
close all
clc
%--------------------------------------------------------------------------
% load('results.mat');
data_robot = loadpopfile('populationsCollated.txt');
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
numObj = 1;
[maxGen, popsize] = size(data_robot.pops);

% keyboard(); 


% Data Extraction
objective_data = reshape([data_robot.pops.obj], [maxGen,popsize]);
%%
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Graphs showing objective function evolution - all members
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------

num = (1:maxGen)';

h3 = figure;
hold on
for i = 1:popsize
        objective_data_plot = objective_data(:,i);
        objective_data_plot = objective_data_plot(objective_data_plot<100);
        plot((1:length(objective_data_plot)), objective_data_plot, 'ro');
end
title('Power loss objective across generations, 1.0 m/s');
xlabel('Generation');
ylabel('Power Loss');
grid on;
grid minor;
hold off
saveas(h3, 'Power_Loss_Scatter.png');

%%
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Find best members based on product of obj. function values
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------

% Based on objective function, find num_best_mem best
% members overall.
% Take the mean and variance of the objective function values of these best
% members and store them

mean_obj_a = zeros(maxGen, 1);
mean_obj_b = zeros(maxGen, 1);
mean_obj_c = zeros(maxGen, 1);
mean_obj_d = zeros(maxGen, 1);

for i = 1:maxGen
    objective_data_sorted = sort([data_robot.pops(i,:).obj]);
    objective_data_sorted = objective_data_sorted(objective_data_sorted<1000);
    
    mean_obj_a(i) = mean(objective_data_sorted);
%     var_obj_a(i) = var(objective_data_sorted);
    
    mean_obj_b(i) = mean(objective_data_sorted(1:min(floor(popsize/2), end)));
%     var_obj_b(i) = var(objective_data_sorted(1:min(floor(popsize/2), end)));
    
    mean_obj_c(i) = mean(objective_data_sorted(1:min(floor(popsize/4), end)));
%     var_obj_c(i) = var(objective_data_sorted(1:min(floor(popsize/4), end)));
    
    mean_obj_d(i) = mean(objective_data_sorted(1:min(floor(popsize*0.1), end)));
%     var_obj_d(i) = var(objective_data_sorted(1:min(floor(popsize*0.1), end)));
end

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Graphs for various objective function averages based on best members
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
num = (1:maxGen)';

h1_e = figure;
hold on
plot(num, mean_obj_a, 'r', 'LineWidth', 1.5);
plot(num, mean_obj_b, 'b', 'LineWidth', 1.5);
plot(num, mean_obj_c, 'g', 'LineWidth', 1.5);
plot(num, mean_obj_d, 'k', 'LineWidth', 1.5);
title('Power loss objective across generations, 1.0 m/s', 'FontSize', 15);
legend('100% population mean', '50% population mean', '20% population mean', '10% population mean');
xlabel('Generation', 'FontSize', 15);
ylabel('Power Loss', 'FontSize', 15);
grid on;
grid minor;
hold off
saveas(h1_e, 'Power_Loss_Line.png');
