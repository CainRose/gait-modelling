clear all
close all
clc
%--------------------------------------------------------------------------
data_robot = loadpopfile('23052017populations.txt');
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
numObj = 1;
[maxGen, popsize] = size(data_robot.pops);

mean_obj_t = zeros(maxGen, numObj); % Objective mean at each generation
pro_obj_t = zeros(maxGen, popsize); % Objective product at each generation for all members
var_obj = zeros(maxGen, numObj);    % Objective variance at each generation

% keyboard(); 
best_num = 10;

objective_data = reshape([data_robot.pops.obj], [maxGen,popsize]);

for i = 1:maxGen
    [objective_data_sorted, objective_data_index] = sort([data_robot.pops(i,:).obj]);
    min_obj(i,:) = objective_data_sorted(1:best_num);
    min_obj_index(i,:) = objective_data_index(1:best_num);
    for j = 1:best_num
        stride_length(i,j) = data_robot.pops(i, min_obj_index(i,j)).var(1);
        phase_FR(i,j) = data_robot.pops(i, min_obj_index(i,j)).var(2);
        phase_HL(i,j) = data_robot.pops(i, min_obj_index(i,j)).var(3);
        phase_HR(i,j) = data_robot.pops(i, min_obj_index(i,j)).var(4);
        beta_FL(i,j) = data_robot.pops(i, min_obj_index(i,j)).var(5);
        beta_FR(i,j) = data_robot.pops(i, min_obj_index(i,j)).var(28);
        beta_HL(i,j) = data_robot.pops(i, min_obj_index(i,j)).var(51);
        beta_HR(i,j) = data_robot.pops(i, min_obj_index(i,j)).var(74);
%         legf_j1(i,j) = (180/pi).*data_robot((popsize*(i-1))+min_obj_index(i,j), 1);
%         legf_j2(i,j) = (180/pi).*data_robot((popsize*(i-1))+min_obj_index(i,j), 2);
%         legf_j3(i,j) = (180/pi).*data_robot((popsize*(i-1))+min_obj_index(i,j), 3);
%         legb_j1(i,j) = (180/pi).*data_robot((popsize*(i-1))+min_obj_index(i,j), 4);
%         legb_j2(i,j) = (180/pi).*data_robot((popsize*(i-1))+min_obj_index(i,j), 5);
%         legb_j3(i,j) = (180/pi).*data_robot((popsize*(i-1))+min_obj_index(i,j), 6);
%         gaits_duty_factor(i,j) = data_robot((popsize*(i-1))+min_obj_index(i,j), 16);
%         gaits_relat_phase(i,j) = data_robot((popsize*(i-1))+min_obj_index(i,j), 17);
%         stride_lengths(i,j) = data_robot((popsize*(i-1))+min_obj_index(i,j), 7);
%         step_lengths(i,j) = stride_lengths(i,j)*gaits_duty_factor(i,j);
%         ftakeoffangle_toe(i,j) = (180/pi).*data_robot((popsize*(i-1))+min_obj_index(i,j), 8);
%         fattackangle_toe(i,j) =(180/pi).* data_robot((popsize*(i-1))+min_obj_index(i,j), 9);
%         ftakeoffangle_hip(i,j) = (180/pi).*data_robot((popsize*(i-1))+min_obj_index(i,j), 10);
%         fattackangle_hip(i,j) = (180/pi).*data_robot((popsize*(i-1))+min_obj_index(i,j), 11);
%         btakeoffangle_toe(i,j) = (180/pi).*data_robot((popsize*(i-1))+min_obj_index(i,j), 12);
%         battackangle_toe(i,j) = (180/pi).*data_robot((popsize*(i-1))+min_obj_index(i,j), 13);
%         btakeoffangle_hip(i,j) = (180/pi).*data_robot((popsize*(i-1))+min_obj_index(i,j), 14);
%         battackangle_hip(i,j) = (180/pi).*data_robot((popsize*(i-1))+min_obj_index(i,j), 15);
    end
end
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%Plot showing best member gaits
h2 = figure
hold on
for k = 1:best_num
    scatter((1:(maxGen))', beta_FL(:,k), 'r');
    scatter((1:(maxGen))', beta_FR(:,k), 'g');
    scatter((1:(maxGen))', beta_HL(:,k), 'b');
    scatter((1:(maxGen))', beta_HR(:,k), 'm');
end
legend('Front Left', 'Front Right', 'Hind Left', 'Hind Right', 'Location', 'Best');
title('Evolution of Duty Factors of best members, 0.1 m/s');
xlabel('Generation');
ylabel('Duty Factor');
grid on;
hold off
%saveas(h2, 'Best_Duty_factors_20131224speed1.png');
%--------------------------------------------------------------------------
h3 = figure
hold on
for k = 1:best_num
    scatter((1:(maxGen))', phase_FR(:,k), 'g');
    scatter((1:(maxGen))', phase_HL(:,k), 'b');
    scatter((1:(maxGen))', phase_HR(:,k), 'm');
end
legend('Front Right', 'Hind Left', 'Hind Right', 'Location', 'Best');
title('Evolution of phase of best members, 0.1 m/s');
xlabel('Generation');
ylabel('Phase');
grid on;
hold off
%--------------------------------------------------------------------------
h3 = figure
hold on
for k = 1:best_num
    scatter((1:(maxGen))', stride_length(:,k), 'k');
end
title('Evolution of stride length of best members, 0.1 m/s');
xlabel('Generation');
ylabel('Stride Length');
grid on;
hold off
% %--------------------------------------------------------------------------
% h3 = figure
% hold on
% for k = 1:best_num
%     plot((1:(maxGen))', gaits_relat_phase(:,k), 'k*', 'MarkerSize', 5);
% end
% legend('Relative Leg Phase (Rear-Hind leg)');
% title('Evolution of Relative Leg Phases of best member, 0.1 m/s');
% xlabel('Generation');
% ylabel('Relative Leg Phase');
% grid on;
% grid minor;
% hold off
% saveas(h3, 'Best_Relative_Leg_phase_20131224speed1.png');
% %--------------------------------------------------------------------------
% h4 = figure
% hold on
% for k = 1:best_num
%     plot((1:(maxGen))', step_lengths(:,k), 'k*', 'MarkerSize', 5);
% end
% legend('stride lengths');
% title('Evolution of Step Lengths of best member, 0.1 m/s');
% xlabel('Generation');
% ylabel('Step Lengths (m)');
% grid on;
% grid minor;
% hold off
% saveas(h4, 'Best_Step_lengths_20131224speed1.png');
% %--------------------------------------------------------------------------
% 
% h5 = figure
% hold on
% for k = 1:best_num
%     plot((1:(maxGen))', ftakeoffangle_toe(:,k), 'bo', (1:(maxGen))', fattackangle_toe(:,k), 'ro', 'MarkerSize', 5);
% %     plot((1:(maxGen))', btakeoffangle_toe(:,k), 'ko', 'MarkerSize', 10);
% %     plot((1:(maxGen))', battackangle_toe(:,k), 'go', 'MarkerSize', 10);
% end
% legend('take off angle for toe', 'attack angle for toe');
% title('Evolution of Take off and attack angles of the toe, of best member, 0.1 m/s');
% xlabel('Generation');
% ylabel('Take off and attack angles of the toe (deg)');
% grid on;
% grid minor;
% hold off
% saveas(h5, 'Best_Takeoff_and_attack_angles_toe_20131224speed1.png');
% %--------------------------------------------------------------------------
% 
% h6 = figure
% hold on
% for k = 1:best_num
%     plot((1:(maxGen))', ftakeoffangle_hip(:,k), 'bo', (1:(maxGen))', fattackangle_hip(:,k), 'ro', 'MarkerSize', 5);
% %     plot((1:(maxGen))', btakeoffangle_hip(:,k), 'ko', 'MarkerSize', 10);
% %     plot((1:(maxGen))', battackangle_hip(:,k), 'go', 'MarkerSize', 10);
% end
% legend('take off angle for hip', 'attack angle for hip');
% title('Evolution of Take off and attack angles of the hip, of best member, 0.1 m/s');
% xlabel('Generation');
% ylabel('Take off and attack angles of the hip (deg)');
% grid on;
% grid minor;
% hold off
% saveas(h6, 'Best_Takeoff_and_attack_angles_hip_20131224speed1.png');
% %--------------------------------------------------------------------------
% 
% h7 = figure
% hold on
% for k = 1:best_num
%     plot((1:(maxGen))', legf_j1(:,k), 'bo', (1:(maxGen))', legf_j2(:,k), 'ro', (1:(maxGen))', legf_j3(:,k), 'ko', 'MarkerSize', 5);
% %     plot((1:(maxGen))', btakeoffangle_hip(:,k), 'ko', 'MarkerSize', 10);
% %     plot((1:(maxGen))', battackangle_hip(:,k), 'go', 'MarkerSize', 10);
% end
% legend('Joint 1 angle', 'Joint 2 angle', 'Joint 3 angle');
% title('Evolution of Joint angles of the front legs, of the best member, 0.1 m/s');
% xlabel('Generation');
% ylabel('Joint angles (deg)');
% grid on;
% grid minor;
% hold off
% saveas(h7, 'Best_joint_angles_front_20131224speed1.png');
% %--------------------------------------------------------------------------
% 
% h8 = figure
% hold on
% for k = 1:best_num
%     plot((1:(maxGen))', legb_j1(:,k), 'bo', (1:(maxGen))', legb_j2(:,k), 'ro', (1:(maxGen))', legb_j3(:,k), 'ko', 'MarkerSize', 5);
% %     plot((1:(maxGen))', btakeoffangle_hip(:,k), 'ko', 'MarkerSize', 10);
% %     plot((1:(maxGen))', battackangle_hip(:,k), 'go', 'MarkerSize', 10);
% end
% legend('Joint 1 angle', 'Joint 2 angle', 'Joint 3 angle');
% title('Evolution of Joint angles of the back legs, of the best member, 0.1 m/s');
% xlabel('Generation');
% ylabel('Joint angles (deg)');
% grid on;
% grid minor;
% hold off
% saveas(h8, 'Best_joint_angles_back_20131224speed1.png');
%--------------------------------------------------------------------------