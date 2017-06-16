function visualize_best( file )
%VISUALIZE_BEST visualize_best( file )
%   gives a graphical representation of the movement of the best member of
%   the given population file

data = loadpopfile(file);
[~, index] = min([data.pops.obj]);
best  = data.pops(index);

Obj = strfind(file, 'Obj');
if strcmp(file(Obj+4:Obj+6), 'Pow')
    optFun = 1;
    velocity = char(regexp(file,'Obj=Pow@[0-9.]+,','match'));
    if isempty(velocity);
        error('Error.\nVelocity Not Found in File Name.'); 
    end
    velocity = str2double(velocity(9:end-1));
    if length(best.var) ~= 96
        error('Error.\nIncorrect number of optimization variables')
    end
elseif strcmp(file(Obj+4:Obj+6), 'Vel')
    optFun = 2;
    if length(best.var) ~= 97
        error('Error.\nIncorrect number of optimization variables')
    end
    velocity = best.var(97);
else
    error('Error.\nUnrecognized Optimization')
end
    

% generate transformation matrices
scale = 1.0;
FLL = scale*0.33;
lg1ps = [FLL*0.43, 0, 0, 0; ...
    FLL*0.40, 0, 0, 0;...
    FLL*0.17, 0, 0, 0];
lg2ps = [FLL*0.43, 0, 0, 0; ...
    FLL*0.40, 0, 0, 0;...
    FLL*0.17, 0, 0, 0];
lg3ps = [FLL*0.39, 0, 0, 0; ...
    FLL*0.36, 0, 0, 0;...
    FLL*0.25, 0, 0, 0];
lg4ps = [FLL*0.39, 0, 0, 0; ...
    FLL*0.36, 0, 0, 0;...
    FLL*0.25, 0, 0, 0];

visualize_movement(best.var, lg1ps, lg2ps, lg3ps, lg4ps, velocity, optFun);

end



function visualize_movement(design_var, lg1ps, lg2ps, lg3ps, lg4ps, velocity, optFun)
% Function: visualize_movement(design_vars, lg1ps, lg2ps, lg3ps, lg4ps)
% Description: show trajectory of all legs given design variables

stride_length = design_var(1);
stride_time = stride_length / velocity;

% get preliminary trajectories for each leg
extra_flag = 0;
[~, y1, ~] = leg_stance_plotter(lg1ps, (design_var(6:8))');
[~, y2, ~] = leg_stance_plotter(lg2ps, (design_var(29:31))');
[~, y3, ~] = leg_stance_plotter(lg3ps, (design_var(52:54))');
[~, y4, ~] = leg_stance_plotter(lg4ps, (design_var(75:77))');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%     Generate Trajectories for each leg                              %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (abs(y1(2)) < abs(y1(1)))||(abs(y1(3)) < abs(y1(2)))||(abs(y1(4)) < abs(y1(3)))||(abs(y2(2)) < abs(y2(1)))||(abs(y2(3)) < abs(y2(2)))||(abs(y2(4)) < abs(y2(3)))||(abs(y3(2)) < abs(y3(1)))||(abs(y3(3)) < abs(y3(2)))||(abs(y3(4)) < abs(y3(3)))||(abs(y4(2)) < abs(y4(1)))||(abs(y4(3)) < abs(y4(2)))||(abs(y4(4)) < abs(y4(3)))
    extra_flag = 1;
end
ansp_ans_FL = [];
ansp_ans_FR = [];
ansp_ans_HL = [];
ansp_ans_HR = [];

% Create joint trajectories for one cycle
if ~extra_flag
    DH_mat = lg1ps;
    [ansp_ans_FL, time_traj] = invkin_traj_gen_modfl(DH_mat, stride_time, design_var(5:15), stride_length);
    DH_mat = lg2ps;
    ansp_ans_FR = invkin_traj_gen_modfr(DH_mat, stride_time, design_var(28:38), stride_length);
    DH_mat = lg3ps;
    ansp_ans_HL = invkin_traj_gen_modhl(DH_mat, stride_time, design_var(51:61), stride_length);
    DH_mat = lg4ps;
    ansp_ans_HR = invkin_traj_gen_modhr(DH_mat, stride_time, design_var(74:84), stride_length);
end
index_ini = 0;
l = min([length(ansp_ans_FL), length(ansp_ans_FR), length(ansp_ans_HL), length(ansp_ans_HR)]);
ansp_ans_FL = ansp_ans_FL(:, 1:l);
ansp_ans_FR = ansp_ans_FR(:, 1:l);
ansp_ans_HL = ansp_ans_HL(:, 1:l);
ansp_ans_HR = ansp_ans_HR(:, 1:l);
time_traj = time_traj(1:l);

if extra_flag
    display('Constraint violation - infeasible joint angles')
    cons(1) = 1;
elseif (isempty(ansp_ans_FL))||(isempty(ansp_ans_FR))||(isempty(ansp_ans_HR))||(isempty(ansp_ans_HL)) % c1 violated - can't proceed
    display('Constraint violation - inv kin did not converge')
    cons(1) = 1;
    %%%keyboard();
elseif (length(ansp_ans_FL) ~= length(ansp_ans_FR))||(length(ansp_ans_FL) ~= length(ansp_ans_HR))||(length(ansp_ans_FL) ~= length(ansp_ans_HL))
    display('Constraint violation - trajectories generated not equal in length')
    cons(1) = 1;
else % c1 not violated - proceed
    % Extend created joint trajectories to multiple cycles based on
    % total desired simulation time and adjust according to
    % relative phases:
    
    % Create leg markers to identify stance(0), swing(1) and
    % AEP(-1)
    [~, length_should_be] = size(ansp_ans_FL);
    leg_marker_FL = zeros(1,length_should_be);
    leg_marker_FR = zeros(1,length_should_be);
    leg_marker_HL = zeros(1,length_should_be);
    leg_marker_HR = zeros(1,length_should_be);
    
    for i = 1:(floor((1-design_var(5))*length_should_be))
        leg_marker_FL(i) = 1;  % Marker has a 1 for swing phase and 0 for stance phase
    end
    for i = 1:(floor((1-design_var(28))*length_should_be))
        leg_marker_FR(i) = 1;  % Marker has a 1 for swing phase and 0 for stance phase
    end
    for i = 1:(floor((1-design_var(51))*length_should_be))
        leg_marker_HL(i) = 1;  % Marker has a 1 for swing phase and 0 for stance phase
    end
    for i = 1:(floor((1-design_var(74))*length_should_be))
        leg_marker_HR(i) = 1;  % Marker has a 1 for swing phase and 0 for stance phase
    end
    leg_marker_FL((floor((1-design_var(5))*length_should_be))+1) = -1;
    leg_marker_FR((floor((1-design_var(28))*length_should_be))+1) = -1;
    leg_marker_HL((floor((1-design_var(51))*length_should_be))+1) = -1;
    leg_marker_HR((floor((1-design_var(74))*length_should_be))+1) = -1;
    
    % adjust trajectories w.r.t. their relative phases
    ansp_FL = ansp_ans_FL;
    [ansp_FL, leg_marker_FL] = modforleg(ansp_ans_FL, leg_marker_FL, 1, 2);
    [ansp_FR, leg_marker_FR] = modforleg(ansp_ans_FR, leg_marker_FR, 1, 2);
    [ansp_HL, leg_marker_HL] = modforleg(ansp_ans_HL, leg_marker_HL, 1, 2);
    [ansp_HR, leg_marker_HR] = modforleg(ansp_ans_HR, leg_marker_HR, 1, 2);
    time_tempo = time_traj;
    time_tempo = time_tempo + stride_time;
    time_traj = [time_traj(:,1:end-1), time_tempo];

    % need to find initialization point
    index_ini = 0; % non-existant index
    for counter = 1:length_should_be
        if ((leg_marker_FL(counter) == 0)||(leg_marker_FL(counter) == -1)||(leg_marker_FR(counter) == 0)||(leg_marker_FR(counter) == -1))&&((leg_marker_HL(counter) == 0)||(leg_marker_HL(counter) == -1)||(leg_marker_HR(counter) == 0)||(leg_marker_HR(counter) == -1))
            index_ini = counter;
            break
        end
    end
end



if(index_ini == 0)
    % if index_ini is zero = such an initialization point does not
    % exist, and thus the solution is infeasible.
    display('Constraint violation - initialization point does not exist')
else
    % if index_ini is non-zero, adjust the trajectories, such that
    % the simulation starts from this initialization point.
    [ansp_FL, leg_marker_FL] = modforleg1(ansp_FL, leg_marker_FL, abs(index_ini/length_should_be));
    [ansp_FR, leg_marker_FR] = modforleg1(ansp_FR, leg_marker_FR, abs(index_ini/length_should_be));
    [ansp_HL, leg_marker_HL] = modforleg1(ansp_HL, leg_marker_HL, abs(index_ini/length_should_be));
    [ansp_HR, leg_marker_HR] = modforleg1(ansp_HR, leg_marker_HR, abs(index_ini/length_should_be));
    
    % Variables to be used by Simulink/Simmechanics model
    TRJ_LG1_J1  = [time_traj; ansp_FL(1,:)];
    TRJ_LG1_J2  = [time_traj; ansp_FL(2,:)];
    TRJ_LG1_J3  = [time_traj; ansp_FL(3,:)];
    
    TRJ_LG2_J1  = [time_traj; ansp_FR(1,:)];
    TRJ_LG2_J2  = [time_traj; ansp_FR(2,:)];
    TRJ_LG2_J3  = [time_traj; ansp_FR(3,:)];
    
    TRJ_LG3_J1  = [time_traj; ansp_HL(1,:)];
    TRJ_LG3_J2  = [time_traj; ansp_HL(2,:)];
    TRJ_LG3_J3  = [time_traj; ansp_HL(3,:)];
    
    TRJ_LG4_J1  = [time_traj; ansp_HR(1,:)];
    TRJ_LG4_J2  = [time_traj; ansp_HR(2,:)];
    TRJ_LG4_J3  = [time_traj; ansp_HR(3,:)];
    
    FL_AEP = find(leg_marker_FL == -1, 1);
    FR_AEP = find(leg_marker_FR == -1, 1);
    HL_AEP = find(leg_marker_HL == -1, 1);
    HR_AEP = find(leg_marker_HR == -1, 1);
    
    [leg1, hip1, knee1, ankle1, toe1] = ...
        plot_trajectory(lg1ps, TRJ_LG1_J1, TRJ_LG1_J2, TRJ_LG1_J3, velocity, FL_AEP, 30);
    figure(leg1);   title(['Trajectory of Front Left Leg at '           num2str(velocity) ' m/s']);
    figure(hip1);   title(['Vertical Position of Front Left Hip at '    num2str(velocity) ' m/s']);
    figure(knee1);  title(['Vertical Position of Front Left Knee at '   num2str(velocity) ' m/s']);
    figure(ankle1); title(['Vertical Position of Front Left Ankle at '  num2str(velocity) ' m/s']);
    figure(toe1);   title(['Vertical Position of Front Left Toe at '    num2str(velocity) ' m/s']);
    
    [leg2, hip2, knee2, ankle2, toe2] = ...
        plot_trajectory(lg2ps, TRJ_LG2_J1, TRJ_LG2_J2, TRJ_LG2_J3, velocity, FR_AEP, 30);
    figure(leg2);   title(['Trajectory of Front Right Leg at '          num2str(velocity) ' m/s']);
    figure(hip2);   title(['Vertical Position of Front Right Hip at '   num2str(velocity) ' m/s']);
    figure(knee2);  title(['Vertical Position of Front Right Knee at '  num2str(velocity) ' m/s']);
    figure(ankle2); title(['Vertical Position of Front Right Ankle at ' num2str(velocity) ' m/s']);
    figure(toe2);   title(['Vertical Position of Front Right Toe at '   num2str(velocity) ' m/s']);
    
    [leg3, hip3, knee3, ankle3, toe3] = ...
        plot_trajectory(lg3ps, TRJ_LG3_J1, TRJ_LG3_J2, TRJ_LG3_J3, velocity, HL_AEP, 30);
    title(['Trajectory of Hind Left Leg at ' num2str(velocity) ' m/s']);
    figure(leg3);   title(['Trajectory of Hind Left Leg at '            num2str(velocity) ' m/s']);
    figure(hip3);   title(['Vertical Position of Hind Left Hip at '     num2str(velocity) ' m/s']);
    figure(knee3);  title(['Vertical Position of Hind Left Knee at '    num2str(velocity) ' m/s']);
    figure(ankle3); title(['Vertical Position of Hind Left Ankle at '   num2str(velocity) ' m/s']);
    figure(toe3);   title(['Vertical Position of Hind Left Toe at '     num2str(velocity) ' m/s']);
    
    [leg4, hip4, knee4, ankle4, toe4] = ...
        plot_trajectory(lg4ps, TRJ_LG4_J1, TRJ_LG4_J2, TRJ_LG4_J3, velocity, HR_AEP, 30);
    title(['Trajectory of Hind Right Leg at ' num2str(velocity) ' m/s']);
    figure(leg4);   title(['Trajectory of Hind Right Leg at '           num2str(velocity) ' m/s']);
    figure(hip4);   title(['Vertical Position of Hind Right Hip at '    num2str(velocity) ' m/s']);
    figure(knee4);  title(['Vertical Position of Hind Right Knee at '   num2str(velocity) ' m/s']);
    figure(ankle4); title(['Vertical Position of Hind Right Ankle at '  num2str(velocity) ' m/s']);
    figure(toe4);   title(['Vertical Position of Hind Right Toe at '    num2str(velocity) ' m/s']);
    
    
    saveas(leg1,    ['Trajectory of Front Left Leg at '         num2str(velocity) '.png']);
    saveas(hip1,    ['Vertical Position of Front Left Hip at '  num2str(velocity) '.png']);
    saveas(knee1,   ['Vertical Position of Front Left Knee at '  num2str(velocity) '.png']);
    saveas(ankle1,  ['Vertical Position of Front Left Ankle at '  num2str(velocity) '.png']);
    saveas(toe1,    ['Vertical Position of Front Left Toe at '  num2str(velocity) '.png']);
    
    saveas(leg2,    ['Trajectory of Front Right Leg at '        num2str(velocity) '.png']);
    saveas(hip2,    ['Vertical Position of Front Right Hip at ' num2str(velocity) '.png']);
    saveas(knee2,   ['Vertical Position of Front Right Knee at ' num2str(velocity) '.png']);
    saveas(ankle2,  ['Vertical Position of Front Right Ankle at ' num2str(velocity) '.png']);
    saveas(toe2,    ['Vertical Position of Front Right Toe at ' num2str(velocity) '.png']);
    
    saveas(leg3,    ['Trajectory of Hind Left Leg at '          num2str(velocity) '.png']);
    saveas(hip3,    ['Vertical Position of Hind Left Hip at '   num2str(velocity) '.png']);
    saveas(knee3,   ['Vertical Position of Hind Left Knee at '   num2str(velocity) '.png']);
    saveas(ankle3,  ['Vertical Position of Hind Left Ankle at '   num2str(velocity) '.png']);
    saveas(toe3,    ['Vertical Position of Hind Left Toe at '   num2str(velocity) '.png']);
    
    saveas(leg4,    ['Trajectory of Hind Right Leg at '         num2str(velocity) '.png']);
    saveas(hip4,    ['Vertical Position of Hind Right Hip at '  num2str(velocity) '.png']);
    saveas(knee4,   ['Vertical Position of Hind Right Knee at '  num2str(velocity) '.png']);
    saveas(ankle4,  ['Vertical Position of Hind Right Ankle at '  num2str(velocity) '.png']);
    saveas(toe4,    ['Vertical Position of Hind Right Toe at '  num2str(velocity) '.png']);
    
    close all
    
    if optFun == 1
        [OF, cons] = objective_function_main(design_var);
    elseif optFun == 2
        [OF, cons] = objective_function_velocity(design_var);
    end
    OF
end
end

