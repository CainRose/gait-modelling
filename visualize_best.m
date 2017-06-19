function visualize_best( file, component )
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
if optFun == 1
    [OF, cons, d] = objective_function_main(design_var);
elseif optFun == 2
    [OF, cons, d] = objective_function_velocity(design_var);
end

if strcmp(component, 'trajectory')
    [leg1, joint1] = ...
        plot_trajectory(d.lg1ps, d.TRJ_LG1_J1, d.TRJ_LG1_J2, d.TRJ_LG1_J3, velocity, d.FL_AEP, 30);
    figure(leg1);   title(['Trajectory of Front Left Leg at '            num2str(velocity) ' m/s']);
    figure(joint1); title(['Vertical Position of Front Left Joints at '  num2str(velocity) ' m/s']);
    %     figure(hip1);   title(['Vertical Position of Front Left Hip at '    num2str(velocity) ' m/s']);
    %     figure(knee1);  title(['Vertical Position of Front Left Knee at '   num2str(velocity) ' m/s']);
    %     figure(ankle1); title(['Vertical Position of Front Left Ankle at '  num2str(velocity) ' m/s']);
    %     figure(toe1);   title(['Vertical Position of Front Left Toe at '    num2str(velocity) ' m/s']);

    [leg2, joint2] = ...
        plot_trajectory(d.lg2ps, d.TRJ_LG2_J1, d.TRJ_LG2_J2, d.TRJ_LG2_J3, velocity, d.FR_AEP, 30);
    figure(leg2);   title(['Trajectory of Front Right Leg at '           num2str(velocity) ' m/s']);
    figure(joint2); title(['Vertical Position of Front Right Joints at ' num2str(velocity) ' m/s']);
    %     figure(hip2);   title(['Vertical Position of Front Right Hip at '   num2str(velocity) ' m/s']);
    %     figure(knee2);  title(['Vertical Position of Front Right Knee at '  num2str(velocity) ' m/s']);
    %     figure(ankle2); title(['Vertical Position of Front Right Ankle at ' num2str(velocity) ' m/s']);
    %     figure(toe2);   title(['Vertical Position of Front Right Toe at '   num2str(velocity) ' m/s']);

    [leg3, joint3] = ...
        plot_trajectory(d.lg3ps, d.TRJ_LG3_J1, d.TRJ_LG3_J2, d.TRJ_LG3_J3, velocity, d.HL_AEP, 30);
    title(['Trajectory of Hind Left Leg at ' num2str(velocity) ' m/s']);
    figure(leg3);   title(['Trajectory of Hind Left Leg at '             num2str(velocity) ' m/s']);
    figure(joint3); title(['Vertical Position of Hind Left Joints at '   num2str(velocity) ' m/s']);
    %     figure(hip3);   title(['Vertical Position of Hind Left Hip at '     num2str(velocity) ' m/s']);
    %     figure(knee3);  title(['Vertical Position of Hind Left Knee at '    num2str(velocity) ' m/s']);
    %     figure(ankle3); title(['Vertical Position of Hind Left Ankle at '   num2str(velocity) ' m/s']);
    %     figure(toe3);   title(['Vertical Position of Hind Left Toe at '     num2str(velocity) ' m/s']);

    [leg4, joint4] = ...
        plot_trajectory(d.lg4ps, d.TRJ_LG4_J1, d.TRJ_LG4_J2, d.TRJ_LG4_J3, velocity, d.HR_AEP, 30);
    title(['Trajectory of Hind Right Leg at ' num2str(velocity) ' m/s']);
    figure(leg4);   title(['Trajectory of Hind Right Leg at '            num2str(velocity) ' m/s']);
    figure(joint4); title(['Vertical Position of Hind Right Joints at '  num2str(velocity) ' m/s']);
    %     figure(hip4);   title(['Vertical Position of Hind Right Hip at '    num2str(velocity) ' m/s']);
    %     figure(knee4);  title(['Vertical Position of Hind Right Knee at '   num2str(velocity) ' m/s']);
    %     figure(ankle4); title(['Vertical Position of Hind Right Ankle at '  num2str(velocity) ' m/s']);
    %     figure(toe4);   title(['Vertical Position of Hind Right Toe at '    num2str(velocity) ' m/s']);


    saveas(leg1,    ['Trajectory of Front Left Leg at '             num2str(velocity) '.png']);
    saveas(joint1,  ['Vertical Position of Front Left Joints at '   num2str(velocity) '.png']);
    %     saveas(hip1,    ['Vertical Position of Front Left Hip at '  num2str(velocity) '.png']);
    %     saveas(knee1,   ['Vertical Position of Front Left Knee at '  num2str(velocity) '.png']);
    %     saveas(ankle1,  ['Vertical Position of Front Left Ankle at '  num2str(velocity) '.png']);
    %     saveas(toe1,    ['Vertical Position of Front Left Toe at '  num2str(velocity) '.png']);

    saveas(leg2,    ['Trajectory of Front Right Leg at '            num2str(velocity) '.png']);
    saveas(joint2,  ['Vertical Position of Front Right Joints at '  num2str(velocity) '.png']);
    %     saveas(hip2,    ['Vertical Position of Front Right Hip at ' num2str(velocity) '.png']);
    %     saveas(knee2,   ['Vertical Position of Front Right Knee at ' num2str(velocity) '.png']);
    %     saveas(ankle2,  ['Vertical Position of Front Right Ankle at ' num2str(velocity) '.png']);
    %     saveas(toe2,    ['Vertical Position of Front Right Toe at ' num2str(velocity) '.png']);

    saveas(leg3,    ['Trajectory of Hind Left Leg at '              num2str(velocity) '.png']);
    saveas(joint3,  ['Vertical Position of Hind Left Joints at '    num2str(velocity) '.png']);
    %     saveas(hip3,    ['Vertical Position of Hind Left Hip at '   num2str(velocity) '.png']);
    %     saveas(knee3,   ['Vertical Position of Hind Left Knee at '   num2str(velocity) '.png']);
    %     saveas(ankle3,  ['Vertical Position of Hind Left Ankle at '   num2str(velocity) '.png']);
    %     saveas(toe3,    ['Vertical Position of Hind Left Toe at '   num2str(velocity) '.png']);

    saveas(leg4,    ['Trajectory of Hind Right Leg at '             num2str(velocity) '.png']);
    saveas(joint4,  ['Vertical Position of Hind Right Joints at '   num2str(velocity) '.png']);
    %     saveas(hip4,    ['Vertical Position of Hind Right Hip at '  num2str(velocity) '.png']);
    %     saveas(knee4,   ['Vertical Position of Hind Right Knee at '  num2str(velocity) '.png']);
    %     saveas(ankle4,  ['Vertical Position of Hind Right Ankle at '  num2str(velocity) '.png']);
    %     saveas(toe4,    ['Vertical Position of Hind Right Toe at '  num2str(velocity) '.png']);
elseif strcmp(component, 'of')
    OF
elseif strcmp(component, 'torque')
    
elseif strcmp(component, 'jointangle')
    
end
end

