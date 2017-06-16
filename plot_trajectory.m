function [leg, hip, knee, ankle, toe] = plot_trajectory(DH_MAT, J1_traj, J2_traj, J3_traj, velocity, AEP, divisions)
% Function: trajectory_visualization(DH_MAT, J1_traj, J2_traj, J3_traj, velocity, divisions)
% Description: show the trajectory of the given leg

    lentraj = floor(length(J1_traj)/2);
    % select times to visualize and calculate the instanataneous 
    % position of the hip at those 
    sample_indices = floor(linspace(1, lentraj, divisions));
    T = J1_traj(1, AEP : AEP + lentraj - 1) - J1_traj(1, AEP);
    hip_offset = [T * velocity; zeros(2, lentraj)];
    
    % joint angle triplets, where each column is at one moment
    q = [   
            J1_traj(2, AEP : AEP + lentraj);
            J2_traj(2, AEP : AEP + lentraj);
            J3_traj(2, AEP : AEP + lentraj)
        ];
    
    % instantiate position variables
    X = zeros(4, lentraj);
    Y = zeros(4, lentraj);
    Z = zeros(4, lentraj);
    
    % populate the position matrix at each position and apply offset
    for i = 1:lentraj
        [X(:, i), Z(:, i), Y(:, i)] = leg_stance_plotter(DH_MAT, q(:, i));
    end
    
    X = X + repmat(hip_offset(1,:), [4,1]);
    X = X - min(X(4,:));
    Y = Y + repmat(hip_offset(2,:), [4,1]);
    Y = Y - min(Y(4,:));
    Z = Z + repmat(hip_offset(3,:), [4,1]);
    Z = Z - min(Z(4,:));
    
    % display and save trajectories
    leg = figure;
    plot3(X(:,sample_indices), Y(:,sample_indices), Z(:,sample_indices),...
        'color', 'k');
    daspect([1, 1, 1]);
    ylim([0, 0.26]);
    view(2);
    xlabel('X');
    ylabel('Z');
    
    hip = figure;
    plot(T, Z(1,:),'color','k');
    xlabel('T');
    ylabel('Z');
    
    knee = figure;
    plot(T, Z(2,:),'color','k');
    xlabel('T');
    ylabel('Z');
    
    ankle = figure;
    plot(T, Z(3,:),'color','k');
    xlabel('T');
    ylabel('Z');
    
    toe = figure;
    plot(T, Z(4,:),'color','k');
    xlabel('T');
    ylabel('Z');
    
end