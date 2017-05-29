function plot_trajectory(DH_MAT, J1_traj, J2_traj, J3_traj, velocity, AEP, divisions)
% Function: trajectory_visualization(DH_MAT, J1_traj, J2_traj, J3_traj, velocity, divisions)
% Description: show the trajectory of the given leg

    % select times to visualize and calculate the instanataneous 
    % position of the hip at those 
    sample_indices = floor(linspace(AEP, AEP + length(J1_traj)/2, divisions))*2;
    hip_offset = [J1_traj(sample_indices-1)*velocity; zeros(2, divisions)];
    
    % joint angle triplets, where each column is at one moment
    q = [   
            J1_traj(sample_indices);
            J2_traj(sample_indices);
            J3_traj(sample_indices)
        ];
    
    % instantiate position variables
    X = zeros(4, divisions);
    Y = zeros(4, divisions);
    Z = zeros(4, divisions);
    
    % populate the position matrix at each position and apply offset
    for i = 1:divisions
        [X(:, i), Y(:, i), Z(:, i)] = leg_stance_plotter(DH_MAT, q(:, i));
    end
    
    X = X + repmat(hip_offset(1,:), [4,1]) - min(X(4,:));
    Y = Y + repmat(hip_offset(2,:), [4,1]) - min(Y(4,:));
    Z = Z + repmat(hip_offset(3,:), [4,1]) - min(Z(4,:));
    
    % display and save trajectory
    plot3(X, Y, Z, 'color', 'k')
    daspect([1, 1, 1]);
    ylim([0, 0.26]);
    view(2);
    xlabel('X');
    ylabel('Y');
end