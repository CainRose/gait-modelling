function [traj_out, leg_marker_out] = modforleg(traj_in, leg_marker_in, obj_ite_rel_phase, num_strides)

[~, length_traj] = size(traj_in);
relative_index = floor(length_traj*(1-obj_ite_rel_phase));
    traj_out = traj_in;
    leg_marker_out = leg_marker_in;
    if (relative_index ~= 0)
        clear traj_out leg_marker_out
        traj_out = [traj_in(:,(relative_index+1):(end)), traj_in(:,1:relative_index)];
        leg_marker_out = [leg_marker_in(relative_index+1:end), leg_marker_in(1:relative_index)];
    end
    
    temp_traj = traj_out;
    temp_mark = leg_marker_out;
    for num = 1:num_strides-1
        traj_out = [traj_out(:,1:end-1), temp_traj];
        leg_marker_out = [leg_marker_out(:,1:end-1), temp_mark];
    end
end

