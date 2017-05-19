function [traj_out, leg_marker_out] = modforleg1(traj_in, leg_marker_in, obj_ite_rel_phase)

[~, length_traj] = size(traj_in);
relative_index = floor(length_traj*(1-obj_ite_rel_phase));

    traj_out = [traj_in(:,(relative_index+1):(end)), traj_in(:,1:relative_index)];
    leg_marker_out = [leg_marker_in(relative_index+1:end), leg_marker_in(1:relative_index)];

end

