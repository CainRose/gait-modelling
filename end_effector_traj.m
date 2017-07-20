function [ net_time, net_x, net_z, net_psi, len ] = end_effector_traj(speed, x_ini, x_fin, z_ini, z_fin, z_dot_ini, z_dot_fin, psi_dot_ini, psi_dot_fin, psi_ini, psi_change, Fc, T_swing, T_stance, kappa)
% Constructs the entire end-effector-trajectory, based on design variables
% speed = x_dot
% x_ini = x_aep
% z_dot_ini = take off angle for hip's stance traj
% z_dot_fin = attack angle for hip's stance traj
% psi_dot_ini = take off angle for orientation traj
% psi_dot_fin = attack angle for orientation traj
% psi_ini = psi of last link at AEP
% psi_change = net change in psi from aep to pep
% kappa = step_length

T_swing = round(T_swing*1000)/1000;
T_stance = round(T_stance*1000)/1000;
time_vec_swing = 0:0.001:T_swing;
% t_steps_swing = length(time_vec_swing);
time_vec_stance = 0:0.001:T_stance;
t_steps_stance = length(time_vec_stance);
% keyboard();
temp_time_swing = time_vec_swing + (T_stance).*ones(1,length(time_vec_swing));
net_time = [time_vec_stance,temp_time_swing(2:end)];
% keyboard();
% X trajectories
x_traj_stance = new_bezier_order3(x_ini, x_fin, -speed, -speed, time_vec_stance);
x_traj_swing = new_bezier_order3(x_fin, x_ini, -speed, -speed, time_vec_swing);

% Z trajectories
z_traj_swing = new_bezier_order4(z_fin-z_ini, 0, -z_dot_fin, -z_dot_ini, Fc*kappa, time_vec_swing);
z_traj_stance = new_bezier_order3(0, z_fin-z_ini, -z_dot_ini, -z_dot_fin, time_vec_stance);

% Psi trajectories
psi_traj_swing = new_bezier_order3(psi_ini-psi_change, psi_ini, psi_dot_fin, psi_dot_ini, time_vec_swing);
psi_traj_stance = new_bezier_order3(psi_ini, psi_ini-psi_change, psi_dot_ini, psi_dot_fin, time_vec_stance);

% keyboard();

net_psi = [psi_traj_swing, psi_traj_stance(2:end)];
net_z = [z_traj_swing, z_traj_stance(2:end)] + z_ini;
net_x = [x_traj_swing, x_traj_stance(2:end)];
len = length(x_traj_swing);
% figure; plot(net_time, net_x);
% figure; plot(net_time, net_z);
% figure; plot(net_time, net_psi);
%keyboard();
end

