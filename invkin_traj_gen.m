function [ansp, time_traj] = invkin_traj_gen(DH_mat, stride_time, design_var, stride_length, pitch)
% Function: [ansp, time_traj] = trajectory_generator(leg_number, DH_matrix, design_var, stride_time)
% 
% Description: Generates the joint trajectory for the selected leg based on
%   the optimization variables. Note: AEP stands for Anterior Extreme
%   Position and PEP stangs for Posterior Extreme Position.
%
% Parameters:
%   DH_mat : The Denavit-Hartenburg coordinates of the leg's joints
%   stride_time : The length of one cycle
%   design_var : The optimization variables specific to the leg (11)
%   stride_length: The distance moved by the body over one cycle
% 
% Return:
%   ansp : The joint angles of the leg's trajectory
%   time_traj : The time associated with each set of angles
%
%   Revision: 1.0  Data: 2017-06-22
%*************************************************************************

beta = design_var(1);
AEP_L1_J1 = design_var(2);  AEP_L1_J2 = design_var(3);  AEP_L1_J3 = design_var(4);
step_length = beta*stride_length;
delta_y = step_length*design_var(5);
delta_psi = design_var(6);
step_clearance = design_var(7);
psi_dot_AEP = design_var(10);
psi_dot_PEP = design_var(11);

%% Step 1: Use forward kinematics function, to find out points. 
% Rearrange DH matrix (theta d a alpha) to suit function
dh = zeros(3, 4);
q0 = [AEP_L1_J1; AEP_L1_J2;AEP_L1_J3];
dh(:, 3) = DH_mat(:, 1);

% Create robot model and find homogeneous transform matrix
leg = SerialLink(dh);
htm = leg.fkine(q0);

% Extract relavent information from homogeneous transform matrix
x_aep = htm(1, 4); y_aep = htm(2, 4); z_aep = htm(3, 4);
rot_mat_aep = htm(1:3, 1:3);
y_pep = y_aep - step_length*sin(pitch);
psi_aep = mod(atan2(rot_mat_aep(2,1),rot_mat_aep(1,1))+pi, 2*pi)-pi;

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% Step 2: Construct all trajectories
T_swing = stride_time*(1 - beta); 
T_stance = stride_time*beta;
speed = design_var(7)/stride_time;
st_x_dot = speed * cos(pitch);
st_y_dot = speed * sin(pitch);
[time_traj, x_traj, y_traj, psi_traj, len] = end_effector_traj(st_x_dot, x_aep, y_aep, y_pep, st_y_dot, st_y_dot, psi_dot_PEP, psi_dot_AEP, psi_aep, delta_psi, step_clearance, T_swing, T_stance, step_length);

% keyboard();
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% Step 3: Use average of first two joint values, plus inverse kinematics 

% Construct homogeneous transform matrices from trajectories
% htm_traj = repmat(eye(4), [1 1 length(time_traj)]);
% htm_traj_temp = repmat(eye(4), [1 1 length(time_traj)]);
% htm_traj_temp(1, 1, :) =  cos(psi_traj);
% htm_traj_temp(1, 2, :) = -sin(psi_traj);
% htm_traj_temp(2, 1, :) =  sin(psi_traj);
% htm_traj_temp(2, 2, :) =  cos(psi_traj);
% htm_traj_temp(1, 4, :) =  x_traj;
% htm_traj_temp(2, 4, :) =  y_traj;
% 
% htm_traj(:, :, end-len+1:end) = htm_traj_temp(:, :, 1:len);
% htm_traj(:, :, 1:end-len) = htm_traj_temp(:, :, len+1:end);
% 
% q_traj = zeros(3, length(time_traj));
% q_traj(:, 1) = q0;
x3_traj = [x_traj(len+1:end) x_traj(1:len)];
y3_traj = [y_traj(len+1:end) y_traj(1:len)];
psi3_traj = [psi_traj(len+1:end) psi_traj(1:len)];
q_traj_guess = invkin2(DH_mat, x3_traj, y3_traj, psi3_traj, q0);
if isempty(q_traj_guess)
    ansp = [];
    return
end
% for i = 1:length(time_traj)-1
%      q_traj(:, i+1) = leg.ikine(htm_traj(:,:,i), q_traj_guess(:,i),[1 1 0 0 0 1], 'alpha', 1);
% end
% q_traj = [q_traj(:,len+1:end) q_traj(:,1:len)];

 % function to find out additional poses
total_steps = length(time_traj);
 % 3.1: setting up invkin function inputs
ansp = zeros(3,total_steps);
% Defining the various x positions
handx = zeros(total_steps, 3);
handro = zeros(total_steps, 3);
handra = zeros(total_steps, 3);
% keyboard();

handx(:,1) = x_traj';
handx(:,2) = y_traj';

% z portion
for i = 1:total_steps
    handx(i,3) = z_aep;
end

% Approach vector and orientation vector

for random_var = 1:(total_steps)
    handra(random_var, :) = [0;0;1];
end

for random_var = 1:(total_steps)
    handro(random_var, :) = [-sin(psi_traj(random_var));cos(psi_traj(random_var));0];
end
% keyboard();
% 3.2: manipulating handx, handro, and handra to match the order AEP -> SWM -> PEP -> STM -> AEP

handx_temp = handx;
handro_temp = handro;
T_swing = round(T_swing*1000)/1000;
T_stance = round(T_stance*1000)/1000;
t_steps_stance = length(0:0.001:T_stance);
handx(1:t_steps_stance,:) = handx_temp(end-t_steps_stance+1:end, :);
handx(t_steps_stance+1:end,:) = handx_temp(1:end-t_steps_stance,:);
handro(1:t_steps_stance,:) = handro_temp(end-t_steps_stance+1:end, :);
handro(t_steps_stance+1:end,:) = handro_temp(1:end-t_steps_stance,:);
% keyboard();
q0 = [AEP_L1_J1; AEP_L1_J2; AEP_L1_J3]; % Initial guess for invkin function
dh_t = zeros(3, 5);
dh_t = [DH_mat(1:3,2), DH_mat(1:3,1), DH_mat(1:3,4), DH_mat(1:3,3), [0;0;0]]; % append the joint type coloumn, and reorder DH_mat according to invkin function
% % keyboard();
% 3.3: calling invkin and pose initialization for CPG
[q, matrices] = invkin(dh_t, handx(1,:), handro(1,:), handra(1,:), q0);
    if isempty(q)            
        ansp = [];
        return
    end
    ansp(:,1) = q;
%     keyboard();
    for loop_var = 2:total_steps
        [q, matrices] = invkin(dh_t, handx(loop_var,:), handro(loop_var,:), handra(loop_var,:), q_traj_guess(:,loop_var));
        if isempty(q)            
%             keyboard();
            ansp = [];

            return
        end 
        ansp(:,loop_var) = q;
    end
% ansp(:,t_steps_swing) =  q(1:3);
% %keyboard();
% for i = 1:(t_steps_swing-1)
% 
%     [q, matrices] = invkin(dh_t, handx(i+1,:), handro(i+1,:), handra(i+1,:), q);
%         if isempty(q)            
%             ansp = [];
%             return
%         end
%         ansp(:,t_steps_swing-i) =  q(1:3);
% %     %keyboard();
% end
% ansp(:,t_steps_swing+1) = [AEP_L1_J1; AEP_L1_J2; AEP_L1_J3];
% for i = t_steps_swing+2:t_steps_swing+t_steps_stance
% 
%     [q, matrices] = invkin(dh_t, handx(i-1,:), handro(i-1,:), handra(i-1,:), q);
%         if isempty(q)            
%             ansp = [];
%             return
%         end
%         ansp(:,(t_steps_swing+t_steps_stance+t_steps_swing+2)-i) =  q(1:3);
% %     %keyboard();
% end
ansp = mod((ansp)+ pi, 2*pi) - pi;       % changing interval
% keyboard();
ansp_temp = ansp;
ansp(:,end-t_steps_stance+1:end) = ansp_temp(:,1:t_steps_stance);
ansp(:,1:end-t_steps_stance) = ansp_temp(:,t_steps_stance+1:end,:);
% keyboard();
% ansp(1:t_steps_stance,:) = ansp_temp(end-t_steps_stance+1:end, :);
% ansp(t_steps_stance+1:end,:) = ansp_temp(1:end-t_steps_stance,:);

% display('everything solved')
%keyboard();
% ansp now is a 3 rows by n coloumn matrix


end
