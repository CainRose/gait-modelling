function [ansp, time] = trajectory_generator(DH_matrix, stride_time, design_var, stride_length)
% Function: [trajectory, time] = trajectory_generator(leg_number, DH_matrix, design_var, stride_time)
% 
% Description: Generates the joint trajectory for the selected leg based on
%   the optimization variables. Note: AEP stands for Anterior Extreme
%   Position and PEP stangs for Posterior Extreme Position.
%
% Parameters:
%   DH_matrix : The Denavit-Hartenburg coordinates of the leg's joints
%   stride_time : The length of one cycle
%   design_var : The optimization variables specific to the leg (11)
%   stride_length: The distance moved by the body over one cycle
% 
% Return:
%   trajectory : The joint angles of the leg's trajectory
%   time : The time associated with each set of angles
%
%   Revision: 1.0  Data: 2017-06-19
%*************************************************************************


%% Get design variables for chosen leg
beta            = design_var(1);        % The ratio of time in the stance to swing phase
step_length     = beta * stride_length;             % The distance travelled during the swing phase
theta_J1_AEP    = design_var(2);    % Angle of the hip at AEP
theta_J2_AEP    = design_var(3);    % Angle of the knee at AEP
theta_J3_AEP    = design_var(4);    % Angle of the ankle at AEP
delta_z         = design_var(5);    % Change in z over one phase
delta_psi       = design_var(6);    % Change in psi over one phase
step_clearance  = design_var(7);    % Height of toe mid-swing
z_dot_AEP       = design_var(8);    % Vertical velocity of toe at AEP
z_dot_PEP       = design_var(9);    % Vertical velocity of toe at PEP
psi_dot_AEP     = design_var(10);    % Angular velocity of toe at AEP
psi_dot_PEP     = design_var(11);   % Angular velocity of toe at PEP

%%% Design variables representing a proportion of step_length
delta_z = step_length * delta_z;

%% Use forward kinematics to find Cartesian cooridnate of leg at AEP
%%% Set DH coordinates at AEP
DH_AEP = DH_matrix;
DH_AEP(1:3, 4) = [theta_J1_AEP; theta_J2_AEP; theta_J3_AEP];

%%% Find position and rotation matrix of leg at AEP using forward kinematics
[x_AEP, z_AEP, y_AEP, rot_mat_AEP] = forward_kin(DH_AEP);
% y_AEP = -y_AEP; % Function calculates (x, y, z), ensure proper direction

%%% Find position at PEP
% x_PEP = x_AEP - delta_x;
z_PEP = z_AEP - delta_z;
% y_PEP = y_AEP;

%%% Find the orientation of the last link
psi_AEP = mod(atan2(rot_mat_AEP(2,1),rot_mat_AEP(1,1))+pi, 2*pi)-pi;
psi_PEP = psi_AEP - delta_psi;

%% Construct trajectories
T_swing  = stride_time * (1 - beta);
T_stance = stride_time * beta;
x_dot    = stride_length / stride_time;
[time, x, y, psi] = end_effector_traj(x_dot, x_AEP, z_AEP, z_PEP, z_dot_PEP, ...
                        z_dot_AEP, psi_dot_PEP, psi_dot_AEP, psi_AEP, ...
                        delta_psi, step_clearance, T_swing, T_stance, step_length);

%% Step 3: Use average of first two joint values, plus inverse kinematics (who knows what this is...)
 % function to find out additional poses
total_steps = length(time);
 % 3.1: setting up invkin function inputs
ansp = zeros(3,total_steps);
% Defining the various x positions
handx = zeros(total_steps, 3);
handro = zeros(total_steps, 3);
handra = zeros(total_steps, 3);
% keyboard();

handx(:,1) = x';
handx(:,2) = y';

% z portion
for i = 1:total_steps
    handx(i,3) = z_AEP;
end

% Approach vector and orientation vector

for random_var = 1:(total_steps)
    handra(random_var, :) = [0;0;1];
end

for random_var = 1:(total_steps)
    handro(random_var, :) = [-sin(psi(random_var));cos(psi(random_var));0];
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
q0 = [theta_J1_AEP; theta_J2_AEP; theta_J3_AEP]; % Initial guess for invkin function
dh_t = zeros(3, 5);
dh_t = [DH_matrix(1:3,2), DH_matrix(1:3,1), DH_matrix(1:3,4), DH_matrix(1:3,3), [0;0;0]]; % append the joint type coloumn, and reorder DH_matrix according to invkin function
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
        [q, matrices] = invkin(dh_t, handx(loop_var,:), handro(loop_var,:), handra(loop_var,:), q);
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
% ansp(:,t_steps_swing+1) = [theta_J1_AEP; theta_J2_AEP; theta_J3_AEP];
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