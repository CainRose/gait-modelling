function [ansp, time_traj] = invkin_traj_gen_modfl(DH_mat, obj_ite_stride_time, design_var, stride_length)

beta1 = design_var(1);
AEP_L1_J1 = design_var(2);  AEP_L1_J2 = design_var(3);  AEP_L1_J3 = design_var(4);
obj_ite_step_length = beta1*stride_length;
y_net = obj_ite_step_length*design_var(5);
psi_net = design_var(6);
step_clearance = design_var(7);
y_speed_attack = design_var(8);
y_speed_take_off = design_var(9);
psi_speed_attack = design_var(10);
psi_speed_take_off = design_var(11);

beta2 = 1 - beta1;

 % duty factor * stride length
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% 
% uses states produced by setup_general, to find the phase oscillator
% trajectories that would relate to angular space
%
% uses the Stochastic Runge Kutta SRKII method for numerical integration
%
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% Step 1: Use forward kinematics function, to find out points. 
DH_temp = DH_mat;
DH_temp(1:3, 4) = [AEP_L1_J1; AEP_L1_J2;AEP_L1_J3];
[x_aep, y_aep, z_aep, rot_mat_aep] = forward_kin(DH_temp);
% x_pep = x_aep - abs(obj_ite_step_length);
y_pep = y_aep - y_net;
% z_pep = z_aep;
psi_aep = mod(atan2(rot_mat_aep(2,1),rot_mat_aep(1,1))+pi, 2*pi)-pi;
psi_pep = psi_aep - psi_net;

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% Step 2: Construct all trajectories
T_swing = obj_ite_stride_time*beta2; 
T_stance = obj_ite_stride_time*beta1;
speed = design_var(7)/obj_ite_stride_time;
[time_traj, x_traj, y_traj, psi_traj] = end_effector_traj(speed, x_aep, y_aep, y_pep, y_speed_take_off, y_speed_attack, psi_speed_take_off, psi_speed_attack, psi_aep, psi_net, step_clearance, T_swing, T_stance, obj_ite_step_length);

% keyboard();
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% Step 3: Use average of first two joint values, plus inverse kinematics 
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
