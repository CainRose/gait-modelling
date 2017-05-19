function [ansp] = invkin_traj_gen_modf(DH_mat, obj_ite_stride_time, design_var)
AEP_L1_J1 = design_var(1);  AEP_L1_J2 = design_var(2);  AEP_L1_J3 = design_var(3);
take_off_angle_toe = design_var(8);
attack_angle_toe = design_var(9);
take_off_angle_hip = design_var(10);
attack_angle_hip = design_var(11);

beta1 = design_var(16);
beta2 = 1 - beta1;
obj_ite_step_length = design_var(7);
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
x_pep = x_aep - abs(obj_ite_step_length);
y_pep = y_aep;
z_pep = z_aep;
rot_mat_pep = rot_mat_aep;
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% Step 2: Construct Cycloidal trajectory, flat trajectory
t_steps_stance = floor(obj_ite_stride_time*beta1/0.001);
t_steps_swing = floor(obj_ite_stride_time*beta2/0.001);
% keyboard();
% 2.1 : Bezier swing trajectory
[theta_sw, ~, ~] = Bezier_trajectory(0, 0, take_off_angle_toe, 0, -attack_angle_toe, 0, obj_ite_step_length, t_steps_swing);
swing_l = theta_sw(1,:);
swing_y = theta_sw(2,:);
% keyboard();
if beta1 > 0.5
    % 2.2.a : Straight line trajectory
    [theta_st, ~, ~] = Bezier_trajectory(0, 0, take_off_angle_hip, 0, -attack_angle_hip, 0, obj_ite_step_length, t_steps_stance);
    stance_l = theta_st(1,:);
    stance_y = zeros(1,length(theta_st));
else
    % 2.2.b : Bezier stance trajectory
    [theta_st, ~, ~] = Bezier_trajectory(0, 0, take_off_angle_hip, 0, -attack_angle_hip, 0, obj_ite_step_length, t_steps_stance);
    stance_l = theta_st(1,:);
    stance_y = -1.*theta_st(2,:);
end
% keyboard();
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% Step 3: Use average of first two joint values, plus inverse kinematics 
 % function to find out additional poses

 % 3.1: setting up invkin function inputs
ansp = zeros(3,t_steps_swing+t_steps_stance);
% Defining the various x positions
handx = zeros(t_steps_swing+t_steps_stance-1, 3);
handro = zeros(t_steps_swing+t_steps_stance-1, 3);
handra = zeros(t_steps_swing+t_steps_stance-1, 3);
% keyboard();
% x portion
temp_handx = zeros(1,t_steps_swing);
for i = 1:t_steps_swing
    temp_handx(i) = x_pep + (swing_l(i)/obj_ite_step_length)*(x_aep-x_pep);
end

handx(1:t_steps_swing, 1) = temp_handx;
clear temp_handx

temp_handx = zeros(1,t_steps_stance-1);
for i = 2:t_steps_stance
    temp_handx(i-1) = x_pep + (stance_l(i)/obj_ite_step_length)*(x_aep-x_pep);
end

handx((t_steps_swing+1):(t_steps_swing+t_steps_stance-1), 1) = fliplr(temp_handx);
clear temp_handx

% z portion
temp_handx = zeros(1,t_steps_swing);
for i = 1:t_steps_swing
    temp_handx(i) = z_pep + (swing_l(i)/obj_ite_step_length)*(z_aep-z_pep);
end

handx(1:t_steps_swing, 3) = temp_handx;
clear temp_handx

temp_handx = zeros(1,t_steps_stance-1);
for i = 2:t_steps_stance
    temp_handx(i-1) = z_pep + (stance_l(i)/obj_ite_step_length)*(z_aep-z_pep);
end

handx((t_steps_swing+1):(t_steps_swing+t_steps_stance-1), 3) = fliplr(temp_handx);
clear temp_handx

% y portion
handx(1:t_steps_swing, 2) = y_aep + swing_y(1:t_steps_swing);
%keyboard();
handx((t_steps_swing+1):(t_steps_swing+t_steps_stance-1), 2) = y_aep + stance_y(1:t_steps_stance-1);
%keyboard();
% Approach vector and orientation vector

for random_var = 1:(t_steps_swing+t_steps_stance-1)
    handra(random_var, :) = rot_mat_pep(1:3, 3);
    handro(random_var, :) = rot_mat_pep(1:3, 2);
end

% 3.2: manipulating handx, handro, and handra to match the order AEP -> SWM -> PEP -> STM -> AEP
handro(1:t_steps_swing, :) = flipud(handro(1:t_steps_swing,:));
handro(t_steps_swing+1:t_steps_swing+t_steps_stance-1,:) = flipud(handro(t_steps_swing+1:t_steps_swing+t_steps_stance-1,:));
handra(1:t_steps_swing, :) = flipud(handra(1:t_steps_swing,:));
handra(t_steps_swing+1:t_steps_swing+t_steps_stance-1,:) = flipud(handra(t_steps_swing+1:t_steps_swing+t_steps_stance-1,:));
handx(1:t_steps_swing, :) = flipud(handx(1:t_steps_swing,:));
handx(t_steps_swing+1:t_steps_swing+t_steps_stance-1,:) = flipud(handx(t_steps_swing+1:t_steps_swing+t_steps_stance-1,:));
%keyboard();


% display('just before solving invkin');
%keyboard();

q0 = [AEP_L1_J1; AEP_L1_J2; AEP_L1_J3]; % Initial guess for invkin function
dh_t = [DH_mat(1:3,2), DH_mat(1:3,1), DH_mat(1:3,4), DH_mat(1:3,3), [0;0;0]]; % append the joint type coloumn, and reorder DH_mat according to invkin function
% %keyboard();
% 3.3: calling invkin and pose initialization for CPG
[q, matrices] = invkin(dh_t, handx(1,:), handro(1,:), handra(1,:), q0);
    if isempty(q)            
        ansp = [];
        return
    end
ansp(:,t_steps_swing) =  q(1:3);
%keyboard();
for i = 1:(t_steps_swing-1)

    [q, matrices] = invkin(dh_t, handx(i+1,:), handro(i+1,:), handra(i+1,:), q);
        if isempty(q)            
            ansp = [];
            return
        end
        ansp(:,t_steps_swing-i) =  q(1:3);
%     %keyboard();
end
ansp(:,t_steps_swing+1) = [AEP_L1_J1; AEP_L1_J2; AEP_L1_J3];
for i = t_steps_swing+2:t_steps_swing+t_steps_stance

    [q, matrices] = invkin(dh_t, handx(i-1,:), handro(i-1,:), handra(i-1,:), q);
        if isempty(q)            
            ansp = [];
            return
        end
        ansp(:,(t_steps_swing+t_steps_stance+t_steps_swing+2)-i) =  q(1:3);
%     %keyboard();
end

ansp = mod((ansp)+ pi, 2*pi) - pi;       % changing interval
% display('everything solved')
%keyboard();
% ansp now is a 3 rows by n coloumn matrix


end

