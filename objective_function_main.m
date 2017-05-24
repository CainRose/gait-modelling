function [OF, cons] = objective_function_main(design_var)
tic;
%% OBJECTIVE_FUNCTION_MAIN 
% Provides the attribute values (OF) evaluated for the 28 design variables (design_var)
%                        
% % design_var = [stride length     #1
%                 phase_FR          #2
%                 phase_HL          #3
%                 phase_HR          #4
%                 
%                 beta_FL           #5
%                 theta1_AEP_FL     #6
%                 theta2_AEP_FL     #7
%                 theta3_AEP_FL     #8
%                 delta_z_FL        #9
%                 delta_psi_FL      #10
%                 Fc_FL             #11
%                 zdot_AEP_FL       #12
%                 zdot_PEP_FL       #13
%                 psidot_AEP_FL     #14
%                 psidot_PEP_FL     #15
%                 Kp_1_sw_FL        #16
%                 Kd_1_sw_FL        #17
%                 Kp_2_sw_FL        #18
%                 Kd_2_sw_FL        #19
%                 Kp_3_sw_FL        #20
%                 Kd_3_sw_FL        #21
%                 Kp_1_st_FL        #22
%                 Kd_1_st_FL        #23
%                 Kp_2_st_FL        #24
%                 Kd_2_st_FL        #25
%                 Kp_3_st_FL        #26
%                 Kd_3_st_FL        #27
%                 
%                 beta_FR           #28
%                 theta1_AEP_FR     #29
%                 theta2_AEP_FR     #30
%                 theta3_AEP_FR     #31
%                 delta_z_FR        #32
%                 delta_psi_FR      #33
%                 Fc_FR             #34
%                 zdot_AEP_FR       #35
%                 zdot_PEP_FR       #36
%                 psidot_AEP_FR     #37
%                 psidot_PEP_FR     #38
%                 Kp_1_sw_FR        #39
%                 Kd_1_sw_FR        #40
%                 Kp_2_sw_FR        #41
%                 Kd_2_sw_FR        #42
%                 Kp_3_sw_FR        #43
%                 Kd_3_sw_FR        #44
%                 Kp_1_st_FR        #45
%                 Kd_1_st_FR        #46
%                 Kp_2_st_FR        #47
%                 Kd_2_st_FR        #48
%                 Kp_3_st_FR        #49
%                 Kd_3_st_FR        #50
%                 
%                 beta_HL           #51
%                 theta1_AEP_HL     #52
%                 theta2_AEP_HL     #53
%                 theta3_AEP_HL     #54
%                 delta_z_HL        #55
%                 delta_psi_HL      #56
%                 Fc_HL             #57
%                 zdot_AEP_HL       #58
%                 zdot_PEP_HL       #59
%                 psidot_AEP_HL     #60
%                 psidot_PEP_HL     #61
%                 Kp_1_sw_HL        #62
%                 Kd_1_sw_HL        #63
%                 Kp_2_sw_HL        #64
%                 Kd_2_sw_HL        #65
%                 Kp_3_sw_HL        #66
%                 Kd_3_sw_HL        #67
%                 Kp_1_st_HL        #68
%                 Kd_1_st_HL        #69
%                 Kp_2_st_HL        #70
%                 Kd_2_st_HL        #71
%                 Kp_3_st_HL        #72
%                 Kd_3_st_HL        #73
%                 
%                 beta_HR           #74
%                 theta1_AEP_HR     #75
%                 theta2_AEP_HR     #76
%                 theta3_AEP_HR     #77
%                 delta_z_HR        #78
%                 delta_psi_HR      #79
%                 Fc_HR             #80
%                 zdot_AEP_HR       #81
%                 zdot_PEP_HR       #82
%                 psidot_AEP_HR     #83
%                 psidot_PEP_HR     #84
%                 Kp_1_sw_HR        #85
%                 Kd_1_sw_HR        #86
%                 Kp_2_sw_HR        #87
%                 Kd_2_sw_HR        #88
%                 Kp_3_sw_HR        #89
%                 Kd_3_sw_HR        #90
%                 Kp_1_st_HR        #91
%                 Kd_1_st_HR        #92
%                 Kp_2_st_HR        #93
%                 Kd_2_st_HR        #94
%                 Kp_3_st_HR        #95
%                 Kd_3_st_HR        #96
                
% OF(1) = energy_density;
% cons = [c1 c2 c3 c4 c5 c6 c7]
% Default value is 0, constraint violation value is 1 
%
% PRE SIMULATION CONSTRAAINTS
% c1 and c2 => Represent constraints regarding pre-simulation failures,
%                 c1) Inverse kinematics failed to get trajectories
%                 c2) Failed to build accelerator model
%
% STOP SIMULATION CONSTRAINTS
% c3 to c4 => Represents the constraint regarding a simulation stop... This happens if either
%                 c3) Maximum Pitch exceeded, 
%                 c4) Maximum height exceeded or height is negative 
%
%                 In this case, simulation_time = time_end
% POST SIMULATION CONSTRAINTS
% c5 to c7 => Represents the constraint regarding stability failures... This happens if either
%                 c5) Maximum Torque exceeded
%                 c6) velocity of toe or height of toe during stance is
%                 non-zero
%                 c7) any pose variable differs from it's initial value at
%                 the end of 1 complete cycle - cycle dependent on stride
%                 time - not a stable cyclic gait    
%
%%
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Section 1: DEFINING POSES IN TERMS OF INPUT VECTOR COMPONENTS
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

scale = 1;
HLL = scale*0.33;
FLL = scale*0.33;
IGD = scale*0.33;
body_pitch_max = pi/8;
body_roll_max = pi/6; 
body_yaw_max = pi/6;
ground_x = 24;
ground_y = 4;
ground_z = 0.1;
front_x = IGD/2;      
back_x = -IGD/2; 
trans_leg_1_rot = [1 0 0; 0 1 0; 0 0 1];
bx =IGD; by =0.5*0.5*IGD; bz =0.25*IGD;
obj_ite_max_torque = 50;
obj_ite_max_torque_change = 50;
density = 1408;
gravity = -9.81;
k_g_v = 7.21e7; n = 2.31;   b_g_v = 3.8e4;  p = 1.1;    q = 1;  S_p = 0.001;    fk = 0.6;

lg1ps = [FLL*0.43, 0, 0, 0; ...
         FLL*0.40, 0, 0, 0;...
         FLL*0.17, 0, 0, 0];
lg2ps = [FLL*0.43, 0, 0, 0; ...
         FLL*0.40, 0, 0, 0;...
         FLL*0.17, 0, 0, 0];
lg3ps = [FLL*0.39, 0, 0, 0; ...
         FLL*0.36, 0, 0, 0;...
         FLL*0.25, 0, 0, 0];
lg4ps = [FLL*0.39, 0, 0, 0; ...
         FLL*0.36, 0, 0, 0;...
         FLL*0.25, 0, 0, 0];     

for i = 1:3
	t_mat_lg1(:,:,i) = transformation_mat(lg1ps(i,:));     
    t_mat_lg2(:,:,i) = transformation_mat(lg2ps(i,:));     
    t_mat_lg3(:,:,i) = transformation_mat(lg3ps(i,:));     
    t_mat_lg4(:,:,i) = transformation_mat(lg4ps(i,:));              
end

leg_length = lg1ps(1,1)+lg1ps(2,1)+lg1ps(3,1);

M1 = [(0.15*lg1ps(1,1))/leg_length;...
      (0.15*lg1ps(2,1))/leg_length;...
      (0.15*lg1ps(3,1))/leg_length];
M2 = [(0.15*lg2ps(1,1))/leg_length;...
      (0.15*lg2ps(2,1))/leg_length;...
      (0.15*lg2ps(3,1))/leg_length];
M3 = [(0.15*lg3ps(1,1))/leg_length;...
      (0.15*lg3ps(2,1))/leg_length;...
      (0.15*lg3ps(3,1))/leg_length];
M4 = [(0.15*lg4ps(1,1))/leg_length;...
      (0.15*lg4ps(2,1))/leg_length;...
      (0.15*lg4ps(3,1))/leg_length];

%-------------------------------------------------------------------------%
L1 = [0.5*sqrt((lg1ps(1,1)^2)+(lg1ps(1,3)^2));...     % Location of centre of mass in front leg 1
    0.5*sqrt((lg1ps(2,1)^2)+(lg1ps(2,3)^2));...
    0.5*sqrt((lg1ps(3,1)^2)+(lg1ps(3,3)^2))...
    ];

l1 = [L1(1) sqrt((lg1ps(1,1))^2+(lg1ps(1,3))^2)-L1(1);...
      L1(2) sqrt((lg1ps(2,1))^2+(lg1ps(2,3))^2)-L1(2);...
      L1(3) sqrt((lg1ps(3,1))^2+(lg1ps(3,3))^2)-L1(3);...
      ];
     
R1 = [sqrt((M1(1))/(pi*density*(l1(1,1)+((l1(1,2))*(((l1(1,1))/(l1(1,2)))^(1.5))))));...
      sqrt((M1(2))/(pi*density*(l1(2,1)+((l1(2,2))*(((l1(2,1))/(l1(2,2)))^(1.5))))));...
      sqrt((M1(3))/(pi*density*(l1(3,1)+((l1(3,2))*(((l1(3,1))/(l1(3,2)))^(1.5))))));...
      ];

r1 = [R1(1) R1(1)*(((l1(1,1))/(l1(1,2)))^1.5);...
      R1(2) R1(2)*(((l1(2,1))/(l1(2,2)))^1.5);...
      R1(3) R1(3)*(((l1(3,1))/(l1(3,2)))^1.5);...
      ];
%-------------------------------------------------------------------------%
L2 = [0.5*sqrt((lg2ps(1,1)^2)+(lg2ps(1,3)^2));...     % Location of centre of mass in front leg 2
    0.5*sqrt((lg2ps(2,1)^2)+(lg2ps(2,3)^2));...
    0.5*sqrt((lg2ps(3,1)^2)+(lg2ps(3,3)^2));...
    ];

l2 = [L2(1) sqrt((lg2ps(1,1))^2+(lg2ps(1,3))^2)-L2(1);...
      L2(2) sqrt((lg2ps(2,1))^2+(lg2ps(2,3))^2)-L2(2);...
      L2(3) sqrt((lg2ps(3,1))^2+(lg2ps(3,3))^2)-L2(3);...
      ];
     
R2 = [sqrt((M2(1))/(pi*density*(l2(1,1)+((l2(1,2))*(((l2(1,1))/(l2(1,2)))^(1.5))))));...
      sqrt((M2(2))/(pi*density*(l2(2,1)+((l2(2,2))*(((l2(2,1))/(l2(2,2)))^(1.5))))));...
      sqrt((M2(3))/(pi*density*(l2(3,1)+((l2(3,2))*(((l2(3,1))/(l2(3,2)))^(1.5))))));...
      ];

r2 = [R2(1) R2(1)*(((l2(1,1))/(l2(1,2)))^1.5);...
      R2(2) R2(2)*(((l2(2,1))/(l2(2,2)))^1.5);...
      R2(3) R2(3)*(((l2(3,1))/(l2(3,2)))^1.5);...
      ];
%-------------------------------------------------------------------------%  
L3 = [0.5*sqrt((lg3ps(1,1)^2)+(lg3ps(1,3)^2));...     % Location of centre of mass in front leg 3
    0.5*sqrt((lg3ps(2,1)^2)+(lg3ps(2,3)^2));...
    0.5*sqrt((lg3ps(3,1)^2)+(lg3ps(3,3)^2));...
    ];

l3 = [L3(1) sqrt((lg3ps(1,1))^2+(lg3ps(1,3))^2)-L3(1);...
      L3(2) sqrt((lg3ps(2,1))^2+(lg3ps(2,3))^2)-L3(2);...
      L3(3) sqrt((lg3ps(3,1))^2+(lg3ps(3,3))^2)-L3(3);...
      ];
     
R3 = [sqrt((M3(1))/(pi*density*(l3(1,1)+((l3(1,2))*(((l3(1,1))/(l3(1,2)))^(1.5))))));...
      sqrt((M3(2))/(pi*density*(l3(2,1)+((l3(2,2))*(((l3(2,1))/(l3(2,2)))^(1.5))))));...
      sqrt((M3(3))/(pi*density*(l3(3,1)+((l3(3,2))*(((l3(3,1))/(l3(3,2)))^(1.5))))));...
      ];

r3 = [R3(1) R3(1)*(((l3(1,1))/(l3(1,2)))^1.5);...
      R3(2) R3(2)*(((l3(2,1))/(l3(2,2)))^1.5);...
      R3(3) R3(3)*(((l3(3,1))/(l3(3,2)))^1.5);...
      ];
%-------------------------------------------------------------------------%  
L4 = [0.5*sqrt((lg4ps(1,1)^2)+(lg4ps(1,3)^2));...     % Location of centre of mass in front leg 4
    0.5*sqrt((lg4ps(2,1)^2)+(lg4ps(2,3)^2));...
    0.5*sqrt((lg4ps(3,1)^2)+(lg4ps(3,3)^2));...
    ];

l4 = [L4(1) sqrt((lg4ps(1,1))^2+(lg4ps(1,3))^2)-L4(1);...
      L4(2) sqrt((lg4ps(2,1))^2+(lg4ps(2,3))^2)-L4(2);...
      L4(3) sqrt((lg4ps(3,1))^2+(lg4ps(3,3))^2)-L4(3);...
      ];
     
R4 = [sqrt((M4(1))/(pi*density*(l4(1,1)+((l4(1,2))*(((l4(1,1))/(l4(1,2)))^(1.5))))));...
      sqrt((M4(2))/(pi*density*(l4(2,1)+((l4(2,2))*(((l4(2,1))/(l4(2,2)))^(1.5))))));...
      sqrt((M4(3))/(pi*density*(l4(3,1)+((l4(3,2))*(((l4(3,1))/(l4(3,2)))^(1.5))))));...
      ];

r4 = [R4(1) R4(1)*(((l4(1,1))/(l4(1,2)))^1.5);...
      R4(2) R4(2)*(((l4(2,1))/(l4(2,2)))^1.5);...
      R4(3) R4(3)*(((l4(3,1))/(l4(3,2)))^1.5);...
      ];    

     
OF = 100000; % Represents a generic value that the members have... multiplied by 10 to represent the worst member
cons = [0 0 0 0 0 0 0 0 0]; % The 8 constraint values. Look at description above

obj_ite_velocity = 0.5;
%--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Design Variable initialization to global variables
obj_ite_stride_length = design_var(1); % Stride length 
obj_ite_duty_factor_FL = design_var(5);
obj_ite_duty_factor_FR = design_var(28);
obj_ite_duty_factor_HL = design_var(51);
obj_ite_duty_factor_HR = design_var(74);

obj_ite_phi_rel_FR = design_var(2);
obj_ite_phi_rel_HL = design_var(3);
obj_ite_phi_rel_HR = design_var(4);

num_strides = 3;
% %%%%%%%%%%%keyboard();
obj_ite_stride_time = floor(100*obj_ite_stride_length/obj_ite_velocity)/100;
obj_ite_simulation_time = floor(100*num_strides*obj_ite_stride_time)/100;
% obj_ite_velocity = floor(100*obj_ite_stride_length/obj_ite_stride_time)/100;
velocity_initial = obj_ite_velocity;
obj_ite_step_length_FL = obj_ite_duty_factor_FL*obj_ite_stride_length;
obj_ite_step_length_FR = obj_ite_duty_factor_FR*obj_ite_stride_length;
obj_ite_step_length_HL = obj_ite_duty_factor_HL*obj_ite_stride_length;
obj_ite_step_length_HR = obj_ite_duty_factor_HR*obj_ite_stride_length;
KPFLsw1 = design_var(16); KPFLsw2 = design_var(18); KPFLsw3 = design_var(20); 
KPFLst1 = design_var(22); KPFLst2 = design_var(24); KPFLst3 = design_var(26); 
KDFLsw1 = design_var(17); KDFLsw2 = design_var(19); KDFLsw3 = design_var(21); 
KDFLst1 = design_var(23); KDFLst2 = design_var(25); KDFLst3 = design_var(27);

KPFRsw1 = design_var(39); KPFRsw2 = design_var(41); KPFRsw3 = design_var(43); 
KPFRst1 = design_var(45); KPFRst2 = design_var(47); KPFRst3 = design_var(49); 
KDFRsw1 = design_var(40); KDFRsw2 = design_var(42); KDFRsw3 = design_var(44); 
KDFRst1 = design_var(46); KDFRst2 = design_var(48); KDFRst3 = design_var(50);

KPHLsw1 = design_var(62); KPHLsw2 = design_var(64); KPHLsw3 = design_var(66); 
KPHLst1 = design_var(68); KPHLst2 = design_var(70); KPHLst3 = design_var(72); 
KDHLsw1 = design_var(63); KDHLsw2 = design_var(65); KDHLsw3 = design_var(67); 
KDHLst1 = design_var(69); KDHLst2 = design_var(71); KDHLst3 = design_var(73);

KPHRsw1 = design_var(85); KPHRsw2 = design_var(87); KPHRsw3 = design_var(89); 
KPHRst1 = design_var(91); KPHRst2 = design_var(93); KPHRst3 = design_var(95); 
KDHRsw1 = design_var(86); KDHRsw2 = design_var(88); KDHRsw3 = design_var(90); 
KDHRst1 = design_var(92); KDHRst2 = design_var(94); KDHRst3 = design_var(96);

%--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
%%
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Section 2: CPG TRAJECTORIES BASED ON POSES
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
extra_flag = 0;
[~, y1, ~] = leg_stance_plotter(lg1ps, (design_var(6:8))');
[~, y2, ~] = leg_stance_plotter(lg2ps, (design_var(29:31))');
[~, y3, ~] = leg_stance_plotter(lg3ps, (design_var(52:54))');
[~, y4, ~] = leg_stance_plotter(lg4ps, (design_var(75:77))');
if (abs(y1(2)) < abs(y1(1)))||(abs(y1(3)) < abs(y1(2)))||(abs(y1(4)) < abs(y1(3)))||(abs(y2(2)) < abs(y2(1)))||(abs(y2(3)) < abs(y2(2)))||(abs(y2(4)) < abs(y2(3)))||(abs(y3(2)) < abs(y3(1)))||(abs(y3(3)) < abs(y3(2)))||(abs(y3(4)) < abs(y3(3)))||(abs(y4(2)) < abs(y4(1)))||(abs(y4(3)) < abs(y4(2)))||(abs(y4(4)) < abs(y4(3)))
    extra_flag = 1;
end
ansp_ans_FL = [];
ansp_ans_FR = [];
ansp_ans_HL = [];
ansp_ans_HR = [];

% Create joint trajectories for one cycle
if ~extra_flag
    DH_mat = lg1ps;
    [ansp_ans_FL, time_traj] = invkin_traj_gen_modfl(DH_mat, obj_ite_stride_time, design_var(5:15), obj_ite_stride_length);
    DH_mat = lg2ps;
    ansp_ans_FR = invkin_traj_gen_modfr(DH_mat, obj_ite_stride_time, design_var(28:38), obj_ite_stride_length);
    DH_mat = lg3ps;
    ansp_ans_HL = invkin_traj_gen_modhl(DH_mat, obj_ite_stride_time, design_var(51:61), obj_ite_stride_length);
    DH_mat = lg4ps;
    ansp_ans_HR = invkin_traj_gen_modhr(DH_mat, obj_ite_stride_time, design_var(74:84), obj_ite_stride_length);
end
        index_ini = 0;
%         design_var(1)

        %%%%%%%%%%%keyboard();
        if extra_flag
            display('Constraint violation - infeasible joint angles')
            cons(1) = 1;
        elseif (isempty(ansp_ans_FL))||(isempty(ansp_ans_FR))||(isempty(ansp_ans_HR))||(isempty(ansp_ans_HL)) % c1 violated - can't proceed           
            display('Constraint violation - inv kin did not converge')
            cons(1) = 1;    
            %%%keyboard();
        elseif (length(ansp_ans_FL) ~= length(ansp_ans_FR))||(length(ansp_ans_FL) ~= length(ansp_ans_HR))||(length(ansp_ans_FL) ~= length(ansp_ans_HL))
            display('Constraint violation - trajectories generated not equal in length')
            cons(1) = 1;
        else % c1 not violated - proceed
            % Extend created joint trajectories to multiple cycles based on
            % total desired simulation time and adjust according to
            % relative phases:
            
            % Create leg markers to identify stance(0), swing(1) and
            % AEP(-1)
            [~, length_should_be] = size(ansp_ans_FL);
            leg_marker_FL = zeros(1,length_should_be);
            leg_marker_FR = zeros(1,length_should_be);
            leg_marker_HL = zeros(1,length_should_be);
            leg_marker_HR = zeros(1,length_should_be);
            
            for i = 1:(floor((1-obj_ite_duty_factor_FL)*length_should_be))  
               leg_marker_FL(i) = 1;  % Marker has a 1 for swing phase and 0 for stance phase
            end
            for i = 1:(floor((1-obj_ite_duty_factor_FR)*length_should_be))  
               leg_marker_FR(i) = 1;  % Marker has a 1 for swing phase and 0 for stance phase
            end
            for i = 1:(floor((1-obj_ite_duty_factor_HL)*length_should_be))  
               leg_marker_HL(i) = 1;  % Marker has a 1 for swing phase and 0 for stance phase
            end
            for i = 1:(floor((1-obj_ite_duty_factor_HR)*length_should_be))  
               leg_marker_HR(i) = 1;  % Marker has a 1 for swing phase and 0 for stance phase
            end
            leg_marker_FL((floor((1-obj_ite_duty_factor_FL)*length_should_be))+1) = -1;
            leg_marker_FR((floor((1-obj_ite_duty_factor_FR)*length_should_be))+1) = -1;
            leg_marker_HL((floor((1-obj_ite_duty_factor_HL)*length_should_be))+1) = -1;
            leg_marker_HR((floor((1-obj_ite_duty_factor_HR)*length_should_be))+1) = -1;           
            
            % adjust trajectories w.r.t. their relative phases
            ansp_FL = ansp_ans_FL;
            [ansp_FL, leg_marker_FL] = modforleg(ansp_ans_FL, leg_marker_FL, 1, num_strides);
            [ansp_FR, leg_marker_FR] = modforleg(ansp_ans_FR, leg_marker_FR, obj_ite_phi_rel_FR, num_strides);
            [ansp_HL, leg_marker_HL] = modforleg(ansp_ans_HL, leg_marker_HL, obj_ite_phi_rel_HL, num_strides);
            [ansp_HR, leg_marker_HR] = modforleg(ansp_ans_HR, leg_marker_HR, obj_ite_phi_rel_HR, num_strides);
%             %%%keyboard();
            time_tempo = time_traj;
            for num = 1:num_strides-1
                time_tempo = time_tempo + obj_ite_stride_time;
                time_traj = [time_traj(:,1:end-1), time_tempo];
            end            
%             %%%keyboard();
            % need to find initialization point
            index_ini = 0; % non-existant index
            for counter = 1:length_should_be
                if ((leg_marker_FL(counter) == 0)||(leg_marker_FL(counter) == -1)||(leg_marker_FR(counter) == 0)||(leg_marker_FR(counter) == -1))&&((leg_marker_HL(counter) == 0)||(leg_marker_HL(counter) == -1)||(leg_marker_HR(counter) == 0)||(leg_marker_HR(counter) == -1))
                    index_ini = counter;
                    break
                end
            end
        end
            
           
            
        if(index_ini == 0)
            % if index_ini is zero = such an initialization point does not
            % exist, and thus the solution is infeasible.
            display('Constraint violation - initialization point does not exist')
            cons(1) = 1;
        else
            cons(1) = 0;
            % if index_ini is non-zero, adjust the trajectories, such that
            % the simulation starts from this initialization point. 
            [ansp_FL, leg_marker_FL] = modforleg1(ansp_FL, leg_marker_FL, abs(index_ini/length_should_be));
            [ansp_FR, leg_marker_FR] = modforleg1(ansp_FR, leg_marker_FR, abs(index_ini/length_should_be));
            [ansp_HL, leg_marker_HL] = modforleg1(ansp_HL, leg_marker_HL, abs(index_ini/length_should_be));
            [ansp_HR, leg_marker_HR] = modforleg1(ansp_HR, leg_marker_HR, abs(index_ini/length_should_be)); 
            
            % Variables to be used by Simulink/Simmechanics model
            TRJ_LG1_J1  = [time_traj; ansp_FL(1,:)]; 
            TRJ_LG1_J2  = [time_traj; ansp_FL(2,:)];
            TRJ_LG1_J3  = [time_traj; ansp_FL(3,:)];

            TRJ_LG2_J1  = [time_traj; ansp_FR(1,:)]; 
            TRJ_LG2_J2  = [time_traj; ansp_FR(2,:)];
            TRJ_LG2_J3  = [time_traj; ansp_FR(3,:)];

            TRJ_LG3_J1  = [time_traj; ansp_HL(1,:)]; 
            TRJ_LG3_J2  = [time_traj; ansp_HL(2,:)];
            TRJ_LG3_J3  = [time_traj; ansp_HL(3,:)];

            TRJ_LG4_J1  = [time_traj; ansp_HR(1,:)]; 
            TRJ_LG4_J2  = [time_traj; ansp_HR(2,:)];
            TRJ_LG4_J3  = [time_traj; ansp_HR(3,:)];
            
            
            TRJ_LG1_J1_TIMESERIES = timeseries(TRJ_LG1_J1(2,:).', (TRJ_LG1_J1(1,:)), 'name', 'TRJ_LG4_J1');
            TRJ_LG1_J2_TIMESERIES = timeseries(TRJ_LG1_J2(2,:).', (TRJ_LG1_J2(1,:)), 'name', 'TRJ_LG4_J1');
            TRJ_LG1_J3_TIMESERIES = timeseries(TRJ_LG1_J3(2,:).', (TRJ_LG1_J3(1,:)), 'name', 'TRJ_LG4_J1');
            
            TRJ_LG2_J1_TIMESERIES = timeseries(TRJ_LG2_J1(2,:).', (TRJ_LG2_J1(1,:)), 'name', 'TRJ_LG4_J1');
            TRJ_LG2_J2_TIMESERIES = timeseries(TRJ_LG2_J2(2,:).', (TRJ_LG2_J2(1,:)), 'name', 'TRJ_LG4_J1');
            TRJ_LG2_J3_TIMESERIES = timeseries(TRJ_LG2_J3(2,:).', (TRJ_LG2_J3(1,:)), 'name', 'TRJ_LG4_J1');
            
            TRJ_LG3_J1_TIMESERIES = timeseries(TRJ_LG3_J1(2,:).', (TRJ_LG3_J1(1,:)), 'name', 'TRJ_LG4_J1');
            TRJ_LG3_J2_TIMESERIES = timeseries(TRJ_LG3_J2(2,:).', (TRJ_LG3_J2(1,:)), 'name', 'TRJ_LG4_J1');
            TRJ_LG3_J3_TIMESERIES = timeseries(TRJ_LG3_J3(2,:).', (TRJ_LG3_J3(1,:)), 'name', 'TRJ_LG4_J1');
            
            TRJ_LG4_J1_TIMESERIES = timeseries(TRJ_LG4_J1(2,:).', (TRJ_LG4_J1(1,:)), 'name', 'TRJ_LG4_J1');
            TRJ_LG4_J2_TIMESERIES = timeseries(TRJ_LG4_J2(2,:).', (TRJ_LG4_J2(1,:)), 'name', 'TRJ_LG4_J1');
            TRJ_LG4_J3_TIMESERIES = timeseries(TRJ_LG4_J3(2,:).', (TRJ_LG4_J3(1,:)), 'name', 'TRJ_LG4_J1');
            
    %%%%%%%%%%%keyboard();

            % FINDING INITIAL BODY HEIGHT AND INITIAL PITCH ANGLE 
            % Case 1:
%             if ((obj_ite_phi_rel_fb)<=(obj_ite_duty_factor))||(obj_ite_duty_factor>=0.5)
            
                DH_temp = lg1ps;
                DH_temp(1:3,4) = [TRJ_LG1_J1(2,1);TRJ_LG1_J2(2,1);TRJ_LG1_J3(2,1)]; % leg 1 is a PEP
                [xFR,zFR,~,~] = forward_kin(DH_temp);    
                DH_temp = lg2ps;
                DH_temp(1:3,4) = [TRJ_LG2_J1(2,1);TRJ_LG2_J2(2,1);TRJ_LG2_J3(2,1)]; % leg 1 is a PEP
                [xFL,zFL,~,~] = forward_kin(DH_temp);
                DH_temp = lg3ps;
                DH_temp(1:3,4) = [TRJ_LG3_J1(2,1);TRJ_LG3_J2(2,1);TRJ_LG3_J3(2,1)]; % leg 1 is a PEP
                [xHR,zHR,~,~] = forward_kin(DH_temp);                
                DH_temp = lg4ps;
                DH_temp(1:3,4) = [TRJ_LG4_J1(2,1);TRJ_LG4_J2(2,1);TRJ_LG4_J3(2,1)]; % leg 4
                [xHL,zHL,~,~] = forward_kin(DH_temp);                
                
                [body_height, pitch_initial] = select_toes(xFR, zFR, xFL, zFL, xHR, zHR, xHL, zHL, IGD);     

%             body_height = body_height + 0.5; % this is to be used to
%             % suspend the rover above ground... change planar joint in
%             % rover model to a revolute/weld joint, and corresponding
%             % transform sensor for body_movement_check block

            % Save created trajectories into files for use by model's rapid
            % accelerator build
            save('TRJ_LG1_J1.mat', 'TRJ_LG1_J1'); save('TRJ_LG1_J2.mat', 'TRJ_LG1_J2'); save('TRJ_LG1_J3.mat', 'TRJ_LG1_J3');
            save('TRJ_LG2_J1.mat', 'TRJ_LG2_J1'); save('TRJ_LG2_J2.mat', 'TRJ_LG2_J2'); save('TRJ_LG2_J3.mat', 'TRJ_LG2_J3');
            save('TRJ_LG3_J1.mat', 'TRJ_LG3_J1'); save('TRJ_LG3_J2.mat', 'TRJ_LG3_J2'); save('TRJ_LG3_J3.mat', 'TRJ_LG3_J3');
            save('TRJ_LG4_J1.mat', 'TRJ_LG4_J1'); save('TRJ_LG4_J2.mat', 'TRJ_LG4_J2'); save('TRJ_LG4_J3.mat', 'TRJ_LG4_J3');
            
%             %%
            % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
            % Section 3: Running model and evaluating objective functions
            % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
            
            mws = get_param('final_model_trial2', 'modelworkspace');
            
            % Timeseries trajectories
            mws.assignin('TRJ_LG1_J1_TIMESERIES', TRJ_LG1_J1_TIMESERIES); mws.assignin('TRJ_LG1_J2_TIMESERIES', TRJ_LG1_J2_TIMESERIES); mws.assignin('TRJ_LG1_J3_TIMESERIES', TRJ_LG1_J3_TIMESERIES);
            mws.assignin('TRJ_LG2_J1_TIMESERIES', TRJ_LG2_J1_TIMESERIES); mws.assignin('TRJ_LG2_J2_TIMESERIES', TRJ_LG2_J2_TIMESERIES); mws.assignin('TRJ_LG2_J3_TIMESERIES', TRJ_LG2_J3_TIMESERIES);
            mws.assignin('TRJ_LG3_J1_TIMESERIES', TRJ_LG3_J1_TIMESERIES); mws.assignin('TRJ_LG3_J2_TIMESERIES', TRJ_LG3_J2_TIMESERIES); mws.assignin('TRJ_LG3_J3_TIMESERIES', TRJ_LG3_J3_TIMESERIES);
            mws.assignin('TRJ_LG4_J1_TIMESERIES', TRJ_LG4_J1_TIMESERIES); mws.assignin('TRJ_LG4_J2_TIMESERIES', TRJ_LG4_J2_TIMESERIES); mws.assignin('TRJ_LG4_J3_TIMESERIES', TRJ_LG4_J3_TIMESERIES);
            
            % Trajectories
            mws.assignin('TRJ_LG1_J1', TRJ_LG1_J1); mws.assignin('TRJ_LG1_J2', TRJ_LG1_J2); mws.assignin('TRJ_LG1_J3', TRJ_LG1_J3);
            mws.assignin('TRJ_LG2_J1', TRJ_LG2_J1); mws.assignin('TRJ_LG2_J2', TRJ_LG2_J2); mws.assignin('TRJ_LG2_J3', TRJ_LG2_J3);
            mws.assignin('TRJ_LG3_J1', TRJ_LG3_J1); mws.assignin('TRJ_LG3_J2', TRJ_LG3_J2); mws.assignin('TRJ_LG3_J3', TRJ_LG3_J3);
            mws.assignin('TRJ_LG4_J1', TRJ_LG4_J1); mws.assignin('TRJ_LG4_J2', TRJ_LG4_J2); mws.assignin('TRJ_LG4_J3', TRJ_LG4_J3);
            
            % Gains
            mws.assignin('KPFLsw1', KPFLsw1); mws.assignin('KPFLsw2', KPFLsw2); mws.assignin('KPFLsw3', KPFLsw3);
            mws.assignin('KPFLst1', KPFLst1); mws.assignin('KPFLst2', KPFLst2); mws.assignin('KPFLst3', KPFLst3);
            mws.assignin('KDFLsw1', KDFLsw1); mws.assignin('KDFLsw2', KDFLsw2); mws.assignin('KDFLsw3', KDFLsw3);
            mws.assignin('KDFLst1', KDFLst1); mws.assignin('KDFLst2', KDFLst2); mws.assignin('KDFLst3', KDFLst3);
            
            mws.assignin('KPFRsw1', KPFRsw1); mws.assignin('KPFRsw2', KPFRsw2); mws.assignin('KPFRsw3', KPFRsw3);
            mws.assignin('KPFRst1', KPFRst1); mws.assignin('KPFRst2', KPFRst2); mws.assignin('KPFRst3', KPFRst3);
            mws.assignin('KDFRsw1', KDFRsw1); mws.assignin('KDFRsw2', KDFRsw2); mws.assignin('KDFRsw3', KDFRsw3);
            mws.assignin('KDFRst1', KDFRst1); mws.assignin('KDFRst2', KDFRst2); mws.assignin('KDFRst3', KDFRst3);
            
            mws.assignin('KPHLsw1', KPHLsw1); mws.assignin('KPHLsw2', KPHLsw2); mws.assignin('KPHLsw3', KPHLsw3);
            mws.assignin('KPHLst1', KPHLst1); mws.assignin('KPHLst2', KPHLst2); mws.assignin('KPHLst3', KPHLst3);
            mws.assignin('KDHLsw1', KDHLsw1); mws.assignin('KDHLsw2', KDHLsw2); mws.assignin('KDHLsw3', KDHLsw3);
            mws.assignin('KDHLst1', KDHLst1); mws.assignin('KDHLst2', KDHLst2); mws.assignin('KDHLst3', KDHLst3);
            
            mws.assignin('KPHRsw1', KPHRsw1); mws.assignin('KPHRsw2', KPHRsw2); mws.assignin('KPHRsw3', KPHRsw3);
            mws.assignin('KPHRst1', KPHRst1); mws.assignin('KPHRst2', KPHRst2); mws.assignin('KPHRst3', KPHRst3);
            mws.assignin('KDHRsw1', KDHRsw1); mws.assignin('KDHRsw2', KDHRsw2); mws.assignin('KDHRsw3', KDHRsw3);
            mws.assignin('KDHRst1', KDHRst1); mws.assignin('KDHRst2', KDHRst2); mws.assignin('KDHRst3', KDHRst3);
            
            % Body Variables
            mws.assignin('body_height', body_height); mws.assignin('pitch_initial', pitch_initial); mws.assignin('velocity_initial', velocity_initial);
            mws.assignin('t_mat_lg1', t_mat_lg1); mws.assignin('t_mat_lg2', t_mat_lg2); mws.assignin('t_mat_lg3', t_mat_lg3); mws.assignin('t_mat_lg4', t_mat_lg4);
            mws.assignin('body_pitch_max', body_pitch_max); mws.assignin('body_roll_max', body_roll_max); mws.assignin('body_yaw_max', body_yaw_max); 
            
            mws.assignin('l1', l1); mws.assignin('l2', l2); mws.assignin('l3', l3); mws.assignin('l4', l4); 
            mws.assignin('r1', r1); mws.assignin('r2', r2); mws.assignin('r3', r3); mws.assignin('r4', r4); 
            mws.assignin('L1', L1); mws.assignin('L2', L2); mws.assignin('L3', L3); mws.assignin('L4', L4); 
            mws.assignin('R1', R1); mws.assignin('R2', R2); mws.assignin('R3', R3); mws.assignin('R4', R4); 
            
            % others
            mws.assignin('lg1ps', lg1ps); mws.assignin('lg2ps', lg2ps); mws.assignin('lg3ps', lg3ps); mws.assignin('lg4ps', lg4ps); 
            mws.assignin('gravity', gravity); mws.assignin('ground_x', ground_x); mws.assignin('ground_y', ground_y); mws.assignin('ground_z', ground_z);
            mws.assignin('back_x', back_x); mws.assignin('front_x', front_x); mws.assignin('density', density);
            mws.assignin('HLL', HLL); mws.assignin('FLL', FLL); mws.assignin('IGD', IGD); 
            mws.assignin('bx', bx); mws.assignin('by', by); mws.assignin('bz', bz);
            mws.assignin('k_g_v', k_g_v); mws.assignin('n', n); mws.assignin('b_g_v', b_g_v); mws.assignin('p', p); mws.assignin('q', q);
            mws.assignin('S_p', S_p); mws.assignin('fk', fk);
            
            % Try catch system to create exception for when model fails to
            % compile... treat is as a constraint violation...
%%%%%%%%%%%keyboard();
%%%keyboard();
%              try
                modelsim = sim('final_model_trial2.mdl', 'SimulationMode', 'normal', 'StartTime', '0', 'StopTime', num2str(obj_ite_simulation_time));
%              catch
%                  OF = 100000;
%                  cons(2) = 1;   
%              end
            %%%%%%%%%%keyboard();

            
            
            results_file_loader  % Reads data from files created by the executable above and stores them in function workspace

            temporary_time = modelsim.get('t_sen_lg1').time;
            
            t_sen_lg1_j1 = modelsim.get('t_sen_lg1').data(:,1);
            t_sen_lg1_j2 = modelsim.get('t_sen_lg1').data(:,2);
            t_sen_lg1_j3 = modelsim.get('t_sen_lg1').data(:,3);

            t_sen_lg2_j1 = modelsim.get('t_sen_lg2').data(:,1);
            t_sen_lg2_j2 = modelsim.get('t_sen_lg2').data(:,2);
            t_sen_lg2_j3 = modelsim.get('t_sen_lg2').data(:,3);

            t_sen_lg3_j1 = modelsim.get('t_sen_lg3').data(:,1);
            t_sen_lg3_j2 = modelsim.get('t_sen_lg3').data(:,2);
            t_sen_lg3_j3 = modelsim.get('t_sen_lg3').data(:,3);

            t_sen_lg4_j1 = modelsim.get('t_sen_lg4').data(:,1);
            t_sen_lg4_j2 = modelsim.get('t_sen_lg4').data(:,2);
            t_sen_lg4_j3 = modelsim.get('t_sen_lg4').data(:,3);

            theta_sen_lg1_j1 = modelsim.get('theta_sen_lg1').data(:,1);
            theta_sen_lg1_j2 = modelsim.get('theta_sen_lg1').data(:,2);
            theta_sen_lg1_j3 = modelsim.get('theta_sen_lg1').data(:,3);

            theta_sen_lg2_j1 = modelsim.get('theta_sen_lg2').data(:,1);
            theta_sen_lg2_j2 = modelsim.get('theta_sen_lg2').data(:,2);
            theta_sen_lg2_j3 = modelsim.get('theta_sen_lg2').data(:,3);

            theta_sen_lg3_j1 = modelsim.get('theta_sen_lg3').data(:,1);
            theta_sen_lg3_j2 = modelsim.get('theta_sen_lg3').data(:,2);
            theta_sen_lg3_j3 = modelsim.get('theta_sen_lg3').data(:,3);

            theta_sen_lg4_j1 = modelsim.get('theta_sen_lg4').data(:,1);
            theta_sen_lg4_j2 = modelsim.get('theta_sen_lg4').data(:,2);
            theta_sen_lg4_j3 = modelsim.get('theta_sen_lg4').data(:,3);

            body_pitch = modelsim.get('body_movement').data(:,1);
            body_height_calc = modelsim.get('body_movement').data(:,2);
            body_velocity = modelsim.get('body_velocity').data(:,1);
            
            leg1_ee_position = modelsim.get('leg1_ee').data(:,2);
            leg2_ee_position = modelsim.get('leg2_ee').data(:,2);
            leg3_ee_position = modelsim.get('leg3_ee').data(:,2);
            leg4_ee_position = modelsim.get('leg4_ee').data(:,2);
            
% cons = [c1 c2 c3 c4 c5 c6 c7 c8]
% Default value is 0, constraint violation value is 1 
%
% PRE SIMULATION CONSTRAAINTS
% c1 and c2 => Represent constraints regarding pre-simulation failures,
%                 c1) Inverse kinematics failed to get trajectories
%                 c2) Failed to build accelerator model
%
% STOP SIMULATION CONSTRAINTS
% c3 to c5 => Represents the constraint regarding a simulation stop... This happens if either
%                 c3) Maximum Pitch exceeded, 
%                 c4) Maximum height exceeded or height is negative 
%                 c5) Simulation end not reached _ due to other purposes  
%                 In this case, simulation_time = time_end
% POST SIMULATION CONSTRAINTS
% c6 to c8 => Represents the constraint regarding stability failures... This happens if either
%                 c6) Maximum Torque exceeded
%                 c7) velocity of toe or height of toe during stance is
%                 non-zero
%                 c8) any pose variable differs from it's initial value at
%                 the end of 1 complete cycle - cycle dependent on stride
%                 time - not a stable cyclic gait         
            
        %---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------  
        % Cons 3,4 and 5: Simulation did not reach end
  %%%%%%%%%%keyboard();
              
                    if isempty(temporary_time)
                        cons(5) = 1;
                    else
                        obj_ite_simulation_time = floor(100*obj_ite_simulation_time)/100;
                        temporary_time(end) = floor(100*temporary_time(end))/100;
                        if (temporary_time(end) - obj_ite_simulation_time >= 0.03)
                            cons(5) = 1;
                        end
                        if (abs(body_pitch(end)) >= body_pitch_max)
                            cons(3) = 1;
                        end 
                        if (-1*body_height_calc(end) >= body_height)
                            cons(4) = 1;
                        end
                        if (abs(body_height_calc(end)) >= 0.10)
                            cons(4) = 1;
                        end 
                    end
    
            if (~cons(1))&&(~cons(2))&&(~cons(3))&&(~cons(4))&&(~cons(5))
        %---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------  
        % Cons 6: Find positions where max torque and torque change was exceeded

                        idx1 = find(abs(t_sen_lg1_j1) >= obj_ite_max_torque, 1);                
                        idx2 = find(abs(t_sen_lg1_j2) >= obj_ite_max_torque, 1);
                        idx3 = find(abs(t_sen_lg1_j3) >= obj_ite_max_torque, 1);
                        idx4 = find(abs(t_sen_lg2_j1) >= obj_ite_max_torque, 1);
                        idx5 = find(abs(t_sen_lg2_j2) >= obj_ite_max_torque, 1);
                        idx6 = find(abs(t_sen_lg2_j3) >= obj_ite_max_torque, 1);
                        idx7 = find(abs(t_sen_lg3_j1) >= obj_ite_max_torque, 1);                
                        idx8 = find(abs(t_sen_lg3_j2) >= obj_ite_max_torque, 1);
                        idx9 = find(abs(t_sen_lg3_j3) >= obj_ite_max_torque, 1);
                        idx10 = find(abs(t_sen_lg4_j1) >= obj_ite_max_torque, 1);
                        idx11 = find(abs(t_sen_lg4_j2) >= obj_ite_max_torque, 1);
                        idx12 = find(abs(t_sen_lg4_j3) >= obj_ite_max_torque, 1);

                        idx13 = find(abs(diff(t_sen_lg1_j1)) >= obj_ite_max_torque_change, 1);                
                        idx14 = find(abs(diff(t_sen_lg1_j2)) >= obj_ite_max_torque_change, 1);
                        idx15 = find(abs(diff(t_sen_lg1_j3)) >= obj_ite_max_torque_change, 1);
                        idx16 = find(abs(diff(t_sen_lg2_j1)) >= obj_ite_max_torque_change, 1);
                        idx17 = find(abs(diff(t_sen_lg2_j2)) >= obj_ite_max_torque_change, 1);
                        idx18 = find(abs(diff(t_sen_lg2_j3)) >= obj_ite_max_torque_change, 1);
                        idx19 = find(abs(diff(t_sen_lg3_j1)) >= obj_ite_max_torque_change, 1);                
                        idx20 = find(abs(diff(t_sen_lg3_j2)) >= obj_ite_max_torque_change, 1);
                        idx21 = find(abs(diff(t_sen_lg3_j3)) >= obj_ite_max_torque_change, 1);
                        idx22 = find(abs(diff(t_sen_lg4_j1)) >= obj_ite_max_torque_change, 1);
                        idx23 = find(abs(diff(t_sen_lg4_j2)) >= obj_ite_max_torque_change, 1);
                        idx24 = find(abs(diff(t_sen_lg4_j3)) >= obj_ite_max_torque_change, 1);
                    if (~isempty(idx1))||(~isempty(idx2))||(~isempty(idx3))||(~isempty(idx4))||(~isempty(idx5))||(~isempty(idx6)||~isempty(idx7))||(~isempty(idx8))||(~isempty(idx9))||(~isempty(idx10))||(~isempty(idx11))||(~isempty(idx12))||(~isempty(idx13))||(~isempty(idx14))||(~isempty(idx15))||(~isempty(idx16))||(~isempty(idx17))||(~isempty(idx18)||~isempty(idx19))||(~isempty(idx20))||(~isempty(idx21))||(~isempty(idx22))||(~isempty(idx23))||(~isempty(idx24)) % Max torque was exceeded
                        cons(6) = 1;
                    end

        %---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------  
        % Cons 7: Checks if vertical position or horizontal velocity of toe
        % during stance is 0 or not
        ccc_1 = 0; ccc_2 = 0; ccc_3 = 0; ccc_4 = 0;
                    if (length(leg1_ee_position) == length(leg2_ee_position))&&(length(leg1_ee_position) == length(leg3_ee_position))&&(length(leg1_ee_position) == length(leg4_ee_position)) &&(length(leg_marker_FL) >= length(leg1_ee_position))
                        for kk = 1:length(leg1_ee_position)
                            % Break if z position during stance isn't 0, or x velocity isn't 0, leg
                            % marker would have value of 0 or -1
                            if((leg_marker_FL(kk)==0)||(leg_marker_FL(kk)==-1))&&(((leg1_ee_position(kk))>0.01))%||(abs(leg1_ee_velocity(kk))>0.1))%||((leg_markers(2,kk)==1))&&((abs(leg1_ee_position(kk))<0.0025))
                                ccc_1 = 1;
                                display('stance problem leg 1')
                                break
                            end
                            if((leg_marker_FR(kk)==0)||(leg_marker_FR(kk)==-1))&&(((leg2_ee_position(kk))>0.01))%||(abs(leg2_ee_velocity(kk))>0.1))%||((leg_markers(3,kk)==1))&&((abs(leg2_ee_position(kk))<0.0025))
                                ccc_2 = 1;
                                display('stance problem leg 2')
                                break
                            end
                            if((leg_marker_HL(kk)==0)||(leg_marker_HL(kk)==-1))&&(((leg3_ee_position(kk))>0.01))%||(abs(leg3_ee_velocity(kk))>0.1))%||((leg_markers(4,kk)==1))&&((abs(leg3_ee_position(kk))<0.0025))
                                ccc_3 = 1;
                                display('stance problem leg 3')
                                break
                            end
                            if((leg_marker_HR(kk)==0)||(leg_marker_HR(kk)==-1))&&(((leg4_ee_position(kk))>0.01))%||(abs(leg4_ee_velocity(kk))>0.1))%||((leg_markers(5,kk)==1))&&((abs(leg4_ee_position(kk))<0.0025))
                                ccc_4 = 1;
%                                 %%%%%%%%%%keyboard();
                                display('stance problem leg 4')
                                break
                            end
                        end
                    else
                        display('leg markers and position don`t match in length')
                        cons(7) = 1;
                    end

                    if (abs((length(find((leg1_ee_position)<=0.001))/length(leg1_ee_position)) - obj_ite_duty_factor_FL) > 0.1)||(abs((length(find((leg2_ee_position)<=0.001))/length(leg2_ee_position)) - obj_ite_duty_factor_FR) > 0.1)||(abs((length(find((leg3_ee_position)<=0.001))/length(leg3_ee_position)) - obj_ite_duty_factor_HL) > 0.1)||(abs((length(find((leg4_ee_position)<=0.001))/length(leg4_ee_position)) - obj_ite_duty_factor_HR) > 0.1)
                        
                        display('duty factor mismatch')
%                         keyboard();
                        cons(7) = 1;
                    end
                    if (ccc_1)||(ccc_2)||(ccc_3)||(ccc_4)
                        cons(7) = 1;
                    end 
                    %%%keyboard();
        %---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------              
        % Cons 8: Checks for joint angles mismatch at every AEP instance of each leg is the same...

                    if ((theta_sen_lg1_j1(1)-theta_sen_lg1_j1(end)) >= 0.035)||((theta_sen_lg1_j2(1)-theta_sen_lg1_j2(end)) >= 0.035)||((theta_sen_lg1_j3(1)-theta_sen_lg1_j3(end)) >= 0.035)||((theta_sen_lg2_j1(1)-theta_sen_lg2_j1(end)) >= 0.035)||((theta_sen_lg2_j2(1)-theta_sen_lg2_j2(end)) >= 0.035)||((theta_sen_lg2_j3(1)-theta_sen_lg2_j3(end)) >= 0.035)||((theta_sen_lg3_j1(1)-theta_sen_lg3_j1(end)) >= 0.035)||((theta_sen_lg3_j2(1)-theta_sen_lg3_j2(end)) >= 0.035)||((theta_sen_lg3_j3(1)-theta_sen_lg3_j3(end)) >= 0.035)||((theta_sen_lg4_j1(1)-theta_sen_lg4_j1(end)) >= 0.035)||((theta_sen_lg4_j2(1)-theta_sen_lg4_j2(end)) >= 0.035)||((theta_sen_lg4_j3(1)-theta_sen_lg4_j3(end)) >= 0.035)
                        cons(8) = 1;
                    end
%                     idx_FR_AEP = find(leg_markers(2,:) == -1); % leg 1 AEPs
%                     idx_FL_AEP = find(leg_markers(3,:) == -1); % leg 2 AEPs
%                     idx_HR_AEP = find(leg_markers(4,:) == -1); % leg 3 AEPs
%                     idx_HL_AEP = find(leg_markers(5,:) == -1); % leg 4 AEPs
%                     temp_len = length(leg_markers(2,:));
%                     cc_1 = 0; cc_2 = 0; cc_3 = 0; cc_4 = 0;
%                     for kk = 1:length(idx_FR_AEP)
%                         if (length(theta_sen_lg1_j1)<temp_len)||(length(theta_sen_lg1_j2)<temp_len)||((length(theta_sen_lg1_j3)<temp_len)||((theta_sen_lg1_j1(idx_FR_AEP(kk))- TRJ_LG1_J1(2,idx_FR_AEP(kk)))>= 0.0175)||((theta_sen_lg1_j2(idx_FR_AEP(kk))- TRJ_LG1_J2(2,idx_FR_AEP(kk)))>= 0.0175)||((theta_sen_lg1_j3(idx_FR_AEP(kk))- TRJ_LG1_J3(2,idx_FR_AEP(kk)))>= 0.0175))
%                             cc_1 = 1;
%                             break
%                         end
%                     end
%                     for kk = 1:length(idx_FL_AEP)
%                         if (length(theta_sen_lg2_j1)<temp_len)||(length(theta_sen_lg2_j2)<temp_len)||(length(theta_sen_lg2_j3)<temp_len)||(((theta_sen_lg2_j1(idx_FL_AEP(kk))- TRJ_LG2_J1(2,idx_FL_AEP(kk)))>= 0.0175)||((theta_sen_lg2_j2(idx_FL_AEP(kk))- TRJ_LG2_J2(2,idx_FL_AEP(kk)))>= 0.0175)||((theta_sen_lg2_j3(idx_FL_AEP(kk))- TRJ_LG2_J3(2,idx_FL_AEP(kk)))>= 0.0175))
%                             cc_2 = 1;
%                             break
%                         end
%                     end
%                     for kk = 1:length(idx_HR_AEP)
%                         if (length(theta_sen_lg3_j1)<temp_len)||(length(theta_sen_lg3_j2)<temp_len)||(length(theta_sen_lg3_j3)<temp_len)||(((theta_sen_lg3_j1(idx_HR_AEP(kk))- TRJ_LG3_J1(2,idx_HR_AEP(kk)))>= 0.0175)||((theta_sen_lg3_j2(idx_HR_AEP(kk))- TRJ_LG3_J2(2,idx_HR_AEP(kk)))>= 0.0175)||((theta_sen_lg3_j3(idx_HR_AEP(kk))- TRJ_LG3_J3(2,idx_HR_AEP(kk)))>= 0.0175))
%                             cc_3 = 1;
%                             break
%                         end
%                     end
%                     for kk = 1:length(idx_HL_AEP)
%                         if (length(theta_sen_lg4_j1)<temp_len)||(length(theta_sen_lg4_j2)<temp_len)||(length(theta_sen_lg4_j3)<temp_len)||(((theta_sen_lg4_j1(idx_HL_AEP(kk))- TRJ_LG4_J1(2,idx_HL_AEP(kk)))>= 0.0175)||((theta_sen_lg4_j2(idx_HL_AEP(kk))- TRJ_LG4_J2(2,idx_HL_AEP(kk)))>= 0.0175)||((theta_sen_lg4_j3(idx_HL_AEP(kk))- TRJ_LG4_J3(2,idx_HL_AEP(kk)))>= 0.0175))
%                             cc_4 = 1;
%                             break
%                         end
%                     end
%                     if (cc_1)||(cc_2)||(cc_3)||(cc_4)
%                         cons(8) = 1;
%                     end    

        %---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------              
        % Cons 9: Checks if velocity of main body is ever negative
        
                    idx_bv = find(body_velocity < 0, 1);
                    if ~isempty(idx_bv)
                        cons(9) = 1;
                    end

            end
%---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------              


            % completely...
%%%%%%%%%%keyboard();

            if (~cons(1))&&(~cons(2))&&(~cons(3))&&(~cons(4))&&(~cons(5))&&(~cons(6))&&(~cons(7))&&(~cons(8))&&(~cons(9))
                OF = at4_power_loss(obj_ite_stride_length, temporary_time, t_sen_lg1_j1, t_sen_lg2_j1, t_sen_lg3_j1,t_sen_lg4_j1, t_sen_lg1_j2, t_sen_lg2_j2, t_sen_lg3_j2, t_sen_lg4_j2, t_sen_lg1_j3, t_sen_lg2_j3, t_sen_lg3_j3, t_sen_lg4_j3, obj_ite_stride_time);
%                 OF = -1.*obj_ite_velocity;
                
                display('Successful run')
            else
                display('Constraint violation')
                cons
                if ((~cons(1))&&(~cons(2))&&(~cons(3))&&(~cons(4))&&(~cons(5)))
                    OF = 1000;
                elseif ((~cons(1))&&(~cons(2)))
                    OF = 4000/temporary_time(end);
                else
                    OF = 100000;
                end
            end
            
        end

time_taken_for_one_iteration = toc

end

